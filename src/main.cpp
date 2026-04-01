#include <Arduino.h>
#include <bluefruit.h>
#include "LSM6DS3.h"
#include "Wire.h"

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#include "settings.h"
#include "protocol.h"

#include <SPI.h>

#include "SdFat.h"
#include <Adafruit_SPIFlash.h>

// for flashTransport definition
#include "flash_config.h"


// Flash Chip declerations, made up from Data Sheets.
//  https://files.seeedstudio.com/wiki/github_weiruanexample/Flash_P25Q16H-UXH-IR_Datasheet.pdf
SPIFlash_Device_t const p25q16h{
  .total_size = (1UL << 21),  // 2MiB
  .start_up_time_us = 10000,
  .manufacturer_id = 0x85,
  .memory_type = 0x60,
  .capacity = 0x15,
  .max_clock_speed_mhz = 55,
  .quad_enable_bit_mask = 0x02,
  .has_sector_protection = 1,
  .supports_fast_read = 1,
  .supports_qspi = 1,
  .supports_qspi_writes = 1,
  .write_status_register_split = 1,
  .single_status_byte = 0,
  .is_fram = 0,
};





Adafruit_SPIFlash flash(&flashTransport);



#define TACH_INT_PIN 2
#define REFRESH_RATE 100 // milliseconds between sensor updates
#define DEBOUNCE_TIME (3 * REFRESH_RATE)

#define LED_ON LED_STATE_ON
#define LED_OFF !LED_STATE_ON

#define LED_PIN LED_RED

#define GEAR_PIN PIN_A2

#define BANDIT_SERVICE_UUID "2E290000-1EF9-11E9-AB14-D663BD873D93"
#define BANDIT_RPM_CHAR_UUID "2E290001-1EF9-11E9-AB14-D663BD873D93"


static BLEDis  bledis;  // device information
static BLEUart nrfuart; // NRF uart over ble
static BLEService banditService = BLEService(BANDIT_SERVICE_UUID);

#define CHARACTERISTIC_BUFFER_LENGTH 18 
BLECharacteristic rpmCharacteristic = BLECharacteristic(BANDIT_RPM_CHAR_UUID, BLERead | BLENotify, CHARACTERISTIC_BUFFER_LENGTH, true);


static void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value);
static void disconnect_callback(uint16_t conn_handle, uint8_t reason);
static void connect_callback(uint16_t conn_handle);
static void startAdv(void);

uint8_t currentGear = -1;
uint16_t currentRPM = 0;

static uint16_t getRPM();
static uint8_t readGEAR();

#define BUFFER_SIZE 4 // Must be a power of 2
volatile uint32_t _micros[BUFFER_SIZE];
volatile uint8_t _head;
volatile uint8_t _tail;
volatile uint32_t _size;


#define IMU_INT_PIN     7

volatile bool _override = false;
volatile int _overrideRPM = 0;
volatile int _overrideGear = 0;


//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A


void setOverride(bool override, int rpm, int gear) {
  _override = override;
  _overrideRPM = rpm;
  _overrideGear = gear;
  digitalWrite(LED_RED, _override?LED_ON:LED_OFF);
}



//The setup function is called once at startup of the sketch
void setup()
{

  Serial.begin(115200);
  uint32_t idx=50;
  bool state = true;
  uint32_t now = millis();
  while(!Serial && millis()-now<5000) {
    yield();
    delay(10);
    if(--idx == 0) { 
      digitalWrite(LED_GREEN, state? LED_ON : LED_OFF );
      idx=50;
      state = !state;
    }
  }
  digitalWrite(LED_GREEN, Serial?LED_ON:LED_OFF);
  digitalWrite(LED_RED, _override?LED_ON:LED_OFF);

  Serial.println("#Starting");

  Serial.println("#Read settings");
  if (!readSettings())
  {
    Serial.println("#Default settings");
    resetSettings();
    Serial.println("#Write settings");
    writeSettings();
  }
  Serial.print("$GEAR1=");
  Serial.println(g_settings.gear1);
  Serial.print("$GEAR2=");
  Serial.println(g_settings.gear2);
  Serial.print("$GEAR3=");
  Serial.println(g_settings.gear3);
  Serial.print("$GEAR4=");
  Serial.println(g_settings.gear4);
  Serial.print("$GEAR5=");
  Serial.println(g_settings.gear5);
  Serial.print("$GEAR6=");
  Serial.println(g_settings.gear6);


  Wire.begin();
  Wire.setClock(400000);

  if (myIMU.begin() != 0) {
      Serial.println("myIMU error");
  } else {
      Serial.println("myIMU OK!");
  }


  Serial.println("Adafruit Serial Flash Info example");
  if(flash.begin(&p25q16h, 1)) {

    // Using a flash device not already listed? Start the flash memory by passing
    // it the array of device settings defined above, and the number of elements
    // in the array.
    // flash.begin(my_flash_devices, flashDevices);

    uint32_t jedec_id = flash.getJEDECID();
    Serial.print("JEDEC ID: 0x");
    Serial.println(jedec_id, HEX);
    Serial.print("Flash size (usable): ");
    Serial.print(flash.size() / 1024);
    Serial.println(" KB");
  } else {
    Serial.println("Flash...KO");
  }

  // Configure INPUT pin
  uint16_t upin = g_ADigitalPinMap[TACH_INT_PIN];
  NRF_GPIO->PIN_CNF[upin] =
      ((uint32_t)GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) | ((uint32_t)GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | ((uint32_t)GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

  // Configure GPIOTE	3 in EVENT mode to sense INPUT pin
  NRF_GPIOTE->CONFIG[3] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
  NRF_GPIOTE->CONFIG[3] |= ((upin << GPIOTE_CONFIG_PSEL_Pos) & GPIOTE_CONFIG_PSEL_Msk) |
                           ((GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) & GPIOTE_CONFIG_POLARITY_Msk);

  // Configure Timer1 as counter mode
  NVIC_DisableIRQ(TIMER2_IRQn);
  NRF_TIMER2->MODE = TIMER_MODE_MODE_Counter;                                                  // Set the timer in Counter Mode
  NRF_TIMER2->TASKS_CLEAR = 1;                                                                 // clear the task first to be usable for later
  NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_08Bit;                                           //Set counter to 08 bit resolution
  NRF_TIMER2->CC[0] = 22;                                                                      // Compare value
  NRF_TIMER2->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos; // Enable Compare0/clear shorts
  NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;

  // Configure PPI channel 0 to increase counter whenever INPUT goes from Hi to Lo
  NRF_PPI->CH[0].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[3];
  NRF_PPI->CH[0].TEP = (uint32_t)&NRF_TIMER2->TASKS_COUNT;
  // Enable PPI channel 0
  NRF_PPI->CHEN = (PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos);

  Serial.println("#Configuring BLE...");

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behavior, but provided
  // here in case you want to control this LED manually via PIN 19
  // Bluefruit.setConnLed(LED_RED);
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_HIGH);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);        // Check bluefruit.h for supported values
  Bluefruit.setName("BANDIT");    // useful testing with multiple central connections
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Configure and Start Device Information Service
  bledis.setManufacturer("Rix");
  bledis.setModel("BANDIT RPM");
  bledis.begin();

  // Configure and Start BANDIT Service
  banditService.begin();
  rpmCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  rpmCharacteristic.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  rpmCharacteristic.begin();

  // Configure and Start UART Service
  nrfuart.begin();


  Serial.println("#Configuring BLE...OK");

  // Init RPM circular buffer
  _head = 0;
  _tail = 0;
  _size = 0;
  _micros[_head] = 0;

  // Start Counting
  NVIC_ClearPendingIRQ(TIMER2_IRQn);
  NVIC_SetPriority(TIMER2_IRQn, 3);
  NVIC_EnableIRQ(TIMER2_IRQn);
  NRF_TIMER2->TASKS_START = 1;


  // Start Advertising
  startAdv();
  Serial.println("#Bandit BLE...started");
}

// The loop function is called in an endless loop
uint32_t previousMillis = 0;
uint8_t lastGear = -1;
uint32_t lastGEARChanged = 0;

static uint8_t characteristic_buffer[CHARACTERISTIC_BUFFER_LENGTH];

void loop()
{
  digitalWrite(LED_GREEN, Serial?LED_ON:LED_OFF);

  processInput();
  

  uint32_t now = millis();

  if (now - previousMillis > REFRESH_RATE)
  {
    uint16_t RPM = getRPM();
    currentRPM = RPM;

    uint8_t g = readGEAR();
    if (g != lastGear)
    {
      lastGEARChanged = now;
    }
    lastGear = g;

    if (lastGEARChanged && millis() - lastGEARChanged >= DEBOUNCE_TIME)
    {
      currentGear = g;
      lastGEARChanged = 0;
    }

    if (Bluefruit.connected())
    {
      // Note: We use .notify instead of .write!
      // If it is connected but CCCD is not enabled
      // The characteristic's value is still updated although notification is not sent
      characteristic_buffer[0] = (RPM)&0xFF;
      characteristic_buffer[1] = (RPM >> 8) & 0xFF;
      characteristic_buffer[2] = currentGear;
      rpmCharacteristic.notify((const unsigned char *)characteristic_buffer, CHARACTERISTIC_BUFFER_LENGTH);
    }




    previousMillis = now;
  }
}


static void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include 'Bandit Service' in Advertising packet
  Bluefruit.Advertising.addService(banditService);

  // Secondary Scan Response packet (optional)
  Bluefruit.ScanResponse.addName();
  Bluefruit.ScanResponse.addService(nrfuart);

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}



// checks accumulated tach signal pulses and calculates engine speed
// returns engine speed in RPM
// Resolution: 30000 * engineCycles / refreshInterval / engineCylinders RPM (for default values = 20 RPM)
static uint16_t getRPM()
{
  if(_override) {
    return _overrideRPM;
  }

  noInterrupts();
  uint32_t m1 = _micros[_head];
  uint32_t m0 = _micros[_tail];
  if (_size > 0 && micros() - m1 > 250000)
  {
    // Reset buffer if last value older than 250ms.
    _size = 0;
    _head = 0;
    _tail = 0;
  }
  uint8_t s = _size;
  interrupts();

  if (s < 2)
    return 0;

  uint32_t RPM = (s - 1) * 6000000 / (m1 - m0); // RPM x 10

  return (uint16_t)RPM;
}

static uint8_t readGEAR()
{
  if(_override) {
    return _overrideGear;
  }

  analogReadResolution(8);
  uint16_t g = analogRead(GEAR_PIN);

  //  90 / 114 / 149 / 179 / 212 / 227 / 242

  if (g < 102)
    g = 1;
  else if (g < 131)
    g = 2;
  else if (g < 164)
    g = 3;
  else if (g < 195)
    g = 4;
  else if (g < 219)
    g = 5;
  else if (g < 234)
    g = 6;
  else
    g = 0;

  return (uint8_t)(g);
}


// callback invoked when central connects
static void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name)-1);

  Serial.print("Connected to ");
  Serial.println(central_name);

}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
static void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}

static void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value)
{
    // Display the raw request packet
    Serial.print("CCCD Updated: ");
    //Serial.printBuffer(request->data, request->len);
    Serial.print(cccd_value);
    Serial.println("");

    // Check the characteristic this CCCD update is associated with in case
    // this handler is used for multiple CCCD records.
    if (chr->uuid == rpmCharacteristic.uuid) {
        if (chr->notifyEnabled(conn_hdl)) {
            Serial.println("RPM Measurement 'Notify' enabled");
        } else {
            Serial.println("RPM Measurement 'Notify' disabled");
        }
    }
}



extern "C"
{
  void TIMER2_IRQHandler(void)
  {

    if (NRF_TIMER2->EVENTS_COMPARE[0] == 1)
    {
      NRF_TIMER2->EVENTS_COMPARE[0] = 0;

      _head = _size == 0 ? _head : (++_head) & (BUFFER_SIZE - 1);
      _micros[_head] = micros();

      if (_size == BUFFER_SIZE)
      {
        _tail = (++_tail) & (BUFFER_SIZE - 1);
      }
      else
      {
        ++_size;
      }
    }
  }
}
