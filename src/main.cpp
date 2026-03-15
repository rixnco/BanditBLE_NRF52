#include <Arduino.h>
#include <bluefruit.h>

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include "settings.h"
#include "protocol.h"

#define SERIAL1_RX PIN_SERIAL1_RX
#define SERIAL1_TX PIN_SERIAL1_TX

#define DEBUG

#ifdef DEBUG
#define dbg_print(...) Serial.print(__VA_ARGS__)
#define dbg_printf(...) Serial.printf(__VA_ARGS__)
#define dbg_println(...) Serial.println(__VA_ARGS__)
#else
#define dbg_print(...) \
  do                   \
  {                    \
  } while (0)
#define dbg_printf(...) \
  do                   \
  {                    \
  } while (0)
#define dbg_println(...) \
  do                     \
  {                      \
  } while (0)
#endif

#define TACH_INT_PIN 2
#define REFRESH_RATE 100 // milliseconds between sensor updates
#define DEBOUNCE_TIME (3 * REFRESH_RATE)

#define LED_ON LED_STATE_ON
#define LED_OFF LED_STATE_OFF

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

static uint16_t getRPM();
static uint8_t readGEAR();

#define BUFFER_SIZE 4 // Must be a power of 2
volatile uint32_t _micros[BUFFER_SIZE];
volatile uint8_t _head;
volatile uint8_t _tail;
volatile uint32_t _size;


#define IMU_INT_PIN     7

static MPU6050 mpu;
static bool dmpReady = false;  // set true if DMP init was successful
static uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
static uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
static uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
static uint8_t fifoBuffer[64]; // FIFO storage buffer


volatile static bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


//The setup function is called once at startup of the sketch
void setup()
{
  pinMode(PIN_LED1, OUTPUT_H0D1);
  pinMode(PIN_LED2, OUTPUT_H0D1);
  pinMode(PIN_LED3, OUTPUT_H0D1);
  pinMode(PIN_LED4, OUTPUT_H0D1);
  digitalWrite(PIN_LED1, LED_OFF);    // Weak LED (HS?)
  digitalWrite(PIN_LED2, LED_OFF);    // OK
  digitalWrite(PIN_LED3, LED_OFF);    // HS
  digitalWrite(PIN_LED4, LED_OFF);    // HS



#ifdef USE_TINYUSB
  // EBO: Call it here instead of main.cpp
  TinyUSB_Device_Init(0);
#endif

#if CFG_DEBUG
  // EBO:  Call it here instead of main.cpp
  // // If Serial is not begin(), call it to avoid hard fault
  if(!Serial) Serial.begin(115200);
#endif


#ifdef DEBUG
  // Serial.setPins(SERIAL1_RX, SERIAL1_TX);
  Serial.begin(115200);
  uint32_t idx=50;
  bool state = true;
  while(!Serial) {
    yield();
    delay(10);
    if(--idx == 0) { 
      digitalWrite(PIN_LED1, state? LED_ON : LED_OFF );
      idx=50;
      state = !state;
    }
  }
  digitalWrite(PIN_LED1, LED_ON);
#endif

  dbg_println("#Starting");

  dbg_println("#Read settings");
  if (!readSettings())
  {
    dbg_println("#Default settings");
    resetSettings();
    dbg_println("#Write settings");
    writeSettings();
  }
  dbg_print("$GEAR1=");
  dbg_println(g_settings.gear1);
  dbg_print("$GEAR2=");
  dbg_println(g_settings.gear2);
  dbg_print("$GEAR3=");
  dbg_println(g_settings.gear3);
  dbg_print("$GEAR4=");
  dbg_println(g_settings.gear4);
  dbg_print("$GEAR5=");
  dbg_println(g_settings.gear5);
  dbg_print("$GEAR6=");
  dbg_println(g_settings.gear6);


  Wire.begin();
  Wire.setClock(400000);

  mpu.initialize();

  devStatus = mpu.dmpInitialize() ;
  mpu.setXGyroOffset(-7);
  mpu.setYGyroOffset(-60);
  mpu.setZGyroOffset(2);
  mpu.setXAccelOffset(-4640); 
  mpu.setYAccelOffset(-531); 
  mpu.setZAccelOffset(1044); 

  if (devStatus == 0)
  {
   dbg_println("IMU...OK");

        //Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(12);
        mpu.CalibrateGyro(12);
#ifdef DEBUG
        mpu.PrintActiveOffsets();
#endif
        mpu.setInterruptMode(0);
        mpu.setInterruptDrive(0);
        mpu.setInterruptLatch(1);
        mpu.setInterruptLatchClear(0);
        mpu.setIntDMPEnabled(true);

        pinMode(IMU_INT_PIN, INPUT);
        // We need to modify WInterrupts.c in order to change GPIOTE IRQ level from 2 to 3.
        // IRQ Level 2 is reserved for softdevice
        attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), dmpDataReady, RISING);

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();

      //   // set our DMP Ready flag so the main loop() function knows it's okay to use it
      //  Serial.println("DMP ready! Waiting for first interrupt...");
        dmpReady = true;

  }
  else
  {
     dbg_println("IMU...KO");
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

  dbg_println("#Configuring BLE...");

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behavior, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.setConnLed(LED_RED);
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


  dbg_println("#Configuring BLE...OK");

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

  // turn on the DMP, now that it's ready
  if(dmpReady) {
    dbg_println("Enabling DMP...");

    mpuIntStatus = mpu.getIntStatus();
    mpuInterrupt = false;
    mpu.setDMPEnabled(true);
  }

  // Start Advertising
  startAdv();
  dbg_println("#Bandit BLE...started");
}

// The loop function is called in an endless loop
uint32_t previousMillis = 0;
bool ledState = LOW;
uint8_t lastGear = -1;
uint32_t lastGEARChanged = 0;

static uint8_t characteristic_buffer[CHARACTERISTIC_BUFFER_LENGTH];

void loop()
{

  if(mpuInterrupt) {
    mpuIntStatus = mpu.getIntStatus();
    mpuInterrupt = false;
    if(mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      // Update BLE buffer
      characteristic_buffer[3] = fifoBuffer[1];    // qw_l
      characteristic_buffer[4] = fifoBuffer[0];    // qw_h
      characteristic_buffer[5] = fifoBuffer[5];    // qx_l
      characteristic_buffer[6] = fifoBuffer[4];    // qx_h
      characteristic_buffer[7] = fifoBuffer[9];    // qy_l
      characteristic_buffer[8] = fifoBuffer[8];    // qy_h
      characteristic_buffer[9] = fifoBuffer[13];   // qz_l
      characteristic_buffer[10] = fifoBuffer[12];  // qz_h

      characteristic_buffer[11] = fifoBuffer[29];  // ax_l
      characteristic_buffer[12] = fifoBuffer[28];  // ax_h
      characteristic_buffer[13] = fifoBuffer[33];  // ay_l
      characteristic_buffer[14] = fifoBuffer[32];  // ay_h
      characteristic_buffer[15] = fifoBuffer[37];  // az_l
      characteristic_buffer[16] = fifoBuffer[36];  // az_h
    }
  }

  //processInput();
  uint32_t now = millis();
  if (now - previousMillis > REFRESH_RATE)
  {
    uint16_t RPM = getRPM();

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
    else
    {
      // ledState = !ledState;
      // digitalWrite(LED_PIN, ledState);
    }

#ifdef DEBUG
      static uint16_t idx=0;
      if (!idx)
      {
        dbg_print("gear=");
        dbg_print(currentGear);
        dbg_print(" rpm=");
        dbg_print(RPM);
        dbg_print(" Q=[");
        dbg_print((int16_t)((characteristic_buffer[4]<<8)|characteristic_buffer[3]));
        dbg_print(',');
        dbg_print((int16_t)((characteristic_buffer[6]<<8)|characteristic_buffer[5]));
        dbg_print(',');
        dbg_print((int16_t)((characteristic_buffer[8]<<8)|characteristic_buffer[7]));
        dbg_print(',');
        dbg_print((int16_t)((characteristic_buffer[10]<<8)|characteristic_buffer[9]));
        dbg_println(']');

      }
      idx = (idx + 1) % 10;
#endif


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
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);


  // Serial.printf("Connection MTU: %d\n", connection->getMtu());
  
  // // request mtu exchange
  // Serial.printf("Request to change MTU: %d\n",128);
  // connection->requestMtuExchange(128);

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
