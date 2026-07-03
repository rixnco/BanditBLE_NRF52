#include <Arduino.h>
#include <bluefruit.h>
#include "LSM6DS3.h"
#include "Wire.h"

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#include "config.h"
#include "settings.h"
#include "protocol.h"



#define REFRESH_RATE          100 // milliseconds between sensor updates
#define BLE_NOTIFY_PERIOD_MS  REFRESH_RATE // BLE notification interval (ms)
#define DEBOUNCE_TIME         (3 * REFRESH_RATE)


#define BUFFER_SIZE 4 // Must be a power of 2



#define BANDIT_SERVICE_UUID  "2E290000-1EF9-11E9-AB14-D663BD873D93"
#define BANDIT_RPM_CHAR_UUID "2E290001-1EF9-11E9-AB14-D663BD873D93"


BLEDis  bledis;  // device information
BLEUart nrfuart; // NRF uart over ble
BLEService banditService = BLEService(BANDIT_SERVICE_UUID);

#define CHARACTERISTIC_BUFFER_LENGTH 18 
BLECharacteristic rpmCharacteristic = BLECharacteristic(BANDIT_RPM_CHAR_UUID, BLERead | BLENotify, CHARACTERISTIC_BUFFER_LENGTH, true);


static void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value);
static void disconnect_callback(uint16_t conn_handle, uint8_t reason);
static void connect_callback(uint16_t conn_handle);
static void startAdv(void);

uint8_t currentGear = -1;
uint16_t currentRPM = 0;
uint16_t currentGearRaw = 0;

static uint16_t getRPM();
static uint8_t readGEAR(uint16_t* adcValue = nullptr);

#ifdef ENABLE_SPEED_SENSOR
uint16_t currentSpeed = 0;  // km/h
static uint16_t getSpeed();
volatile uint32_t _wheelMicros[BUFFER_SIZE];
volatile uint8_t _wheelHead, _wheelTail, _wheelSize;
#endif

// Crankshaft (RPM) buffer
volatile uint32_t _crankMicros[BUFFER_SIZE];
volatile uint8_t _crankHead, _crankTail, _crankSize;



volatile bool _override = false;
volatile int _overrideRPM = 0;
volatile int _overrideGear = 0;


// #define IMU_WATCHDOG_TIMEOUT  5000  // Reset DMP if no valid packet for 5s
// #define IMU_INT_PIN           7
// MPU6050 DMP variables (kept for reference, LSM6DS3 used for now)
// static LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A
// static uint32_t lastValidIMUPacket = 0;   // Timestamp of last valid quaternion


void setOverride(bool override, int rpm, int gear) {
  _override = override;
  _overrideRPM = rpm;
  _overrideGear = gear;
  digitalWrite(LED_RED, _override?LED_ON:LED_OFF);
}

// ============================================================================
// SENSOR BUFFER MANAGEMENT - Dual sensor support (crankshaft + wheel ABS)
// ============================================================================

void crankshaft_buffer_init() {
  _crankHead = _crankTail = _crankSize = 0;
}

#if ENABLE_SPEED_SENSOR
void wheel_buffer_init() {
  _wheelHead = _wheelTail = _wheelSize = 0;
}
#endif


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

  initSettingsStorage();

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


  // Wire.begin();
  // Wire.setClock(400000);

  // if (myIMU.begin() != 0) {
  //     Serial.println("myIMU error");
  // } else {
  //     Serial.println("myIMU OK!");
  // }


  // Configure INPUT pin
  uint16_t upin = g_ADigitalPinMap[CRANKSHAFT_INT_PIN];
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
  NRF_TIMER2->CC[0] = CRANKSHAFT_TEETH;                                                       // Compare value (= teeth per revolution)
  NRF_TIMER2->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos; // Enable Compare0/clear shorts
  NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;

  // Configure PPI channel 0 to increase counter whenever INPUT goes from Hi to Lo
  NRF_PPI->CH[0].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[3];
  NRF_PPI->CH[0].TEP = (uint32_t)&NRF_TIMER2->TASKS_COUNT;
  // Enable PPI channel 0
  NRF_PPI->CHEN = (PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos);

  // ==================== WHEEL ABS SENSOR CONFIGURATION ====================
#if ENABLE_SPEED_SENSOR

  // Configure WHEEL_INT_PIN (P0.05) as INPUT
  uint16_t wheel_pin = g_ADigitalPinMap[WHEEL_INT_PIN];
  NRF_GPIO->PIN_CNF[wheel_pin] =
      ((uint32_t)GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
      ((uint32_t)GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
      ((uint32_t)GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
      ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
      ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

  // Configure GPIOTE[4] in EVENT mode for wheel ABS sensor
  NRF_GPIOTE->CONFIG[4] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
  NRF_GPIOTE->CONFIG[4] |= ((wheel_pin << GPIOTE_CONFIG_PSEL_Pos) & GPIOTE_CONFIG_PSEL_Msk) |
                           ((GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) & GPIOTE_CONFIG_POLARITY_Msk);

  // Enable GPIOTE interrupt for wheel sensor
  NRF_GPIOTE->INTENSET = (GPIOTE_INTENSET_IN4_Enabled << GPIOTE_INTENSET_IN4_Pos);
  NVIC_EnableIRQ(GPIOTE_IRQn);
#endif
  
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

  // Init RPM and Speed circular buffers
  crankshaft_buffer_init();
#if ENABLE_SPEED_SENSOR
  wheel_buffer_init();
#endif

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
  
  // // IMU watchdog: reset if no valid packet for too long
  // uint32_t now = millis();
  // if (now - lastValidIMUPacket > IMU_WATCHDOG_TIMEOUT) {
  //   Serial.println("#IMU watchdog: reinitializing");
  //   lastValidIMUPacket = now;
  //   // In a real MPU6050 DMP scenario, you'd reset the DMP here:
  //   // mpu.resetFIFO();
  //   // mpu.setDMPEnabled(false);
  //   // delay(10);
  //   // mpu.setDMPEnabled(true);
  // }
  uint32_t now = millis();

  if (now - previousMillis > BLE_NOTIFY_PERIOD_MS)
  {
    uint16_t RPM = getRPM();
    currentRPM = RPM;

#ifdef ENABLE_SPEED_SENSOR
    uint16_t speed = getSpeed();
    currentSpeed = speed;
#endif

    uint16_t gearRaw = 0;
    uint8_t g = readGEAR(&gearRaw);
    if (g != lastGear)
    {
      lastGEARChanged = now;
    }
    lastGear = g;

    if (lastGEARChanged && millis() - lastGEARChanged >= DEBOUNCE_TIME)
    {
      currentGear = g;
      currentGearRaw = gearRaw;
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
    #ifdef ENABLE_SPEED_SENSOR
      // Speed: bytes 3-4 (uint16_t LE)
      characteristic_buffer[3] = (speed) & 0xFF;
      characteristic_buffer[4] = (speed >> 8) & 0xFF;
    #else
      // Speed feature disabled: keep payload format and force speed bytes to 0xFF
      characteristic_buffer[3] = 0xFF;
      characteristic_buffer[4] = 0xFF;
    #endif
      // Bytes 5-17 reserved for quaternion/future data
      rpmCharacteristic.notify((const unsigned char *)characteristic_buffer, CHARACTERISTIC_BUFFER_LENGTH);
      
      // // Mark successful update for IMU watchdog
      // if(RPM > 0 || speed > 0) {
      //   lastValidIMUPacket = now;
      // }
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
  uint32_t m1 = _crankMicros[_crankHead];
  uint32_t m0 = _crankMicros[_crankTail];
  if (_crankSize > 0 && micros() - m1 > 250000)
  {
    // Reset buffer if last value older than 250ms.
    _crankSize = 0;
    _crankHead = 0;
    _crankTail = 0;
  }
  uint8_t s = _crankSize;
  interrupts();

  if (s < 2)
    return 0;

  uint32_t RPM = (s - 1) * 6000000 / (m1 - m0); // RPM x 10

  return (uint16_t)RPM;
}

#if ENABLE_SPEED_SENSOR
// checks accumulated wheel ABS pulses and calculates vehicle speed
// returns vehicle speed in km/h
// Formula: (pulses_per_second * 60 / WHEEL_ABS_TEETH) * (WHEEL_CIRCUMF_MM / 1000) * 3.6
static uint16_t getSpeed()
{
  noInterrupts();
  uint32_t m1 = _wheelMicros[_wheelHead];
  uint32_t m0 = _wheelMicros[_wheelTail];
  if (_wheelSize > 0 && micros() - m1 > 500000)
  {
    // Reset buffer if last value older than 500ms (vehicle stopped)
    _wheelSize = 0;
    _wheelHead = 0;
    _wheelTail = 0;
  }
  uint8_t s = _wheelSize;
  interrupts();

  if (s < 2)
    return 0;

  // Calculate frequency: Hz = (s-1) / (m1-m0) * 1,000,000
  // Speed = (Hz / WHEEL_ABS_TEETH * 60) * (WHEEL_CIRCUMF_MM / 1000) * 3.6
  // Simplified: Speed (km/h) = (s-1) * 60 * 3.6 * WHEEL_CIRCUMF_MM / (WHEEL_ABS_TEETH * (m1-m0) * 1000)
  uint32_t speed = (s - 1) * 60 * 3600 * WHEEL_CIRCUMF_MM / (WHEEL_ABS_TEETH * (m1 - m0) * 1000);

  return (uint16_t)(speed > 300 ? 0 : speed);  // Sanity check: max ~300 km/h on motorcycle
}
#endif


static uint8_t readGEAR(uint16_t* rawValue)
{
  if(_override) {
    if (rawValue != nullptr) {
      *rawValue = 0;
    }
    return _overrideGear;
  }

  analogReadResolution(8);
  uint16_t g = analogRead(GEAR_PIN);
  if (rawValue != nullptr) {
    *rawValue = g;
  }

  // Thresholds are stored in settings and can be tuned through serial protocol.
  if (g < g_settings.gear1)
    g = 1;
  else if (g < g_settings.gear2)
    g = 2;
  else if (g < g_settings.gear3)
    g = 3;
  else if (g < g_settings.gear4)
    g = 4;
  else if (g < g_settings.gear5)
    g = 5;
  else if (g < g_settings.gear6)
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
    // Crankshaft VRS sensor interrupt handler

    if (NRF_TIMER2->EVENTS_COMPARE[0] == 1)
    {
      NRF_TIMER2->EVENTS_COMPARE[0] = 0;

      _crankHead = _crankSize == 0 ? _crankHead : (++_crankHead) & (BUFFER_SIZE - 1);
      _crankMicros[_crankHead] = micros();

      if (_crankSize == BUFFER_SIZE)
      {
        _crankTail = (++_crankTail) & (BUFFER_SIZE - 1);
      }
      else
      {
        ++_crankSize;
      }
    }
  }

  void GPIOTE_IRQHandler(void)
  {
#if ENABLE_SPEED_SENSOR
    // Wheel ABS sensor interrupt handler (GPIOTE[4])
    
    if ((NRF_GPIOTE->EVENTS_IN[4] == 1) && (NRF_GPIOTE->INTENSET & (1 << 4)))
    {
      NRF_GPIOTE->EVENTS_IN[4] = 0;
      
      _wheelHead = _wheelSize == 0 ? _wheelHead : (++_wheelHead) & (BUFFER_SIZE - 1);
      _wheelMicros[_wheelHead] = micros();

      if (_wheelSize == BUFFER_SIZE)
      {
        _wheelTail = (++_wheelTail) & (BUFFER_SIZE - 1);
      }
      else
      {
        ++_wheelSize;
      }
    }
#endif
  }
}
