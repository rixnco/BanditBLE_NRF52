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


// ============================================================================
// ACCELEROMETER INITIALIZATION
// ============================================================================
#ifdef ENABLE_ACCELEROMETER

#ifdef IMU_USE_LSM6DS3
static LSM6DS3 myIMU(I2C_MODE, IMU_I2C_ADDR);
#endif

#ifdef IMU_USE_MPU6050_DMP
static MPU6050 mpu;
static uint8_t mpuIntStatus;
static uint16_t packetSize;
static uint8_t fifoBuffer[64];
static Quaternion q;
static uint32_t lastValidDMPPacket = 0;
#endif

struct {
  int16_t x;
  int16_t y;
  int16_t z;
} currentAccel = {0, 0, 0};

static bool readAcceleration();

#endif

BLEDis  bledis;  // device information
BLEUart nrfuart; // NRF uart over ble
BLEService banditService = BLEService(BANDIT_SERVICE_UUID);

#define CHARACTERISTIC_BUFFER_LENGTH 11 
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


  // ==================== ACCELEROMETER INITIALIZATION ====================
#ifdef ENABLE_ACCELEROMETER
  Wire.begin();
  Wire.setClock(400000);

#ifdef IMU_USE_LSM6DS3
  Serial.println("#Initializing LSM6DS3...");
  if (myIMU.begin() != 0) {
    Serial.println("LSM6DS3 init error!");
  } else {
    Serial.println("LSM6DS3 OK!");
    // Configure for optimal precision
    myIMU.settings.accelRange = IMU_ACCEL_SCALE;  // ±2g
    myIMU.settings.accelSampleRate = IMU_ACCEL_ODR;  // 104 Hz
    myIMU.settings.accelBandWidth = 0;  // Auto
    myIMU.settings.gyroRange = 245;  // ±245 deg/s (we don't use gyro but set it)
    myIMU.settings.gyroSampleRate = IMU_ACCEL_ODR;
  }
#endif

#ifdef IMU_USE_MPU6050_DMP
  Serial.println("#Initializing MPU6050 DMP...");
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
  } else {
    Serial.println("MPU6050 connection OK!");
    // Initialize DMP
    uint8_t devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
      mpu.setDMPEnabled(true);
      packetSize = mpu.dmpGetFIFOPacketSize();
      lastValidDMPPacket = millis();  // Initialize watchdog timer
      Serial.println("DMP initialized successfully!");
    } else {
      Serial.print("DMP init failed (code ");
      Serial.print(devStatus);
      Serial.println(")");
    }
  }
#endif

#endif

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

  // Initialize BLE characteristic buffer
  // Structure: RPM(2) | Gear(1) | Speed(2) | AccelXYZ(6) = 11 bytes
  memset(characteristic_buffer, 0, CHARACTERISTIC_BUFFER_LENGTH);

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

    #ifdef ENABLE_ACCELEROMETER
      // Read accelerometer data at same rate as BLE updates
      readAcceleration();
      
      // Accelerometer: bytes 5-10 (3x int16_t LE, normalized to milli-g)
      // Both LSM6DS3 and MPU6050 normalized to ±2000 milli-g range
      // X-axis (bytes 5-6)
      characteristic_buffer[5] = (currentAccel.x) & 0xFF;
      characteristic_buffer[6] = (currentAccel.x >> 8) & 0xFF;
      // Y-axis (bytes 7-8)
      characteristic_buffer[7] = (currentAccel.y) & 0xFF;
      characteristic_buffer[8] = (currentAccel.y >> 8) & 0xFF;
      // Z-axis (bytes 9-10)
      characteristic_buffer[9] = (currentAccel.z) & 0xFF;
      characteristic_buffer[10] = (currentAccel.z >> 8) & 0xFF;
    #else
      // Accelerometer feature disabled: force accel bytes to 0xFF
      characteristic_buffer[5] = 0xFF;
      characteristic_buffer[6] = 0xFF;
      characteristic_buffer[7] = 0xFF;
      characteristic_buffer[8] = 0xFF;
      characteristic_buffer[9] = 0xFF;
      characteristic_buffer[10] = 0xFF;
    #endif

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


// ============================================================================
// ACCELEROMETER DATA READING
// ============================================================================
#ifdef ENABLE_ACCELEROMETER

static bool readAcceleration()
{
#ifdef IMU_USE_LSM6DS3
  // Read LSM6DS3 accelerometer data
  // Normalized to milli-g (1000 = 1g)
  
  myIMU.readAccelData();
  
  // Get acceleration in g's (LSM6DS3 provides calibrated data)
  // Range: ±2g = ±2000 milli-g
  float accelX = myIMU.calcAccel(myIMU.accelX);  // in g
  float accelY = myIMU.calcAccel(myIMU.accelY);  // in g
  float accelZ = myIMU.calcAccel(myIMU.accelZ);  // in g
  
  // Convert from g to milli-g (multiply by 1000) for int16_t
  currentAccel.x = (int16_t)(accelX * 1000);
  currentAccel.y = (int16_t)(accelY * 1000);
  currentAccel.z = (int16_t)(accelZ * 1000);
  
  return true;
#endif

#ifdef IMU_USE_MPU6050_DMP
  // DMP watchdog: reset if no valid packet for too long
  uint32_t now = millis();
  if (now - lastValidDMPPacket > IMU_DMP_WATCHDOG) {
    Serial.println("#DMP watchdog: reinitializing FIFO");
    mpu.resetFIFO();
    lastValidDMPPacket = now;
  }
  
  // Read MPU6050 raw acceleration and normalize to milli-g
  // MPU6050 raw acceleration scale: LSB/g depends on range
  // At ±16g (default): 2048 LSB/g → raw_value / 2048 = acceleration in g
  // Normalized: raw_value / 2.048 = acceleration in milli-g
  
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  
  // Normalize MPU6050 raw values to milli-g
  // At ±16g: LSB/g = 2048
  // milli-g = raw * 1000 / 2048 ≈ raw * 0.488
  // For consistency with LSM (±2g = ±2000 milli-g), scale appropriately:
  // raw / 2048 (g) * 1000 = raw / 2.048 (milli-g)
  currentAccel.x = (int16_t)(ax / 2.048f);
  currentAccel.y = (int16_t)(ay / 2.048f);
  currentAccel.z = (int16_t)(az / 2.048f);
  
  // Check for DMP data (optional: for future quaternion use)
  mpuIntStatus = mpu.getIntStatus();
  uint16_t fifoCount = mpu.getFIFOCount();
  
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // FIFO overflow detected
    Serial.println("FIFO overflow, resetting...");
    mpu.resetFIFO();
    return false;
  } 
  else if (mpuIntStatus & 0x02) {
    // DMP data ready
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    lastValidDMPPacket = now;  // Update watchdog timestamp
    
    return true;
  }
  
  return true;
#endif

  return true;
}

#endif


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
