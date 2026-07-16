#include <Arduino.h>
#include <bluefruit.h>
#include "LSM6DS3.h"
#include "Wire.h"

#include <I2Cdev.h>
#include <MPU6050.h>

#include "config.h"
#include "settings.h"
#include "SettingsStore.h"
#include "SettingsManager.h"
#include "ProtocolHandler.h"
#include "protocol.h"



#define REFRESH_RATE          100 // milliseconds between sensor updates
#define BLE_NOTIFY_PERIOD_MS  REFRESH_RATE // BLE notification interval (ms)
#define DEBOUNCE_TIME         (3 * REFRESH_RATE)


#define RPM_TIMESTAMP_BUFFER_SIZE 4 // Must be a power of 2



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

static uint8_t characteristic_buffer[CHARACTERISTIC_BUFFER_LENGTH];

static uint16_t getRPM();
static uint8_t readGEAR(uint16_t* adcValue = nullptr);

#ifdef ENABLE_SPEED_SENSOR
uint16_t currentSpeed = 0;  // km/h
static uint16_t getSpeed();
volatile uint32_t _wheelMicros[RPM_TIMESTAMP_BUFFER_SIZE];
volatile uint8_t _wheelHead, _wheelTail, _wheelSize;
#endif

// Crankshaft (RPM) buffer
volatile uint32_t _crankMicros[RPM_TIMESTAMP_BUFFER_SIZE];
volatile uint8_t _crankHead, _crankTail, _crankSize;



volatile bool _override = false;
volatile int _overrideRPM = 0;
volatile int _overrideGear = 0;


void setOverride(bool override, int rpm, int gear) {
  _override = override;
  _overrideRPM = rpm;
  _overrideGear = gear;
  digitalWrite(LED_RED, _override?LED_ON:LED_OFF);
}

// Single settings instance (unique source of truth), backed by the
// compile-time selected persistence store.
static SettingsManager settings(getDefaultSettingsStore());

#ifdef ENABLE_ACCELEROMETER
void calibrateIMU();
#endif

// Application façade exposed to the protocol layer through interfaces,
// replacing the previous extern-global coupling.
class BanditApp : public SensorProvider, public BanditController {
public:
  uint8_t  getCurrentGear() override    { return currentGear; }
  uint16_t getCurrentRPM() override     { return currentRPM; }
  uint16_t getCurrentGearRaw() override { return currentGearRaw; }

  void onOverride(bool enabled, int rpm, int gear) override {
    setOverride(enabled, rpm, gear);
  }

  void onCalibrateIMU() override {
#ifdef ENABLE_ACCELEROMETER
    calibrateIMU();
#endif
  }
};

static BanditApp banditApp;


// ============================================================================
// SENSOR BUFFER MANAGEMENT - Dual sensor support (crankshaft + wheel ABS)
// ============================================================================

void crankshaft_buffer_init() {
  _crankHead = _crankTail = _crankSize = 0;
}

#ifdef ENABLE_SPEED_SENSOR
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

  settings.init();

  Serial.println("#Read settings");
  if (!settings.load())
  {
    Serial.println("#Default settings");
    Serial.println("#Write settings");
    settings.save();
  }
  for (uint8_t i = 1; i <= 6; ++i)
  {
    Serial.print("$GEAR");
    Serial.print(i);
    Serial.print("=");
    Serial.println(settings.getGear(i));
  }


  // ==================== ACCELEROMETER INITIALIZATION ====================
#ifdef ENABLE_ACCELEROMETER
  // Initialize I2C buses with correct pin assignments for nRF52
  // Note: nRF52 Wire.begin() uses default pins from variant
  // For custom pins, use Wire.setPins() if available
  
#ifdef IMU_USE_LSM6DS3
  // LSM6DS3 on I2C1 (Wire1): SDA=P0_07, SCL=P0_27
  // Try to configure Wire1 with custom pins (nRF52 specific)
  #if defined(NRF52)
    Wire1.setPins(IMU_I2C_SDA, IMU_I2C_SCL);  // P0_07, P0_27
  #endif
  Wire1.begin();
  Wire1.setClock(400000);
  Serial.println("#I2C1 initialized for LSM6DS3 (SDA=P0_07, SCL=P0_27)");
#endif

#ifdef IMU_USE_MPU6050_DMP
  // MPU6050 on I2C0 (Wire): SDA=P0_04, SCL=P0_05
  // Configure Wire with custom pins for MPU6050
  #if defined(NRF52)
    Wire.setPins(IMU_I2C_SDA, IMU_I2C_SCL);  // P0_04, P0_05
  #endif
  Wire.begin();
  Wire.setClock(400000);
  Serial.println("#I2C0 initialized for MPU6050 (SDA=P0_04, SCL=P0_05)");
#endif

#if !defined(IMU_USE_LSM6DS3) && !defined(IMU_USE_MPU6050_DMP)
  // Fallback: initialize default Wire
  Wire.begin();
  Wire.setClock(400000);
#endif

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
  Serial.println("#Initializing MPU6050...");
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
  } else {
    Serial.println("MPU6050 connection OK!");
    
    // Configure accelerometer full-scale range
    // 0=±2g, 1=±8g, 2=±16g (IMU_ACCEL_RANGE default=1 for ±8g)
    mpu.setFullScaleAccelRange(IMU_ACCEL_RANGE);
    Serial.print("#MPU6050 Full-Scale: ±");
    Serial.print((2 << IMU_ACCEL_RANGE));
    Serial.println("g");
    
    // Configure Digital Low Pass Filter (DLPF)
    // Reduces noise, especially useful for motorcycle vibrations
    // Mode 4 = 21Hz cutoff frequency (good balance between noise and responsiveness)
    mpu.setDLPFMode(IMU_DLPF_BW);
    Serial.print("#MPU6050 DLPF mode: ");
    Serial.println(IMU_DLPF_BW);
    
#ifdef IMU_ZERO_CALIB
    // Zero-g calibration: read 100 samples at rest to get average offsets
    // This compensates for manufacturing offsets and orientation bias
    if (getRPM() == 0)  // Only calibrate when engine is stopped
    {
      Serial.println("#Calibrating MPU6050 zero-g offsets...");
      int16_t ax_sum = 0, ay_sum = 0, az_sum = 0;
      const int calib_samples = 100;
      
      for (int i = 0; i < calib_samples; i++) {
        int16_t ax, ay, az;
        mpu.getAcceleration(&ax, &ay, &az);
        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;
        delay(10);
      }
      
      // Calculate average offsets (works at any orientation: horizontal or kickstand)
      int16_t ax_offset = ax_sum / calib_samples;
      int16_t ay_offset = ay_sum / calib_samples;
      int16_t az_offset = az_sum / calib_samples;
      // Note: We use raw offsets without gravity compensation.
      // This ensures calibration works whether bike is upright, on kickstand, or at any angle.
      
      mpu.setXAccelOffset(ax_offset);
      mpu.setYAccelOffset(ay_offset);
      mpu.setZAccelOffset(az_offset);
      Serial.print("#MPU6050 offsets: X=");
      Serial.print(ax_offset);
      Serial.print(" Y=");
      Serial.print(ay_offset);
      Serial.print(" Z=");
      Serial.println(az_offset);
    }
#endif
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
#ifdef ENABLE_SPEED_SENSOR

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
#ifdef ENABLE_SPEED_SENSOR
  wheel_buffer_init();
#endif

  // Initialize BLE characteristic buffer to 0
  memset(characteristic_buffer, 0, CHARACTERISTIC_BUFFER_LENGTH);

  // Start Counting
  NVIC_ClearPendingIRQ(TIMER2_IRQn);
  NVIC_SetPriority(TIMER2_IRQn, 3);
  NVIC_EnableIRQ(TIMER2_IRQn);
  NRF_TIMER2->TASKS_START = 1;

  // Initialize Protocol Handler
  initProtocol(settings, banditApp, banditApp);

  // Start Advertising
  startAdv();
  Serial.println("#Bandit BLE...started");
}

// The loop function is called in an endless loop
uint32_t previousMillis = 0;
uint8_t lastGear = -1;
uint32_t lastGEARChanged = 0;

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

#ifdef ENABLE_SPEED_SENSOR
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
  if (g < settings.getGear(1))
    g = 1;
  else if (g < settings.getGear(2))
    g = 2;
  else if (g < settings.getGear(3))
    g = 3;
  else if (g < settings.getGear(4))
    g = 4;
  else if (g < settings.getGear(5))
    g = 5;
  else if (g < settings.getGear(6))
    g = 6;
  else
    g = 0;

  return (uint8_t)(g);
}


// ============================================================================
// ACCELEROMETER CALIBRATION
// ============================================================================
#ifdef ENABLE_ACCELEROMETER

void calibrateIMU()
{
  Serial.println("#IMU calibration starting...");
  
#ifdef IMU_USE_LSM6DS3
  // LSM6DS3 software calibration: read 100 samples and store averages
  float ax_sum = 0.0f, ay_sum = 0.0f, az_sum = 0.0f;
  const int calib_samples = 100;
  
  for (int i = 0; i < calib_samples; i++) {
    float ax = myIMU.readFloatAccelX();
    float ay = myIMU.readFloatAccelY();
    float az = myIMU.readFloatAccelZ();
    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    delay(10);
  }
  
  // Calculate average offsets in milli-g
  int16_t imu_x = (int16_t)(ax_sum / calib_samples * 1000.0f);
  int16_t imu_y = (int16_t)(ay_sum / calib_samples * 1000.0f);
  int16_t imu_z = (int16_t)(az_sum / calib_samples * 1000.0f);
  settings.setImuOffsets(imu_x, imu_y, imu_z);
  
  Serial.print("#LSM6DS3 calibration: X=");
  Serial.print(imu_x);
  Serial.print(" Y=");
  Serial.print(imu_y);
  Serial.print(" Z=");
  Serial.println(imu_z);
#endif

#ifdef IMU_USE_MPU6050_DMP
  // MPU6050 hardware calibration: read raw offsets and apply via registers
  int16_t ax_sum = 0, ay_sum = 0, az_sum = 0;
  const int calib_samples = 100;
  
  for (int i = 0; i < calib_samples; i++) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    delay(10);
  }
  
  // Calculate average offsets (raw units at ±8g: ~4096 LSB/g)
  int16_t ax_offset = ax_sum / calib_samples;
  int16_t ay_offset = ay_sum / calib_samples;
  int16_t az_offset = az_sum / calib_samples;
  
  // Apply hardware offsets
  mpu.setXAccelOffset(ax_offset);
  mpu.setYAccelOffset(ay_offset);
  mpu.setZAccelOffset(az_offset);
  
  // Store in settings as milli-g (for reference: 4096 LSB/g at ±8g = 244 milli-g/LSB)
  int16_t imu_x = (int16_t)(ax_offset / 4.096f);
  int16_t imu_y = (int16_t)(ay_offset / 4.096f);
  int16_t imu_z = (int16_t)(az_offset / 4.096f);
  settings.setImuOffsets(imu_x, imu_y, imu_z);
  
  Serial.print("#MPU6050 calibration: X=");
  Serial.print(imu_x);
  Serial.print(" Y=");
  Serial.print(imu_y);
  Serial.print(" Z=");
  Serial.println(imu_z);
#endif
  
  Serial.println("#IMU calibration complete - use $< to save settings");
}

#endif

// ============================================================================
// ACCELEROMETER DATA READING
// ============================================================================
#ifdef ENABLE_ACCELEROMETER

static bool readAcceleration()
{
#ifdef IMU_USE_LSM6DS3
  // Read LSM6DS3 accelerometer and normalize to milli-g
  // Seeed LSM6DS3 library provides readFloatAccelX/Y/Z() in g
  // For ±2g range: sensitivity = 0.061 mg/LSB
  
  float accelX = myIMU.readFloatAccelX();  // in g
  float accelY = myIMU.readFloatAccelY();  // in g
  float accelZ = myIMU.readFloatAccelZ();  // in g
  
  // Convert to milli-g
  int16_t accel_x_mg = (int16_t)(accelX * 1000);
  int16_t accel_y_mg = (int16_t)(accelY * 1000);
  int16_t accel_z_mg = (int16_t)(accelZ * 1000);
  
  // Apply calibration offsets (subtract to remove bias)
  int16_t off_x, off_y, off_z;
  settings.getImuOffsets(off_x, off_y, off_z);
  currentAccel.x = accel_x_mg - off_x;
  currentAccel.y = accel_y_mg - off_y;
  currentAccel.z = accel_z_mg - off_z;
  
  return true;
#endif

#ifdef IMU_USE_MPU6050_DMP
  // Read MPU6050 raw acceleration and normalize to milli-g
  // MPU6050 at ±8g: 4096 LSB/g → raw / 4.096 = milli-g
  
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  
  // Normalize to milli-g
  int16_t accel_x_mg = (int16_t)(ax / 4.096f);
  int16_t accel_y_mg = (int16_t)(ay / 4.096f);
  int16_t accel_z_mg = (int16_t)(az / 4.096f);
  
  // Apply calibration offsets (hardware offsets already applied at register level)
  int16_t off_x, off_y, off_z;
  settings.getImuOffsets(off_x, off_y, off_z);
  currentAccel.x = accel_x_mg - off_x;
  currentAccel.y = accel_y_mg - off_y;
  currentAccel.z = accel_z_mg - off_z;
  
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

      if (_crankSize > 0) {
        ++_crankHead;
        _crankHead &= (RPM_TIMESTAMP_BUFFER_SIZE - 1);
      }
      _crankMicros[_crankHead] = micros();

      if (_crankSize == RPM_TIMESTAMP_BUFFER_SIZE)
      {
        _crankTail++;
        _crankTail &= (RPM_TIMESTAMP_BUFFER_SIZE - 1);
      }
      else
      {
        ++_crankSize;
      }
    }
  }

  void GPIOTE_IRQHandler(void)
  {
#ifdef ENABLE_SPEED_SENSOR
    // Wheel ABS sensor interrupt handler (GPIOTE[4])
    
    if ((NRF_GPIOTE->EVENTS_IN[4] == 1) && (NRF_GPIOTE->INTENSET & (1 << 4)))
    {
      NRF_GPIOTE->EVENTS_IN[4] = 0;
      
      if (_wheelSize > 0) {
        ++_wheelHead;
        _wheelHead &= (RPM_TIMESTAMP_BUFFER_SIZE - 1);
      }
      _wheelMicros[_wheelHead] = micros();

      if (_wheelSize == RPM_TIMESTAMP_BUFFER_SIZE)
      {
        ++_wheelTail;
        _wheelTail &= (RPM_TIMESTAMP_BUFFER_SIZE - 1);
      }
      else
      {
        ++_wheelSize;
      }
    }
#endif
  }
}
