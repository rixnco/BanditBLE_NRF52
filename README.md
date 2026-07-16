# BanditBLE_NRF52

Firmware for a Bluetooth Low Energy (BLE) tachometer designed for the **Suzuki Bandit** motorcycle. Runs on a **nRF52840 MCU**.
Compatible boards:
- **Seeed XIAO nRF52840 Sense**  
- **Makerdiary nrf52840 MDK**
- **Nordic nRF52840 DK board**


Measures engine RPM via a variable reluctance sensor (VRS) on the crankshaft, reads the current gear from an analog position sensor, and streams both values over BLE every 100 ms.

---

## Hardware

| Component | Description |
|---|---|
| MCU | nRF52840 |
| RPM sensor | Variable reluctance sensor on crankshaft |
| Tach input pin | `TACH_INT_PIN` |
| Gear sensor | Potentiometer on gearbox → `GEAR_PIN` |
| External flash | P25Q16H 2 MB QSPI (on Seeed XIAO Sense board only) |
| LED | Red LED = override active |

---

## RPM Measurement

RPM is measured using a fully hardware-driven pipeline — the CPU is only involved when a timestamp needs to be recorded.

```
VRS sensor → GPIOTE (Hi→Lo edge) → PPI → TIMER2 counter
                                              │ every CRANKSHAFT_TEETH pulses
                                              ▼
                                      TIMER2_IRQHandler
                                      (timestamp in circular buffer)
```

### 1. GPIOTE — Edge detection
Channel 3 of GPIOTE monitors `TACH_INT_PIN` and generates a hardware event on every **falling edge** (Hi→Lo).

### 2. PPI — Zero-CPU routing
PPI channel 0 connects the GPIOTE event directly to `TIMER2 TASKS_COUNT`. Each falling edge increments TIMER2 without CPU involvement.

### 3. TIMER2 — Pulse counter
TIMER2 runs in **counter mode** (8-bit). When the count reaches `CRANKSHAFT_TEETH` (= 22, one crankshaft revolution), it fires a compare interrupt and auto-resets via a SHORT.

### 4. ISR — Circular timestamp buffer
`TIMER2_IRQHandler` records `micros()` into a power-of-2 circular buffer (size `RPM_TIMESTAMP_BUFFER_SIZE` = 4). The head/tail use bitmask arithmetic (`& (RPM_TIMESTAMP_BUFFER_SIZE-1)`) for efficiency. When the buffer is full the oldest entry is overwritten.

### 5. RPM Calculation
Called every `REFRESH_RATE` ms from the main loop:

```
RPM = (size - 1) × 6 000 000 / (t_head - t_tail)
```

- `size - 1` = number of complete revolution intervals in the buffer
- `t_head - t_tail` = elapsed time in microseconds
- Constant `6 000 000 = 60 s/min × 10⁶ µs/s ÷ 10` (result has 10 RPM resolution)

If the last timestamp is older than **250 ms**, the buffer is reset and RPM returns 0 (engine stopped).

---

## Gear Detection

`GEAR_PIN` (A2) is connected to a potentiometer that produces a different voltage per gear. The ADC is read at 8-bit resolution every `REFRESH_RATE` ms.

| Gear | Typical ADC | Threshold define |
|---|---|---|
| 1 | 114 | `GEAR_ADC_THR_12` = 102 |
| 2 | 149 | `GEAR_ADC_THR_23` = 131 |
| 3 | 179 | `GEAR_ADC_THR_34` = 164 |
| 4 | 212 | `GEAR_ADC_THR_45` = 195 |
| 5 | 227 | `GEAR_ADC_THR_56` = 219 |
| 6 | 242 | `GEAR_ADC_THR_6N` = 234 |
| N | 90  | (below `GEAR_ADC_THR_12`) |

A **debounce of 300 ms** (`3 × REFRESH_RATE`) filters transitions before updating the active gear.

---

## BLE

- **Device name**: `BANDIT`
- **Service UUID**: `2E290000-1EF9-11E9-AB14-D663BD873D93`
- **Characteristic UUID**: `2E290001-1EF9-11E9-AB14-D663BD873D93` (Read + Notify)
- **Notification interval**: `BLE_NOTIFY_PERIOD_MS` = 100 ms

### Characteristic payload (11 bytes)

| Bytes | Content | Notes |
|---|---|---|
| 0–1 | RPM (uint16_t LE) | Engine RPM × 10 resolution |
| 2 | Gear (uint8_t) | 0 = neutral, 1–6 = gear position |
| 3–4 | Speed (uint16_t LE) | Vehicle speed in km/h (or 0xFFFF if disabled) |
| 5–6 | AccelX (int16_t LE) | X-axis acceleration in milli-g (or 0xFFFF if disabled) |
| 7–8 | AccelY (int16_t LE) | Y-axis acceleration in milli-g (or 0xFFFF if disabled) |
| 9–10 | AccelZ (int16_t LE) | Z-axis acceleration in milli-g (or 0xFFFF if disabled) |

---

## Accelerometer (Optional)

The firmware supports dual accelerometer configurations via the `ENABLE_ACCELEROMETER` flag in [config.h](config.h).

### Supported IMUs

| IMU | Interface | I2C Bus | Address | Range | Notes |
|---|---|---|---|---|---|
| **LSM6DS3** | Seeed XIAO onboard | Wire1 (I2C1) | 0x6A | ±2g | Software calibration, 104 Hz |
| **MPU6050** | External module | Wire (I2C0) | 0x68 | ±8g | Hardware + software calibration, 100 Hz |

### Configuration

In [config.h](config.h):
- Uncomment `#define ENABLE_ACCELEROMETER` to enable
- Select one: `#define IMU_USE_LSM6DS3` OR `#define IMU_USE_MPU6050_DMP`
- Enable calibration with `#define IMU_ZERO_CALIB` (removed, manual calibration via command instead)

### Calibration

Calibration is triggered **on-demand** via the serial command `$IMU CLB`:

1. Device reads 100 acceleration samples at rest (1 second, 10 ms per sample)
2. Computes average offset for each axis (works at any orientation)
3. Stores offsets in settings as `imu_x`, `imu_y`, `imu_z` (milli-g units)
4. Use `$<` to save settings to persistent storage

**Note:** Calibration works at any bike orientation (upright, kickstand, etc.)

**Usage examples:**
```
$IMU CLB              → Calibrate now (motion sensors must be at rest)
$IMU                  → Read current offsets (response: $IMU=x,y,z)
$IMU=-50,20,-100      → Manually set offsets
$<                    → Save to flash
```

---

## Serial Protocol

Text-based, line-terminated (`\n`). The first character of each line is the opcode.

**All commands work on both Serial (USB) and BLE UART simultaneously.** Responses are sent to both channels if a BLE client is connected.

### `$` — Parameters

| Command | Description |
|---|---|
| `$$` | Read all parameters |
| `$GEAR1` | Read GEAR1 RPM threshold |
| `$GEAR1=370` | Write GEAR1 RPM threshold |
| `$IMU` | Read accelerometer calibration triplet (format: `$IMU=x,y,z`) |
| `$IMU=50,-20,100` | Write calibration offsets (x, y, z in milli-g) |
| `$IMU CLB` | **Calibrate accelerometer** (100 samples, 1 second) |
| `$<` | Save settings to persistent storage |
| `$>` | Load settings from persistent storage |

Response format: `$PARAM=value`

### `?` — Report

Syntax: `?[*][value]`

| Command | Description |
|---|---|
| `?` | Single report: `>gear,rpm` (gear mode) |
| `?*` | Single report: `>*adc,rpm` (raw ADC mode) |
| `?500` | Periodic report every 500 ms (gear mode) |
| `?*500` | Periodic report `>*adc,rpm` every 500 ms (raw ADC mode) |
| `?[*]0` | Stop periodic report |

### `!` — Override

| Command | Description |
|---|---|
| `!2,3500` | Force gear = 2, RPM = 3500 (test mode) |
| `!!` | Disable override |

When override is active the **red LED** is on and the real sensor inputs are ignored.

### Response tokens

| Prefix | Meaning |
|---|---|
| `!OK` | Command accepted |
| `!ERROR,msg` | Command rejected |
| `>gear,rpm` | Report in gear mode (e.g., `>3,4500` = gear 3, 4500 RPM) |
| `>*adc,rpm` | Report in raw ADC mode (e.g., `>*179,4500` = ADC 179, 4500 RPM) |
| `#...` | Informational / boot messages |
| `$PARAM=val` | Parameter value |

---

## Settings

Settings are stored as a `settings_t` struct (26 bytes) validated by a magic word and version number.

| Field | Default | Description |
|---|---|---|
| `gear1` | 370 | RPM threshold for gear 1 indication |
| `gear2` | 477 | RPM threshold for gear 2 |
| `gear3` | 600 | RPM threshold for gear 3 |
| `gear4` | 720 | RPM threshold for gear 4 |
| `gear5` | 810 | RPM threshold for gear 5 |
| `gear6` | 867 | RPM threshold for gear 6 |
| `imu_x` | 0 | X-axis accelerometer calibration offset (milli-g) |
| `imu_y` | 0 | Y-axis accelerometer calibration offset (milli-g) |
| `imu_z` | 0 | Z-axis accelerometer calibration offset (milli-g) |

### Storage backends

Controlled by `#define SETTINGS_USE_EXTERNAL_FLASH` in `settings.h`:

| Define | Backend | Notes |
|---|---|---|
| defined | P25Q16H QSPI flash | Last 4 KB sector (`0x1FF000`). Sector erased before each write. |
| commented | `BLEBondStore` | nRF SoftDevice internal storage, slot 0. |

Call `initSettingsStorage()` once in `setup()` before `readSettings()`. On first boot (blank flash) settings default to the `DEFAULT_GEARx` values and are written automatically.

---

## Key Compile-time Defines

| Define | File | Default | Description |
|---|---|---|---|
| `CRANKSHAFT_TEETH` | `main.cpp` | 22 | VRS pulses per crankshaft revolution |
| `REFRESH_RATE` | `main.cpp` | 100 ms | Main loop update interval |
| `BLE_NOTIFY_PERIOD_MS` | `main.cpp` | 100 ms | BLE notification interval |
| `ENABLE_SPEED_SENSOR` | `config.h` | 1 | Enable/disable rear wheel speed measurement |
| `ENABLE_ACCELEROMETER` | `config.h` | 1 | Enable/disable accelerometer |
| `IMU_USE_LSM6DS3` | `config.h` | — | Select LSM6DS3 (Seeed onboard) |
| `IMU_USE_MPU6050_DMP` | `config.h` | — | Select MPU6050 (external module) |
| `IMU_ACCEL_RANGE` | `config.h` | 1 | ±2g (LSM6DS3) or ±8g (MPU6050) |
| `IMU_DLPF_BW` | `config.h` | 4 | Digital Low Pass Filter mode (MPU6050 only) |
| `DEFAULT_GEAR1..6` | `config.h` | 102–234 | ADC thresholds for gear detection |
| `SETTINGS_USE_EXTERNAL_FLASH` | `config.h` | defined | Select flash vs BLEBondStore |
| `SETTINGS_FLASH_ADDR` | `config.h` | `0x1FF000` | Flash address for settings page |

### Build Configuration Examples

To customize the build, add `build_flags` to your PlatformIO environment:

```ini
# Disable wheel speed sensor
build_flags =
    -DENABLE_SPEED_SENSOR=0

# Disable accelerometer
build_flags =
    -DENABLE_ACCELEROMETER=0

# Enable accelerometer with MPU6050 (external I2C0)
build_flags =
    -DENABLE_ACCELEROMETER=1
    -DIMU_USE_MPU6050_DMP=1
```
