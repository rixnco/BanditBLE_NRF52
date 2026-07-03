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
`TIMER2_IRQHandler` records `micros()` into a power-of-2 circular buffer (size `BUFFER_SIZE` = 4). The head/tail use bitmask arithmetic (`& (BUFFER_SIZE-1)`) for efficiency. When the buffer is full the oldest entry is overwritten.

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

### Characteristic payload (18 bytes)

| Byte | Content |
|---|---|
| 0 | RPM low byte |
| 1 | RPM high byte |
| 2 | Current gear (0 = neutral) |
| 3–17 | Reserved |

---

## Serial Protocol

Text-based, line-terminated (`\n`). The first character of each line is the opcode.

### `$` — Parameters

| Command | Description |
|---|---|
| `$$` | Read all parameters |
| `$GEAR1` | Read GEAR1 RPM threshold |
| `$GEAR1=370` | Write GEAR1 RPM threshold |
| `$<` | Save settings to persistent storage |
| `$>` | Load settings from persistent storage |

Response format: `$PARAM=value`

### `?` — Report

| Command | Description |
|---|---|
| `?` | Single report: `>gear,rpm` |
| `?500` | Periodic report every 500 ms |
| `?0` | Stop periodic report |

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
| `>gear,rpm` | Periodic or on-demand report |
| `#...` | Informational / boot messages |
| `$PARAM=val` | Parameter value |

---

## Settings

Settings are stored as a `settings_t` struct (20 bytes) validated by a magic word and version number.

| Field | Default | Description |
|---|---|---|
| `gear1` | 370 | RPM threshold for gear 1 indication |
| `gear2` | 477 | RPM threshold for gear 2 |
| `gear3` | 600 | RPM threshold for gear 3 |
| `gear4` | 720 | RPM threshold for gear 4 |
| `gear5` | 810 | RPM threshold for gear 5 |
| `gear6` | 867 | RPM threshold for gear 6 |

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
| `GEAR_ADC_THR_12..6N` | `main.cpp` | — | ADC gear position thresholds |
| `DEFAULT_GEAR1..6` | `settings.h` | 370–867 | Default RPM thresholds per gear |
| `SETTINGS_USE_EXTERNAL_FLASH` | `settings.h` | defined | Select flash vs BLEBondStore |
| `SETTINGS_FLASH_ADDR` | `settings.h` | `0x1FF000` | Flash address for settings page |
