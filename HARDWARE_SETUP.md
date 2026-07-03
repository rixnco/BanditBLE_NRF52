# BanditBLE GSF650 - Hardware Setup Guide

## Overview
Complete hardware integration for dual-sensor motorcycle telemetry:
- **Crankshaft RPM** via VRS sensor (engine)
- **Wheel Speed** via ABS sensor (rear wheel, isolated via opto-coupler)

---

## Bill of Materials (BOM)

### Main Components

| Part | Qty | Value | Purpose | Cost |
|---|---|---|---|---|
| nRF52840 Seeed XIAO | 1 | - | Main microcontroller | €50 |
| **ABS Sensor Interface** | | | | |
| TLP291 Opto-coupler | 1 | - | Galvanic isolation | €1.50 |
| Resistor | 1 | 100Ω | LED current limit | €0.05 |
| Resistor | 1 | 10kΩ | Pull-up to 3.3V | €0.05 |
| Resistor | 1 | 10kΩ | Pull-down ABS signal | €0.05 |
| Capacitor | 1 | 100nF | Decoupling (photo side) | €0.10 |
| Capacitor | 1 | 10µF | Decoupling (LED side) | €0.10 |
| Diode | 1 | 1N4148 | Protection reverse polarity | €0.05 |
| **Connectivity** | | | | |
| Breadboard | 1 | 830-pin | Development/prototyping | €3 |
| Jumper wires | 1 | 65-piece | Connections | €2 |
| USB-C cable | 1 | - | nRF52 programming | €5 |
| **Optional** | | | | |
| Shielded twisted pair cable | 1m | - | ABS signal line | €2 |
| Ferrite toroid | 1 | 16mm OD | EMI suppression | €1 |
| Multimeter | 1 | Digital | Troubleshooting | €15 |
| **Total (essentials)** | | | | **~€60** |
| **Total (with optional)** | | | | **~€80** |

---

## Wiring Diagram - Opto-Coupler Isolation

```
┌──────────────────────────────────────────────────────────────┐
│                    GSF650 ABS SENSOR                         │
│         (rear wheel, 46 teeth, signal 0-5V)                  │
└────────────────┬──────────────────────────────────────────────┘
                 │
        ┌────────┴──────────┐
        │                   │
    (5V pulse)      ┌───────┴───────┐
        │           │               │
        │       ┌───┴────┐      ┌───┴────┐
        │       │  LED+  │      │ Photo+ │
        │       │ TLP291 │      │ TLP291 │
        │       │   D    │      │   C    │
        │   ┌───┤   │    │      │   │    ├───────┐
        │   │   │   K    │      │   E    │       │
        │   │   └───┬────┘      └───┬────┘       │
        │   │       │                │           │
        │   │   ┌───┴────────┐  ┌────┴───────┐   │
        │   │   │ R 100Ω    │  │ R 10kΩ     │   │
        │   │   │ (limit I) │  │ (Pull-up)  │   │
        │   │   └───┬───────┘  └────┬───────┘   │
        │   │       │               │           │
        │   │   ┌───┴───────┐       │           │
        │   │   │ C 10µF    │       │           │
        │   │   │ (bypass)  │       │           │
        │   │   └───┬───────┘       │           │
        │   │       │               │           │
        │   └───────┤───GND         │           │
        │           │               │           │
        ├───────────┴───GND  3.3V ──┤           │
        │                           │           │
    Module ABS               ┌──────┴──────┐    │
    (OEM)                    │ R 10kΩ      │    │
    Signal restored 5V       │ (Pull-down) │    │
                             └──────┬──────┘    │
                                    │           │
                                    │      [Diode 1N4148]
                                    │           │
                                    └───────────┤
                                                │
                                           GPIO P0.05
                                           nRF52840
```

---

## Pin Assignment

### nRF52840 SEEED XIAO Pinout

```
┌──────────────────────────────┐
│    SEEED XIAO nRF52840       │
├──────────────────────────────┤
│ GND   ░░░░░░░░░░░░░░░░   5V  │
│ GPIO2 ░░░░░░░░░░░░░░░░   P0  │ (Crankshaft VRS)
│ GPIO3 ░░░░░░░░░░░░░░░░   SDA │
│ GPIO4 ░░░░░░░░░░░░░░░░   SCL │
│ GPIO5 ░░░░░░░░░░░░░░░░   RST │ (Wheel ABS)
│ 3V3   ░░░░░░░░░░░░░░░░   GND │
└──────────────────────────────┘

TACH_INT_PIN = 2   (D2)  → Crankshaft VRS
WHEEL_INT_PIN = 5  (D5)  → Wheel ABS (via opto-isolator)
GEAR_PIN = PIN_A2  (A2)  → Gear ADC (existing)
```

---

## Breadboard Layout (top view)

```
        Column:  1  2  3  4  5  6  7  8  9 10
        ┌──────────────────────────────────────┐
      1 │ GND  GND  GND  GND  GND  GND  GND GND│
      2 │      LED+ │      │       │      │    │
      3 │ ┌────┬────┴────┬─┐     │       │    │
      4 │ │TLP │ R100Ω   │ │ │   │       │    │
      5 │ │291 │         │ │ │   │       │    │
      6 │ │    │ C 10µF  │ │ │   │       │    │
      7 │ └────┴─────────┴─┘ │   │       │    │
      8 │                    │   │       │    │
      9 │  Photo+            │   │       │    │
     10 │  ┌──────────────┬──┘   │       │    │
     11 │  │ R 10kΩ(pull) │      │       │    │
     12 │  │ to 3.3V      │      │       │    │
     13 │  └──────────────┘      │       │    │
     14 │                        │       │    │
     15 │  Signal ABS input ─────┘       │    │
     16 │  (from motorcycle ABS)         │    │
     17 │                                │    │
     18 │  R 10kΩ (pull-down) ───────────┘    │
     19 │  │                                  │
     20 │  └─── to GPIO P0.05 (nRF52)        │
        └──────────────────────────────────────┘

Legend:
- TLP291 = 8-pin DIP opto-coupler
- All resistors: 1/4W, 5% tolerance
- Capacitors: 50V rated minimum
```

---

## Connection Details

### 1. Signal ABS Input (from motorcycle)

```
GSF650 ABS Connector (2-pin Superseal, yellow)
├─ Pin 1 (Signal) ──[Shielded twisted pair]──→ Breadboard col.15
└─ Pin 2 (GND)    ──[Black wire]─────────────→ Breadboard GND rail

Shielding:
- Twist 2 wires together (signal + drain)
- Connect shield to motorcycle GND (not nRF52 GND yet)
- Use ferrite toroid around cable (optional but recommended)
```

### 2. Opto-Coupler Connections (TLP291)

```
TLP291 pinout (8-pin DIP):
  1 = Anode (LED+)      ──→ R 100Ω ──→ col 5
  2 = Cathode (LED-)    ──→ GND rail
  3 = No connect
  4 = GND (photo side)  ──→ GND rail
  5 = GND (photo side)  ──→ GND rail
  6 = Emitter (E)       ──→ R 10kΩ pull-down
  7 = Collector (C)     ──→ 3.3V pull-up via R 10kΩ
  8 = No connect
```

### 3. Pull-Up / Pull-Down Network

```
        3.3V
         │
      [R 10kΩ]  ← Pull-up to 3.3V
         │
    ┌────┴────┐
    │ Photo C │  (TLP291 collector)
    └────┬────┘
         │
    ┌────┴─────────────→ GPIO P0.05
    │
 [R 10kΩ]  ← Pull-down (optional, circuit works without)
    │
   GND
```

---

## Testing Procedure (Breadboard)

### Phase 1: Component Verification (Key OFF)

```
□ Visual inspection
  ├─ TLP291 seated correctly (no bent pins)
  ├─ Resistors in correct holes
  ├─ Capacitors polarity correct (stripe = -)
  └─ No jumper bridges or solder bridges

□ Multimeter continuity test (power OFF)
  ├─ Measure: col 5 (LED+) to GND (LED-) → R 100Ω measured
  ├─ Measure: col 7 (Photo C) to GND → R 10kΩ measured
  └─ Measure: GPIO P0.05 to GND via pull-down → R 10kΩ
```

### Phase 2: Voltage Test (Key ON, engine stopped)

```
Engine OFF, battery connected:

□ Power supply
  ├─ nRF52 3.3V rail: 3.3V ± 0.1V
  └─ Breadboard GND: 0V reference

□ Opto-coupler idle (ABS not moving)
  ├─ Measure pin 7 (Photo C) to GND: ~3.3V (HIGH)
  ├─ Measure GPIO P0.05 to GND: ~3.3V (HIGH)
  └─ LED should be OFF (dim or no light)

□ Signal input idle (motorcycle parked)
  ├─ Measure col 15 (ABS in) to GND: ~5V (high level)
  └─ Measure col 5 (LED+) to GND: ~0V (LED cathode is GND)
```

### Phase 3: Dynamic Test (engine idling, no movement)

```
Engine idling (motorcycle stationary):

□ Signal detection (crankshaft pulses)
  ├─ Measure GPIO P0.02 (crankshaft): 0-5V pulse at ~300 Hz
  ├─ Check Serial output: RPM should show > 0
  └─ LED indicator (if added): blinks at crankshaft rate

□ ABS sensor idle (wheel stopped)
  ├─ Measure col 15 (ABS in): 5V DC (no pulses)
  ├─ Measure GPIO P0.05 (ABS out): 3.3V DC (no pulses)
  ├─ Check Serial output: Speed should show 0 km/h
  └─ Photo LED off
```

### Phase 4: Rolling Test (motorcycle in motion)

```
Ride at constant 50 km/h (target frequency ~543 Hz):

□ ABS sensor active
  ├─ Measure col 15 (ABS in): 0-5V square pulse
  ├─ Oscilloscope (if available): observe pulse shape
  ├─ Frequency counter: should be ~540 ± 50 Hz
  └─ BLE app: Speed shows ~50 km/h (compare with OEM speedometer)

□ Both sensors synchronized
  ├─ RPM shows engine speed (typically 3000-4000 idle+cruise)
  ├─ Speed shows wheel speed (50 km/h in this test)
  ├─ Gear shows current gear (4-5 for cruise)
  └─ All update every 100ms

□ Validation
  ├─ Speed reading within 5% of OEM cluster
  ├─ No glitches or missing pulses
  ├─ Module ABS still functioning (test brakes gently)
  └─ No dashboard warnings
```

---

## Troubleshooting Matrix

| Symptom | Cause | Fix |
|---|---|---|
| Speed = 0 always | Signal not reaching GPIO | Check col 15 → col 7 connection |
| Speed = 999 km/h | Signal too weak/noisy | Add ferrite toroid, shield cable |
| Intermittent speed | Bad breadboard contact | Reseat all jumpers, add solder bridges |
| ABS warning light | Signal voltage too low | Check 3.3V pull-up resistor value |
| nRF52 won't boot | GND loop or short | Check all GND connections isolated |
| Speed lags by 200ms | Buffer timeout | Reduce WHEEL_TIMEOUT in firmware |
| Compiler errors | Pin conflicts | Check WHEEL_INT_PIN ≠ other pins |

---

## Safe Integration Checklist (before first ride)

- [ ] Breadboard fully wired and tested (Phase 1-3 passing)
- [ ] ABS signal verified with multimeter (0-5V pulses)
- [ ] GPIO voltage acceptable (0-3.3V pulses)
- [ ] Motorcycle battery healthy (12.6V+ with engine off)
- [ ] Module ABS functional (test with gentle brake)
- [ ] nRF52 firmware compiled without errors
- [ ] BLE connection stable (phone connects and reads data)
- [ ] Speed matches OEM cluster ± 5 km/h (test at 50 km/h)
- [ ] Crankshaft RPM reasonable (~3000 rpm at idle)
- [ ] No dashboard warning lights after integration
- [ ] Road test at low speed (~20 km/h) in parking lot
- [ ] Test emergency braking - ABS must engage normally
- [ ] Final validation at highway speed (100+ km/h)

---

## Production Wiring (after breadboard validated)

Once breadboard works, solder permanent connections:

```
□ Print PCB or use perf-board layout
□ Solder TLP291 and passives
□ Use stranded shielded cable (motorcycle grade)
□ Conformal coat to protect from vibration/moisture
□ Potted or enclosed for weather resistance
□ Mount near battery with proper strain relief
```

---

## Files Generated

- `schematic_tlp291.txt` - ASCII wiring diagram (this file)
- `BOM_parts.csv` - Supplier links and part numbers
- `firmware/main.cpp` - nRF52 code (already deployed)
- `test_results.log` - Template for documenting test results

---

## Next Steps

1. **Order parts** from BOM (or use spare components)
2. **Assemble breadboard** following layout section
3. **Run Phase 1-2 tests** with motorcycle parked
4. **Test Phase 3** with engine running (idling)
5. **First ride** (low speed, parking lot)
6. **Validate data** with BLE app on phone
7. **Document results** in test log
8. **Submit pull request** with test photos/data

---

Last updated: 2026-07-03
Status: Ready for breadboard assembly
