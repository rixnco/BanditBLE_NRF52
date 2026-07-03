# BanditBLE Hardware Testing Log

**Date**: _______________  
**Tester**: _______________  
**Motorcycle**: Suzuki GSF650 / _______________  
**Location**: _______________

---

## Pre-Assembly Checklist

- [ ] All parts received and verified against BOM
- [ ] Breadboard inspected (no bent holes, dust)
- [ ] nRF52840 tested with USB (bootloader works)
- [ ] Multimeter battery fresh and working
- [ ] Motorcycle battery healthy (test with multimeter)

---

## PHASE 1: Component Verification (Motorcycle Key OFF, Battery Disconnected)

### Visual Inspection

```
TLP291 Opto-coupler
  └─ Orientation correct (notch visible): ☐ YES ☐ NO
  └─ No bent pins: ☐ YES ☐ NO
  └─ Pins 1-8 clearly visible: ☐ YES ☐ NO

Resistors (R 100Ω, R 10kΩ x2)
  └─ Color bands readable: ☐ YES ☐ NO
  └─ Correct positions on breadboard: ☐ YES ☐ NO
  
Capacitors (C 100nF, C 10µF)
  └─ Polarity marked correctly: ☐ YES ☐ NO
  └─ No visible damage/leakage: ☐ YES ☐ NO

Diode 1N4148
  └─ Black stripe visible: ☐ YES ☐ NO
  └─ Correct orientation (stripe = cathode): ☐ YES ☐ NO
```

### Continuity Testing (Multimeter on Ω × 1 scale)

| Connection | Expected | Measured | Pass? |
|---|---|---|---|
| col 5 (LED+) to GND | ~100Ω | ______ Ω | ☐ |
| col 7 (Photo C) to GND | ~10kΩ | ______ Ω | ☐ |
| GPIO P0.05 to GND | ~10kΩ | ______ Ω | ☐ |
| col 15 (ABS in) to GND | OL (open) | ______ | ☐ |

**Result**: ☐ PASS ☐ FAIL

---

## PHASE 2: Voltage Test (Motorcycle Key ON, Engine Stopped)

**Time**: _______________  
**Battery Voltage**: ______ V (should be 12.4-12.8V)

### 3.3V Supply Check

| Test Point | Expected | Measured | Pass? |
|---|---|---|---|
| nRF52 3.3V rail | 3.3V ±0.1V | ______ V | ☐ |
| Breadboard 3.3V rail | 3.3V ±0.1V | ______ V | ☐ |
| Breadboard GND | 0V ref | ______ V | ☐ |

### Opto-Coupler Idle Voltage (ABS Not Moving)

| Pin | Expected | Measured | Pass? |
|---|---|---|---|
| TLP291 pin 1 (Anode) | 2.5-3.0V | ______ V | ☐ |
| TLP291 pin 7 (Collector) | ~3.3V (HIGH) | ______ V | ☐ |
| GPIO P0.05 | ~3.3V (HIGH) | ______ V | ☐ |

### ABS Signal Input (Motorcycle Parked)

| Test Point | Expected | Measured | Pass? |
|---|---|---|---|
| col 15 (ABS in) | ~5.0V | ______ V | ☐ |
| col 5 (LED+) thru R100 | ~4.8V | ______ V | ☐ |
| TLP291 pin 1 (Anode) | ~3.5V | ______ V | ☐ |

### Serial Monitor Check

```
Connect USB to nRF52, open serial terminal (115200 baud)
Copy output below:
_________________________________________________
_________________________________________________
_________________________________________________

Expected: RPM > 0 (from crankshaft)
          Speed = 0 (wheel stationary)
          Gear detected
          
Result: ☐ PASS ☐ FAIL
```

---

## PHASE 3: Dynamic Test (Engine Idling, Motorcycle Stationary)

**Time**: _______________  
**Engine Status**: Idling at ______ RPM (OEM cluster)  
**Environmental**: Temperature ______°C, Humidity ______%

### Crankshaft VRS Sensor (GPIO P0.02)

| Measurement | Tool | Expected | Measured | Pass? |
|---|---|---|---|---|
| Frequency | Oscilloscope or freq counter | 300-500 Hz | ______ Hz | ☐ |
| Voltage swing | Multimeter AC | 0-5V | 0-______ V | ☐ |
| Signal quality | Visual | Clean square | ☐ Good ☐ Noisy | ☐ |

### Crankshaft RPM Output

| Parameter | Expected | Measured | Pass? |
|---|---|---|---|
| RPM readout (serial) | > 500 | ______ RPM | ☐ |
| RPM on BLE app | > 500 | ______ RPM | ☐ |
| Consistency | Stable ±50 RPM | Variance: ______ | ☐ |

### ABS Sensor Input (Wheel Stopped)

| Measurement | Expected | Measured | Pass? |
|---|---|---|---|
| col 15 voltage | 5V DC (no pulses) | ______ V | ☐ |
| GPIO P0.05 voltage | 3.3V DC (no pulses) | ______ V | ☐ |
| Speed readout (serial) | 0 km/h | ______ km/h | ☐ |
| Speed on BLE app | 0 km/h | ______ km/h | ☐ |

### Gear Detection

| Gear Position | Expected ADC | Measured | Pass? |
|---|---|---|---|
| Neutral | > 234 | ______ | ☐ |
| Gear 1 | < 102 | ______ | ☐ |
| Gear 3 | 164-195 | ______ | ☐ |

**Result**: ☐ PASS ☐ FAIL

---

## PHASE 4: Rolling Test (Constant Speed Cruise)

### Test Parameters

- **Target Speed**: 50 km/h  
- **Test Duration**: 5 minutes minimum  
- **Road Conditions**: Safe, empty parking lot or test track  
- **Time**: _______________  

### Data Collection (every 30 seconds)

| Time | OEM Speed | BLE Speed | Delta | RPM | Gear | Pass? |
|---|---|---|---|---|---|---|
| 0:00 | _____ | _____ | _____ | _____ | _____ | ☐ |
| 0:30 | _____ | _____ | _____ | _____ | _____ | ☐ |
| 1:00 | _____ | _____ | _____ | _____ | _____ | ☐ |
| 1:30 | _____ | _____ | _____ | _____ | _____ | ☐ |
| 2:00 | _____ | _____ | _____ | _____ | _____ | ☐ |

**Average Delta**: ______ km/h  
**% Error**: ______ % (should be < 5%)

### ABS Sensor Frequency Check

Using frequency counter or oscilloscope, measure ABS pulses at 50 km/h:

```
Expected frequency = 50 km/h × 46 teeth / 2.15m = ~543 Hz

Measured frequency: ______ Hz
Deviation: ______ Hz (Acceptable: ±50 Hz)

Result: ☐ PASS ☐ FAIL
```

### Brake Test (Emergency Braking Check)

```
At 30 km/h, test gentle braking:
  └─ ABS engages normally: ☐ YES ☐ NO
  └─ No error messages: ☐ YES ☐ NO
  └─ Speed drops correctly: ☐ YES ☐ NO

At 50 km/h, test harder braking:
  └─ ABS pulsation felt: ☐ YES ☐ NO
  └─ Motorcycle stops safely: ☐ YES ☐ NO
  └─ No loss of control: ☐ YES ☐ NO
```

### Dashboard / Warning Lights

```
After 5 minutes of riding:
  □ ABS warning light OFF: ☐ YES ☐ NO ☐ NOT VISIBLE
  □ Check Engine light OFF: ☐ YES ☐ NO ☐ NOT VISIBLE
  □ Other warnings: ___________________________
```

**Result**: ☐ PASS ☐ FAIL

---

## PHASE 5: Extended Validation (Optional)

### Speed Range Test

| Speed | OEM Value | BLE Value | Delta | Error % | Pass? |
|---|---|---|---|---|---|
| 20 km/h | _____ | _____ | _____ | _____ % | ☐ |
| 50 km/h | _____ | _____ | _____ | _____ % | ☐ |
| 80 km/h | _____ | _____ | _____ | _____ % | ☐ |
| 100 km/h | _____ | _____ | _____ | _____ % | ☐ |
| 120 km/h | _____ | _____ | _____ | _____ % | ☐ |

### Acceleration / Deceleration Response

```
Rapid acceleration (0 → 80 km/h in 5 sec):
  └─ BLE speed follows smoothly: ☐ YES ☐ NO
  └─ No glitches in data: ☐ YES ☐ NO
  └─ RPM correlates with speed: ☐ YES ☐ NO

Rapid deceleration (engine braking):
  └─ Speed drops smoothly: ☐ YES ☐ NO
  └─ ABS doesn't trigger: ☐ YES ☐ NO
  └─ Gear detection accurate: ☐ YES ☐ NO
```

### Environmental Tolerance

```
Test in various conditions:
  □ Full sunlight: ☐ Works ☐ Fails
  □ Heavy rain: ☐ Works ☐ Fails
  □ Cold (~5°C): ☐ Works ☐ Fails
  □ Heat (~35°C): ☐ Works ☐ Fails
  □ High vibration (rough road): ☐ Works ☐ Fails
```

**Result**: ☐ PASS ☐ FAIL

---

## FINAL ASSESSMENT

### Overall Result

```
☐ PASS - All phases completed, no critical issues
☐ PASS WITH NOTES - Minor issues documented below
☐ FAIL - See issues section
```

### Issues Encountered

```
Issue 1: ___________________________________________________________
  └─ Impact: ☐ Critical ☐ Major ☐ Minor
  └─ Solution: _____________________________________________________

Issue 2: ___________________________________________________________
  └─ Impact: ☐ Critical ☐ Major ☐ Minor
  └─ Solution: _____________________________________________________

Issue 3: ___________________________________________________________
  └─ Impact: ☐ Critical ☐ Major ☐ Minor
  └─ Solution: _____________________________________________________
```

### Recommendations

```
For production hardware:
□ Solder permanent connections on perfboard
□ Add waterproof potting around opto-coupler
□ Upgrade to industrial-grade cable shielding
□ Integrate into motorcycle fairing for vibration dampening
□ Add status LED for power indication
□ Consider redundant sensor (backup speed measurement)
```

---

## Sign-Off

**Tester Name**: _______________  
**Date**: _______________  
**Signature**: _______________  

**Reviewed By**: _______________  
**Review Date**: _______________  

---

## Attachments

- [ ] Photos of breadboard assembly
- [ ] Screenshots of BLE app data
- [ ] Oscilloscope capture (signal waveform)
- [ ] Test result CSV export from firmware
- [ ] Notes from observations

---

**End of Test Log**
