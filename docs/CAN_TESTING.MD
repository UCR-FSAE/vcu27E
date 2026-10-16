# VCU: Pedal Input Reading and Bench Test Setup

**Branch:** `paul-vcu-can`
**Files changed:** `src/main.cpp`
**Status:** Not yet tested on hardware. The calibration values are placeholders.

## Summary

Before this change the VCU never read its inputs. The state machine stayed in
`NOT_READY` and the inverter only ever got torque 0 with enable off.

This change:
- reads the pedal sensors (APPS1, APPS2, BSE) and the TS_ACTIVE and DRIVER_ACT switches
- converts the APPS readings to a 0–100% pedal position
- detects brake pressed and pedal faults
- prints a debug line over USB Serial

The VCU can now be taken through its full startup sequence on a test bench.

## Changes

### 1. Input reading: `read_inputs()`
Called every 10 ms tick, before the state machine runs.

| Field | How it's set |
|---|---|
| `apps1_raw`, `apps2_raw`, `bse_raw` | `analogRead` on pins 41, 40, 39 (10-bit, 0–1023) |
| `ts_active`, `driver_act` | `digitalRead` on pins 3, 2 |
| `apps_percentage` | Each APPS value is scaled from `APPS_RAW_MIN`–`APPS_RAW_MAX` to 0–100%, clamped, and the two are averaged |
| `brake_pressed` | On above raw 300, off below raw 250. The gap stops it flickering at the threshold |
| `pedal_fault` | Set after 100 ms of a fault condition (see below) |

A pedal fault is triggered if, for 100 ms:
- any pedal sensor (APPS1, APPS2 or BSE) reads below 10 or above 1013 raw, meaning
  an open circuit or a short; or
- APPS1 and APPS2 differ by more than 10 percentage points (the FSAE APPS
  implausibility rule).

When `pedal_fault` is set in `DRIVE`, the state machine goes to `FAULT`, which
sends torque 0 with the inverter disabled. **`FAULT` is latched: power-cycle or
reset the Teensy to leave it.**

### 2. Calibration constants
These are at the top of `main.cpp` so they can be tuned without changing the
logic. **All values are placeholders.** Measure the real sensors and update
them.

| Constant | Value | Meaning |
|---|---|---|
| `APPS_RAW_MIN` | 50 | Raw ADC value with the pedal released |
| `APPS_RAW_MAX` | 970 | Raw ADC value with the pedal fully pressed |
| `PEDAL_RAW_FAULT_LOW` | 10 | Below this = open circuit or short to GND |
| `PEDAL_RAW_FAULT_HIGH` | 1013 | Above this = short to 3.3 V |
| `APPS_MAX_DIFF` | 10 | Maximum allowed APPS1/APPS2 difference (percentage points) |
| `APPS_FAULT_TICKS` | 10 | Ticks (10 ms each) a fault condition must last before `pedal_fault` is set |
| `BSE_ON_RAW` / `BSE_OFF_RAW` | 300 / 250 | Brake pressed / released thresholds |

### 3. Debug output: `print_debug()`
USB Serial at 115200 baud prints one line at 10 Hz:

    DRIVE         t=412   | apps1= 510 apps2= 505 bse=  80 apps= 49.7% | ts=1 drv=1 brk=0 pf=0 | tq= 497 en=1

The fields are: state, ticks in that state, raw sensor values, pedal %, input
flags (TS active, driver active, brake, pedal fault), then the torque command
and enable flag sent to the inverter.

### 4. Digital input pins
TS_ACTIVE (pin 3) and DRIVER_ACT (pin 2) now use the Teensy's internal
pulldown resistors (`INPUT_PULLDOWN`), so they read 0 when nothing is
connected. These pulldowns are weak (around 100 kΩ). **Check this setting
before connecting the real car signals.** Active-low or optocoupler inputs will
need a different setup.

## Bench Test Setup

### Hardware
- Teensy 4.1 with a 3.3 V CAN transceiver (e.g. SN65HVD230)
- Arduino Uno with a CAN module, acting as a simulated inverter
- 3 potentiometers (APPS1, APPS2, BSE)
- 2 jumper wires or switches (TS_ACTIVE, DRIVER_ACT)

### Wiring

| Signal | Teensy pin | Connection |
|---|---|---|
| APPS1 | 41 | Pot: ends to **3.3 V** / GND, wiper to pin |
| APPS2 | 40 | Pot: ends to **3.3 V** / GND, wiper to pin |
| BSE | 39 | Pot: ends to **3.3 V** / GND, wiper to pin |
| TS_ACTIVE | 3 | Jumper or switch to 3.3 V |
| DRIVER_ACT | 2 | Jumper or switch to 3.3 V |
| CAN TX / RX | 22 / 23 | Transceiver CTX / CRX |

> ⚠️ **Teensy 4.1 pins are not 5 V tolerant.** Power the pots from the Teensy's
> 3.3 V pin, never from the Uno's 5 V.

CAN bus:
- Connect CANH to CANH and CANL to CANL, and tie the GNDs of both boards together.
- Put a 120 Ω resistor at each end of the bus. With power off, CANH to CANL
  should measure about 60 Ω.
- The bus runs at 500 kbit/s.

### Running
1. Flash the Teensy and open the Serial monitor at 115200 baud.
2. Turn the pots off their end stops. A pot at the very end of its travel reads
   out of range and latches `FAULT`.
3. Note the raw APPS values at each end of travel and update `APPS_RAW_MIN` and
   `APPS_RAW_MAX`.

## Test Checklist

| # | Action | Expected |
|---|---|---|
| 1 | Power up with nothing jumpered | `NOT_READY`, `tq=0 en=0` |
| 2 | BSE above threshold, then jumper TS_ACTIVE and DRIVER_ACT | `NOT_READY` → `RTD` → `INVERTER_INIT` |
| 3 | Wait | After 0.5 s (50 ticks) in `INVERTER_INIT`, goes to `DRIVE` with `en=1` |
| 4 | Sweep both APPS pots together | `tq` moves smoothly from 0 to 1000 with no jumps at the ends |
| 5 | Move one APPS pot away from the other (>10%) | `pf=1` after 100 ms → `FAULT`, `tq=0 en=0` |
| 6 | Reset, get back to `DRIVE`, remove the TS_ACTIVE jumper | `NOT_READY` on the next tick, `tq=0 en=0` |
| 7 | Turn any pedal pot to its end stop | `pf=1` → `FAULT` |
| 8 | Watch the inverter sim on the Uno | 0x0C0 received at about 100 Hz. At 50% pedal the bytes are `F4 01 00 00 00 01 00 00` |

### Inverter command frame (0x0C0) reference

| Byte | Content |
|---|---|
| 0–1 | Torque command, little-endian (0–1000) |
| 2–3 | Speed command (0, torque mode) |
| 4 | Direction (0 = forward) |
| 5 | Inverter enable (1 = enabled) |
| 6–7 | Torque limit (0 = inverter default) |

## Known Limitations / TODO
- The calibration constants are placeholders and must be measured on the real
  pedal sensors.
- Brake/APPS plausibility (brake pressed with APPS > 25% → cut torque) is not
  implemented.
- The RTD state does not sound the buzzer or wait 3 s yet.
- The watchdog is not fed yet.
- `DRIVE` sends the current pedal position as soon as it starts. Consider
  requiring APPS ≈ 0% before entering `DRIVE`.
- The Uno inverter simulator sketch is not written yet.
