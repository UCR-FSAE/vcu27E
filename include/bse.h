#pragma once
#include <stdint.h>

/*
    Brake System Encoder (BSE) - FSAE 2027 T.4.3
    - T.4.3.3: a BSE failure persisting > 100 ms must be detected and motor power stopped
    - T.4.3.4: analog signal outside the operating range (e.g. <0.5 V / >4.5 V) = failed.
               An open circuit must read out of range -> pull-down/up resistor in hardware.

    No Arduino dependencies: the caller reads the ADC and passes in the raw count and the
    current time in ms (millis()), so this can also be unit-tested on a desktop.
*/

// Front end (TUNE to your hardware) 
constexpr float   ADC_VREF       = 3.3f;      // Teensy 4.x ADC reference
constexpr float   ADC_MAX_COUNTS = 4095.0f;   // 12-bit resolution
constexpr float   BSE_DIVIDER    = 0.6667f;   // V_pin / V_sensor (resistor divider 5 V -> 3.3 V)

//  Fault limits (sensor volts, i.e. before the divider) 
// Keep the legitimate sensor range INSIDE these limits with margin. A 0.5-4.5 V
// transducer rests near 0.5 V, so a hard 0.5 V limit would false-trip.
constexpr float   BSE_V_FAULT_LOW  = 0.30f;
constexpr float   BSE_V_FAULT_HIGH = 4.70f;

//  Persistence (time-based) 
// The failure must be continuous for this long before the fault latches.
constexpr uint32_t BSE_FAULT_MS = 100;
static_assert(BSE_FAULT_MS < 101, "BSE_FAULT_MS must not persist more than 100 msec(T.4.3.3)");

//  Brake-applied threshold with hysteresis (TUNE from measured data) 
// Must correspond to HYDRAULIC actuation (T.3.2.4: first 90% of travel may be regen only).
constexpr float   BSE_V_ON  = 0.80f;
constexpr float   BSE_V_OFF = 0.70f;

struct BseState {
    uint32_t bad_since_ms  = 0;       // time the current continuous failure began
    bool     bad_active    = false;   // true while the signal is currently failed
    bool     fault         = false;   // latched until power cycle
    bool     brake_pressed = false;
};

// Converts a raw 12-bit ADC count to the sensor voltage (undoing the divider).
float bse_sensor_volts(uint16_t raw);

// Call with each new sample, at any rate.
// sample_ok = false if the conversion timed out or returned ADC_ERROR_VALUE.
// now_ms    = millis() at the time of the sample.
void  bse_update(BseState& s, uint16_t raw, bool sample_ok, uint32_t now_ms);
