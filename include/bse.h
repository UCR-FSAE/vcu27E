#pragma once
#include <stdint.h>

/*
    Brake System Encoder (BSE) - FSAE 2027 T.4.3
    - T.4.3.3: a BSE failure persisting > 100 ms must be detected and motor power stopped
    - T.4.3.4: analog signal outside the operating range (e.g. <0.5 V / >4.5 V) = failed.
               An open circuit must read out of range -> pull-down/up resistor in hardware.
*/

constexpr float   ADC_VREF       = 3.3f;      // Teensy 4.x ADC reference
constexpr float   ADC_MAX_COUNTS = 4095.0f;   // 12-bit resolution
constexpr float   BSE_DIVIDER    = 0.6667f;   // V_pin / V_sensor (resistor divider 5 V -> 3.3 V)

//  Fault limits (sensor volts, i.e. before the divider)
// Keep the legitimate sensor range INSIDE these limits with margin. A 0.5-4.5 V
// transducer rests near 0.5 V, so a hard 0.5 V limit would false-trip.
constexpr float   BSE_V_FAULT_LOW  = 0.30f;
constexpr float   BSE_V_FAULT_HIGH = 4.70f;

// Persistence: 5 ticks x 10 ms tick = 50 ms. Worst-case detection is ~60 ms after the
// failure, leaving ~40 ms of the 100 ms budget for CAN + inverter reaction.
constexpr uint8_t BSE_FAULT_TICKS  = 5;

// Brake-applied threshold with hysteresis
// Must correspond to HYDRAULIC actuation (T.3.2.4: first 90% of travel may be regen only).
constexpr float   BSE_V_ON  = 0.80f;
constexpr float   BSE_V_OFF = 0.70f;

struct BseState {
    uint8_t bad_ticks     = 0;
    bool    fault         = false;   // latched until power cycle
    bool    brake_pressed = false;
};

// Converts a raw 12-bit ADC count to the sensor voltage (undoing the divider).
float bse_sensor_volts(uint16_t raw);

// Call once per tick with the raw ADC count.
// sample_ok = false if the conversion timed out or returned ADC_ERROR_VALUE.
void  bse_update(BseState& s, uint16_t raw, bool sample_ok);
