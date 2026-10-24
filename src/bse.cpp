#include "bse.h"

float bse_sensor_volts(uint16_t raw) {
    return (raw * (ADC_VREF / ADC_MAX_COUNTS)) / BSE_DIVIDER;
}

void bse_update(BseState& s, uint16_t raw, bool sample_ok) {
    const float v = bse_sensor_volts(raw);

    // 1. Range / sample check (T.4.3.4). An open circuit is caught because the
    //    hardware pull-down drags the pin below BSE_V_FAULT_LOW.
    const bool bad = !sample_ok || v < BSE_V_FAULT_LOW || v > BSE_V_FAULT_HIGH;

    // 2. Persistence timer (T.4.3.3). Any good sample restarts the count.
    if (bad) {
        if (s.bad_ticks < UINT8_MAX) { s.bad_ticks++; }
    } else {
        s.bad_ticks = 0;
    }
    if (s.bad_ticks >= BSE_FAULT_TICKS) { s.fault = true; }   // latched

    // 3. Brake applied, with hysteresis. Forced false while faulted or on a bad
    //    sample so the RTD sequence can't start on bad data.
    if (s.fault || !sample_ok) {
        s.brake_pressed = false;
        return;
    }
    if (!s.brake_pressed && v > BSE_V_ON) {
        s.brake_pressed = true;
    } else if (s.brake_pressed && v < BSE_V_OFF) {
        s.brake_pressed = false;
    }
}
