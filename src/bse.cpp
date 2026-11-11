#include "bse.h"

float bse_sensor_volts(uint16_t raw) {
    return (raw * (ADC_VREF / ADC_MAX_COUNTS));
}

void bse_update(BseState& s, uint16_t raw, bool sample_ok, uint32_t now_ms) {
    if (!sample_ok) {
        s.brake_pressed = false;
        return;
    }

    const float v = bse_sensor_volts(raw);

    if (v > BRAKES_ACTIVATION_THRESHOLD) {
        s.brake_pressed = true;
    }
}
