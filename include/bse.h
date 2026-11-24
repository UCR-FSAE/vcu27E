#pragma once
#include <stdint.h>

constexpr float   ADC_VREF       = 3.3f;
constexpr float   ADC_MAX_COUNTS = 4095.0f;

//  Brake-applied threshold
constexpr float   BRAKES_ACTIVATION_THRESHOLD  = 0.50f;

struct BseState {
    bool     brake_pressed = false;
};

float bse_sensor_volts(uint16_t raw);

void  bse_update(BseState& s, uint16_t raw, bool sample_ok, uint32_t curr_ms);
