#pragma once
#include <stdint.h>

struct Inputs {
    float apps1_raw = 0.0f;
    float apps2_raw = 0.0f;
    float bse_raw = 0.0f;
    float apps_percentage = 0.0f;
    bool  ts_active = false;
    bool  driver_act = false;
    bool  inverter_enabled = false;
    bool  brake_pressed = false;
    bool  pedal_fault = false;
};

struct Outputs {
    float torque_cmd = 0.0f;
    bool  inverter_enable = false;
    bool  buzzer = false;
    bool  brake_light = false;
};