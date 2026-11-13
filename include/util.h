#pragma once

#include <Arduino.h>
#include "vcu_types.h"

// ------------- Pinouts -------------
// GPIO OUT
constexpr uint8_t BUZZER_PIN = 14;
constexpr uint8_t BRAKE_LIGHT_PIN = 15;

// GPIO IN
constexpr uint8_t TS_ACTIVE_PIN = 3;
constexpr uint8_t DRIVER_ACT_PIN = 2;

// ADC IN
constexpr uint8_t APPS1_PIN = 41;
constexpr uint8_t APPS2_PIN = 40;
constexpr uint8_t BSE_PIN = 39;

void setup_gpio();
void read_inputs(Inputs& in);
void process_gpio(const Outputs &out);