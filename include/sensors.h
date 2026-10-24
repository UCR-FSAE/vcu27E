#pragma once

#include <stdint.h>

// Initialize the ADC hardware used by the sensor module.
void sensors_init();

// Read the BSE ADC count.
// Returns false if the conversion times out or reports an ADC error.
bool sensors_read_bse(uint16_t& raw);
