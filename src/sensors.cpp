#include "sensors.h"

#include <Arduino.h>
#include <ADC.h>

// BSE ADC input
constexpr uint8_t BSE_PIN = 39;

static ADC* adc = new ADC();

void sensors_init() {
    pinMode(BSE_PIN, INPUT);

    adc->adc1->setAveraging(4);
    adc->adc1->setResolution(12);
    adc->adc1->setConversionSpeed(ADC_settings::ADC_CONVERSION_SPEED::HIGH_SPEED);
    adc->adc1->setSamplingSpeed(ADC_settings::ADC_SAMPLING_SPEED::HIGH_SPEED);
}

bool sensors_read_bse(uint16_t& raw) {

    static bool conversion_started = false;

    // Should start a conversion if it hasn't alreadu 
    if (conversion_started == true) {
        if (!adc->adc1->startSingleRead(BSE_PIN)) {
            return false;
        }
        conversion_started = true;
        return false;
    }
    // check if the it is complete
    if (!adc->adc1->isComplete()) {
        return false;
    }

    const int value = adc->adc1->readSingle();
    conversion_started = false;
    
    if (value == ADC_ERROR_VALUE) {
        return false;
    }

    raw = static_cast<uint16_t>(value);
    return true;
}
