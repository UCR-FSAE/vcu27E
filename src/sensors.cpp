#include "sensors.h"

#include <Arduino.h>
#include <ADC.h>

// BSE ADC input
constexpr uint8_t BSE_PIN = 39;

// Maximum time to wait for one ADC conversion, in microseconds.
constexpr uint32_t ADC_TIMEOUT_US = 300;

static ADC* adc = new ADC();

void sensors_init() {
    pinMode(BSE_PIN, INPUT);

    adc->adc1->setAveraging(4);
    adc->adc1->setResolution(12);
    adc->adc1->setConversionSpeed(ADC_settings::ADC_CONVERSION_SPEED::HIGH_SPEED);
    adc->adc1->setSamplingSpeed(ADC_settings::ADC_SAMPLING_SPEED::HIGH_SPEED);
}

bool sensors_read_bse(uint16_t& raw) {
    if (!adc->adc1->startSingleRead(BSE_PIN)) {
        return false;
    }

    const uint32_t t0 = micros();

    while (!adc->adc1->isComplete()) {
        if (micros() - t0 > ADC_TIMEOUT_US) {
            return false;
        }
    }

    const int value = adc->adc1->readSingle();

    if (value == ADC_ERROR_VALUE) {
        return false;
    }

    raw = static_cast<uint16_t>(value);
    return true;
}
