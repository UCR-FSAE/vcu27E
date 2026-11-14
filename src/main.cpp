#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Watchdog_t4.h>
#include <ADC.h>
#include <elapsedMillis.h>

#include "fsm.h"

// ------------- Pinouts -------------
// GPIO OUT
constexpr uint8_t BUZZER_PIN        = 14;
constexpr uint8_t BRAKE_LIGHT_PIN   = 15;

// GPIO IN
constexpr uint8_t TS_ACTIVE_PIN     = 3;
constexpr uint8_t DRIVER_ACT_PIN    = 2;

// ADC IN
constexpr uint8_t APPS1_PIN         = 41;
constexpr uint8_t APPS2_PIN         = 40;
constexpr uint8_t BSE_PIN           = 39;


elapsedMillis tick;
constexpr uint32_t TICK_DURATION = 10; // 100 hz/10 ms

// ------------- VCU Types -------------
CurrentState    curState;
Inputs          in;
Outputs         out;

ADC *adc = new ADC();

float apps1Min = 1.4;
float apps1Max = 3.2;
float apps2Min = 0.4;
float apps2Max = 1.4;
float voltScale = 3.3;

constexpr float ADC_MAX_COUNTS = 4095.0f;

float sensorPercent(float volts, float voltMin, float voltMax) {
    float percent = (volts - voltMin) / (voltMax - voltMin) * 100;
    if (percent < 0) {
        percent = 0;
    }
    if (percent > 100) {
        percent = 100;
    }
    return percent;
}

float calcAppsPercentage(float apps1Raw, float apps2Raw) {
    float apps1Volts = apps1Raw * voltScale / ADC_MAX_COUNTS;
    float apps2Volts = apps2Raw * voltScale / ADC_MAX_COUNTS;

    float apps1Percent = sensorPercent(apps1Volts, apps1Min, apps1Max);
    float apps2Percent = sensorPercent(apps2Volts, apps2Min, apps2Max);

    return (apps1Percent + apps2Percent) / 2;
}

void canRead(const CAN_message_t &msg) {
    switch(msg.id) {
        case 0x0AA:
            in.inverter_enabled = (msg.buf[6] & 0x01) != 0;
            break;
    }
}

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
WDT_T4<WDT3> wdt;

constexpr uint32_t WDT_TIMEOUT_MS = 100; 
constexpr float BSE_THRESHOLD = 1000;

void setup() {

   pinMode(BUZZER_PIN, OUTPUT);
   pinMode(BRAKE_LIGHT_PIN, OUTPUT);
   pinMode(TS_ACTIVE_PIN, INPUT);
   pinMode(DRIVER_ACT_PIN, INPUT);

   // setting up ADC
   adc->adc0->setResolution(12);
   adc->adc0->setAveraging(4);
   adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED);
   adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED);
   adc->adc1->setResolution(12);
   adc->adc1->setAveraging(4);
   adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED);
   adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED);

   // setting up can1
   can1.begin();
   can1.setBaudRate(500000);
   can1.enableFIFO();
   can1.enableFIFOInterrupt();
   can1.setFIFOFilter(REJECT_ALL);
   can1.setFIFOFilter(0, 0x0AA, STD);
   can1.onReceive(canRead);

   //setting up watchdog
    WDT_timings_t wdt_config;
    wdt_config.timeout = WDT_TIMEOUT_MS;

   wdt.begin(wdt_config);
}

void loop() {
    can1.events();
    if (tick >= TICK_DURATION) {
        tick -= TICK_DURATION;

        in.apps1_raw = adc->adc0->analogRead(APPS1_PIN);
        in.apps2_raw = adc->adc1->analogRead(APPS2_PIN);
        in.bse_raw = adc->adc1->analogRead(BSE_PIN);

        in.apps_percentage = calcAppsPercentage(in.apps1_raw, in.apps2_raw);

        if (in.bse_raw > BSE_THRESHOLD) {
            in.brake_pressed = true;
        } else {
            in.brake_pressed = false;
        }

        in.ts_active = (digitalRead(TS_ACTIVE_PIN) == HIGH);
        in.driver_act = (digitalRead(DRIVER_ACT_PIN) == HIGH);

        run_fsm(curState, in, out);
        wdt.feed();
    }
}