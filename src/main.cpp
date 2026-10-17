#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Watchdog_t4.h>
#include <ADC.h>
#include <elapsedMillis.h>
#include "inverter.h"

#include "fsm.h"

// ------------- Pinouts -------------
constexpr uint8_t POT_PIN = 17;

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

elapsedMillis tick;
constexpr uint32_t TICK_DURATION = 10; // 100 hz/10 ms

// ------------- Debug -------------
elapsedMillis debugTimer;
constexpr uint32_t DEBUG_PERIOD = 100; // 10 hz
const char *STATE_NAMES[] = {"NOT_READY", "RTD", "INVERTER_INIT", "DRIVE", "FAULT"};

// ------------- VCU Types -------------
CurrentState curState;
Inputs in;
Outputs out;

// ------------- Library Setup -------------
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
ADC *adc = new ADC();

void print_debug() {
    if (debugTimer < DEBUG_PERIOD)
        return;
    debugTimer = 0;

    Serial.printf("%-13s t=%-5u tq=%4d en=%d\n",
                  STATE_NAMES[curState.state], curState.state_ticks,
                  (int)out.torque_cmd, out.inverter_enable);
}

void setup() {
    /*
    TODO:
    Set up GPIO pins
    Set up CAN, ADC, WDT
    */
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BRAKE_LIGHT_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    // ADC setup
    adc->adc0->setAveraging(16);                                      // average 16 samples
    adc->adc0->setResolution(12);                                     // 12-bit: 0-4095
    adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
    adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
    adc->adc0->startSingleRead(POT_PIN);

    Serial.begin(115200);

    //    can setup
    can1.begin();
    can1.setBaudRate(500000);
}

uint16_t potRaw = 0;
uint32_t blink_timer = 0;

void loop() {
    uint32_t time_now = millis();
    if (time_now - blink_timer >= 100) {
        blink_timer = time_now;

        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_BUILTIN, LOW);
    can1.events();

    if (tick >= TICK_DURATION) {
        tick -= TICK_DURATION;
        // read_inputs(in);

        if (adc->adc0->isComplete()) {
            potRaw = adc->adc0->readSingle();
            adc->adc0->startSingleRead(POT_PIN);
        }
        
        in.apps_percentage = (float)potRaw * 100u / 4095u;
        // in.apps_percentage = potRaw;
        in.ts_active = true;
        in.driver_act = true;
        in.brake_pressed = true;

        run_fsm(curState, in, out);
        Inverter_Process(out);
        print_debug();

        // TODO: feed watchdog
    }
}