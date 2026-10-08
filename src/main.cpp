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

void setup() {
    /*
    TODO: 
    Set up GPIO pins
    Set up CAN, ADC, WDT
    */
   pinMode(BUZZER_PIN, OUTPUT);
   pinMode(BRAKE_LIGHT_PIN, OUTPUT);
   pinMode(TS_ACTIVE_PIN, INPUT);
   pinMode(DRIVER_ACT_PIN, INPUT);
}

void loop() {
    if (tick >= TICK_DURATION) {
        tick -= TICK_DURATION;
        // TODO: read_inputs(in)
        /*
            read apps1_raw, apps2_raw, bse_raw
            process apps into apps_percentage
            toggle brake_pressed if bse_raw above threshold
        */
        run_fsm(curState, in, out);
        // TODO: send torque commands over CAN
        // TODO: feed watchdog
    }
}