#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Watchdog_t4.h>
#include <ADC.h>
#include <elapsedMillis.h>

elapsedMillis tick;
constexpr uint32_t TICK_DURATION = 10; // 100 hz/10 ms

void setup() {
    /*
    Set up GPIO pins
    Set up CAN, ADC, WDT
    */
}

void loop() {
    if (tick >= TICK_DURATION) {
        tick -= TICK_DURATION;
        // read inputs
        // run fsm
        // send can
        // feed wdt
    }
}