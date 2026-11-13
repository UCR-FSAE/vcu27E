#include <Arduino.h>
#include <Watchdog_t4.h>
#include <ADC.h>
#include <elapsedMillis.h>
#include "inverter.h"
#include "util.h"
#include "fsm.h"

elapsedMillis tick;
constexpr uint32_t TICK_DURATION = TICK_MS; // 100 hz/10 ms

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

void print_debug() {
    if (debugTimer < DEBUG_PERIOD)
        return;
    debugTimer = 0;

    Serial.printf("%-13s t=%-5u tq=%4d en=%d\n",
                  STATE_NAMES[curState.state], curState.state_ticks,
                  (int)out.torque_cmd, out.inverter_enable);
}

void setup() {
    setup_gpio();

    Serial.begin(115200);

    //    can setup
    can1.begin();
    can1.setBaudRate(500000);
    can1.enableFIFO();
    can1.enableFIFOInterrupt();
    can1.setFIFOFilter(REJECT_ALL);
    can1.setFIFOFilter(0, 0x0AA, STD);
    can1.onReceive(canRead);

    tick = 0;
}

uint32_t blink_timer = 0;
uint32_t blink_thres = 10000;

void loop() {
    if (curState.state == INVERTER_INIT) { blink_thres = 1000; }
    if (curState.state == DRIVE) { blink_thres = 100; }

    if (millis() - blink_timer >= blink_thres) {
        blink_timer = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }

    can1.events();
    
    if (tick >= TICK_DURATION) {
        tick -= TICK_DURATION;
        
        Inverter_UpdateState();
        read_inputs(in);
        run_fsm(curState, in, out);
        Inverter_Process(out);
        process_gpio(out);
        // print_debug();

        // TODO: feed watchdog
    }
}