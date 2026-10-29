#include <Arduino.h>
#include <Watchdog_t4.h>
#include <ADC.h>
#include <elapsedMillis.h>
#include "inverter.h"

#include "fsm.h"

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
    /*
    TODO:
    Set up GPIO pins
    Set up CAN, ADC, WDT
    */
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BRAKE_LIGHT_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);

    //    can setup
    can1.begin();
    can1.setBaudRate(500000);
    can1.enableFIFO();
    can1.enableFIFOInterrupt();
    can1.setFIFOFilter(REJECT_ALL);
    can1.setFIFOFilter(0, 0x0AA, STD);
    can1.onReceive(canRead);
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
        // read_inputs(in);
        run_fsm(curState, in, out);
        Inverter_Process(out);
        // print_debug();

        // TODO: feed watchdog
    }
}