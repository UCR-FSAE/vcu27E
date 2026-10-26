#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Watchdog_t4.h>
#include <elapsedMillis.h>

#include "inverter.h"
#include "fsm.h"
#include "bse.h"
#include "sensors.h"

// ------------- GPIO Pinouts -------------
constexpr uint8_t BUZZER_PIN = 14;
constexpr uint8_t BRAKE_LIGHT_PIN = 15;
constexpr uint8_t TS_ACTIVE_PIN = 3;
constexpr uint8_t DRIVER_ACT_PIN = 2;

elapsedMillis bse_timer;
constexpr uint32_t BSE_SAMPLE_PERIOD_MS = 1;
elapsedMillis tick;
constexpr uint32_t TICK_DURATION = 10; // 100 Hz / 10 ms

// ------------- Debug -------------
elapsedMillis debugTimer;
constexpr uint32_t DEBUG_PERIOD = 100; // 10 Hz

const char* STATE_NAMES[] = {
    "NOT_READY",
    "RTD",
    "INVERTER_INIT",
    "DRIVE",
    "FAULT"
};

// ------------- VCU Types -------------
CurrentState curState;
Inputs in;
Outputs out;
BseState bseState;

static uint16_t last_bse_raw = 0;

// ------------- Library Setup -------------
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

void print_debug() {
    if (debugTimer < DEBUG_PERIOD)
        return;
    debugTimer = 0;

    Serial.printf("%-13s t=%-5u tq=%4d en=%d bse=%.2fV brk=%d bse_fault=%d\n",
        STATE_NAMES[curState.state],
        curState.state_ticks,
        (int)out.torque_cmd,
        out.inverter_enable,
        bse_sensor_volts(last_bse_raw),
        in.brake_pressed,
        in.bse_fault
    );
}

void setup() {
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BRAKE_LIGHT_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);

    // Initialize sensor hardware.
    sensors_init();

    // CAN setup.
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

    if (bse_timer >= BSE_SAMPLE_PERIOD_MS) {
        bse_timer = 0;

        uint16_t bse_raw = 0;
        const bool bse_ok = sensors_read_bse(bse_raw);

        if (bse_ok) {
            last_bse_raw = bse_raw;
            in.bse_raw = static_cast<float>(bse_raw);
        }

        bse_update(bseState, bse_raw, bse_ok, millis());
        in.bse_fault = bseState.fault;
        in.brake_pressed = bseState.brake_pressed;
    }

    if (tick >= TICK_DURATION) {
        tick -= TICK_DURATION;

        if (internalState.is_new) {
            in.inverter_enabled = ((internalState.byte6 & 0x01) == 0x01);
            in.inverter_locked  = ((internalState.byte6 & 0x80) != 0x00);
            internalState.is_new = false;
        }

        run_fsm(curState, in, out);
        digitalWrite(BRAKE_LIGHT_PIN, out.brake_light);
        Inverter_Process(out);
        print_debug();

        // TODO: Feed watchdog.
    }
}
