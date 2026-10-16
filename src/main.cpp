#include <Arduino.h>
#include <FlexCAN_T4.h>
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
constexpr uint32_t TICK_DURATION = 10; // 100 hz/10 ms

// ------------- Pedal Calibration (raw ADC, 10-bit 0-1023) -------------
constexpr float APPS_RAW_MIN = 50.0f;          // pedal released
constexpr float APPS_RAW_MAX = 970.0f;         // pedal fully pressed
constexpr float PEDAL_RAW_FAULT_LOW = 10.0f;   // below = open / short to GND
constexpr float PEDAL_RAW_FAULT_HIGH = 1013.0f; // above = short to 3.3V
constexpr float APPS_MAX_DIFF = 10.0f;         // % points allowed between APPS1/APPS2
constexpr uint8_t APPS_FAULT_TICKS = 10;       // implausible for 100 ms -> fault
constexpr float BSE_ON_RAW = 300.0f;           // brake pressed above this
constexpr float BSE_OFF_RAW = 250.0f;          // brake released below this

// ------------- Debug -------------
elapsedMillis debugTimer;
constexpr uint32_t DEBUG_PERIOD = 100; // 10 hz
const char *STATE_NAMES[] = {"NOT_READY", "RTD", "INVERTER_INIT", "DRIVE", "FAULT"};

// ------------- VCU Types -------------
CurrentState curState;
Inputs in;
Outputs out;

// ------------- CAN -------------
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

float apps_to_percent(float raw)
{
    float pct = (raw - APPS_RAW_MIN) / (APPS_RAW_MAX - APPS_RAW_MIN) * 100.0f;
    return constrain(pct, 0.0f, 100.0f);
}

bool raw_out_of_range(float raw)
{
    return raw < PEDAL_RAW_FAULT_LOW || raw > PEDAL_RAW_FAULT_HIGH;
}

void read_inputs(Inputs &in)
{
    static uint8_t implausible_ticks = 0;

    in.apps1_raw = analogRead(APPS1_PIN);
    in.apps2_raw = analogRead(APPS2_PIN);
    in.bse_raw = analogRead(BSE_PIN);

    in.ts_active = digitalRead(TS_ACTIVE_PIN);
    in.driver_act = digitalRead(DRIVER_ACT_PIN);

    // Hysteresis so brake_pressed doesn't chatter at the threshold
    if (in.bse_raw > BSE_ON_RAW)
        in.brake_pressed = true;
    else if (in.bse_raw < BSE_OFF_RAW)
        in.brake_pressed = false;

    float apps1 = apps_to_percent(in.apps1_raw);
    float apps2 = apps_to_percent(in.apps2_raw);
    in.apps_percentage = (apps1 + apps2) / 2.0f;

    // Sensor out of range or APPS disagree by > 10% for 100 ms -> pedal fault
    bool implausible = raw_out_of_range(in.apps1_raw) || raw_out_of_range(in.apps2_raw) ||
                       raw_out_of_range(in.bse_raw) || fabsf(apps1 - apps2) > APPS_MAX_DIFF;
    if (!implausible)
        implausible_ticks = 0;
    else if (implausible_ticks < APPS_FAULT_TICKS)
        implausible_ticks++;
    in.pedal_fault = implausible_ticks >= APPS_FAULT_TICKS;
}

void print_debug()
{
    if (debugTimer < DEBUG_PERIOD)
        return;
    debugTimer = 0;

    Serial.printf("%-13s t=%-5u | apps1=%4d apps2=%4d bse=%4d apps=%5.1f%% | ts=%d drv=%d brk=%d pf=%d | tq=%4d en=%d\n",
                  STATE_NAMES[curState.state], curState.state_ticks,
                  (int)in.apps1_raw, (int)in.apps2_raw, (int)in.bse_raw, in.apps_percentage,
                  in.ts_active, in.driver_act, in.brake_pressed, in.pedal_fault,
                  (int)out.torque_cmd, out.inverter_enable);
}

void setup()
{
    /*
    TODO:
    Set up GPIO pins
    Set up CAN, ADC, WDT
    */
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BRAKE_LIGHT_PIN, OUTPUT);
    pinMode(TS_ACTIVE_PIN, INPUT_PULLDOWN);
    pinMode(DRIVER_ACT_PIN, INPUT_PULLDOWN);

    Serial.begin(115200);

    //    can setup
    can1.begin();
    can1.setBaudRate(500000);
}

void loop()
{
    can1.events();
    if (tick >= TICK_DURATION)
    {
        tick -= TICK_DURATION;
        read_inputs(in);
        run_fsm(curState, in, out);
        Inverter_Process(out);
        print_debug();

        // TODO: feed watchdog
    }
}