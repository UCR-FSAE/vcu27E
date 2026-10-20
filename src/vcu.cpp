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

constexpr uint32_t ADC_TIMEOUT_US   = 300;  // 300ms (conversion wt)


elapsedMillis tick;
constexpr uint32_t TICK_DURATION = 10; // 100 hz/10 ms

// ------------- VCU Types -------------
CurrentState    curState;
Inputs          in;
Outputs         out;
BseState        bseState; // Will implement in the loop() section for range checks,etc.

ADC* adc = new ADC();

// Reads the BSE once. Returns false on timeout or ADC error.
// raw : stores the actual sensor reading
static bool read_bse(uint16_t& raw) {
    // If the starting conversion failes this condition should become true and the function returns false
    if (!adc -> adc1 -> startSingleRead(BSE_PIN)) { return false; }

    const int v = adc->adc1->readSingle();
    if (v == ADC_ERROR_VALUE) { return false; }
    // gets the digital reading adn stores it in v
    raw = (uint16_t)v;
    return true;
}

void setup() {
    /*
    TODO: 
    Set up CAN, WDT
    */
   pinMode(BUZZER_PIN, OUTPUT);
   pinMode(BRAKE_LIGHT_PIN, OUTPUT);
   pinMode(TS_ACTIVE_PIN, INPUT);
   pinMode(DRIVER_ACT_PIN, INPUT);

   // ADC (BSE only for now)
   pinMode(BSE_PIN, INPUT);
   adc->adc1->setAveraging(4); // 4 readings
   adc->adc1->setResolution(12); //3.3V of Teensy 4.1  so the ADC reading is 4095 so resolution set to 12 bits 2^12
   adc->adc1->setConversionSpeed(ADC_settings::HIGH_SPEED); //is guaranteed to be the highest possible speed within specs for resolutions
   adc->adc1->setSamplingSpeed(ADC_settings::HIGH_SPEED); // adds +4 ADCK, 6 in total -> 20MHz ADClibrary documentation
}


//Note : Have to impliment the vcu.h to set up the BSE status in the struct of BseState and create bse_update

void loop() {
    if (tick >= TICK_DURATION) {
        tick -= TICK_DURATION;
        // TODO: read_inputs(in)
        /*
            read apps1_raw, apps2_raw, bse_raw
            process apps into apps_percentage
            toggle brake_pressed if bse_raw above threshold
        */

        // raw = Raw ADC reading (12 bit)
        // ok = determines where the reading succedeed
        uint16_t bse_raw = 0;
        const bool bse_ok = read_bse(bse_raw);
        bse_update(bseState, bse_raw, bse_ok);
        in.bse_fault = bseState.fault;
        in.brake_pressed = bseState.brake_pressed;

        run_fsm(curState, in, out);

        digitalWrite(BRAKE_LIGHT_PIN, out.brake_light);
        // TODO: send torque commands over CAN
        // TODO: feed watchdog
    }
}
