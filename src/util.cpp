#include "util.h"

void setup_gpio() {
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    pinMode(BRAKE_LIGHT_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(TS_ACTIVE_PIN, INPUT_PULLDOWN);
    pinMode(DRIVER_ACT_PIN, INPUT_PULLDOWN); 
}

void read_inputs(Inputs& in) {
    in.ts_active = digitalRead(TS_ACTIVE_PIN) == HIGH;
    in.driver_act = digitalRead(DRIVER_ACT_PIN) == HIGH;
    in.brake_pressed = true;
}

void process_gpio(const Outputs &out) {
    digitalWrite(BUZZER_PIN, out.buzzer);
}
