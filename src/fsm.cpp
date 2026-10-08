#include "fsm.h"

/*
    NOT_READY,      // Peripherals/Pins init, waiting for RTD Sequence
    RTD,            // Buzzer
    INVERTER_INIT,  // Inverter init sequence 
    DRIVE,          // read pedals, send torque cmd, check implausibility
    FAULT           // Send 0 torque cmd, disable inverter
*/

/*
    resets state_ticks
*/
void next_state(CurrentState& curSt, State s) {
    curSt.state = s;
    curSt.state_ticks = 0;
}

void run_fsm(CurrentState& curSt, const Inputs& in, Outputs& out) {
    out = {};

    switch (curSt.state) {
        case NOT_READY:
            if (in.ts_active && in.driver_act && in.brake_pressed) { next_state(curSt, RTD); }
            break;
        case RTD:
            if (!in.ts_active || !in.brake_pressed) { 
                next_state(curSt, NOT_READY); 
                break;
            }
            // TODO: turn on buzzer -> if 3 sec pass -> turn off buzzer -> next state
            next_state(curSt, INVERTER_INIT);
            break;
        case INVERTER_INIT:
            if (!in.ts_active) {
                next_state(curSt, NOT_READY);
                break;
            }
            // TODO: inverter init sequence
            next_state(curSt, DRIVE);
            break;
        case DRIVE:
            if (!in.ts_active) {
                next_state(curSt, NOT_READY);
                break;
            }
            if (in.pedal_fault) {
                next_state(curSt, FAULT);
                break;
            }

            // TODO: enable inverter, calculate torque command
            break;
        case FAULT:
            out.torque_cmd = 0;
            out.inverter_enable = false;
            break; 
    }

    if (curSt.state_ticks < UINT16_MAX) { curSt.state_ticks++; }
}