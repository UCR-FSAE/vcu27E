#pragma once

#include "vcu_types.h"

enum State { 
    NOT_READY,      // Peripherals/Pins init, waiting for RTD Sequence
    RTD,            // Buzzer
    INVERTER_INIT,  // Inverter init sequence 
    DRIVE,          // read pedals, send torque cmd, check implausibility
    FAULT           // Send 0 torque cmd
};

/*
    Struct to track state of vcu
    state_ticks: gets incremented every 10ms, 
        used to track how long fsm has been in a state,
        used by RTD and INVERTER_INIT states
*/
struct CurrentState {
    State    state = NOT_READY;
    uint16_t state_ticks = 0;
};

void next_state(CurrentState& curSt, State s);
void run_fsm(CurrentState& curSt, const Inputs& in, Outputs& out);
