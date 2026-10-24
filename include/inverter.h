/*
 * inverter.h
 *
 *  Created on: Oct 11, 2026
 *      Author: paul do
 */

#pragma once

#include <cstdint>
#include <FlexCAN_T4.h>
#include "vcu_types.h"

// Inverter functions
bool Inverter_Process(const Outputs &out);
float Inverter_CalcTorqueCommand(float appsPercentage);
void Inverter_EnableInverter(void);
void Inverter_DisableInverter(void);
bool Inverter_ClearInverterFaults(void);
void Inverter_ProcessAnalogInputs(void);
bool Inverter_TransmitCANMessage(uint16_t torque, uint8_t direction, uint8_t inverterEnable);
void canRead(const CAN_message_t &msg);

struct InternalState {
    uint32_t byte6 = 0x00;
    bool is_new = false;
};

extern volatile InternalState internalState;

#define Inverter_INVERTER_COMMAND_ID 0x0C0
#define Inverter_INVERTER_CLEAR_ID 0x0C1
#define Inverter_INVERTER_STATUS_ID 0x0AA

#define Inverter_DIRECTION_FORWARD 0x00
#define Inverter_DIRECTION_REVERSE 0x01
#define Inverter_INVERTER_ENABLE 0x01
#define Inverter_INVERTER_DISABLE 0x00

#define MAX_TORQUE_COMMAND 50

// Parameter message (0x0C1): fault clear
#define Inverter_PARAM_FAULT_CLEAR_ADDR 20
#define Inverter_PARAM_WRITE 0x01

// Init sequence: hold disable this long before enabling (ticks of 10 ms)
#define Inverter_INIT_DISABLE_TICKS 50
