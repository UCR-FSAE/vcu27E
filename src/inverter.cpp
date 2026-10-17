/*
 * inverter.c
 *
 *  Created on: Oct 11, 2026
 *      Author: Paul Do
 */

#include "inverter.h"
#include <FlexCAN_T4.h>
#include <Arduino.h>

extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

/**
 * @brief  Convert accelerator pedal position to an inverter torque command
 * @param  appsPercentage: pedal position (0 - 100)
 * @retval Torque command in inverter units (0 - MAX_TORQUE_COMMAND)
 */
float Inverter_CalcTorqueCommand(float appsPercentage)
{
    if (appsPercentage < 0.0f)
        appsPercentage = 0.0f;
    if (appsPercentage > 100.0f)
        appsPercentage = 100.0f;

    return (appsPercentage / 100.0f) * MAX_TORQUE_COMMAND;
}

/**
 * @brief  Send the FSM's torque/enable outputs to the inverter (call once per tick)
 * @param  out: FSM outputs; torque_cmd is in inverter units (0 - MAX_TORQUE_COMMAND)
 * @retval true if the frame was accepted by the CAN controller
 */
bool Inverter_Process(const Outputs &out)
{
    /* Clamp so out-of-range or negative values never reach the bus */
    float torque = out.torque_cmd;
    if (torque < 100.0f) // be at 100 for now since breadboard pot value floats
        torque = 0.0f;
    if (torque > MAX_TORQUE_COMMAND)
        torque = MAX_TORQUE_COMMAND;

    /* Never command torque while disabled */
    if (!out.inverter_enable)
        torque = 0.0f;

    return Inverter_TransmitCANMessage((uint16_t)torque,
                                       Inverter_DIRECTION_FORWARD,
                                       out.inverter_enable ? Inverter_INVERTER_ENABLE
                                                           : Inverter_INVERTER_DISABLE);
}

/**
 * @brief  Transmit CAN message to inverter
 * @param  torque: Torque command value (0-32767)
 * @param  direction: Direction command (0=forward, 1=reverse)
 * @param  inverterEnable: Inverter enable state (0=disable, 1=enable)
 * @retval None
 */
bool Inverter_TransmitCANMessage(uint16_t torque, uint8_t direction, uint8_t inverterEnable)
{
    CAN_message_t msg;

    /* Configure transmission */
    msg.id = Inverter_INVERTER_COMMAND_ID;
    msg.flags.extended = 0;
    msg.len = 8;

    /* Pack torque command (little-endian) */
    msg.buf[0] = torque & 0xFF;
    msg.buf[1] = (torque >> 8) & 0xFF;

    /* Speed command (0 for torque control mode) */
    msg.buf[2] = 0;
    msg.buf[3] = 0;

    /* Direction and inverter control */
    msg.buf[4] = direction;
    msg.buf[5] = inverterEnable;

    /* Torque limits (using default) */
    msg.buf[6] = 0;
    msg.buf[7] = 0;

    return can1.write(msg) == 1;
}

/**
 * @brief  Enable inverter
 * @retval None
 */
void Inverter_EnableInverter(void) { Inverter_TransmitCANMessage(0, Inverter_DIRECTION_FORWARD, Inverter_INVERTER_ENABLE); }

void Inverter_DisableInverter(void) { Inverter_TransmitCANMessage(0, 0, Inverter_INVERTER_DISABLE); }

/**
 * @brief  Clear active inverter faults (parameter write to fault-clear address)
 * @retval true if the frame was accepted by the CAN controller
 */
bool Inverter_ClearInverterFaults(void)
{
    CAN_message_t msg;

    msg.id = Inverter_INVERTER_CLEAR_ID;
    msg.flags.extended = 0;
    msg.len = 8;

    /* Parameter address (little-endian) */
    msg.buf[0] = Inverter_PARAM_FAULT_CLEAR_ADDR & 0xFF;
    msg.buf[1] = (Inverter_PARAM_FAULT_CLEAR_ADDR >> 8) & 0xFF;

    /* Read/write: write */
    msg.buf[2] = Inverter_PARAM_WRITE;
    msg.buf[3] = 0;

    /* Data: 0 clears faults */
    msg.buf[4] = 0;
    msg.buf[5] = 0;
    msg.buf[6] = 0;
    msg.buf[7] = 0;

    return can1.write(msg) == 1;
}