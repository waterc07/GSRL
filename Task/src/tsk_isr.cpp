/**
 ******************************************************************************
 * @file           : tsk_isr.cpp
 * @brief          : 中断服务程序
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "tsk_isr.hpp"
#include "crt_chassis.hpp"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/
extern CrtChassis chassis;

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/
void dr16RxCallback(uint8_t *Buffer, uint16_t Length)
{
    chassis.receiveRemoteControlDataFromISR(Buffer);
}

void can1RxCallback(can_rx_message_t *pRxMsg)
{
    // CAN1: 驱动电机 (M3508)
    chassis.receiveChassisDriveMotorDataFromISR(pRxMsg);
}

void can2RxCallback(can_rx_message_t *pRxMsg)
{
    // CAN2: 舵向电机 (GM6020)
    chassis.receiveChassisSteerMotorDataFromISR(pRxMsg);
}
