/*******************************************************************************
 * @file           : crt_chassis.cpp
 * @brief          : 舵轮底盘控制器实现
 *******************************************************************************
 * @attention
 *
 * 本文件实现舵轮底盘控制器的具体逻辑，包括：
 * - 底盘速度指令的接收与保存
 * - 舵轮运动学计算
 * - 多舵轮模块的统一更新与调度
 *
 * 本文件不包含 FreeRTOS 任务循环，不直接读取遥控或传感器数据，
 * 仅作为底盘功能逻辑模块，由 Task 层调用。
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "crt_chassis.hpp"
#include "stm32f4xx_hal_can.h"
#include "tsk_chassis.cpp"
#include  "drv_misc.h"
/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/
Chassis::Chassis(MotorM6020* front_SteerLeftMotor, MotorGM6020* front_SteerRightMotor,
                 MotorGM6020* back_SteerLeftMotor, MotorGM6020* back_SteerRightMotor,
                 MotorM3508* front_WheelLeftMotor,MotorM3508* front_WheelRightMotor,
                 MotorM3508* back_WheelLeftMotor,MotorM3508* back_WheelRightMotor,
                 IMU* imu)
    : m_front_SteerLeftMotor(front_SteerLeftMotor),
      m_front_SteerRightMotor(front_SteerRightMotor),
      m_back_SteerLeftMotor(back_SteerLeftMotor),
      m_back_SteerRightMotor(back_SteerRightMotor),
      m_front_WheelLeftMotor(front_WheelLeftMotor),
      m_front_WheelRightMotor(front_WheelRightMotor),
      m_back_WheelLeftMotor(back_WheelLeftMotor),
      m_back_WheelRightMotor(back_WheelRightMotor),
      m_imu(imu) {}

void Chassis::init()
{
    DWT_Init();
    CAN_Init(&hcan1, can1RxCallback);
    CAN_Init(&hcan2, can2RxCallback);
    UART_Init(&huart3, dr16RxCallback, 36);
    m_imu->init();
    m_isInitComplete = true;
}

void Chassis::controlLoop()
{
    if (!m_isInitComplete) return;
    //模式选择
    modeSelect();
    //速度解算
    targetSpeedplan();
    //电机控制
    motorControl();
    //数据发送
    transmitChassisMotorData();
}

void Chassis::imuLoop()
{
    if (!m_isInitComplete) return;
    m_eulerAngle = m_imu->solveAttitude();
}

void Chassis::receiveChassisMotorDataFromISR(const can_rx_message_t *rxMessage)
{
    if (m_front_SteerLeftMotor && m_front_WheelLeftMotor->decodeCanRxMessageFromISR(rxMessage)) return;
    if (m_front_SteerRightMotor && m_front_WheelRightMotor->decodeCanRxMessageFromISR(rxMessage)) return;
    if (m_back_SteerLeftMotor && m_back_WheelLeftMotor->decodeCanRxMessageFromISR(rxMessage)) return;
    if (m_back_SteerRightMotor && m_back_WheelRightMotor->decodeCanRxMessageFromISR(rxMessage)) return;
}

void Chassis::receiveRemoteControlDataFromISR(const uint8_t *rxData)
{
    m_remoteControl.receiveRxDataFromISR(rxData);
}

void Chassis::modeSelect()
{
    m_remoteControl.updateEvent();
    if (!m_remoteControl.isDr16RemoteControlConnected()) {
        m_chassisMode  = CHASSIS_NO_FORCE;
        return;
    }

    switch (m_remoteControl.getLeftSwitchStatus()) {
        case Dr16RemoteControl::SWITCH_DOWN:
            m_chassisMode  = CHASSIS_NO_FORCE;
            break;

        case Dr16RemoteControl::SWITCH_MIDDLE:
            m_chassisMode  = MANUAL_CONTROL;
            break;

        case Dr16RemoteControl::SWITCH_UP:
            m_chassisMode  = AUTO_CONTROL;
            break;

        default:
            break;
    }
}

void Chassis::targetSpeedplan()
{
    //速度解算
}

void Chassis::motorControl()
{
    //电机控制
}

void Chassis::transmitChassisMotorData()
{
    //看具体接线再写
}

inline fp32 Chassis::rcStickDeadZoneFilter(const fp32 &rcStickValue)
{
    if (rcStickValue > DT7_STICK_DEAD_ZONE)
        return (rcStickValue - DT7_STICK_DEAD_ZONE) / (1.0f - DT7_STICK_DEAD_ZONE);
    else if (rcStickValue < -DT7_STICK_DEAD_ZONE)
        return (rcStickValue + DT7_STICK_DEAD_ZONE) / (1.0f - DT7_STICK_DEAD_ZONE);
    else
        return 0.0f;
}






