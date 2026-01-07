/**
 *******************************************************************************
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
Chassis::Chassis(MotorGM6020* front_SteerLeftMotor, MotorGM6020* front_SteerRightMotor,
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
    fp32 vx = 0.0f; // 前后速度 (m/s)
    fp32 vy = 0.0f; // 左右速度 (m/s)
    fp32 wz = 0.0f; // 旋转角速度 (rad/s)

    if (m_chassisMode == MANUAL_CONTROL)
    {
        const fp32 MAX_SPEED = 3.0f;
        const fp32 MAX_OMEGA = 3.0f;

        vx = rcStickDeadZoneFilter(m_remoteControl.getLeftStickY()) * MAX_SPEED;
        vy = rcStickDeadZoneFilter(m_remoteControl.getLeftStickX()) * MAX_SPEED;
        wz = rcStickDeadZoneFilter(m_remoteControl.getRightStickX()) * MAX_OMEGA;
    }
    else if (m_chassisMode == CHASSIS_NO_FORCE)
    {
        vx = 0.0f;
        vy = 0.0f;
        wz = 0.0f;
    }

    // 计算各轮速度分量
    fp32 rx = halfWheelBase;   
    fp32 ry = halfTrackWidth;

    struct WheelVel { fp32 vx; fp32 vy; };
    WheelVel wheelVels[4];

    // 计算每个舵轮的目标线速度
    wheelVels[0].vx = vx - wz * ry;
    wheelVels[0].vy = vy + wz * rx;

    wheelVels[1].vx = vx - wz * ry;
    wheelVels[1].vy = vy - wz * rx;

    wheelVels[2].vx = vx + wz * ry;
    wheelVels[2].vy = vy - wz * rx;

    wheelVels[3].vx = vx + wz * ry;
    wheelVels[3].vy = vy + wz * rx;

    // 应用目标速度和角度到电机
    auto updateMotorPair = [&](MotorGM6020* steer, MotorM3508* drive, fp32 t_vx, fp32 t_vy) {
        if (steer == nullptr || drive == nullptr) return;

        fp32 targetSpeed = sqrtf(t_vx * t_vx + t_vy * t_vy);

        // 如果速度非常小，则保持当前角度或归零速度
        if (targetSpeed < 0.01f) {
            drive->setTargetAngularVelocity(0.0f);
            return;
        }

        fp32 targetAngle = atan2f(t_vy, t_vx);

        fp32 currentAngle = steer->getCurrentAngle();
        fp32 angleDiff = targetAngle - currentAngle;

        // 确保角度差限制在 [-PI, PI] 之间
        while (angleDiff > MATH_PI) angleDiff -= 2.0f * MATH_PI;
        while (angleDiff < -MATH_PI) angleDiff += 2.0f * MATH_PI;

        // 如果角度差大于 90 度，则反向驱动
        if (fabsf(angleDiff) > (MATH_PI / 2.0f)) {
            angleDiff = (angleDiff > 0) ? angleDiff - MATH_PI : angleDiff + MATH_PI;
            targetSpeed = -targetSpeed;
        }

        // 计算最终目标角度
        fp32 finalTargetAngle = currentAngle + angleDiff;
        steer->setTargetAngle(finalTargetAngle);

        // 将速度转换为角速度并设置给驱动电机
        fp32 targetOmega = targetSpeed / wheelRadius;
        drive->setTargetAngularVelocity(targetOmega);
    };

    // 更新每个舵轮电机
    updateMotorPair(m_front_SteerLeftMotor, m_front_WheelLeftMotor, wheelVels[0].vx, wheelVels[0].vy);
    updateMotorPair(m_back_SteerLeftMotor, m_back_WheelLeftMotor, wheelVels[1].vx, wheelVels[1].vy);
    updateMotorPair(m_back_SteerRightMotor, m_back_WheelRightMotor, wheelVels[2].vx, wheelVels[2].vy);
    updateMotorPair(m_front_SteerRightMotor, m_front_WheelRightMotor, wheelVels[3].vx, wheelVels[3].vy);
}


void Chassis::motorControl()
{
    // 无力模式时停止所有电机
    if (m_chassisMode == CHASSIS_NO_FORCE)
    {
        auto stopMotor = [](Motor* m) {
            if (m) m->setTargetTorqueCurrent(0); // 或者使用 openloopControl(0)
        };
        stopMotor(m_front_SteerLeftMotor);
        stopMotor(m_front_WheelLeftMotor);
        stopMotor(m_back_SteerLeftMotor);
        stopMotor(m_back_WheelLeftMotor);
        stopMotor(m_back_SteerRightMotor);
        stopMotor(m_back_WheelRightMotor);
        stopMotor(m_front_SteerRightMotor);
        stopMotor(m_front_WheelRightMotor);
        return;
    }

    // 正常模式下进行闭环控制
    auto runControl = [](MotorGM6020* steer, MotorM3508* drive) {
        if (steer) steer->angleClosedloopControl();
        if (drive) drive->angularVelocityClosedloopControl();
    };

    // 运行电机控制
    runControl(m_front_SteerLeftMotor, m_front_WheelLeftMotor);
    runControl(m_back_SteerLeftMotor, m_back_WheelLeftMotor);
    runControl(m_back_SteerRightMotor, m_back_WheelRightMotor);
    runControl(m_front_SteerRightMotor, m_front_WheelRightMotor);

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






