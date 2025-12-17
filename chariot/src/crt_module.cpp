/**
 *******************************************************************************
 * @file           : crt_module.cpp
 * @brief          : 单舵轮模块控制实现
 *******************************************************************************
 * @attention
 *
 * 本文件实现单个舵轮模块的具体控制逻辑，包括：
 * - 转向角度控制
 * - 驱动速度控制
 * - 舵轮最短转向与方向优化（如启用）
 *
 * 本模块不涉及底盘整体运动学与任务调度，
 * 由底盘控制器统一调用。
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "crt_module.hpp"
#include <cmath>

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

CrtModule::CrtModule(MotorGM6020 *steerMotor, MotorM3508 *driveMotor)
{
    /* 绑定电机对象（外部需确保非空） */
    m_steerMotor = steerMotor;
    m_driveMotor = driveMotor;

    /* 初始化目标指令 */
    targetAngle = 0.0f;
    targetSpeed = 0.0f;

    /* 初始化角度状态 */
    currentAngle     = 0.0f;
    lastRawAngle     = 0.0f;
    angleAccumulated = 0.0f;
}

void CrtModule::setTarget(float targetAngle, float targetSpeed)
{
    /* 保存目标指令，不在此处进行任何处理 */
    this->targetAngle = targetAngle;
    this->targetSpeed = targetSpeed;
}

void CrtModule::update(float dt)
{
    /* ====================== 0. 基本保护 ====================== */
    if (m_steerMotor == nullptr || m_driveMotor == nullptr)
    {
        return;
    }

    /* ====================== 1. 读取舵向原始角度（0 ~ 2π） ====================== */
    /* GSRL: getCurrentAngle() 返回 rad 且范围 [0, 2PI) */
    float rawAngle = m_steerMotor->getCurrentAngle();

    /* ====================== 2. 构建连续舵向角 ====================== */
    float deltaAngle = rawAngle - lastRawAngle;

    /* 处理跨越 0 / 2π 边界产生的角度跳变 */
    if (deltaAngle > M_PI)
    {
        deltaAngle -= 2.0f * M_PI;
    }
    else if (deltaAngle < -M_PI)
    {
        deltaAngle += 2.0f * M_PI;
    }

    angleAccumulated += deltaAngle;
    currentAngle = angleAccumulated;
    lastRawAngle = rawAngle;

    /* ====================== 3. 最短转向与方向优化 ====================== */
    /* 先将误差压到 (-π, π] */
    float angleError = wrapToPi(targetAngle - currentAngle);

    /* 默认驱动目标不反向 */
    float driveSpeedCmd = targetSpeed;

    /* 若误差超过 ±90°，执行 π 翻转并反向驱动轮方向 */
    if (angleError > (M_PI * 0.5f))
    {
        angleError -= M_PI;
        driveSpeedCmd = -driveSpeedCmd;
    }
    else if (angleError < -(M_PI * 0.5f))
    {
        angleError += M_PI;
        driveSpeedCmd = -driveSpeedCmd;
    }

    /* ====================== 4. 将“最短误差”转回“电机目标角” ====================== */
    /* 电机接口 setTargetAngle() 期望目标角为 [0, 2π)，所以这里构造一个“就近目标角” */
    float optimizedTargetAngleContinuous = currentAngle + angleError;
    float optimizedTargetAngleWrapped = wrapTo0To2Pi(optimizedTargetAngleContinuous);

    /* ====================== 5. 舵向角度闭环（GM6020） ====================== */
    m_steerMotor->setTargetAngle(optimizedTargetAngleWrapped);
    (void)m_steerMotor->angleClosedloopControl(); /* 生成电机控制数据 */

    /* ====================== 6. 驱动速度闭环（M3508） ====================== */
    /* GSRL: getCurrentAngularVelocity() 为 rad/s，建议 targetSpeed 同口径 */
    m_driveMotor->setTargetAngularVelocity(driveSpeedCmd);
    (void)m_driveMotor->angularVelocityClosedloopControl(); /* 生成电机控制数据 */

    /* ====================== 7. 说明 ====================== */
    /* 本模块不发送 CAN，只更新电机对象的控制数据；发送由既有的 CAN/Task 统一完成 */
}

float CrtModule::wrapToPi(float angle)
{
    while (angle > M_PI)
    {
        angle -= 2.0f * M_PI;
    }
    while (angle <= -M_PI)
    {
        angle += 2.0f * M_PI;
    }
    return angle;
}

float CrtModule::wrapTo0To2Pi(float angle)
{
    /* 将角度映射到 [0, 2π) */
    float twoPi = 2.0f * (float)M_PI;
    float wrapped = std::fmod(angle, twoPi);
    if (wrapped < 0.0f)
    {
        wrapped += twoPi;
    }
    return wrapped;
}
