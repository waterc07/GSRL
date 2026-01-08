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
#include  "drv_misc.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/
CrtChassis::CrtChassis(const std::vector<Config> &modules)
    : m_modules(modules)
{
}

void CrtChassis::init()
{
    // 控制器层 init：只做状态初始化，不做 HAL/CAN/UART 初始化
    m_vx = 0.0f;
    m_vy = 0.0f;
    m_wz = 0.0f;
    m_isInitComplete = true;
}

void CrtChassis::setCmd(float vx, float vy, float wz)
{
    m_vx = vx;
    m_vy = vy;
    m_wz = wz;
}

void CrtChassis::update(float dt)
{
    (void)dt;
    if (!m_isInitComplete) return;

    for (auto &cfg : m_modules)
    {
        if (cfg.module == nullptr) continue;
        if (cfg.wheelRadius <= 1e-6f) continue;

        // Swerve kinematics (body frame):
        // wheelV = [vx, vy] + wz x r  => [vx - wz*y, vy + wz*x]
        const float wheelVx = m_vx - m_wz * cfg.wheelY;
        const float wheelVy = m_vy + m_wz * cfg.wheelX;

        const float targetAngle = std::atan2(wheelVy, wheelVx); // rad
        const float linearSpeed = std::sqrt(wheelVx * wheelVx + wheelVy * wheelVy); // m/s
        const float targetSpeed = linearSpeed / cfg.wheelRadius; // rad/s

        cfg.module->setTarget(targetAngle, targetSpeed);
        cfg.module->update(dt);
    }
}