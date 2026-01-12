/**
 *******************************************************************************
 * @file           : crt_module.hpp
 * @brief          : 单舵轮模块接口定义
 *******************************************************************************
 * @attention
 *
 * 本文件定义单个舵轮模块（Module）的控制接口。
 * 一个舵轮模块通常包含：
 * - 转向电机（Steer）
 * - 驱动电机（Drive）
 *
 * 该模块负责根据目标角度与目标速度完成舵轮自身的控制更新，
 * 不感知整车运动学，仅接受底盘分配后的目标量。
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 *******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include <cstdint>
#include "dvc_motor.hpp"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/

/**
 * @brief 单个舵轮模块类
 * @note  仅负责舵轮自身的执行层控制，不包含整车运动学与任务调度
 */
class CrtModule
{
public:
    CrtModule(MotorGM6020 *steerMotor, MotorM3508 *driveMotor);

    void setTarget(float targetAngle, float targetSpeed);
    void update(float dt);

    MotorGM6020* getSteerMotor() const { return m_steerMotor; }
    MotorM3508*  getDriveMotor() const { return m_driveMotor; }

private:
    /* -------- 电机对象 -------- */
    MotorGM6020 *m_steerMotor;   // 转向电机（GM6020）
    MotorM3508  *m_driveMotor;   // 驱动电机（M3508）

    /* -------- 目标指令 -------- */
    float mTargetAngle;          // 目标舵向角（rad，连续）
    float mTargetSpeed;          // 目标驱动角速度（rad/s）

    /* -------- 角度状态 -------- */
    float mCurrentAngle;         // 当前连续舵向角（rad）
    float mLastRawAngle;         // 上一次原始角度（rad，0~2π）
    float mAngleAccumulated;     // 连续角度累计量（rad）

private:
    /* -------- 工具函数 -------- */
    float wrapToPi(float angle);
    float wrapTo0To2Pi(float angle);
};
