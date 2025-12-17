/**
 *******************************************************************************
 * @file           : para_chassis.hpp
 * @brief          : 舵轮底盘参数定义
 *******************************************************************************
 * @attention
 *
 * 本文件集中定义舵轮底盘相关的静态参数，包括但不限于：
 * - 轮距、轴距等几何参数
 * - 舵轮模块安装位置
 * - 电机与编码器相关配置
 * - 控制器参数（PID 等）
 *
 * 本文件仅用于参数配置，不包含任何控制逻辑代码。
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
#include "alg_pid.hpp"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 舵轮编号（逆时针）
 * @note 0=左前(FL), 1=左后(BL), 2=右后(BR), 3=右前(FR)
 */
enum class ChassisWheelID : uint8_t
{
    FrontLeft  = 0,   // FL
    BackLeft   = 1,   // BL
    BackRight  = 2,   // BR
    FrontRight = 3,   // FR
    WheelCount
};

/* Exported constants --------------------------------------------------------*/

/* ====================== 底盘几何参数 ====================== */

/**
 * @brief 底盘前后轮距的一半
 * @note 单位：m
 */
static constexpr float halfWheelBase = 0.10f;

/**
 * @brief 底盘左右轮距的一半
 * @note 单位：m
 */
static constexpr float halfTrackWidth = 0.10f;

/**
 * @brief 轮子半径
 * @note 单位：m
 */
static constexpr float wheelRadius = 0.03f;

/* ====================== 舵向电机（GM6020）参数 ====================== */

/**
 * @brief 舵向电机角度环 PID 参数
 * @note 单位：rad
 */
static constexpr SimplePID::PIDParam steerPidParam = {
    .Kp = 8.0f,
    .Ki = 0.0f,
    .Kd = 0.1f,
    .outputLimit    = 16000.0f,
    .intergralLimit = 3000.0f
};

/**
 * @brief 舵向电机机械零点偏移（GM6020）
 * @note 顺序严格对应编号 0~3（FL, BL, BR, FR）
 * @attention
 *   舵向零点仅通过 GM6020 的 encoderOffset 处理，
 *   Module / Chassis 层不得再进行角度补偿
 */
static constexpr uint16_t steerZeroOffset[4] = {
    0,  // [0] FL 左前
    0,  // [1] BL 左后
    0,  // [2] BR 右后
    0   // [3] FR 右前
};

/* ====================== 驱动电机（M3508）参数 ====================== */

/**
 * @brief 驱动电机速度环 PID 参数
 * @note 单位：rad/s
 */
static constexpr SimplePID::PIDParam drivePidParam = {
    .Kp = 10.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .outputLimit    = 16000.0f,
    .intergralLimit = 3000.0f
};

/**
 * @brief M3508 驱动电机减速比
 * @note 若使用 19:1 减速箱，则填写 19
 */
static constexpr uint8_t driveGearboxRatio = 19;

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
