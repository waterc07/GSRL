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

/* Exported constants --------------------------------------------------------*/

/* ====================== 底盘几何参数 ====================== */

/**
 * @brief 底盘前后轮距的一半
 * @note 单位：m
 */
static constexpr float halfWheelBase = 0.186975f;

/**
 * @brief 底盘左右轮距的一半
 * @note 单位：m
 */
static constexpr float halfTrackWidth = 0.186975f;

/**
 * @brief 轮子半径
 * @note 单位：m
 */
static constexpr float wheelRadius = 0.0525f;

/**
 * @brief 旋转速度增益系数
 * @note 用于放大自转角速度，使得自转速度与平移速度相匹配
 *       计算公式：1 / sqrt(halfWheelBase² + halfTrackWidth²)
 *       当前值：1 / sqrt(0.186975² + 0.186975²) ≈ 3.78
 */
static constexpr float rotationGain = 3.78f;

/* ====================== 舵向电机（GM6020）参数 ====================== */

static constexpr SimplePID::PIDParam steerAngleOuterPidParam = {
    .Kp             = 50.0f,
    .Ki             = 0.0f,
    .Kd             = 0.0f,
    .outputLimit    = 30.0f,
    .intergralLimit = 0.0f};

static constexpr SimplePID::PIDParam steerSpeedInnerPidParam = {
    .Kp             = 500.0f,
    .Ki             = 0.0f,
    .Kd             = 0.0f,
    .outputLimit    = 16000.0f,
    .intergralLimit = 3000.0f};

/* ====================== 驱动电机（M3508）参数 ====================== */

/**
 * @brief 驱动电机速度环 PID 参数
 * @note 单位：rad/s
 */
static constexpr SimplePID::PIDParam drivePidParam = {
    .Kp             = 500.0f,
    .Ki             = 0.0f,
    .Kd             = 0.0f,
    .outputLimit    = 16000.0f,
    .intergralLimit = 3000.0f};

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
