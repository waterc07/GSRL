/**
 ******************************************************************************
 * @file           : tsk_chassis.cpp
 * @brief          : 底盘控制任务（舵轮）
 ******************************************************************************
 * @attention
 *
 * 本文件完成底盘任务层的实例化与调度，包括：
 * - PID 控制器对象创建与参数绑定
 * - 8 个电机对象（4 舵向 GM6020 + 4 驱动 M3508）实例化
 * - 4 个舵轮模块 CrtModule 实例化
 * - CrtChassis 实例化
 * - FreeRTOS 周期调用 chassis.controlLoop()
 *
 * 注意：
 * - 底盘几何参数、CAN ID、CAN口分配、电机 DJI ID 以装车为准
 * - 当前默认：舵向 1~4，驱动 5~8（占位）
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "crt_chassis.hpp"
#include "crt_module.hpp"
#include "para_chassis.hpp"
#include "alg_pid.hpp"

#include "FreeRTOS.h"
#include "task.h"

#include <vector>

/* Typedef -------------------------------------------------------------------*/
/* Define --------------------------------------------------------------------*/
/* Macro ---------------------------------------------------------------------*/
/* Variables -----------------------------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/
/* User code -----------------------------------------------------------------*/

/******************************************************************************
 *                            电机与控制器对象
 ******************************************************************************/

/* ---------- 1) PID 参数拷贝（必须是非 const，才能传 PIDParam&） ---------- */
static SimplePID::PIDParam steerPidParamFl = steerPidParam;
static SimplePID::PIDParam steerPidParamBl = steerPidParam;
static SimplePID::PIDParam steerPidParamBr = steerPidParam;
static SimplePID::PIDParam steerPidParamFr = steerPidParam;

static SimplePID::PIDParam drivePidParamFl = drivePidParam;
static SimplePID::PIDParam drivePidParamBl = drivePidParam;
static SimplePID::PIDParam drivePidParamBr = drivePidParam;
static SimplePID::PIDParam drivePidParamFr = drivePidParam;

/* ---------- 2) PID 控制器对象 ----------
 * 说明：SimplePID 构造为 (PIDMode mode, PIDParam& param, Filter* filter)
 * 这里用 (SimplePID::PIDMode)0 来避免枚举名差异导致编译失败
 */
static SimplePID steerPidFl((SimplePID::PIDMode)0, steerPidParamFl, nullptr);
static SimplePID steerPidBl((SimplePID::PIDMode)0, steerPidParamBl, nullptr);
static SimplePID steerPidBr((SimplePID::PIDMode)0, steerPidParamBr, nullptr);
static SimplePID steerPidFr((SimplePID::PIDMode)0, steerPidParamFr, nullptr);

static SimplePID drivePidFl((SimplePID::PIDMode)0, drivePidParamFl, nullptr);
static SimplePID drivePidBl((SimplePID::PIDMode)0, drivePidParamBl, nullptr);
static SimplePID drivePidBr((SimplePID::PIDMode)0, drivePidParamBr, nullptr);
static SimplePID drivePidFr((SimplePID::PIDMode)0, drivePidParamFr, nullptr);

/* ---------- 3) DJI 电机 ID（占位：装车后修改） ---------- */
static constexpr uint8_t steerDjiIdFl = 1;
static constexpr uint8_t steerDjiIdBl = 2;
static constexpr uint8_t steerDjiIdBr = 3;
static constexpr uint8_t steerDjiIdFr = 4;

static constexpr uint8_t driveDjiIdFl = 5;
static constexpr uint8_t driveDjiIdBl = 6;
static constexpr uint8_t driveDjiIdBr = 7;
static constexpr uint8_t driveDjiIdFr = 8;

/* ---------- 4) 电机对象 ----------
 * - 舵向零点：仅通过 encoderOffset 生效（你前面已强调）
 * - 3508 gearboxRatio：dvc_motor.hpp 里注释写的是“减速比倒数”，所以传 1/ratio
 */
static constexpr fp32 driveGearboxRatioInv = 1.0f / (fp32)driveGearboxRatio;

static MotorGM6020 steerMotorFl(steerDjiIdFl, &steerPidFl, steerZeroOffset[0]);
static MotorGM6020 steerMotorBl(steerDjiIdBl, &steerPidBl, steerZeroOffset[1]);
static MotorGM6020 steerMotorBr(steerDjiIdBr, &steerPidBr, steerZeroOffset[2]);
static MotorGM6020 steerMotorFr(steerDjiIdFr, &steerPidFr, steerZeroOffset[3]);

static MotorM3508 driveMotorFl(driveDjiIdFl, &drivePidFl, 0, driveGearboxRatioInv);
static MotorM3508 driveMotorBl(driveDjiIdBl, &drivePidBl, 0, driveGearboxRatioInv);
static MotorM3508 driveMotorBr(driveDjiIdBr, &drivePidBr, 0, driveGearboxRatioInv);
static MotorM3508 driveMotorFr(driveDjiIdFr, &drivePidFr, 0, driveGearboxRatioInv);

/* ---------- 5) 舵轮模块对象 ---------- */
static CrtModule moduleFl(&steerMotorFl, &driveMotorFl);
static CrtModule moduleBl(&steerMotorBl, &driveMotorBl);
static CrtModule moduleBr(&steerMotorBr, &driveMotorBr);
static CrtModule moduleFr(&steerMotorFr, &driveMotorFr);

/******************************************************************************
 *                            底盘对象（全局供 ISR 引用）
 ******************************************************************************/

/* 坐标约定（占位，装车后可统一）：
 * - +X：车头方向（forward）
 * - +Y：车左方向（left）
 * - 原点：底盘几何中心
 * 模块顺序固定：0=FL, 1=BL, 2=BR, 3=FR
 */
static std::vector<CrtChassis::Config> chassisModules = {
    {&moduleFl, +halfWheelBase, +halfTrackWidth, wheelRadius}, // FL
    {&moduleBl, -halfWheelBase, +halfTrackWidth, wheelRadius}, // BL
    {&moduleBr, -halfWheelBase, -halfTrackWidth, wheelRadius}, // BR
    {&moduleFr, +halfWheelBase, -halfTrackWidth, wheelRadius}  // FR
};

/* IMU 当前不是刚需：先传 nullptr，后续小陀螺/底盘跟随再接入 */
CrtChassis chassis(chassisModules, nullptr);

/******************************************************************************
 *                            FreeRTOS 任务入口
 ******************************************************************************/

extern "C" void chassis_task(void *argument)
{
    (void)argument;

    TickType_t taskLastWakeTime = xTaskGetTickCount();

    chassis.init();

    while (1)
    {
        chassis.controlLoop();
        vTaskDelayUntil(&taskLastWakeTime, pdMS_TO_TICKS(1)); // 1ms 周期
    }
}
