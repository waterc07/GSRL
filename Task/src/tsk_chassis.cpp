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
#include "FreeRTOS.h"
#include "task.h"
/* Typedef -------------------------------------------------------------------*/
/* Define --------------------------------------------------------------------*/
/* Macro ---------------------------------------------------------------------*/
/* Variables -----------------------------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/
/* User code -----------------------------------------------------------------*/



extern CrtChassis chassis;

extern "C" void chassis_task(void *argument)
{
    (void)argument;

    TickType_t taskLastWakeTime = xTaskGetTickCount();

    chassis.init();

    while (1)
    {
        chassis.controlLoop();
        vTaskDelayUntil(&taskLastWakeTime, pdMS_TO_TICKS(1));
    }
}