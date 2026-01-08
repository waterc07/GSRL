/**
 *******************************************************************************
 * @file           : crt_chassis.hpp
 * @brief          : 舵轮底盘控制器接口定义
 *******************************************************************************
 * @attention
 *
 * 本文件定义舵轮底盘（Chassis）的对外控制接口，用于接收底盘速度指令
 * （vx, vy, wz），并统一管理多个舵轮模块的运动学分配与控制更新。
 *
 * 该文件仅包含接口与类声明，不包含任何任务调度或硬件相关实现，
 * 由 Task 层周期性调用。
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 *******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include <vector>
#include "crt_module.hpp"     // CrtModule
// #include "para_chassis.hpp" // 如果你这里暂时没用到参数文件，可以先不 include

class CrtChassis
{
public:
    struct Config
    {
        CrtModule *module; // 舵轮模块实例（外部管理生命周期）
        float wheelX;      // 安装位置 X (m)
        float wheelY;      // 安装位置 Y (m)
        float wheelRadius; // 轮子半径 (m)
    };

    explicit CrtChassis(const std::vector<Config> &modules);

    void init();
    void setCmd(float vx, float vy, float wz);
    void update(float dt);

private:
    std::vector<Config> m_modules;

    float m_vx{0.0f};
    float m_vy{0.0f};
    float m_wz{0.0f};

    bool m_isInitComplete{false};
};
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
