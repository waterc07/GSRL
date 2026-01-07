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
#include "crt_module.hpp"
#include "para_chassis.hpp"
#include "dvc_imu.hpp"
#include "dvc_remotecontrol.hpp"
#include "gsrl_common.h"

/* Exported types ------------------------------------------------------------*/

class CrtChassis
{
public:
    // 底盘控制模式
    enum ChassisMode
    {
        CHASSIS_NO_FORCE, // 无力/急停
        MANUAL_CONTROL,   // 手动遥控
        AUTO_CONTROL      // 自动/视觉控制
    };

    struct Config
    {
        CrtModule *module; // 舵轮模块实例（需外部管理生命周期）
        float wheelX;      // 安装位置 X (m)
        float wheelY;      // 安装位置 Y (m)
        float wheelRadius; // 轮子半径 (m)，用于线速度转角速度
    };

    explicit CrtChassis(const std::vector<Config> &modules);

    CrtChassis(MotorGM6020* front_SteerLeftMotor, MotorGM6020* front_SteerRightMotor,
               MotorGM6020* back_SteerLeftMotor, MotorGM6020* back_SteerRightMotor,
               MotorM3508* front_WheelLeftMotor, MotorM3508* front_WheelRightMotor,
               MotorM3508* back_WheelLeftMotor, MotorM3508* back_WheelRightMotor,
               IMU* imu);
    
    CrtChassis() = default;

    void init();
    void setCmd(float vx, float vy, float wz);
    void controlLoop(); // Task loop
    void imuLoop();     // IMU update loop (high freq)
    void receiveChassisMotorDataFromISR(const can_rx_message_t *rxMessage);
    void receiveRemoteControlDataFromISR(const uint8_t *rxData);
    void update(float dt);

private:
    std::vector<Config> m_modules; // 内部持有的模块列表
    
    ChassisMode m_chassisMode;
    
    float m_vx; // 缓存的目标 Vx
    float m_vy; // 缓存的目标 Vy
    float m_wz; // 缓存的目标 Wz

    // 电机
    MotorGM6020 *m_front_SteerLeftMotor;
    MotorGM6020 *m_front_SteerRightMotor;
    MotorGM6020 *m_back_SteerLeftMotor;
    MotorGM6020 *m_back_SteerRightMotor;
    MotorM3508 *m_front_WheelLeftMotor;
    MotorM3508 *m_front_WheelRightMotor;
    MotorM3508 *m_back_WheelLeftMotor;
    MotorM3508 *m_back_WheelRightMotor;
    
    // IMU
    IMU *m_imu;
    EulerAngle m_eulerAngle;

    // 遥控器
    Dr16RemoteControl m_remoteControl;

    // 标志位
    bool m_isInitComplete;

private:
    void modeSelect();
    void targetSpeedplan();
    void motorControl();
    void transmitChassisMotorData();
    inline fp32 rcStickDeadZoneFilter(const fp32 &rcStickValue);
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
