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
 * - mModules 顺序固定为：0=FL, 1=BL, 2=BR, 3=FR
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
#include "crt_module.hpp"
#include "drv_can.h"
#include "drv_uart.h"
#include "dvc_remotecontrol.hpp"
#include "dvc_imu.hpp"

/* Exported types ------------------------------------------------------------*/

class CrtChassis
{
public:
    /* 底盘控制模式 */
    enum ChassisMode {
        ChassisNoForce = 0, // 无力/急停
        ManualControl,      // 手动遥控
        AutoControl         // 自动/视觉控制（预留）
    };

    /* 舵轮模块配置（mModules 顺序固定：0=FL, 1=BL, 2=BR, 3=FR） */
    struct Config {
        CrtModule *module; // 舵轮模块实例（外部管理生命周期）
        float wheelX;      // 安装位置 X (m)
        float wheelY;      // 安装位置 Y (m)
        float wheelRadius; // 轮子半径 (m)
    };

public:
    /**
     * @brief 构造函数
     * @param modules 舵轮模块列表（顺序固定：0=FL, 1=BL, 2=BR, 3=FR）
     * @param imu IMU 指针（可选：后续小陀螺/底盘跟随使用；当前非刚需可传 nullptr）
     */
    explicit CrtChassis(const std::vector<Config> &modules, IMU *imu = nullptr);

    /**
     * @brief 初始化（对齐战队通用控制器风格）
     * @note  内部会初始化 DWT、CAN、UART；若传入 IMU 指针则初始化 IMU
     */
    void init();

    /**
     * @brief 底盘控制主循环（Task 周期调用）
     * @note  内部执行：modeSelect -> targetSpeedPlan -> motorControl -> transmitChassisMotorData
     */
    void controlLoop();

    /**
     * @brief IMU 高频更新入口（可选：Task 高频调用）
     */
    void imuLoop();

    /**
     * @brief ISR：接收底盘驱动电机 CAN 数据入口（由 CAN1 回调转发）
     * @note  处理 4 个 M3508 驱动电机的反馈
     */
    void receiveChassisDriveMotorDataFromISR(const can_rx_message_t *rxMessage);

    /**
     * @brief ISR：接收底盘舵向电机 CAN 数据入口（由 CAN2 回调转发）
     * @note  处理 4 个 GM6020 舵向电机的反馈
     */
    void receiveChassisSteerMotorDataFromISR(const can_rx_message_t *rxMessage);

    /**
     * @brief ISR：接收遥控器数据入口（由 dr16RxCallback 转发）
     */
    void receiveRemoteControlDataFromISR(const uint8_t *rxData);

    /**
     * @brief 设置底盘速度指令（算法/上位机/自动控制可调用）
     */
    void setCmd(float vx, float vy, float wz);

    /**
     * @brief 仅执行运动学分配 + module 更新（保留给需要“算法接口”的调用方式）
     * @param dt 控制周期（s）
     */
    void update(float dt);

private:
    /* Variables -------------------------------------------------------------*/
    std::vector<Config> mModules;

    ChassisMode mChassisMode;

    float mVx; // 前向速度 (m/s)，向前为正
    float mVy; // 横向速度 (m/s)，向左为正
    float mWz; // 旋转角速度 (rad/s)，顺时针为正

    IMU *mImu;
    GSRLMath::Vector3f mEulerAngle;

    Dr16RemoteControl mRemoteControl;

    bool mIsInitComplete;

private:
    /* Function prototypes ---------------------------------------------------*/
    void modeSelect();
    void targetSpeedPlan();
    void motorControl();
    void transmitChassisMotorData();
};

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
/* Defines -------------------------------------------------------------------*/
