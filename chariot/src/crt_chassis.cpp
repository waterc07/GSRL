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
 * 注意：
 * - mModules 顺序固定：0=FL, 1=BL, 2=BR, 3=FR
 * - 底盘几何参数、CAN ID、CAN口分配可后续调整；仅需修改发送拆分逻辑
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "crt_chassis.hpp"
#include <cmath>
#include "stm32f4xx_hal_can.h"
#include "tsk_isr.hpp"
#include "drv_misc.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

CrtChassis::CrtChassis(const std::vector<Config> &modules, IMU *imu)
    : mModules(modules),
      mChassisMode(ChassisNoForce),
      mVx(0.0f),
      mVy(0.0f),
      mWz(0.0f),
      mImu(imu),
      mEulerAngle{0.0f, 0.0f, 0.0f},
      mRemoteControl(0.05f),   // 摇杆死区：统一交给 Dr16RemoteControl 处理
      mIsInitComplete(false)
{
}

void CrtChassis::init()
{
    DWT_Init();

    CAN_Init(&hcan1, can1RxCallback);
    // CAN_Init(&hcan2, can2RxCallback);

    UART_Init(&huart3, dr16RxCallback, 36);

    if (mImu != nullptr) {
        mImu->init();
    }

    mVx = 0.0f;
    mVy = 0.0f;
    mWz = 0.0f;
    mChassisMode = ChassisNoForce;

    mIsInitComplete = true;
}

void CrtChassis::controlLoop()
{
    if (!mIsInitComplete) return;

    modeSelect();
    targetSpeedPlan();
    motorControl();
    transmitChassisMotorData();
}

void CrtChassis::imuLoop()
{
    if (!mIsInitComplete) return;
    if (mImu == nullptr) return;

    /* dvc_imu.cpp：solveAttitude 返回 IMU::Vector3f 引用 */
    mEulerAngle = mImu->solveAttitude();
}

void CrtChassis::receiveChassisMotorDataFromISR(const can_rx_message_t *rxMessage)
{
    if (rxMessage == nullptr) return;

    for (auto &cfg : mModules) {
        if (cfg.module == nullptr) continue;

        MotorGM6020 *steerMotor = cfg.module->getSteerMotor();
        MotorM3508  *driveMotor = cfg.module->getDriveMotor();

        if (steerMotor != nullptr) {
            if (steerMotor->decodeCanRxMessageFromISR(rxMessage)) return;
        }
        if (driveMotor != nullptr) {
            if (driveMotor->decodeCanRxMessageFromISR(rxMessage)) return;
        }
    }
}

void CrtChassis::receiveRemoteControlDataFromISR(const uint8_t *rxData)
{
    if (rxData == nullptr) return;
    mRemoteControl.receiveRxDataFromISR(rxData);
}

void CrtChassis::setCmd(float vx, float vy, float wz)
{
    mVx = vx;
    mVy = vy;
    mWz = wz;
}

void CrtChassis::update(float dt)
{
    if (!mIsInitComplete) return;

    for (auto &cfg : mModules) {
        if (cfg.module == nullptr) continue;
        if (cfg.wheelRadius <= 1e-6f) continue;

        /* Swerve kinematics (body frame):
           wheelV = [vx, vy] + wz x r => [vx - wz*y, vy + wz*x] */
        const float wheelVx = mVx - mWz * cfg.wheelY;
        const float wheelVy = mVy + mWz * cfg.wheelX;

        const float targetAngle = std::atan2(wheelVy, wheelVx);                     // rad
        const float linearSpeed = std::sqrt(wheelVx * wheelVx + wheelVy * wheelVy); // m/s
        const float targetSpeed = linearSpeed / cfg.wheelRadius;                    // rad/s

        cfg.module->setTarget(targetAngle, targetSpeed);
        cfg.module->update(dt);
    }
}

void CrtChassis::modeSelect()
{
    mRemoteControl.updateEvent();

    if (!mRemoteControl.isDr16RemoteControlConnected()) {
        mChassisMode = ChassisNoForce;
        return;
    }

    /* 先提供最基础三挡：后续小陀螺/底盘跟随在此扩展 */
    switch (mRemoteControl.getRightSwitchStatus()) {
        case Dr16RemoteControl::SWITCH_DOWN:
            mChassisMode = ChassisNoForce;
            break;

        case Dr16RemoteControl::SWITCH_MIDDLE:
            mChassisMode = ManualControl;
            break;

        case Dr16RemoteControl::SWITCH_UP:
            mChassisMode = AutoControl;
            break;

        default:
            break;
    }
}

void CrtChassis::targetSpeedPlan()
{
    if (mChassisMode == ChassisNoForce) {
        mVx = 0.0f;
        mVy = 0.0f;
        mWz = 0.0f;
        return;
    }

    if (mChassisMode == ManualControl) {
        /* 遥控器摇杆死区已由 Dr16RemoteControl::applyStickDeadZone 统一处理 */
        const fp32 leftX  = mRemoteControl.getLeftStickX();
        const fp32 leftY  = mRemoteControl.getLeftStickY();
        const fp32 rightX = mRemoteControl.getRightStickX();

        /* 约定：左摇杆平移，右摇杆X旋转（坐标系符号后续统一） */
        mVx = leftY;
        mVy = leftX;
        mWz = rightX;
        return;
    }

    /* AutoControl：预留（后续视觉/导航接管 setCmd 或在此写入 mVx/mVy/mWz） */
}

void CrtChassis::motorControl()
{
    /* dt：先用固定周期（与 Task 周期一致即可），后续可改为 DWT 计算 */
    constexpr float controlDt = 0.001f; // 1ms
    update(controlDt);
}

void CrtChassis::transmitChassisMotorData()
{
    /* mModules 顺序固定：0=FL, 1=BL, 2=BR, 3=FR */
    if (mModules.size() < 4) return;

    CrtModule *moduleFL = mModules[0].module;
    CrtModule *moduleBL = mModules[1].module;
    CrtModule *moduleBR = mModules[2].module;
    CrtModule *moduleFR = mModules[3].module;

    if (moduleFL == nullptr || moduleBL == nullptr || moduleBR == nullptr || moduleFR == nullptr) return;

    MotorGM6020 *steerFL = moduleFL->getSteerMotor();
    MotorGM6020 *steerBL = moduleBL->getSteerMotor();
    MotorGM6020 *steerBR = moduleBR->getSteerMotor();
    MotorGM6020 *steerFR = moduleFR->getSteerMotor();

    MotorM3508 *driveFL = moduleFL->getDriveMotor();
    MotorM3508 *driveBL = moduleBL->getDriveMotor();
    MotorM3508 *driveBR = moduleBR->getDriveMotor();
    MotorM3508 *driveFR = moduleFR->getDriveMotor();

    /* 默认：舵向4电机同一控制帧（常见0x1FF），驱动4电机同一控制帧（常见0x200），均在 CAN1
       装车后若 CAN口/控制ID 分配变化，仅需在此函数拆分/调整发送，不影响上层控制链路。 */

    if (steerFL && steerBL && steerBR && steerFR) {
        HAL_CAN_AddTxMessage(&hcan1,
                             steerFL->getMotorControlHeader(),
                             (*steerFL + *steerBL + *steerBR + *steerFR).getMotorControlData(),
                             NULL);
    }

    if (driveFL && driveBL && driveBR && driveFR) {
        HAL_CAN_AddTxMessage(&hcan1,
                             driveFL->getMotorControlHeader(),
                             (*driveFL + *driveBL + *driveBR + *driveFR).getMotorControlData(),
                             NULL);
    }
}
