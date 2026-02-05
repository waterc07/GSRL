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
#include "crt_module.hpp"
#include "para_chassis.hpp"
#include "alg_pid.hpp"
#include <vector>

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* --- 调试用全局变量：用于实时监看底盘速度指令 --- */
float debugVx = 0.0f; // 底盘横向速度 (m/s)
float debugVy = 0.0f; // 底盘前后速度 (m/s)
float debugWz = 0.0f; // 底盘旋转角速度 (rad/s)

/* --- 舵向：每轮串级 PID 参数复制（因为构造函数要非 const 引用） --- */
static SimplePID::PIDParam steerOuterParamFl = steerAngleOuterPidParam;
static SimplePID::PIDParam steerInnerParamFl = steerSpeedInnerPidParam;
static CascadePID steerPidFl(steerOuterParamFl, steerInnerParamFl);

static SimplePID::PIDParam steerOuterParamBl = steerAngleOuterPidParam;
static SimplePID::PIDParam steerInnerParamBl = steerSpeedInnerPidParam;
static CascadePID steerPidBl(steerOuterParamBl, steerInnerParamBl);

static SimplePID::PIDParam steerOuterParamBr = steerAngleOuterPidParam;
static SimplePID::PIDParam steerInnerParamBr = steerSpeedInnerPidParam;
static CascadePID steerPidBr(steerOuterParamBr, steerInnerParamBr);

static SimplePID::PIDParam steerOuterParamFr = steerAngleOuterPidParam;
static SimplePID::PIDParam steerInnerParamFr = steerSpeedInnerPidParam;
static CascadePID steerPidFr(steerOuterParamFr, steerInnerParamFr);

/* --- 舵向电机：controller 换成 CascadePID --- */
static SimplePID::PIDParam drivePidParamFl = drivePidParam;
static SimplePID::PIDParam drivePidParamBl = drivePidParam;
static SimplePID::PIDParam drivePidParamBr = drivePidParam;
static SimplePID::PIDParam drivePidParamFr = drivePidParam;

static SimplePID drivePidFl((SimplePID::PIDMode)0, drivePidParamFl, nullptr);
static SimplePID drivePidBl((SimplePID::PIDMode)0, drivePidParamBl, nullptr);
static SimplePID drivePidBr((SimplePID::PIDMode)0, drivePidParamBr, nullptr);
static SimplePID drivePidFr((SimplePID::PIDMode)0, drivePidParamFr, nullptr);

/* 舵向零点：只用 encoderOffset*/
static MotorGM6020 steerMotorFl(1, &steerPidFl, 6442);
static MotorGM6020 steerMotorBl(2, &steerPidBl, 4520);
static MotorGM6020 steerMotorBr(3, &steerPidBr, 2418);
static MotorGM6020 steerMotorFr(4, &steerPidFr, 2360);

static MotorM3508 driveMotorFl(1, &drivePidFl, 0, 14.882353f);
static MotorM3508 driveMotorBl(2, &drivePidBl, 0, 14.882353f);
static MotorM3508 driveMotorBr(3, &drivePidBr, 0, 14.882353f);
static MotorM3508 driveMotorFr(4, &drivePidFr, 0, 14.882353f);

static CrtModule moduleFl(&steerMotorFl, &driveMotorFl);
static CrtModule moduleBl(&steerMotorBl, &driveMotorBl);
static CrtModule moduleBr(&steerMotorBr, &driveMotorBr);
static CrtModule moduleFr(&steerMotorFr, &driveMotorFr);

/* mModules 顺序固定：0=FL, 1=BL, 2=BR, 3=FR */
static std::vector<CrtChassis::Config> chassisModules = {
    {&moduleFl, +halfWheelBase, +halfTrackWidth, wheelRadius},
    {&moduleBl, -halfWheelBase, +halfTrackWidth, wheelRadius},
    {&moduleBr, -halfWheelBase, -halfTrackWidth, wheelRadius},
    {&moduleFr, +halfWheelBase, -halfTrackWidth, wheelRadius}};

/* IMU 暂时 nullptr：你想保留 controlLoop，后续需要小陀螺再接 IMU 指针 */
CrtChassis chassis(chassisModules, nullptr);
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
      mRemoteControl(0.05f), // 摇杆死区：统一交给 Dr16RemoteControl 处理
      mIsInitComplete(false)
{
}

void CrtChassis::init()
{
    DWT_Init();

    CAN_Init(&hcan1, can1RxCallback);
    CAN_Init(&hcan2, can2RxCallback);

    UART_Init(&huart3, dr16RxCallback, 36);

    if (mImu != nullptr) {
        mImu->init();
    }

    mVx          = 0.0f;
    mVy          = 0.0f;
    mWz          = 0.0f;
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

void CrtChassis::receiveChassisDriveMotorDataFromISR(const can_rx_message_t *rxMessage)
{
    if (rxMessage == nullptr) return;

    // CAN1: 处理驱动电机 (M3508)
    for (auto &cfg : mModules) {
        if (cfg.module == nullptr) continue;

        MotorM3508 *driveMotor = cfg.module->getDriveMotor();
        if (driveMotor != nullptr) {
            if (driveMotor->decodeCanRxMessageFromISR(rxMessage)) return;
        }
    }
}

void CrtChassis::receiveChassisSteerMotorDataFromISR(const can_rx_message_t *rxMessage)
{
    if (rxMessage == nullptr) return;

    // CAN2: 处理舵向电机 (GM6020)
    for (auto &cfg : mModules) {
        if (cfg.module == nullptr) continue;

        MotorGM6020 *steerMotor = cfg.module->getSteerMotor();
        if (steerMotor != nullptr) {
            if (steerMotor->decodeCanRxMessageFromISR(rxMessage)) return;
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
           速度定义: mVx=横向(X), mVy=前后(Y), mWz=旋转角速度
           wheelV = [vx, vy] + wz x r => [vx + wz*x, vy - wz*y]
           注意：mWz 需要乘以 rotationGain 来匹配平移速度量级 */
        const float wheelVx = mVx + mWz * rotationGain * cfg.wheelX; // X分量：横向速度
        const float wheelVy = mVy - mWz * rotationGain * cfg.wheelY; // Y分量：前后速度

        const float linearSpeed = std::sqrt(wheelVx * wheelVx + wheelVy * wheelVy); // m/s

        /* 当速度接近0时，保持当前舵向角度，避免回到原点 */
        constexpr float SPEED_THRESHOLD = 0.001f; // 速度阈值 (m/s)

        if (linearSpeed < SPEED_THRESHOLD) {
            /* 速度为0：保持当前实际角度，速度设为0 */
            MotorGM6020 *steerMotor = cfg.module->getSteerMotor();
            float currentAngle      = steerMotor ? steerMotor->getCurrentAngle() : 0.0f;
            cfg.module->setTarget(currentAngle, 0.0f);
            cfg.module->update(dt);
            continue;
        }

        const float targetAngle = std::atan2(wheelVy, wheelVx) - (float)M_PI / 2.0f;
        const float targetSpeed = linearSpeed / cfg.wheelRadius; // rad/s

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
        const fp32 leftX = mRemoteControl.getLeftStickX();
        // const fp32 leftY  = mRemoteControl.getLeftStickY();
        const fp32 rightX = mRemoteControl.getRightStickX();
        const fp32 rightY = mRemoteControl.getRightStickY();

        /* 约定：左摇杆平移，右摇杆X旋转（坐标系符号后续统一） */
        mVx = -rightX; // 右摇杆X → 横向
        mVy = rightY;  // 右摇杆Y → 前后
        mWz = -leftX;  // 左摇杆X
        return;
    }

    /* AutoControl：预留（后续视觉/导航接管 setCmd 或在此写入 mVx/mVy/mWz） */
}

void CrtChassis::motorControl()
{
    constexpr float controlDt = 0.001f;

    if (mChassisMode == ChassisNoForce) {
        for (auto &cfg : mModules) {
            if (cfg.module == nullptr) continue;
            if (auto *s = cfg.module->getSteerMotor()) s->openloopControl(0.0f);
            if (auto *d = cfg.module->getDriveMotor()) d->openloopControl(0.0f);
        }
        return;
    }

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

    /* CAN 分配：
       - CAN1: 驱动电机 M3508 (4个，ID 1-4，TX ID 0x200)
       - CAN2: 舵向电机 GM6020 (4个，ID 1-4，TX ID 0x1FF)
       装车后若 CAN口/控制ID 分配变化，仅需在此函数拆分/调整发送，不影响上层控制链路。 */

    // CAN2: 舵向电机
    if (steerFL && steerBL && steerBR && steerFR) {
        HAL_CAN_AddTxMessage(&hcan2,
                             steerFL->getMotorControlHeader(),
                             (*steerFL + *steerBL + *steerBR + *steerFR).getMotorControlData(),
                             NULL);
    }

    // CAN1: 驱动电机
    if (driveFL && driveBL && driveBR && driveFR) {
        HAL_CAN_AddTxMessage(&hcan1,
                             driveFL->getMotorControlHeader(),
                             (*driveFL + *driveBL + *driveBR + *driveFR).getMotorControlData(),
                             NULL);
    }
}
