/**
  ******************************************************************************
  * @file           : ChassisControl.c
  * @brief          : 底盘控制 —— ZDT 电机 ↔ 运动学 ↔ 全局位姿
  *
  * ============================== 控制链路 ==============================
  *
  *   上位机 CarMove { direction, distance }
  *     │  direction = 目标方向角 (rad), distance = 目标位移量 (m)
  *     │
  *     ▼  [Protocol 回调]
  *   ┌────────────────────────┐
  *   │  命令 → 目标位姿        │
  *   │  target.x  = cos(direction) * distance
  *   │  target.y  = sin(direction) * distance
  *   │  target.θ  = direction   (车头指向目标方向)
  *   └────────┬───────────────┘
  *            │
  *     ┌──────▼──── 定时器回调 (50Hz~100Hz) ──────────────┐
  *     │                                                  │
  *     │  ① Chassis_UpdateOdometry(&pose)                 │
  *     │     读四轮脉冲 → 脉冲增量 Δpulse → 轮位移 Δs       │
  *     │     → Kinematics_Forward → (Δx, Δy, Δθ) → 累积   │
  *     │                                                  │
  *     │  ② Chassis_MoveTo(&target, &pose)                │
  *     │     P 控制: err = target - pose                  │
  *     │     v_cmd = Kp * err  (clamp to max speed)       │
  *     │     → Kinematics_Inverse → 四轮线速度 V1..V4      │
  *     │     → V→RPM → ZDT_SetVelocity(四轮, sync_wait)   │
  *     │     → ZDT_SyncTrigger()                          │
  *     │                                                   │
  *     │  ③ 终止判断: |err| < 阈值 → ZDT_Stop()             │
  *     │                                                   │
  *     └───────────────────────────────────────────────────┘
  *
  * ============================== 坐标约定 ==============================
  *
  *   底盘俯视:  轮1(前,+X): 驱动方向∥Y, 向左为正
  *              轮2(左,+Y): 驱动方向∥X, 向前为正
  *              轮3(后,-X): 驱动方向∥Y, 向左为正
  *              轮4(右,-Y): 驱动方向∥X, 向前为正
  *   旋转: CCW = +ω
  *
  * ============================== 关键换算 ==============================
  *
  *   V(m/s) → RPM:   RPM = V / (2π·Rwheel) × 60
  *   RPM   → V(m/s): V   = RPM × 2π·Rwheel / 60
  *   脉冲 → 轮位移:    Δs  = Δpulse / PPR × 2πRwheel
  *   轮位移 → 底盘:    Kinematics_Forward(Δs1..Δs4)
  *
  * ============================== ZDT 电机映射 ==============================
  *
  *   MotorMap.motor[0] = 前轮地址  (对应 V1)
  *   MotorMap.motor[1] = 左轮地址  (对应 V2)
  *   MotorMap.motor[2] = 后轮地址  (对应 V3)
  *   MotorMap.motor[3] = 右轮地址  (对应 V4)
  *
  ******************************************************************************
  */

#include "ChassisControl.h"
#include "bsp_zdt.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include <math.h>
#include <string.h>

/* ====================================================================== */
/*  常量                                                                  */
/* ====================================================================== */

#define V_TO_RPM          (60.0f / (2.0f * 3.14159265f * WHEEL_RADIUS))
#define PULSE_TO_M        ((2.0f * 3.14159265f * WHEEL_RADIUS) / (float)PULSES_PER_REV)

#define POS_KP            2.0f     /* 位置 P 增益 */
#define ANGLE_KP          1.5f     /* 朝向 P 增益 */
#define MAX_LIN_SPEED     1.0f     /* 最大线速度 m/s */
#define MAX_ANG_SPEED     3.0f     /* 最大角速度 rad/s */

#define CTRL_PERIOD_MS    10       /* 控制周期 ms */
#define STOP_THRESHOLD    0.02f    /* 到位阈值 m */
#define ANGLE_THRESHOLD   0.05f    /* 到位阈值 rad */

/* ====================================================================== */
/*  静态变量                                                              */
/* ====================================================================== */

static MotorMap_t  motor_map;
static Pose_t      current_pose;
static Pose_t      target_pose;
static bool        target_active = false;
static bool        initialized   = false;

/* ====================================================================== */
/*  工具                                                                  */
/* ====================================================================== */

static inline float clamp(float v, float limit)
{
    if (v >  limit) return  limit;
    if (v < -limit) return -limit;
    return v;
}

static inline uint16_t rpm_to_reg(float rpm)
{
    if (rpm < 0) return 0;
    if (rpm > (float)MOTOR_MAX_RPM) return (uint16_t)MOTOR_MAX_RPM;
    return (uint16_t)rpm;
}

/* m/s 线速度 → RPM, 带符号方向 */
static void speed_to_motor(float wheel_speed, uint8_t *dir, uint16_t *rpm)
{
    if (wheel_speed >= 0.0f) {
        *dir = ZDT_DIR_CW;
        *rpm = rpm_to_reg(wheel_speed * V_TO_RPM);
    } else {
        *dir = ZDT_DIR_CCW;
        *rpm = rpm_to_reg(-wheel_speed * V_TO_RPM);
    }
}

/* ====================================================================== */
/*  Chassis_Init                                                          */
/* ====================================================================== */

void Chassis_Init(const MotorMap_t *map)
{
    if (map == NULL) return;

    motor_map = *map;
    memset(&current_pose, 0, sizeof(current_pose));

    for (int i = 0; i < 4; i++) {
        ZDT_ReadPosition(motor_map.motor[i]);
    }

    for (int i = 0; i < 4; i++) {
        ZDT_MotorStatus_t *st = ZDT_GetStatus(motor_map.motor[i]);
        if (st) st->prev_position = st->position;
    }

    /* 使能全部电机 */
    for (int i = 0; i < 4; i++) {
        ZDT_Enable(motor_map.motor[i]);
    }

    initialized = true;
}

void Chassis_InitTask(void)
{
    const osThreadAttr_t attr = {
        .name       = "chassis",
        .stack_size = 512 * 4,
        .priority   = osPriorityAboveNormal,
    };
    osThreadNew(Chassis_Task, NULL, &attr);
}

/* ====================================================================== */
/*  Chassis_UpdateOdometry                                                */
/* ====================================================================== */

void Chassis_UpdateOdometry(void)
{
    if (!initialized) return;

    float ds[4];

    for (int i = 0; i < 4; i++) {
        ZDT_MotorStatus_t *st = ZDT_GetStatus(motor_map.motor[i]);
        if (st == NULL) { ds[i] = 0.0f; continue; }

        int32_t cur  = st->position;
        int32_t prev = st->prev_position;
        st->prev_position = cur;

        ds[i] = (float)(cur - prev) * PULSE_TO_M;
    }

    WheelSpeed_t wheel_delta = { ds[0], ds[1], ds[2], ds[3] };
    ChassisSpeed_t chassis_delta;
    Kinematics_Forward(&wheel_delta, &chassis_delta);

    current_pose.x     += chassis_delta.vx;
    current_pose.y     += chassis_delta.vy;
    current_pose.theta += chassis_delta.w;
}

/* ====================================================================== */
/*  Chassis_SetVelocity                                                   */
/* ====================================================================== */

void Chassis_SetVelocity(const ChassisSpeed_t *spd)
{
    if (!initialized || spd == NULL) return;

    /* 逆运动学 */
    WheelSpeed_t wheels;
    Kinematics_Inverse(spd, &wheels);

    float target[4] = { wheels.v1, wheels.v2, wheels.v3, wheels.v4 };

    /* 轮速 → 电机方向+RPM → ZDT */
    uint8_t  dir[4];
    uint16_t rpm[4];

    for (int i = 0; i < 4; i++) {
        speed_to_motor(target[i], &dir[i], &rpm[i]);
        ZDT_SetVelocity(motor_map.motor[i], dir[i], rpm[i], 50,
                        ZDT_SYNC_WAIT);

        /* 回填当前指令 RPM */
        ZDT_MotorStatus_t *st = ZDT_GetStatus(motor_map.motor[i]);
        if (st) {
            st->velocity_rpm = (dir[i] == ZDT_DIR_CW) ? (int16_t)rpm[i]
                                                       : -(int16_t)rpm[i];
        }
    }

    ZDT_SyncTrigger();
}

/* ====================================================================== */
/*  Chassis_MoveTo (位置 P 控制)                                          */
/* ====================================================================== */

void Chassis_MoveTo(const Pose_t *target)
{
    if (!initialized || target == NULL) return;

    float ex = target->x - current_pose.x;
    float ey = target->y - current_pose.y;
    float eθ = target->theta - current_pose.theta;

    /* P 控制 */
    ChassisSpeed_t cmd;
    cmd.vx = clamp(ex * POS_KP,   MAX_LIN_SPEED);
    cmd.vy = clamp(ey * POS_KP,   MAX_LIN_SPEED);
    cmd.w  = clamp(eθ * ANGLE_KP, MAX_ANG_SPEED);

    Chassis_SetVelocity(&cmd);
}

/* ====================================================================== */
/*  Chassis_OnCarMove                                                      */
/* ====================================================================== */

void Chassis_OnCarMove(const CarMove_t *cmd)
{
    if (cmd == NULL) return;

    target_pose.x     = cosf(cmd->direction) * cmd->distance;
    target_pose.y     = sinf(cmd->direction) * cmd->distance;
    target_pose.theta = cmd->direction;
    target_active     = true;
}

/* ====================================================================== */
/*  Chassis_Task (FreeRTOS 线程, 100Hz 闭环)                               */
/* ====================================================================== */

void Chassis_Task(void *argument)
{
    (void)argument;
    // Chassis_Init(&motor_map);
    ZDT_Enable(0x01);

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CTRL_PERIOD_MS));

        // if (!initialized) continue;

        // /* ① 更新里程计 */
        // Chassis_UpdateOdometry();

        // if (!target_active) continue;

        // /* ② 计算误差 */
        // float ex = target_pose.x     - current_pose.x;
        // float ey = target_pose.y     - current_pose.y;
        // float eθ = target_pose.theta - current_pose.theta;

        // /* ③ 到位判断 */
        // if (fabsf(ex) < STOP_THRESHOLD &&
        //     fabsf(ey) < STOP_THRESHOLD &&
        //     fabsf(eθ) < ANGLE_THRESHOLD) {
        //     Chassis_Stop();
        //     target_active = false;
        //     continue;
        // }

        // /* ④ P 控制 + 发速度 */
        // ChassisSpeed_t cmd;
        // cmd.vx = clamp(ex * POS_KP,   MAX_LIN_SPEED);
        // cmd.vy = clamp(ey * POS_KP,   MAX_LIN_SPEED);
        // cmd.w  = clamp(eθ * ANGLE_KP, MAX_ANG_SPEED);

        // Chassis_SetVelocity(&cmd);
        // uint8_t dir[4] = { 0x11, 0x22, 0x33, 0x44};
        // CAN_Transmit_STD(&hfdcan1, 0x100, dir, 4);
        ZDT_SetVelocity(0x01, ZDT_DIR_CW, 500, 10, ZDT_SYNC_IMMEDIATE);
        // ZDT_SetPosition(0x01, ZDT_DIR_CW, 500, 10,
                // 80000, ZDT_POS_RELATIVE, ZDT_SYNC_IMMEDIATE);
        // ZDT_SetPosition(0x02, ZDT_DIR_CW, 500, 10,
        //         80000, ZDT_POS_RELATIVE, ZDT_SYNC_IMMEDIATE);
        // ZDT_SetPosition(0x03, ZDT_DIR_CW, 500, 10,
        //         80000, ZDT_POS_RELATIVE, ZDT_SYNC_IMMEDIATE);
        // ZDT_SetPosition(0x04, ZDT_DIR_CW, 500, 10,
        //         80000, ZDT_POS_RELATIVE, ZDT_SYNC_IMMEDIATE);
        osDelay(2);
    }
}

/* ====================================================================== */
/*  Chassis_Stop / Chassis_Enable                                         */
/* ====================================================================== */

void Chassis_Stop(void)
{
    if (!initialized) return;

    for (int i = 0; i < 4; i++) {
        ZDT_Stop(motor_map.motor[i]);
    }
}

void Chassis_Enable(bool en)
{
    if (!initialized) return;

    for (int i = 0; i < 4; i++) {
        if (en) ZDT_Enable(motor_map.motor[i]);
        else    ZDT_Disable(motor_map.motor[i]);
    }
}

/* ====================================================================== */
/*  Chassis_GetPulses                                                     */
/* ====================================================================== */

void Chassis_GetPulses(int32_t pulses[4])
{
    if (pulses == NULL) return;

    for (int i = 0; i < 4; i++) {
        ZDT_MotorStatus_t *st = ZDT_GetStatus(motor_map.motor[i]);
        pulses[i] = st ? st->position : 0;
    }
}
