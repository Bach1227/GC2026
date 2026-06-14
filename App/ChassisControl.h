/**
  ******************************************************************************
  * @file           : ChassisControl.h
  * @brief          : 底盘控制 —— ZDT 电机 ↔ 运动学 ↔ 全局位置的中间层
  *
  *  依赖: MoveControl (运动学), bsp_zdt (电机驱动), protocol (命令输入)
  *
  *  坐标约定:
  *    X = 前, Y = 左, θ = CCW 为正
  *    轮编号: 1=前(+X), 2=左(+Y), 3=后(-X), 4=右(-Y)
  ******************************************************************************
  */

#ifndef __CHASSIS_CONTROL_H__
#define __CHASSIS_CONTROL_H__

#include <stdint.h>
#include <stdbool.h>
#include "MoveControl.h"
#include "protocol.h"
#include "bsp_zdt.h"  
#include "bsp_can.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================== */
/*  结构参数 (按实际修改)                                                   */
/* ====================================================================== */

#define WHEEL_RADIUS        0.05f    /* 轮半径 (m) */
#define PULSES_PER_REV      2000     /* 电机一圈的编码器脉冲数 (4x 后) */
#define WHEEL_BASE_RADIUS   0.2f     /* 轮心到底盘中心距离 (m) */
#define MOTOR_MAX_RPM       3000     /* 电机最大转速 */

/* ====================================================================== */
/*  Types                                                                 */
/* ====================================================================== */

/** 全局位置 */
typedef struct {
    float x;       /* 前向位置 (m) */
    float y;       /* 左向位置 (m) */
    float theta;   /* 朝向 (rad), CCW 为正 */
} Pose_t;

/** 电机地址映射: addr[4] = {前轮, 左轮, 后轮, 右轮} */
typedef struct {
    uint8_t motor[4];
} MotorMap_t;

/* ====================================================================== */
/*  API                                                                   */
/* ====================================================================== */

/** 初始化: 绑定电机地址, 使能全部电机, 创建控制任务 */
void Chassis_Init(const MotorMap_t *map);
void Chassis_InitTask(void);

/** 读取电机位置, 更新内部里程计 (定时器回调中周期调用) */
void Chassis_UpdateOdometry(void);

/** 速度开环: 底盘速度 → 四轮 RPM → ZDT */
void Chassis_SetVelocity(const ChassisSpeed_t *spd);

/** 位置闭环: 目标位姿 → P 控制输出速度 → ZDT */
void Chassis_MoveTo(const Pose_t *target);

/** 协议命令入口: CarMove → 目标位姿 (由 Protocol_Dispatch 直接调用) */
void Chassis_OnCarMove(const CarMove_t *cmd);

/** 控制任务: 周期更新里程计 + P 控朝向目标 */
void Chassis_Task(void *argument);

/** 急停 */
void Chassis_Stop(void);

/** 使能/失能全部电机 */
void Chassis_Enable(bool en);

/** 获取上次读取的四轮脉冲数 (原始值) */
void Chassis_GetPulses(int32_t pulses[4]);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_CONTROL_H__ */
