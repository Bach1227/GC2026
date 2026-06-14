/**
  ******************************************************************************
  * @file           : MoveControl.h
  * @brief          : 十字形全向四轮底盘运动学解算
  *
  *  轮布局 (俯视, X=前, Y=左):
  *    轮1(前): +X 轴上, 驱动方向 ∥ Y  (向左为正)
  *    轮2(左): +Y 轴上, 驱动方向 ∥ X  (向前为正)
  *    轮3(后): -X 轴上, 驱动方向 ∥ Y  (向左为正)
  *    轮4(右): -Y 轴上, 驱动方向 ∥ X  (向前为正)
  ******************************************************************************
  */

#ifndef __MOVE_CONTROL_H__
#define __MOVE_CONTROL_H__

#include <stdint.h>
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================== */
/*  Types                                                                 */
/* ====================================================================== */

/** 底盘目标速度 (世界坐标系) */
typedef struct {
    float vx;    /* 前向速度 m/s */
    float vy;    /* 左向速度 m/s */
    float w;     /* 角速度 rad/s (CCW 为正) */
} ChassisSpeed_t;

/** 四轮目标线速度 */
typedef struct {
    float v1;    /* 前轮 */
    float v2;    /* 左轮 */
    float v3;    /* 后轮 */
    float v4;    /* 右轮 */
} WheelSpeed_t;

/* ====================================================================== */
/*  API                                                                   */
/* ====================================================================== */

/**
 * @brief 逆运动学: 底盘速度 → 四轮线速度
 */
void Kinematics_Inverse(const ChassisSpeed_t *in, WheelSpeed_t *out);

/**
 * @brief 正运动学: 四轮线速度 → 底盘速度 (里程计)
 */
void Kinematics_Forward(const WheelSpeed_t *in, ChassisSpeed_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __MOVE_CONTROL_H__ */
