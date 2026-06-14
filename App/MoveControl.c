/**
  ******************************************************************************
  * @file           : MoveControl.c
  * @brief          : 十字形全向四轮底盘运动学
  *
  *  逆运动学:
  *    V1 = vy + L·ω      V2 = vx - L·ω
  *    V3 = vy - L·ω      V4 = vx + L·ω
  *
  *  正运动学:
  *    vx = (V2 + V4) / 2
  *    vy = (V1 + V3) / 2
  *    ω  = ((V1 - V3) + (V4 - V2)) / (4·L)
  ******************************************************************************
  */

#include "MoveControl.h"

/* 轮心到底盘中心距离 (米), 可按实际结构修改 */
#define WHEEL_BASE_RADIUS  0.2f

/* ====================================================================== */
/*  逆运动学                                                              */
/* ====================================================================== */

void Kinematics_Inverse(const ChassisSpeed_t *in, WheelSpeed_t *out)
{
    if (in == NULL || out == NULL) return;

    float rw = WHEEL_BASE_RADIUS * in->w;

    out->v1 =  in->vy + rw;
    out->v2 =  in->vx - rw;
    out->v3 =  in->vy - rw;
    out->v4 =  in->vx + rw;
}

/* ====================================================================== */
/*  正运动学 (里程计)                                                     */
/* ====================================================================== */

void Kinematics_Forward(const WheelSpeed_t *in, ChassisSpeed_t *out)
{
    if (in == NULL || out == NULL) return;

    out->vx = (in->v2 + in->v4) * 0.5f;
    out->vy = (in->v1 + in->v3) * 0.5f;
    out->w  = ((in->v1 - in->v3) + (in->v4 - in->v2))
            / (4.0f * WHEEL_BASE_RADIUS);
}
