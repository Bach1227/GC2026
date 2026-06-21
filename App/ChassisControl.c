#include "ChassisControl.h"
#include "bsp_zdt.h"
#include "cmsis_os2.h"
#include <math.h>

/* ====================================================================== */
/*  常量                                                                  */
/* ====================================================================== */

#define WHEEL_RADIUS     0.075f
#define PULSES_PER_REV   2000
#define WHEEL_BASE       0.2f
#define POS_SPEED_RPM    200
#define POS_ACCEL        128

/* 脉冲 / 米: PPR / (2πR) */
#define PULSE_PER_M      ((float)PULSES_PER_REV / (2.0f * 3.14159265f * WHEEL_RADIUS))

/* ====================================================================== */
/*  工具: 轮位移(m) → ZDT_SetPosition                                    */
/* ====================================================================== */

static void wheel_position(uint8_t id, float disp_m)
{
    int32_t pulses = (int32_t)(disp_m * PULSE_PER_M);
    if (pulses == 0) return;

    uint8_t dir  = (pulses > 0) ? ZDT_DIR_CW : ZDT_DIR_CCW;
    uint16_t rpm = (uint16_t)((pulses > 0) ? pulses : -pulses);
    if (rpm > POS_SPEED_RPM) rpm = POS_SPEED_RPM;

    if (pulses < 0) pulses = -pulses;

    ZDT_SetPosition(id, dir, rpm, POS_ACCEL, pulses,
                    ZDT_POS_RELATIVE, ZDT_SYNC_WAIT);
}

/* ====================================================================== */
/*  Chassis_InitTask                                                      */
/* ====================================================================== */

void Chassis_InitTask(void)
{
    const osThreadAttr_t attr = {
        .name       = "chassis",
        .stack_size = 256 * 4,
        .priority   = osPriorityNormal,
    };
    osThreadNew(Chassis_Task, NULL, &attr);
}

/* ====================================================================== */
/*  Chassis_OnCarMove                                                      */
/*  上位机方向+距离 → 车体位移 → 逆运动学 → 四轮脉冲 → ZDT_SetPosition    */
/* ====================================================================== */

void Chassis_OnCarMove(const CarMove_t *cmd)
{
    if (cmd == NULL) return;

    /* 方向角(rad) → 车体位移 */
    float dx = cosf(cmd->direction) * cmd->distance;
    float dy = sinf(cmd->direction) * cmd->distance;
    float dt = cmd->direction;          /* 旋转角 = 朝向 */

    /* 逆运动学: 车体位移 → 四轮线位移 */
    float rw  = WHEEL_BASE * dt;
    float s1  = dy + rw;                /* 前轮 */
    float s2  = dx - rw;                /* 左轮 */
    float s3  = dy - rw;                /* 后轮 */
    float s4  = dx + rw;                /* 右轮 */

    /* 四轮定位 */
    wheel_position(1, s1);
    osDelay(1);
    wheel_position(2, s2);
    osDelay(1);
    wheel_position(3, s3);
    osDelay(1);
    wheel_position(4, s4);
    osDelay(1);
    
    ZDT_SyncTrigger();
    osDelay(1);
}

/* ====================================================================== */
/*  Chassis_Task                                                          */
/* ====================================================================== */

void Chassis_Task(void *argument)
{
    (void)argument;

    ZDT_Enable(1);
    ZDT_Enable(2);
    ZDT_Enable(3);
    ZDT_Enable(4);
    
    // int32_t pulses = (int32_t)(2 * PULSE_PER_M);
    // ZDT_SetPosition(2, ZDT_DIR_CW, 200, 128, pulses, ZDT_POS_RELATIVE, ZDT_SYNC_IMMEDIATE);
    CarMove_t cmd = {0, 10};
    Chassis_OnCarMove(&cmd);
    for (;;) {
        osDelay(10);
    }
}

/* ====================================================================== */
/*  Chassis_Stop / Chassis_Enable                                         */
/* ====================================================================== */

void Chassis_Stop(void)
{
    ZDT_Stop(1); ZDT_Stop(2); ZDT_Stop(3); ZDT_Stop(4);
}

void Chassis_Enable(bool en)
{
    if (en) {
        ZDT_Enable(1); ZDT_Enable(2); ZDT_Enable(3); ZDT_Enable(4);
    } else {
        ZDT_Disable(1); ZDT_Disable(2); ZDT_Disable(3); ZDT_Disable(4);
    }
}
