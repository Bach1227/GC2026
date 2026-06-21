#ifndef __CHASSIS_CONTROL_H__
#define __CHASSIS_CONTROL_H__

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"
#include "bsp_zdt.h"

#ifdef __cplusplus
extern "C" {
#endif

void Chassis_InitTask(void);

void Chassis_OnCarMove(const CarMove_t *cmd);

void Chassis_Task(void *argument);

void Chassis_Stop(void);

void Chassis_Enable(bool en);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_CONTROL_H__ */
