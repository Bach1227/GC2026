#ifndef __BSP_ZDT_H
#define __BSP_ZDT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <string.h>

/* Address range */
#define ZDT_ADDR_MIN            1
#define ZDT_ADDR_MAX            255
#define ZDT_ADDR_BROADCAST      0

/* Max motors tracked */
#define ZDT_MAX_MOTORS          16

/* Function codes */
#define ZDT_FC_ENABLE_CTL       0xF3U
#define ZDT_FC_VELOCITY         0xF6U
#define ZDT_FC_POSITION         0xFDU
#define ZDT_FC_STOP             0xFEU
#define ZDT_FC_SYNC             0xFFU
#define ZDT_FC_READ_STATUS      0x33U
#define ZDT_FC_READ_POS         0x36U

/* Sub-codes */
#define ZDT_SUBCODE_ENABLE      0xABU
#define ZDT_SUBCODE_STOP        0x98U

/* Direction */
#define ZDT_DIR_CW              0x00U
#define ZDT_DIR_CCW             0x01U

/* Sync flag */
#define ZDT_SYNC_IMMEDIATE      0x00U
#define ZDT_SYNC_WAIT           0x01U

/* Position mode */
#define ZDT_POS_RELATIVE        0x00U
#define ZDT_POS_ABSOLUTE        0x01U

/* Default checksum (fixed 0x6B) */
#define ZDT_CHECKSUM            0x6BU

/* Build extended CAN ID: (addr << 8) | packet_index */
#define ZDT_EXT_ID(addr, pkt)   (((uint32_t)(addr) << 8) | (pkt))

/* Types */

typedef struct {
    uint8_t  addr;
    uint8_t  status_flags;
    int32_t  position;         /* 当前累积脉冲 (CAN 回调更新) */
    int32_t  prev_position;    /* 上一采样点脉冲 (ChassisControl 更新) */
    int16_t  velocity_rpm;     /* 当前转速 RPM (ChassisControl 更新) */
    uint32_t last_rx_tick;
} ZDT_MotorStatus_t;

/* API */

void ZDT_Init(FDCAN_HandleTypeDef *hfdcan);
ZDT_MotorStatus_t* ZDT_GetStatus(uint8_t addr);

/** CAN 中断收到 ZDT 帧时调用, 自动解析 position / status */
void ZDT_OnRxMessage(uint32_t ext_id, const uint8_t *data, uint8_t dlc);

HAL_StatusTypeDef ZDT_Enable(uint8_t addr);
HAL_StatusTypeDef ZDT_Disable(uint8_t addr);
HAL_StatusTypeDef ZDT_Stop(uint8_t addr);
HAL_StatusTypeDef ZDT_SyncTrigger(void);

HAL_StatusTypeDef ZDT_SetVelocity(uint8_t addr, uint8_t dir,
                                   uint16_t speed_rpm, uint8_t accel,
                                   uint8_t sync_flag);

HAL_StatusTypeDef ZDT_SetPosition(uint8_t addr, uint8_t dir,
                                   uint16_t speed_rpm, uint8_t accel,
                                   int32_t pulses, uint8_t rel_abs,
                                   uint8_t sync_flag);

HAL_StatusTypeDef ZDT_ReadPosition(uint8_t addr);
HAL_StatusTypeDef ZDT_ReadStatus(uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ZDT_H */
