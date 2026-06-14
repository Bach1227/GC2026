#include "bsp_zdt.h"
#include "bsp_can.h"

static FDCAN_HandleTypeDef   *zdt_hfdcan   = &hfdcan1;
static ZDT_MotorStatus_t      zdt_motors[ZDT_MAX_MOTORS];
static uint8_t                zdt_motor_cnt = 0;

/* ---- helpers ---- */

static HAL_StatusTypeDef ZDT_SendFrame(uint8_t addr, uint8_t pkt,
                                        uint8_t *data, uint32_t len)
{
    if (zdt_hfdcan == NULL) return HAL_ERROR;
    return CAN_Transmit_EXT(zdt_hfdcan, ZDT_EXT_ID(addr, pkt), data, len);
}

static int ZDT_FindSlot(uint8_t addr)
{
    for (int i = 0; i < ZDT_MAX_MOTORS; i++)
    {
        if (zdt_motors[i].addr == addr) return i;
    }
    if (zdt_motor_cnt < ZDT_MAX_MOTORS)
    {
        int slot = zdt_motor_cnt++;
        memset(&zdt_motors[slot], 0, sizeof(ZDT_MotorStatus_t));
        zdt_motors[slot].addr = addr;
        return slot;
    }
    return -1;
}

/* ---- init ---- */

void ZDT_Init(FDCAN_HandleTypeDef *hfdcan)
{
    zdt_hfdcan = hfdcan;
    zdt_motor_cnt = 0;
    memset(zdt_motors, 0, sizeof(zdt_motors));
}

ZDT_MotorStatus_t* ZDT_GetStatus(uint8_t addr)
{
    int slot = ZDT_FindSlot(addr);
    return (slot >= 0) ? &zdt_motors[slot] : NULL;
}

void ZDT_OnRxMessage(uint32_t ext_id, const uint8_t *data, uint8_t dlc)
{
    if (dlc < 2) return;
    uint8_t addr = data[0];
    uint8_t func = data[1];

    ZDT_MotorStatus_t *st = ZDT_GetStatus(addr);
    if (st == NULL) return;

    st->last_rx_tick = HAL_GetTick();

    switch (func) {
    case ZDT_FC_READ_POS:
        if (dlc >= 7) {
            st->position = ((int32_t)data[2] << 24) |
                           ((int32_t)data[3] << 16) |
                           ((int32_t)data[4] << 8)  |
                            (int32_t)data[5];
        }
        break;

    case ZDT_FC_READ_STATUS:
        if (dlc >= 3) {
            st->status_flags = data[2];
        }
        break;

    default:
        break;
    }
}

/* ---- commands ---- */
/* 协议格式: data[0]=addr, data[1]=func, ...params..., 最后字节=checksum(0x6B) */

HAL_StatusTypeDef ZDT_Enable(uint8_t addr)
{
    uint8_t buf[] = { addr, ZDT_FC_ENABLE_CTL, ZDT_SUBCODE_ENABLE, 0x01, 0x00 };
    buf[4] = ZDT_CHECKSUM;
    return ZDT_SendFrame(addr, 0, buf, 5);
}

HAL_StatusTypeDef ZDT_Disable(uint8_t addr)
{
    uint8_t buf[] = { addr, ZDT_FC_ENABLE_CTL, ZDT_SUBCODE_ENABLE, 0x00, 0x00 };
    buf[4] = ZDT_CHECKSUM;
    return ZDT_SendFrame(addr, 0, buf, 5);
}

HAL_StatusTypeDef ZDT_Stop(uint8_t addr)
{
    uint8_t buf[] = { addr, ZDT_FC_STOP, ZDT_SUBCODE_STOP, 0x00 };
    buf[3] = ZDT_CHECKSUM;
    return ZDT_SendFrame(addr, 0, buf, 4);
}

HAL_StatusTypeDef ZDT_SyncTrigger(void)
{
    uint8_t buf[] = { 0x00, ZDT_FC_SYNC, 0x66, 0x00 };
    buf[3] = ZDT_CHECKSUM;
    return ZDT_SendFrame(ZDT_ADDR_BROADCAST, 0, buf, 4);
}

HAL_StatusTypeDef ZDT_ReadPosition(uint8_t addr)
{
    uint8_t buf[] = { addr, ZDT_FC_READ_POS, 0x00 };
    buf[2] = ZDT_CHECKSUM;
    return ZDT_SendFrame(addr, 0, buf, 3);
}

HAL_StatusTypeDef ZDT_ReadStatus(uint8_t addr)
{
    uint8_t buf[] = { addr, ZDT_FC_READ_STATUS, 0x00 };
    buf[2] = ZDT_CHECKSUM;
    return ZDT_SendFrame(addr, 0, buf, 3);
}

HAL_StatusTypeDef ZDT_SetVelocity(uint8_t addr, uint8_t dir,
                                   uint16_t speed_rpm, uint8_t accel,
                                   uint8_t sync_flag)
{
    uint8_t buf[8] = {
        addr, ZDT_FC_VELOCITY, dir,
        (uint8_t)(speed_rpm >> 8),
        (uint8_t)(speed_rpm & 0xFF),
        accel, sync_flag,
    };
    buf[7] = ZDT_CHECKSUM;
    return ZDT_SendFrame(addr, 0, buf, 8);
}

HAL_StatusTypeDef ZDT_SetPosition(uint8_t addr, uint8_t dir,
                                   uint16_t speed_rpm, uint8_t accel,
                                   int32_t pulses, uint8_t rel_abs,
                                   uint8_t sync_flag)
{
    uint8_t pkt0[8] = {
        addr, ZDT_FC_POSITION, dir,
        (uint8_t)(speed_rpm >> 8),
        (uint8_t)(speed_rpm & 0xFF),
        accel,
        (uint8_t)(pulses >> 24),
        (uint8_t)(pulses >> 16),
    };

    uint8_t pkt1[7] = {
        addr, ZDT_FC_POSITION,
        (uint8_t)(pulses >> 8),
        (uint8_t)(pulses & 0xFF),
        rel_abs, sync_flag,
    };
    pkt1[6] = ZDT_CHECKSUM;

    if (ZDT_SendFrame(addr, 0, pkt0, 8) != HAL_OK) return HAL_ERROR;
    return ZDT_SendFrame(addr, 1, pkt1, 7);
}
