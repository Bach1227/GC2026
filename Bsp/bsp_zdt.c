#include "bsp_zdt.h"
#include "bsp_can.h"

static FDCAN_HandleTypeDef   *zdt_hfdcan   = NULL;
static ZDT_RxCallback         zdt_rx_cb    = NULL;
static ZDT_MotorStatus_t      zdt_motors[ZDT_MAX_MOTORS];
static uint8_t                zdt_motor_cnt = 0;

/* ---- helpers ---- */

static inline uint8_t ZDT_Checksum(const uint8_t *data, uint8_t len)
{
    (void)data;
    (void)len;
    return ZDT_CHECKSUM;
}

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

/* ---- Rx dispatch (registered with bsp_can) ---- */

static void ZDT_RxDispatch(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxFrame_t *frame)
{
    (void)hfdcan;

    uint8_t addr = (uint8_t)(frame->ID >> 8);
    uint8_t func = frame->data[0];

    int slot = ZDT_FindSlot(addr);
    if (slot >= 0)
    {
        zdt_motors[slot].last_rx_tick = HAL_GetTick();

        if (func == ZDT_FC_READ_POS && frame->DLC >= 5)
        {
            zdt_motors[slot].position = ((int32_t)frame->data[1] << 24) |
                                         ((int32_t)frame->data[2] << 16) |
                                         ((int32_t)frame->data[3] << 8)  |
                                          (int32_t)frame->data[4];
        }
        else if (func == ZDT_FC_READ_STATUS && frame->DLC >= 2)
        {
            zdt_motors[slot].status_flags = frame->data[1];
        }
    }

    if (zdt_rx_cb != NULL)
        zdt_rx_cb(addr, func, frame->data, frame->DLC);
}

/* ---- init ---- */

void ZDT_Init(FDCAN_HandleTypeDef *hfdcan)
{
    zdt_hfdcan = hfdcan;
    zdt_motor_cnt = 0;
    memset(zdt_motors, 0, sizeof(zdt_motors));
    CAN_RegisterRxCallback(ZDT_RxDispatch);
}

void ZDT_SetRxCallback(ZDT_RxCallback callback)
{
    zdt_rx_cb = callback;
}

ZDT_MotorStatus_t* ZDT_GetStatus(uint8_t addr)
{
    int slot = ZDT_FindSlot(addr);
    return (slot >= 0) ? &zdt_motors[slot] : NULL;
}

/* ---- commands ---- */

HAL_StatusTypeDef ZDT_Enable(uint8_t addr)
{
    uint8_t buf[] = { ZDT_FC_ENABLE_CTL, ZDT_SUBCODE_ENABLE, 0x01, 0x00 };
    buf[3] = ZDT_Checksum(buf, 3);
    return ZDT_SendFrame(addr, 0, buf, 4);
}

HAL_StatusTypeDef ZDT_Disable(uint8_t addr)
{
    uint8_t buf[] = { ZDT_FC_ENABLE_CTL, ZDT_SUBCODE_ENABLE, 0x00, 0x00 };
    buf[3] = ZDT_Checksum(buf, 3);
    return ZDT_SendFrame(addr, 0, buf, 4);
}

HAL_StatusTypeDef ZDT_Stop(uint8_t addr)
{
    uint8_t buf[] = { ZDT_FC_STOP, ZDT_SUBCODE_STOP, 0x00 };
    buf[2] = ZDT_Checksum(buf, 2);
    return ZDT_SendFrame(addr, 0, buf, 3);
}

HAL_StatusTypeDef ZDT_SyncTrigger(void)
{
    uint8_t buf[] = { ZDT_FC_SYNC, 0x66 };
    buf[1] = ZDT_Checksum(buf, 1);
    return ZDT_SendFrame(ZDT_ADDR_BROADCAST, 0, buf, 2);
}

HAL_StatusTypeDef ZDT_ReadPosition(uint8_t addr)
{
    uint8_t buf[] = { ZDT_FC_READ_POS };
    return ZDT_SendFrame(addr, 0, buf, 1);
}

HAL_StatusTypeDef ZDT_ReadStatus(uint8_t addr)
{
    uint8_t buf[] = { ZDT_FC_READ_STATUS };
    return ZDT_SendFrame(addr, 0, buf, 1);
}

HAL_StatusTypeDef ZDT_SetVelocity(uint8_t addr, uint8_t dir,
                                   uint16_t speed_rpm, uint8_t accel,
                                   uint8_t sync_flag)
{
    uint8_t buf[7];
    buf[0] = ZDT_FC_VELOCITY;
    buf[1] = dir;
    buf[2] = (uint8_t)(speed_rpm >> 8);
    buf[3] = (uint8_t)(speed_rpm & 0xFF);
    buf[4] = accel;
    buf[5] = sync_flag;
    buf[6] = ZDT_Checksum(buf, 6);
    return ZDT_SendFrame(addr, 0, buf, 7);
}

HAL_StatusTypeDef ZDT_SetPosition(uint8_t addr, uint8_t dir,
                                   uint16_t speed_rpm, uint8_t accel,
                                   int32_t pulses, uint8_t rel_abs,
                                   uint8_t sync_flag)
{
    /* Packet 0: FC + dir + speedH + speedL + accel + pulse3 + pulse2 + pulse1 */
    uint8_t pkt0[8];
    pkt0[0] = ZDT_FC_POSITION;
    pkt0[1] = dir;
    pkt0[2] = (uint8_t)(speed_rpm >> 8);
    pkt0[3] = (uint8_t)(speed_rpm & 0xFF);
    pkt0[4] = accel;
    pkt0[5] = (uint8_t)(pulses >> 24);
    pkt0[6] = (uint8_t)(pulses >> 16);
    pkt0[7] = (uint8_t)(pulses >> 8);

    /* Packet 1: FC + pulse0 + rel_abs + sync_flag + checksum */
    uint8_t pkt1[5];
    pkt1[0] = ZDT_FC_POSITION;
    pkt1[1] = (uint8_t)(pulses & 0xFF);
    pkt1[2] = rel_abs;
    pkt1[3] = sync_flag;
    pkt1[4] = ZDT_Checksum(pkt1, 4);

    if (ZDT_SendFrame(addr, 0, pkt0, 8) != HAL_OK) return HAL_ERROR;
    return ZDT_SendFrame(addr, 1, pkt1, 5);
}
