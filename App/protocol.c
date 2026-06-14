/**
  ******************************************************************************
  * @file           : protocol.c
  * @brief          : 工创通讯协议解析器实现
  ******************************************************************************
  */

#include "protocol.h"
#include "ChassisControl.h"
#include <string.h>

/* ====================================================================== */
/*  通用帧构建                                                            */
/* ====================================================================== */

/**
 * 构建一帧完整数据
 * @return 帧长度, 0 表示缓冲区不足
 */
static uint16_t Protocol_BuildFrame(uint8_t seq, uint8_t type, uint8_t cmd,
                                    const uint8_t *payload, uint16_t len,
                                    uint8_t *out, uint16_t out_size)
{
    uint16_t frame_len = PROTOCOL_MIN_FRAME_LEN + len; /* Header(2) + ... + 校验和(2) + payload */

    if (out_size < frame_len || out == NULL)
    {
        return 0;
    }

    uint16_t idx = 0;
    out[idx++] = PROTOCOL_FRAME_HEADER_0;   /* 0xAA */
    out[idx++] = PROTOCOL_FRAME_HEADER_1;   /* 0x55 */
    out[idx++] = seq;
    out[idx++] = type;
    out[idx++] = cmd;
    out[idx++] = (uint8_t)(len & 0xFFu);              /* LEN LO */
    out[idx++] = (uint8_t)((len >> 8) & 0xFFu);       /* LEN HI */

    if (payload != NULL && len > 0)
    {
        memcpy(out + idx, payload, len);
        idx += len;
    }

    /* 校验和: 覆盖 out[2..idx-1] (SEQ..PAYLOAD) */
    uint16_t ck = 0;
    for (uint16_t i = 2; i < idx; i++) ck += out[i];
    out[idx++] = (uint8_t)(ck & 0xFFu);
    out[idx++] = (uint8_t)((ck >> 8) & 0xFFu);

    return idx;
}

/* ====================================================================== */
/*  组帧 API                                                              */
/* ====================================================================== */

uint16_t Protocol_BuildCmd(uint8_t seq, uint8_t cmd,
                           const uint8_t *payload, uint16_t len,
                           uint8_t *out, uint16_t out_size)
{
    return Protocol_BuildFrame(seq, MSG_TYPE_CMD, cmd, payload, len, out, out_size);
}

uint16_t Protocol_BuildAck(uint8_t seq, uint8_t ack_code,
                           const uint8_t *payload, uint16_t len,
                           uint8_t *out, uint16_t out_size)
{
    return Protocol_BuildFrame(seq, MSG_TYPE_ACK, ack_code, payload, len, out, out_size);
}

uint16_t Protocol_BuildStatus(uint8_t seq,
                              const uint8_t *payload, uint16_t len,
                              uint8_t *out, uint16_t out_size)
{
    return Protocol_BuildFrame(seq, MSG_TYPE_STATUS, 0x00, payload, len, out, out_size);
}

uint16_t Protocol_BuildEvent(uint8_t seq,
                             const uint8_t *payload, uint16_t len,
                             uint8_t *out, uint16_t out_size)
{
    return Protocol_BuildFrame(seq, MSG_TYPE_EVENT, 0x00, payload, len, out, out_size);
}

uint16_t Protocol_BuildHeartbeat(uint8_t seq,
                                 uint8_t *out, uint16_t out_size)
{
    return Protocol_BuildFrame(seq, MSG_TYPE_HEARTBEAT, 0x00, NULL, 0, out, out_size);
}

uint16_t Protocol_BuildError(uint8_t seq, uint8_t error_code,
                             uint8_t *out, uint16_t out_size)
{
    return Protocol_BuildFrame(seq, MSG_TYPE_ERROR, error_code, NULL, 0, out, out_size);
}

/* 段内推进一个字节 */
#define ADVANCE() do { \
    p++; rem--; consumed++; \
    if (rem == 0 && seg_idx == 0 && seg[1].len > 0) { \
        p = seg[1].ptr; rem = seg[1].len; seg_idx = 1; \
    } \
} while(0)

/* ====================================================================== */
/*  命令分发 (直接调用, 不走回调)                                           */
/* ====================================================================== */

static void Protocol_Dispatch(const ProtocolFrame_t *frame)
{
    switch (frame->type) {

    case MSG_TYPE_CMD:
        switch (frame->cmd) {
        case CMD_CAR_MOVE: {
            CarMove_t cmd;
            if (UnpackCarMove(frame->payload_ptr, frame->len, &cmd)) {
                Chassis_OnCarMove(&cmd);
            }
            break;
        }
        default:
            break;
        }
        break;

    default:
        break;
    }
}

uint16_t Protocol_ParseBuffer(const segment_t seg[2])
{
    uint16_t consumed = 0;
    uint16_t total = seg[0].len + seg[1].len;

    if (total < PROTOCOL_MIN_FRAME_LEN) return 0;

    const uint8_t *p = seg[0].ptr;
    uint16_t rem = seg[0].len;
    uint8_t  seg_idx = 0;

    while (consumed + PROTOCOL_MIN_FRAME_LEN <= total) {

        /* ---- 1. 找 0xAA ---- */
        if (*p != PROTOCOL_FRAME_HEADER_0) {
            ADVANCE();
            continue;
        }

        uint16_t frame_start = consumed;

        /* AA 已确认, 推进 */
        ADVANCE();

        /* ---- 2. 0x55 ---- */
        if (total - consumed < 1) { consumed = frame_start; break; }
        if (*p != PROTOCOL_FRAME_HEADER_1) continue;
        ADVANCE();

        /* ---- 3. SEQ TYPE CMD LEN_LO LEN_HI ---- */
        if (total - consumed < 5) { consumed = frame_start; break; }

        uint8_t seq    = *p; ADVANCE();
        uint8_t type   = *p; ADVANCE();
        uint8_t cmd    = *p; ADVANCE();
        uint8_t len_lo = *p; ADVANCE();
        uint8_t len_hi = *p; ADVANCE();

        uint16_t len = len_lo | ((uint16_t)len_hi << 8);
        if (len > PROTOCOL_MAX_PAYLOAD) continue;

        /* ---- 4. 检查帧是否完整 ---- */
        uint16_t frame_len = PROTOCOL_MIN_FRAME_LEN + len;
        if (total - frame_start < frame_len) {
            consumed = frame_start;
            break;
        }

        /* ---- 5. 校验和 ---- */
        uint16_t ck = (uint16_t)seq + type + cmd + len_lo + len_hi;

        const uint8_t *payload_ptr = p;
        for (uint16_t i = 0; i < len; i++) {
            ck += *p;
            ADVANCE();
        }

        uint8_t ck_lo = *p; ADVANCE();
        uint8_t ck_hi = *p; ADVANCE();

        /* ---- 6. 校验通过 → 回调 ---- */
        if (ck == (ck_lo | ((uint16_t)ck_hi << 8))) {
            ProtocolFrame_t frame;
            frame.seq         = seq;
            frame.type        = type;
            frame.cmd         = cmd;
            frame.len         = len;
            frame.checksum    = ck;
            frame.payload_ptr = payload_ptr;
            Protocol_Dispatch(&frame);
        }
    }

    return consumed;
}

/* ====================================================================== */
/*  PAYLOAD 封包 (结构体 → 字节流)                                        */
/* ====================================================================== */

uint16_t PackCarMove(const CarMove_t *cmd, uint8_t *out, uint16_t out_size)
{
    if (cmd == NULL || out == NULL || out_size < 4) return 0;

    out[0] = (uint8_t)(cmd->direction & 0xFFu);
    out[1] = (uint8_t)((cmd->direction >> 8) & 0xFFu);
    out[2] = (uint8_t)(cmd->distance & 0xFFu);
    out[3] = (uint8_t)((cmd->distance >> 8) & 0xFFu);

    return 4;
}

uint16_t PackGrasp(const Grasp_t *cmd, uint8_t *out, uint16_t out_size)
{
    if (cmd == NULL || out == NULL || out_size < 7) return 0;

    out[0] = (uint8_t)(cmd->x_error & 0xFFu);
    out[1] = (uint8_t)((cmd->x_error >> 8) & 0xFFu);
    out[2] = (uint8_t)(cmd->y_error & 0xFFu);
    out[3] = (uint8_t)((cmd->y_error >> 8) & 0xFFu);
    out[4] = cmd->grasp;
    out[5] = (uint8_t)(cmd->tray_num & 0xFFu);
    out[6] = (uint8_t)((cmd->tray_num >> 8) & 0xFFu);

    return 7;
}

uint16_t PackEmergency(const Emergency_t *cmd, uint8_t *out, uint16_t out_size)
{
    if (cmd == NULL || out == NULL || out_size < 1) return 0;

    out[0] = cmd->correct;

    return 1;
}

uint16_t PackAckAck(const AckAck_t *ack, uint8_t *out, uint16_t out_size)
{
    if (ack == NULL || out == NULL || out_size < 3) return 0;

    out[0] = ack->received;
    out[1] = (uint8_t)(ack->sequence & 0xFFu);
    out[2] = (uint8_t)((ack->sequence >> 8) & 0xFFu);

    return 3;
}

uint16_t PackAckEvent(const AckEvent_t *evt, uint8_t *out, uint16_t out_size)
{
    if (evt == NULL || out == NULL || out_size < 1) return 0;

    out[0] = evt->finished;

    return 1;
}

uint16_t PackPosition(const Position_t *pos, uint8_t *out, uint16_t out_size)
{
    if (pos == NULL || out == NULL || out_size < 4) return 0;

    out[0] = (uint8_t)(pos->x & 0xFFu);
    out[1] = (uint8_t)((pos->x >> 8) & 0xFFu);
    out[2] = (uint8_t)(pos->y & 0xFFu);
    out[3] = (uint8_t)((pos->y >> 8) & 0xFFu);

    return 4;
}

/* ====================================================================== */
/*  PAYLOAD 拆包 (字节流 → 结构体)                                        */
/* ====================================================================== */

bool UnpackCarMove(const uint8_t *data, uint16_t len, CarMove_t *cmd)
{
    if (data == NULL || cmd == NULL || len < 4) return false;

    cmd->direction = (int16_t)(data[0] | ((uint16_t)data[1] << 8));
    cmd->distance  = (int16_t)(data[2] | ((uint16_t)data[3] << 8));

    return true;
}

bool UnpackGrasp(const uint8_t *data, uint16_t len, Grasp_t *cmd)
{
    if (data == NULL || cmd == NULL || len < 7) return false;

    cmd->x_error  = (int16_t)(data[0] | ((uint16_t)data[1] << 8));
    cmd->y_error  = (int16_t)(data[2] | ((uint16_t)data[3] << 8));
    cmd->grasp     = data[4];
    cmd->tray_num  = (int16_t)(data[5] | ((uint16_t)data[6] << 8));

    return true;
}

bool UnpackEmergency(const uint8_t *data, uint16_t len, Emergency_t *cmd)
{
    if (data == NULL || cmd == NULL || len < 1) return false;

    cmd->correct = data[0];

    return true;
}

bool UnpackAckAck(const uint8_t *data, uint16_t len, AckAck_t *ack)
{
    if (data == NULL || ack == NULL || len < 3) return false;

    ack->received = data[0];
    ack->sequence  = (int16_t)(data[1] | ((uint16_t)data[2] << 8));

    return true;
}

bool UnpackAckEvent(const uint8_t *data, uint16_t len, AckEvent_t *evt)
{
    if (data == NULL || evt == NULL || len < 1) return false;

    evt->finished = data[0];

    return true;
}

bool UnpackPosition(const uint8_t *data, uint16_t len, Position_t *pos)
{
    if (data == NULL || pos == NULL || len < 4) return false;

    pos->x = (int16_t)(data[0] | ((uint16_t)data[1] << 8));
    pos->y = (int16_t)(data[2] | ((uint16_t)data[3] << 8));

    return true;
}

/* ====================================================================== */
/*  自动拆包 (根据 TYPE/CMD 判断)                                         */
/* ====================================================================== */

bool Protocol_UnpackPayload(ProtocolFrame_t *frame)
{
    if (frame == NULL) return false;

    const uint8_t *data = frame->payload_ptr ? frame->payload_ptr : frame->payload.raw;
    uint16_t len = frame->len;

    switch (frame->type)
    {
    case MSG_TYPE_CMD:
        switch (frame->cmd)
        {
        case CMD_CAR_MOVE:
            return UnpackCarMove(data, len, &frame->payload.car_move);
        case CMD_GRASP:
            return UnpackGrasp(data, len, &frame->payload.grasp);
        case CMD_EMERGENCY:
            return UnpackEmergency(data, len, &frame->payload.emergency);
        default:
            return false; /* 未知 CMD */
        }

    case MSG_TYPE_ACK:
        switch (frame->cmd)
        {
        case ACK_ACK_ACK:
            return UnpackAckAck(data, len, &frame->payload.ack_ack);
        case ACK_ACK_EVENT:
            return UnpackAckEvent(data, len, &frame->payload.ack_event);
        default:
            return false;
        }

    case MSG_TYPE_STATUS:
    case MSG_TYPE_EVENT:
        /* STATUS/EVENT 默认携带 Position */
        if (len >= 4)
        {
            return UnpackPosition(data, len, &frame->payload.position);
        }
        return false;

    case MSG_TYPE_HEARTBEAT:
        /* 心跳无 PAYLOAD, 直接成功 */
        return true;

    case MSG_TYPE_ERROR:
        /* 错误帧 PAYLOAD[0] 即错误码, 已存在 raw 中 */
        return true;

    default:
        return false;
    }
}
