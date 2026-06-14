/**
  ******************************************************************************
  * @file           : protocol.h
  * @brief          : 工创通讯协议解析器
  *
  * 帧格式: AA 55 | SEQ | TYPE | CMD | LEN(2B) | PAYLOAD(LEN bytes) | CHECKSUM(2B)
  *
  *   - AA 55: 帧头 (2 bytes)
  *   - SEQ:   序列号 (1 byte), 用于 ACK 对账
  *   - TYPE:  消息类型 (1 byte) → MsgType_t
  *   - CMD:   命令字 (1 byte) → CmdCode_t / AckCode_t (由 TYPE 决定)
  *   - LEN:   PAYLOAD 长度 (2 bytes, little-endian)
  *   - PAYLOAD: 可变长度数据
  *   - CHECKSUM: 16-bit 算术和校验 (2 bytes, little-endian)
  *
  ******************************************************************************
  */

#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <stdint.h>
#include <stdbool.h>

#include "stm32h7xx_hal.h"
#include "DataStructure.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================== */
/*  Frame constants                                                       */
/* ====================================================================== */

#define PROTOCOL_FRAME_HEADER_0  0xAAu
#define PROTOCOL_FRAME_HEADER_1  0x55u
#define PROTOCOL_MIN_FRAME_LEN   9u    /* 2 + SEQ(1) + TYPE(1) + CMD(1) + LEN(2) + CHECKSUM(2) */
#define PROTOCOL_MAX_PAYLOAD     255u

/* ====================================================================== */
/*  Enums                                                               */
/* ====================================================================== */

/** 消息类型 TYPE */
typedef enum {
    MSG_TYPE_CMD       = 0x01u,  /* 上位机 → 下位机 控制命令 */
    MSG_TYPE_ACK       = 0x02u,  /* 下位机 → 上位机 命令应答 */
    MSG_TYPE_STATUS    = 0x03u,  /* 下位机 → 上位机 周期状态上报 */
    MSG_TYPE_EVENT     = 0x04u,  /* 下位机 → 上位机 动作完成事件 */
    MSG_TYPE_HEARTBEAT = 0x05u,  /* 心跳包 */
    MSG_TYPE_ERROR     = 0x06u   /* 错误上报 */
} MsgType_t;

/** TYPE_CMD 下的命令字 CMD */
typedef enum {
    CMD_CAR_MOVE    = 0x01u,     /* 车移动 */
    CMD_GRASP       = 0x02u,     /* 抓取 */
    CMD_EMERGENCY   = 0x03u      /* 紧急矫正 */
} CmdCode_t;

/** TYPE_ACK 下的命令字 CMD */
typedef enum {
    ACK_ACK_ACK   = 0x01u,       /* 反馈是否接收到信息 */
    ACK_ACK_EVENT = 0x02u        /* 是否完成事件 */
} AckCode_t;

/* ====================================================================== */
/*  PAYLOAD 结构体                                                        */
/* ====================================================================== */

/**
 * 车移动 (CMD = 0x01, TYPE = MSG_TYPE_CMD)
 */
typedef struct {
    int16_t direction;           /* 方向 (正负号控制前后) */
    int16_t distance;            /* 距离 (度, 正方向: 车头) */
} CarMove_t;

/**
 * 抓取 (CMD = 0x02, TYPE = MSG_TYPE_CMD)
 */
typedef struct {
    int16_t x_error;             /* X 轴误差 (像素) */
    int16_t y_error;             /* Y 轴误差 (像素) */
    uint8_t grasp;               /* 0=松开, 1=抓取 */
    int16_t tray_num;            /* 目标料盘编号 */
} Grasp_t;

/**
 * 紧急矫正 (CMD = 0x03, TYPE = MSG_TYPE_CMD)
 */
typedef struct {
    uint8_t correct;             /* 0=不矫正, 1=矫正 */
} Emergency_t;

/**
 * ACK 应答 (CMD = 0x01, TYPE = MSG_TYPE_ACK)
 * 下位机反馈是否接收到信息
 */
typedef struct {
    uint8_t received;            /* 0=未收到, 1=已收到 */
    int16_t sequence;            /* 对应的 SEQ */
} AckAck_t;

/**
 * ACK 事件 (CMD = 0x02, TYPE = MSG_TYPE_ACK)
 * 下位机上报是否完成事件
 */
typedef struct {
    uint8_t finished;            /* 0=未完成, 1=已完成 */
} AckEvent_t;

/**
 * 定位信息 (TYPE = MSG_TYPE_STATUS / MSG_TYPE_EVENT)
 */
typedef struct {
    int16_t x;                   /* X 坐标 (地图左下角为原点) */
    int16_t y;                   /* Y 坐标 */
} Position_t;

/* ====================================================================== */
/*  解析后的帧结构                                                        */
/* ====================================================================== */

typedef struct {
    uint8_t  seq;
    uint8_t  type;
    uint8_t  cmd;
    uint16_t len;
    uint16_t checksum;

    /* 零拷贝: 指向 seg 内部的 payload 首字节, 回调期间有效 */
    const uint8_t *payload_ptr;

    /* 拷贝方式: 预定义类型解包目标 */
    union {
        uint8_t    raw[PROTOCOL_MAX_PAYLOAD];
        CarMove_t  car_move;
        Grasp_t    grasp;
        Emergency_t emergency;
        AckAck_t   ack_ack;
        AckEvent_t ack_event;
        Position_t position;
    } payload;
} ProtocolFrame_t;

/* ====================================================================== */
/*  API                                                                   */
/* ====================================================================== */

/**
 * 零拷贝分段解析, 适配 RingBuffer_ParseFunc 签名
 * @return 成功消费的字节数; 0 = 数据不足
 */
uint16_t Protocol_ParseBuffer(const segment_t seg[2]);

/* ====================================================================== */
/*  组帧函数 (发送端用)                                                    */
/* ====================================================================== */

/**
 * 构建 TYPE_CMD 帧
 * @param seq     序列号
 * @param cmd     命令字 CmdCode_t
 * @param payload PAYLOAD 数据
 * @param len     PAYLOAD 长度
 * @param out     输出缓冲区
 * @param out_size 输出缓冲区大小
 * @return 实际写入的帧长度, 0 表示失败
 */
uint16_t Protocol_BuildCmd(uint8_t seq, uint8_t cmd,
                           const uint8_t *payload, uint16_t len,
                           uint8_t *out, uint16_t out_size);

/**
 * 构建 TYPE_ACK 帧
 */
uint16_t Protocol_BuildAck(uint8_t seq, uint8_t ack_code,
                           const uint8_t *payload, uint16_t len,
                           uint8_t *out, uint16_t out_size);

/**
 * 构建 TYPE_STATUS 帧
 */
uint16_t Protocol_BuildStatus(uint8_t seq,
                              const uint8_t *payload, uint16_t len,
                              uint8_t *out, uint16_t out_size);

/**
 * 构建 TYPE_EVENT 帧
 */
uint16_t Protocol_BuildEvent(uint8_t seq,
                             const uint8_t *payload, uint16_t len,
                             uint8_t *out, uint16_t out_size);

/**
 * 构建 TYPE_HEARTBEAT 帧 (无 PAYLOAD)
 */
uint16_t Protocol_BuildHeartbeat(uint8_t seq,
                                 uint8_t *out, uint16_t out_size);

/**
 * 构建 TYPE_ERROR 帧
 * @param error_code 错误码 (1 byte, 放在 PAYLOAD[0])
 */
uint16_t Protocol_BuildError(uint8_t seq, uint8_t error_code,
                             uint8_t *out, uint16_t out_size);

/* ====================================================================== */
/*  PAYLOAD 拆包/封包 工具函数                                              */
/* ====================================================================== */

/** CarMove 封包 → 字节数组, 返回字节数 */
uint16_t PackCarMove(const CarMove_t *cmd, uint8_t *out, uint16_t out_size);

/** CarMove 拆包 ← 字节数组, 返回成功/失败 */
bool UnpackCarMove(const uint8_t *data, uint16_t len, CarMove_t *cmd);

/** Grasp 封包 */
uint16_t PackGrasp(const Grasp_t *cmd, uint8_t *out, uint16_t out_size);

/** Grasp 拆包 */
bool UnpackGrasp(const uint8_t *data, uint16_t len, Grasp_t *cmd);

/** Emergency 封包 */
uint16_t PackEmergency(const Emergency_t *cmd, uint8_t *out, uint16_t out_size);

/** Emergency 拆包 */
bool UnpackEmergency(const uint8_t *data, uint16_t len, Emergency_t *cmd);

/** AckAck 封包 */
uint16_t PackAckAck(const AckAck_t *ack, uint8_t *out, uint16_t out_size);

/** AckAck 拆包 */
bool UnpackAckAck(const uint8_t *data, uint16_t len, AckAck_t *ack);

/** AckEvent 封包 */
uint16_t PackAckEvent(const AckEvent_t *evt, uint8_t *out, uint16_t out_size);

/** AckEvent 拆包 */
bool UnpackAckEvent(const uint8_t *data, uint16_t len, AckEvent_t *evt);

/** Position 封包 */
uint16_t PackPosition(const Position_t *pos, uint8_t *out, uint16_t out_size);

/** Position 拆包 */
bool UnpackPosition(const uint8_t *data, uint16_t len, Position_t *pos);

/**
 * 根据 type/cmd 自动解包 payload_ptr 到 frame->payload 的对应结构体
 * @return true 解包成功, false 格式不对
 */
bool Protocol_UnpackPayload(ProtocolFrame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_H__ */
