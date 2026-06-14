#ifndef DATASTRUCTURE_H
#define DATASTRUCTURE_H

#include "stdint.h"
#include <stdbool.h>

//为了实现零拷贝的环形缓冲区，定义segment_t，使解析函数能处理逻辑上连续，空间上断开的数据包

typedef struct {
    uint8_t* ptr;
    uint16_t len;    
}segment_t;

typedef struct
{
    uint16_t size;
    uint16_t read_index;
    uint16_t write_index;
    uint8_t* buffer;
}RingBuffer_t;

// 环形缓冲区解析函数类型
// 零拷贝: 直接读取 data[2] 分段描述符, 无需额外拷贝
// 返回值: 成功消费的字节数; 0 = 数据不足
typedef uint16_t (*RingBuffer_ParseFunc)(const segment_t data[2]);

void RingBuffer_Init(RingBuffer_t *pCircularRingBuffer, uint8_t *pBuffer, uint16_t bufferSize);

// 写入数据 (ISR 上下文, 生产者)
// 返回值: 实际写入字节数; 0 = 缓冲区满
uint16_t RingBuffer_Write(RingBuffer_t *rb, const uint8_t *data, uint16_t len);

// 零拷贝解析 (任务上下文, 消费者)
// 内部自动更新 read_index
// 返回值: 本次消费的字节数; 0 = 无数据或解析未完成
uint16_t RingBuffer_Parse(RingBuffer_t *pRingBuffer, RingBuffer_ParseFunc parseFunc);

/* 双缓冲: 两个缓冲区轮换写入, 消费者异步消费时不会覆盖 */
typedef struct {
    uint8_t *buf[2];
    uint8_t  idx;
    uint16_t size;
} DoubleBuffer_t;

/* fill: 填充 buf[0..size-1], 返回字节数, 0=无数据 */
typedef uint16_t (*DoubleBuffer_FillFunc)(uint8_t *buf, uint16_t size);
/* submit: 提交 buf[0..len-1], 返回 1=OK 0=忙 */
typedef uint8_t  (*DoubleBuffer_SubmitFunc)(const uint8_t *buf, uint16_t len);

void     DoubleBuffer_Init(DoubleBuffer_t *db, uint8_t *b0, uint8_t *b1, uint16_t sz);
uint8_t* DoubleBuffer_GetWriteBuf(DoubleBuffer_t *db);
void     DoubleBuffer_Swap(DoubleBuffer_t *db);
/* 填充 → 提交, 成功自动 Swap。返回 1=OK 0=忙 2=fill返回0 */
uint8_t  DoubleBuffer_Produce(DoubleBuffer_t *db,
                              DoubleBuffer_FillFunc fill,
                              DoubleBuffer_SubmitFunc submit);

#endif
