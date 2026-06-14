#include "DataStructure.h"
#include "string.h"

void RingBuffer_Init(RingBuffer_t *pCircularRingBuffer, uint8_t *pBuffer, uint16_t bufferSize)
{
    pCircularRingBuffer->buffer = pBuffer;
    pCircularRingBuffer->size = bufferSize;
    pCircularRingBuffer->read_index = 0;
    pCircularRingBuffer->write_index = 0;
}

uint16_t RingBuffer_Parse(RingBuffer_t *pRingBuffer, RingBuffer_ParseFunc parseFunc)
{
    uint16_t available;
    
    // 计算可用数据长度
    if (pRingBuffer->write_index >= pRingBuffer->read_index) {
        available = pRingBuffer->write_index - pRingBuffer->read_index;
    } else {
        available = pRingBuffer->size - pRingBuffer->read_index + pRingBuffer->write_index;
    }
    
    if (available == 0) {
        return 0;
    }

    segment_t temp_seg[2];
    
    // 判断是否跨越边界
    if (pRingBuffer->read_index + available > pRingBuffer->size) {
        uint16_t firstLen = pRingBuffer->size - pRingBuffer->read_index;
        temp_seg[0].ptr = pRingBuffer->buffer + pRingBuffer->read_index;
        temp_seg[0].len = firstLen;
        temp_seg[1].ptr = pRingBuffer->buffer;
        temp_seg[1].len = available - firstLen;
    } else {
        temp_seg[0].ptr = pRingBuffer->buffer + pRingBuffer->read_index;
        temp_seg[0].len = available;
        temp_seg[1].ptr = NULL;
        temp_seg[1].len = 0;
    }
    
    uint16_t consumed = parseFunc(temp_seg);

    if (consumed > 0 && consumed <= available) {
        pRingBuffer->read_index = (pRingBuffer->read_index + consumed) % pRingBuffer->size;
    }
    return consumed;
}

void DoubleBuffer_Init(DoubleBuffer_t *db, uint8_t *b0, uint8_t *b1, uint16_t sz)
{
    db->buf[0] = b0;
    db->buf[1] = b1;
    db->idx    = 0;
    db->size   = sz;
} 

uint8_t* DoubleBuffer_GetWriteBuf(DoubleBuffer_t *db)
{
    return db->buf[db->idx];
}

void DoubleBuffer_Swap(DoubleBuffer_t *db)
{
    db->idx ^= 1;
}

uint8_t DoubleBuffer_Produce(DoubleBuffer_t *db,
                             DoubleBuffer_FillFunc fill,
                             DoubleBuffer_SubmitFunc submit)
{
    uint8_t *buf = db->buf[db->idx];
    uint16_t len = fill(buf, db->size);
    if (len == 0) {
        return 2;  /* 无数据 */
    }
    if (!submit(buf, len)) {
        return 0;  /* 消费者忙, 未交换 */
    }
    db->idx ^= 1;
    return 1;
}
