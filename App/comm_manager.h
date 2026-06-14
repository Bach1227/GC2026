/**
  ******************************************************************************
  * @file           : comm_manager.h
  * @brief          : DMA+IDLE 接收 + 任务通知 + 零拷贝协议解析
  *
  *  使用:
  *    1. main() 中调用 Comm_Init()
  *    2. 创建 Comm_Task 线程
  *    3. USART1_IRQHandler 中调用 Comm_OnUartIdle()
  *    4. Protocol_SetRxCallback() 注册帧回调
  ******************************************************************************
  */

#ifndef __COMM_MANAGER_H__
#define __COMM_MANAGER_H__

#include <stdint.h>
#include "protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

void Comm_Init(void);
void Comm_InitTask(void);
void Comm_OnUartIdle(uint16_t received_size);
void Comm_Task(void *argument);

/* ---- 发送 ---- */

uint16_t Comm_SendFrame(const uint8_t *data, uint16_t len);
uint16_t Comm_SendCmd(uint8_t seq, uint8_t cmd,
                      const uint8_t *payload, uint16_t len);
uint16_t Comm_SendAckAck(uint8_t seq, uint8_t received, int16_t ack_seq);
uint16_t Comm_SendAckEvent(uint8_t seq, uint8_t finished);
uint16_t Comm_SendPosition(uint8_t seq, uint8_t type, int16_t x, int16_t y);
uint16_t Comm_SendHeartbeat(uint8_t seq);
uint16_t Comm_SendError(uint8_t seq, uint8_t error_code);

#ifdef __cplusplus
}
#endif

#endif /* __COMM_MANAGER_H__ */
