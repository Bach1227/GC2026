/**
  ******************************************************************************
  * @file           : comm_manager.c
  * @brief          : DMA+IDLE 接收 → RingBuffer 零拷贝 → 协议解析
  *
  *  DMA 循环写入 dma_rx_buf, ISR 更新 write_index,
  *  Comm_Task 调用 RingBuffer_Parse → Protocol_ParseBuffer 零拷贝解析.
  ******************************************************************************
  */

#include "comm_manager.h"
#include "usart.h"
#include "dma.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "DataStructure.h"
#include "protocol.h"

/* ====================================================================== */
/*  常量                                                                  */
/* ====================================================================== */

#define DMA_RX_BUF_SIZE    256u

/* ====================================================================== */
/*  静态变量                                                              */
/* ====================================================================== */

uint8_t       dma_rx_buf[256];
static RingBuffer_t  uart_rb;
static TaskHandle_t  comm_task_handle = NULL;

/* 发送缓冲区 */
static uint8_t tx_frame_buf[PROTOCOL_MIN_FRAME_LEN + PROTOCOL_MAX_PAYLOAD];

extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;

/* ====================================================================== */
/*  Comm_Init                                                             */
/* ====================================================================== */

void Comm_Init(void)
{
    RingBuffer_Init(&uart_rb, dma_rx_buf, DMA_RX_BUF_SIZE);
    HAL_UART_Receive_DMA(&huart1, dma_rx_buf, DMA_RX_BUF_SIZE);
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_IDLEF);
    SET_BIT(huart1.Instance->CR1, USART_CR1_IDLEIE);
}

void Comm_InitTask(void)
{
    const osThreadAttr_t attr = {
        .name       = "comm",
        .stack_size = 256 * 4,
        .priority   = osPriorityNormal,
    };
    osThreadNew(Comm_Task, NULL, &attr);
}

/* ====================================================================== */
/*  Comm_OnUartIdle (ISR 上下文)                                          */
/* ====================================================================== */

void Comm_OnUartIdle(uint16_t received_size)
{
    uart_rb.write_index = received_size % DMA_RX_BUF_SIZE;

    if (comm_task_handle != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(comm_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* ====================================================================== */
/*  Comm_Task (FreeRTOS 线程)                                             */
/* ====================================================================== */

void Comm_Task(void *argument)
{
    (void)argument;
    comm_task_handle = xTaskGetCurrentTaskHandle();

    for (;;) {
        while (RingBuffer_Parse(&uart_rb, Protocol_ParseBuffer) > 0);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

/* ====================================================================== */
/*  发送函数                                                              */
/* ====================================================================== */

uint16_t Comm_SendFrame(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) return 0;

    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, (uint8_t *)data,
                                                  len, HAL_MAX_DELAY);
    return (status == HAL_OK) ? len : 0;
}

uint16_t Comm_SendCmd(uint8_t seq, uint8_t cmd,
                      const uint8_t *payload, uint16_t len)
{
    uint16_t frame_len = Protocol_BuildCmd(seq, cmd, payload, len,
                                           tx_frame_buf, sizeof(tx_frame_buf));
    return Comm_SendFrame(tx_frame_buf, frame_len);
}

uint16_t Comm_SendAckAck(uint8_t seq, uint8_t received, int16_t ack_seq)
{
    AckAck_t ack = { .received = received, .sequence = ack_seq };
    uint8_t payload[3];
    uint16_t plen = PackAckAck(&ack, payload, sizeof(payload));

    uint16_t frame_len = Protocol_BuildAck(seq, ACK_ACK_ACK, payload, plen,
                                           tx_frame_buf, sizeof(tx_frame_buf));
    return Comm_SendFrame(tx_frame_buf, frame_len);
}

uint16_t Comm_SendAckEvent(uint8_t seq, uint8_t finished)
{
    AckEvent_t evt = { .finished = finished };
    uint8_t payload[1];
    uint16_t plen = PackAckEvent(&evt, payload, sizeof(payload));

    uint16_t frame_len = Protocol_BuildAck(seq, ACK_ACK_EVENT, payload, plen,
                                           tx_frame_buf, sizeof(tx_frame_buf));
    return Comm_SendFrame(tx_frame_buf, frame_len);
}

uint16_t Comm_SendPosition(uint8_t seq, uint8_t type, int16_t x, int16_t y)
{
    Position_t pos = { .x = x, .y = y };
    uint8_t payload[4];
    uint16_t plen = PackPosition(&pos, payload, sizeof(payload));

    uint16_t frame_len;
    if (type == MSG_TYPE_EVENT) {
        frame_len = Protocol_BuildEvent(seq, payload, plen,
                                        tx_frame_buf, sizeof(tx_frame_buf));
    } else {
        frame_len = Protocol_BuildStatus(seq, payload, plen,
                                         tx_frame_buf, sizeof(tx_frame_buf));
    }
    return Comm_SendFrame(tx_frame_buf, frame_len);
}

uint16_t Comm_SendHeartbeat(uint8_t seq)
{
    uint16_t frame_len = Protocol_BuildHeartbeat(seq,
                                                  tx_frame_buf, sizeof(tx_frame_buf));
    return Comm_SendFrame(tx_frame_buf, frame_len);
}

uint16_t Comm_SendError(uint8_t seq, uint8_t error_code)
{
    uint16_t frame_len = Protocol_BuildError(seq, error_code,
                                              tx_frame_buf, sizeof(tx_frame_buf));
    return Comm_SendFrame(tx_frame_buf, frame_len);
}
