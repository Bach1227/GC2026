/**
  ******************************************************************************
  * @file           : interrupthandle.c
  * @brief          : 中断处理函数实现
  *
  *  外设中断的第二层处理逻辑
  ******************************************************************************
  */

#include "interrupthandle.h"
#include "comm_manager.h"
#include "usart.h"
#include "tim.h"
#include "fdcan.h"
#include "bsp_zdt.h"


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }

  if (htim == &htim2)
  {
    /* code */
  }

  if (htim == &htim3)
  {
    /* code */
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart1)
  {
    Comm_OnUartIdle(Size);
  }
  
}

void HAL_CAN_RxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hcan)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t               rx_data[8];

    if (HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        return;

    ZDT_OnRxMessage(rx_header.Identifier, rx_data, rx_header.DataLength);
}