#ifndef __CAN_BASIC_H
#define __CAN_BASIC_H

#define STM32H7


#ifdef STM32F1

    #include "stm32f1xx_hal.h"

    void CAN_Init(void);
    void CAN_Transmit_STD(uint32_t ID, uint8_t* data, uint8_t len);
    void CAN_Transmit_EXT(uint32_t ID, uint8_t* data, uint8_t Len);
    uint8_t RxFifoX_IsEmpty(uint32_t RxFifo);
    void CAN_FilterBank0Init(void);

#endif

#ifdef STM32F4

    #include "stm32f4xx_hal.h"

    void CAN_Init(void);
    void CAN_Transmit_STD(CAN_HandleTypeDef* hcan_num, uint32_t ID, uint8_t* data, uint8_t len);
    void CAN_Transmit_EXT(CAN_HandleTypeDef* hcan_num, uint32_t ID, uint8_t* data, uint8_t Len);
    uint8_t RxFifoX_IsEmpty(CAN_HandleTypeDef* hcan_num, uint32_t RxFifo);
    void CAN_FilterInit(void);

#endif

#ifdef STM32H7

#include "stm32h7xx_hal.h"
#include "fdcan.h"

typedef struct
{
    uint16_t ID;
    uint8_t DLC;
    uint8_t data[8];
    uint32_t TimeStamp;
    uint8_t Status;
}CAN_STD_Frame;

typedef struct
{
    uint32_t ID;
    uint8_t DLC;
    uint8_t data[8];
    uint32_t TimeStamp;
} FDCAN_RxFrame_t;

typedef void (*CAN_RxCallback)(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxFrame_t *frame);

void CAN_FilterInit(void);
HAL_StatusTypeDef CAN_Transmit_STD(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
HAL_StatusTypeDef CAN_Transmit_EXT(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint32_t len);
void CAN_RegisterRxCallback(CAN_RxCallback callback);

#ifdef HAL_FDCAN_MODULE_ENABLED

HAL_StatusTypeDef FDCAN_Transmit_STD_Buffer(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len, uint8_t buffer_index);

#endif


#endif // !__CAN_HANDLER_H

#endif
