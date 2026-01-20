#ifndef __DRV_CAN_H
#define __DRV_CAN_H

#include "can.h"
#include "stm32f4xx_hal.h"

typedef struct{
    CAN_RxHeaderTypeDef Header;
    uint8_t Data[8];
}CAN_Rx_Buffer;

typedef void (*CAN_Callback)(CAN_Rx_Buffer *);

typedef struct{
    CAN_HandleTypeDef *CAN_Handler;
    CAN_Callback Callback;
}CAN_Structure;

extern CAN_Structure CAN1_Structure;
extern CAN_Structure CAN2_Structure;

extern uint8_t CAN_0x200_Data[8];
extern uint8_t CAN_0x1FF_Data[8];

void CAN_Init(CAN_HandleTypeDef * hcan,CAN_Callback Callback);

void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID);

uint8_t CAN_Send_Data(CAN_HandleTypeDef * hcan,uint16_t ID, uint8_t *Data, uint16_t Length);

#endif
