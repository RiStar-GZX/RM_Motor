#ifndef __DRV_USART_H
#define __DRV_USART_H

#include "stm32f4xx_hal.h"
#include "usart.h"

void UART_SendData(UART_HandleTypeDef * husart,uint8_t * data,uint16_t length);

//void StartUartTask(void const * argument);

#endif
