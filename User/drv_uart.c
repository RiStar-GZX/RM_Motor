#include "drv_uart.h"
#include "drv_motor.h"

void UART_SendData(UART_HandleTypeDef * husart,uint8_t * data,uint16_t length){
		HAL_UART_Transmit(husart,data,length,HAL_MAX_DELAY);
}

//static char str[100]="UART Test!\n";

/*void StartUartTask(void const * argument)
{
  //for(;;)
  //{
		//Motor_Structure * motor = &Motors[1];
		//sprintf(str,"pid");//:%f,%f,%f\n",motor->speed_pid.Actual,motor->speed_pid.Target,motor->speed_pid.Out);
		//UART_SendData(&huart6,(uint8_t*)str,strlen(str));
    //osDelay(10);
  //}
}*/
