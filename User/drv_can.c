#include "drv_can.h"

CAN_Structure CAN1_Structure;
CAN_Structure CAN2_Structure;

uint8_t CAN_0x200_Data[8];
uint8_t CAN_0x1FF_Data[8];

#define CAN_ID_ESC_FEEDBACK_BASE 0X200

void CAN_Filter_Init(CAN_HandleTypeDef *hcan){
	
		CAN_FilterTypeDef FilterConfig;

    /*FilterConfig.FilterBank = 0; 
    FilterConfig.SlaveStartFilterBank = 14;
    FilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
    FilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
    FilterConfig.FilterIdHigh = (0x200+1)<<5;
    FilterConfig.FilterIdLow = (0x200+2)<<5;
//  can_filter.FilterMaskIdHigh = (CAN_ID_ESC_FEEDBACK_BASE+1)<<5;
//  can_filter.FilterMaskIdLow = (CAN_ID_ESC_FEEDBACK_BASE+1)<<5;
    FilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    FilterConfig.FilterActivation = ENABLE;
	  HAL_CAN_ConfigFilter(&hcan1, &FilterConfig);*/
		FilterConfig.FilterBank = 0;
		FilterConfig.SlaveStartFilterBank = 14;
		FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
		FilterConfig.FilterIdHigh = 0x0000;
		FilterConfig.FilterIdLow = 0x0000;
		FilterConfig.FilterMaskIdHigh = 0x0000;
		FilterConfig.FilterMaskIdLow = 0x0000;
		FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
		FilterConfig.FilterActivation = ENABLE;
	
		if (HAL_CAN_ConfigFilter(hcan, &FilterConfig) != HAL_OK) {
				Error_Handler();
		}
}

void CAN_Init(CAN_HandleTypeDef * hcan,CAN_Callback Callback){
		if (hcan->Instance == CAN1)
    {
			CAN1_Structure.CAN_Handler = hcan;
			CAN1_Structure.Callback = Callback;
			CAN_Filter_Init(&hcan1);
		}
		else if (hcan->Instance == CAN2)
    {
     CAN2_Structure.CAN_Handler = hcan;
     CAN2_Structure.Callback = Callback;
    }
	
		CAN_Filter_Init(&hcan1);
		
		if (HAL_CAN_Start(&hcan1)!= HAL_OK)
		{
			Error_Handler();
		}
	
		if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
		{
			Error_Handler();
		}
}

uint8_t CAN_Send_Data(CAN_HandleTypeDef * hcan,uint16_t ID, uint8_t *Data, uint16_t Length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    tx_header.StdId = ID;
    //tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = Length;
		tx_header.TransmitGlobalTime = DISABLE;
	
    return (HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &tx_mailbox));
}



//CAN发送数据回调
/*void TIM_CAN_PeriodElapsedCallback()
{
    static int mod10 = 0;

    mod10++;

    // CAN1
    CAN_Send_Data(&hcan1, 0x1ff, CAN1_0x1ff_Tx_Data, 8);
    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
    CAN_Send_Data(&hcan1, 0x2ff, CAN1_0x2ff_Tx_Data, 8);

    // CAN2
    // CAN_Send_Data(&hcan2, 0x1ff, CAN2_0x1ff_Tx_Data, 8);
    // CAN_Send_Data(&hcan2, 0x200, CAN2_0x200_Tx_Data, 8);
    // CAN_Send_Data(&hcan2, 0x2ff, CAN2_0x2ff_Tx_Data, 8);

    if (mod10 == 10 - 1)
    {
        mod10 = 0;
        //  CAN_Send_Data(&hcan1, 0x220, CAN1_0x220_Tx_Data, 8);
    }
}*/

//FIFO0中断
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    //数据缓冲区
    static CAN_Rx_Buffer can_rx_buffer;

    //数据转发给对应的CAN
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_rx_buffer.Header, can_rx_buffer.Data);
        if(CAN1_Structure.Callback!=NULL)CAN1_Structure.Callback(&can_rx_buffer);
    }
    else if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &can_rx_buffer.Header, can_rx_buffer.Data);
        if(CAN2_Structure.Callback!=NULL)CAN2_Structure.Callback(&can_rx_buffer);
    }
}


