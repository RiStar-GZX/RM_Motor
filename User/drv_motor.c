#include "drv_motor.h"
#include "cmsis_os.h"
#include "usart.h"

uint8_t TX_DATA[8]={0}; 

Motor_Structure Motors[8+1]={0};

Motor_Structure Motor_1;

//单电机数据回调
void Motor_CAN_Callback(CAN_Rx_Buffer * rx_buffer){
	static int circle_num=0;
	static float last_angle=0,new_angle;
	//获取电调ID
	uint32_t StdId = rx_buffer->Header.StdId;
	//int motor_id = StdId-0x200;
	
	int16_t machine_angle,speed,torque;
	int8_t temp; 
	machine_angle	 = rx_buffer->Data[0]<<8;
	machine_angle += rx_buffer->Data[1];
	speed  = rx_buffer->Data[2]<<8;
	speed += rx_buffer->Data[3];
	torque  = rx_buffer->Data[4]<<8;
	torque += rx_buffer->Data[5];
	temp = rx_buffer->Data[6];
	
	new_angle = (float)machine_angle*360/8191;
	if(last_angle<=360&&last_angle>=270&&new_angle>=0&&new_angle<=90)circle_num++;
	if(new_angle<=360&&new_angle>=270&&last_angle>=0&&last_angle<=90)circle_num--;
	
	Motor_1.speed_pid.Actual = speed;
	Motor_1.angle_pid.Actual = circle_num*360+new_angle;
	if(Motor_1.type!=MOTOR_TYPE_M2006_C610){
		Motor_1.temp = temp;
	}
	last_angle = new_angle;
}

//多电机数据回调
void Motors_CAN_Callback(CAN_Rx_Buffer * rx_buffer){
	//获取电调ID
	uint32_t StdId = rx_buffer->Header.StdId;
	int motor_id = StdId-0x200;
	Motor_Structure * motor = &Motors[motor_id];
	
	//获取数据
	int16_t machine_angle,speed,torque;
	int8_t temp; 
	machine_angle	 = rx_buffer->Data[0]<<8;
	machine_angle += rx_buffer->Data[1];
	speed  = rx_buffer->Data[2]<<8;
	speed += rx_buffer->Data[3];
	torque  = rx_buffer->Data[4]<<8;
	torque += rx_buffer->Data[5];
	temp = rx_buffer->Data[6];
	
	//机械角转角度
	float last_angle,new_angle;
	last_angle = (float)motor->machine_angle*360/8191;
	new_angle = (float)machine_angle*360/8191;
	if(last_angle<=360&&last_angle>=270&&new_angle>=0&&new_angle<=90)motor->circle_num++;
	if(new_angle<=360&&new_angle>=270&&last_angle>=0&&last_angle<=90)motor->circle_num--;
	
	//保存原始数据
	motor->machine_angle = machine_angle;
	motor->speed = speed;
	motor->torque = torque;
	if(motor->type!=MOTOR_TYPE_M2006_C610){
		motor->temp = temp;
	}
	
	//修改pid实际值
	motor->speed_pid.Actual = speed;
	motor->angle_pid.Actual = motor->circle_num*360+new_angle;
}


void Motor_Init(Motor_Structure * motor,CAN_HandleTypeDef *hcan,MOTOR_ID id,MOTOR_TYPE type){
	motor->hcan = hcan;
	motor->id = id;
	motor->type = type;
	if(type==MOTOR_TYPE_M2006_C610){
		motor->speed_pid.Kp = 0.5;
		motor->speed_pid.Ki = 0.005;
		motor->speed_pid.Kd = 0;
		motor->speed_pid.out_limit = 10000;
		motor->speed_pid.able=ENABLE;
		motor->angle_pid.Kp = 0;
		motor->angle_pid.Ki = 0;
		motor->angle_pid.Kd = 0;
		motor->angle_pid.able=DISABLE;
	}
	if(type==MOTOR_TYPE_M3508_C620){
		motor->speed_pid.Kp = 2.9;
		motor->speed_pid.Ki = 0.0215;
		motor->speed_pid.Kd = 0;
		motor->speed_pid.out_limit = 16384;
		motor->speed_pid.able=ENABLE;
		motor->angle_pid.Kp = 3;
		motor->angle_pid.Ki = 0;
		motor->angle_pid.Kd = 9.75;
		motor->angle_pid.offset = 130;
		motor->angle_pid.deadband = 3;
		motor->angle_pid.able=DISABLE;
	}
	if(type==MOTOR_TYPE_GM6020){
		motor->speed_pid.Kp = 0;
		motor->speed_pid.Ki = 0;
		motor->speed_pid.Kd = 0;
		motor->speed_pid.out_limit = 16384;
		motor->speed_pid.able=ENABLE;
		motor->angle_pid.Kp = 0;
		motor->angle_pid.Ki = 0;
		motor->angle_pid.Kd = 0;
		motor->angle_pid.able=DISABLE;
	}
}


//电机数据发送
void Motor_Set_Current(Motor_Structure * motor,int16_t current){
	uint16_t ID;
	
	if(motor->type==MOTOR_TYPE_M2006_C610||motor->type==MOTOR_TYPE_M3508_C620){
		ID = C6XX_CURRENT_1_4;
		if(motor->id>MOTOR_ID_4&&motor->id<=MOTOR_ID_8){
				ID = C6XX_CURRENT_5_8;
				motor->id-=MOTOR_ID_4;
		}
	}
	
	if(motor->type==MOTOR_TYPE_GM6020){
		ID = GM6020_CURRENT_1_4;
		if(motor->id>MOTOR_ID_4&&motor->id<=MOTOR_ID_8){
				ID = GM6020_CURRENT_5_8;
				motor->id-=MOTOR_ID_4;
		}
	}
	
	uint8_t pos = (motor->id-1)*2;
	TX_DATA[pos]=(uint8_t)(current>>8);
	TX_DATA[pos+1]=(uint8_t)(current & 0xFF);
	CAN_Send_Data(motor->hcan, ID, TX_DATA, 8);
}

void Motor_Set_Voltage(Motor_Structure * motor,uint16_t voltage){
	uint16_t ID;
	
	if(motor->type==MOTOR_TYPE_M2006_C610||motor->type==MOTOR_TYPE_M3508_C620){
		return;
	}
	
	if(motor->type==MOTOR_TYPE_GM6020){
		ID = GM6020_VOLTAGE_1_4;
		if(motor->id>MOTOR_ID_4&&motor->id<=MOTOR_ID_8){
				ID = GM6020_VOLTAGE_5_8;
				motor->id-=MOTOR_ID_4;
		}
	}
	
	uint8_t pos = (motor->id-1)*2;
	TX_DATA[pos]=(uint8_t)(voltage>>8);
	TX_DATA[pos+1]=(uint8_t)(voltage & 0xFF);
	CAN_Send_Data(motor->hcan, ID, TX_DATA, 8);
}




/*----------------------------电机pid控制认任务-------------------------------------------------------*/

int flag=0;

char buf[300];

int16_t flag0_current=0; 

void MotorTask(void const * argument)
{
	//Motor_Init(&Motor_1,&hcan1,MOTOR_ID_1,MOTOR_TYPE_M2006_C610);
	//Motor_Init(&Motor_1,&hcan1,MOTOR_ID_4,MOTOR_TYPE_M3508_C620);
	Motor_Init(&Motor_1,&hcan1,MOTOR_ID_2,MOTOR_TYPE_GM6020);
	for(;;)
  {
		if(flag==0){
			Motor_Set_Current(&Motor_1,flag0_current);
			continue;
		}
		
		if(Motor_1.angle_pid.able){
			pid_calc(&Motor_1.angle_pid);
			Motor_1.speed_pid.Target = Motor_1.angle_pid.Out;
		}
		pid_calc(&Motor_1.speed_pid);		
	
		//Motor_Set_Current(MOTOR_ID_4,Motor_1.speed_pid.Out);
		//Motor_Set_Current(&Motor_1,Motor_1.speed_pid.Out);
		Motor_Set_Voltage(&Motor_1,Motor_1.speed_pid.Out);
		//串口发送数据
		//sprintf(buf,"pid:%f,%f,%f\n",Motor_1.speed_pid.Target,Motor_1.speed_pid.Actual,Motor_1.speed_pid.Out);
		sprintf(buf,"pid:%f,%f,%f\n",Motor_1.angle_pid.Target,Motor_1.angle_pid.Actual,Motor_1.angle_pid.Out);
		
		//sprintf(buf,"pid:%f,%f,%f,%f,%f,%f\n",Motor_1.speed_pid.Target,Motor_1.speed_pid.Actual,Motor_1.speed_pid.Out,
		//Motor_1.angle_pid.Target,Motor_1.angle_pid.Actual,Motor_1.angle_pid.Out);
		HAL_UART_Transmit(&huart3,(uint8_t*)buf,strlen(buf),100);
		osDelay(1);
  }
}

//用户接口
void Motor_Set_Angle(Motor_Structure * motor,float angle){
	motor->angle_pid.able = ENABLE;
	motor->angle_pid.Target = angle;
}

void Motor_Set_Speed(Motor_Structure * motor,int speed){
	motor->speed_pid.Target = speed;
}