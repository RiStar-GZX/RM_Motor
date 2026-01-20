#ifndef _MOTOR_H_
#define _MOTOR_H_
 
#include "math.h"
#include "stm32f4xx_hal.h"
#include "drv_can.h"
#include "pid.h"

//M2006-C610和M3508-C620
#define C6XX_CURRENT_1_4 0x200
#define C6XX_CURRENT_5_8 0x1FF

#define C6XX_FEEDBACK 0x200

//GM6020
#define GM6020_VOLTAGE_1_4 0x1FF
#define GM6020_VOLTAGE_5_8 0x2FF

#define GM6020_CURRENT_1_4 0x1FE
#define GM6020_CURRENT_5_8 0x2FE

#define GM6020_FEEDBACK 0x204


//电机的编号
typedef enum {
	MOTOR_ID_1 = 1,
	MOTOR_ID_2,
	MOTOR_ID_3,
	MOTOR_ID_4,
	MOTOR_ID_5,
	MOTOR_ID_6,
	MOTOR_ID_7,
	MOTOR_ID_8,
}MOTOR_ID;

//电机类型
typedef enum {
	MOTOR_TYPE_M2006_C610 = 1,
	MOTOR_TYPE_M3508_C620 ,
	MOTOR_TYPE_GM6020 ,
}MOTOR_TYPE;

//电机结构体
typedef struct{
	bool able;
	
	CAN_HandleTypeDef *hcan;
	
	MOTOR_TYPE type;
	
	MOTOR_ID id;
	
	//原始数据
	int16_t machine_angle;
	int16_t speed;
	int16_t torque;
	int8_t temp;
	int circle_num;
	
	//pid
	Pid_t speed_pid;
	Pid_t angle_pid;
}Motor_Structure;

extern Motor_Structure Motors[8+1];	//电调ID是从1到8

void Motor_Init(Motor_Structure * motor,CAN_HandleTypeDef *hcan,MOTOR_ID id,MOTOR_TYPE type);

void Motor_CAN_Callback(CAN_Rx_Buffer * rx_buffer);

void MotorTask(void const * argument);

void Motor_Set_Current(Motor_Structure * motor,int16_t current);

void Motor_Set_Angle(Motor_Structure * motor,float angle);

void Motor_Set_Speed(Motor_Structure * motor,int speed);

#endif
