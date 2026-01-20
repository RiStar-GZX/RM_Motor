 #ifndef __PID_H_
 #define __PID_H_

 #include "stdbool.h"
 #include "stm32f4xx_hal.h"

 typedef struct Pid_t{
	bool able;
	float Target;
	float Actual;
	float Out;
	float Kp;
	float Ki;
	float Kd;
	float Error0;
	float Error1;
	float ErrorInt;
	float out_limit;
	float offset;
	float deadband;
}Pid_t;
 
int pid_calc(Pid_t * pid);

#endif
