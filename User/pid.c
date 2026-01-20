#include "pid.h"
#include "math.h"

int pid_calc(Pid_t * pid){
			if(!pid->able)return 0;
			
			pid->Error1 = pid->Error0;			
			pid->Error0 = pid->Target - pid->Actual;
					
			if (pid->Ki != 0)				
			{
				pid->ErrorInt += pid->Error0;		
			}
			else						
			{
				pid->ErrorInt = 0;	
			}
			
			
			if(pid->out_limit>0&&pid->ErrorInt>pid->out_limit/pid->Ki)pid->ErrorInt=pid->out_limit/pid->Ki;
			if(pid->out_limit>0&&pid->ErrorInt<-pid->out_limit/pid->Ki)pid->ErrorInt=-pid->out_limit/pid->Ki;
			
			
			
			if(pid->deadband>0&&fabs(pid->Error0)<pid->deadband){
				pid->Out=0;
			}	
			else{
				pid->Out = (pid->Kp) * pid->Error0 + 
							 pid->Ki * pid->ErrorInt + 
							 pid->Kd * (pid->Error0 - pid->Error1);	
				if(pid->offset!=0){
					if(pid->Out>0)pid->Out+=pid->offset;
					if(pid->Out<0)pid->Out-=pid->offset;
				}
			}
				
			if(pid->out_limit>0&&pid->Out>pid->out_limit)pid->Out=pid->out_limit;
			if(pid->out_limit>0&&pid->Out<-pid->out_limit)pid->Out=-pid->out_limit;
			
}	

