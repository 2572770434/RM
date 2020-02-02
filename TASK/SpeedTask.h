#ifndef __SPEEDTASK_H__
#define __SPEEDTASK_H__
#include <stm32f4xx.h>
#include "RemoteTask.h"

#define CHASSIS_SPEED_ATTENUATION   (2.5f)

typedef enum
{
	near_distance,
	lost_distance,
	normal_distance,
	catwalk,
	camera_distance,
}distance_mode;

extern int16_t count_temp;
extern distance_mode distance_choice;
	
void GimbalYawControlModeSwitch(void);
void GMPitchControlLoop(void);
void GMYawControlLoop(void);
void SetGimbalMotorOutput(void);
void CMControlLoop(void);
void ShooterMControlLoop(void);
void SelectChassisMode(Time_Count *Encoder_Delay_Time);
void PIDparameterchange(void);
void Jscope_test(float Jscope_a,float Jscope_b,float Jscope_c,float Jscope_d,float Jscope_e);
void frictionwheelmotor(float  speed1,float speed2);
distance_mode getdistancemode(void);
void setdistancemode(void);

#endif
