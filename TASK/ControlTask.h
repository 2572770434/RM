#ifndef __CONTROLTASK_H__
#define __CONTROLTASK_H__
//#include "pid.h"
#include "ramp.h"
#include "pid_regulator.h"

#define PREPARE_TIME_TICK_MS 	2000
#define MOTOR_TIME_TICK      2000
#define ROTATE_TIME_TICK_MS  50

//gimbal pitth position pid control

extern RampGen_t GMPitchRamp;
extern RampGen_t GMYawRamp;
extern RampGen_t MotorRotateRamp ;

extern PID_Regulator_t GMPPositionPID;     
extern PID_Regulator_t GMPSpeedPID;
extern PID_Regulator_t GMYPositionPID;			
extern PID_Regulator_t GMYSpeedPID;

extern PID_Regulator_t CMRotatePID; 
extern PID_Regulator_t CM1SpeedPID;
extern PID_Regulator_t CM2SpeedPID;
extern PID_Regulator_t CM3SpeedPID;
extern PID_Regulator_t CM4SpeedPID;
extern PID_Regulator_t Speed_Offset;

extern PID_Regulator_t CAN2_CM1SpeedPID;
extern PID_Regulator_t CAN2_CM2SpeedPID;

extern PID_Regulator_t ShootMotorPositionPID;
extern PID_Regulator_t ShootMotorSpeedPID;
extern uint32_t time_tick_1ms;

void Control_Task(void);
void ControtLoopTaskInit(void);
void ChassisMotorInit(void);
void engineerpower_Init(void);
#endif
