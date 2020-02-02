#include "main.h"
#include "KALMANTEST.h"

#define YAW_POSITION_KP_DEFAULTS  25
#define YAW_POSITION_KI_DEFAULTS  0
#define YAW_POSITION_KD_DEFAULTS  0

#define YAW_SPEED_KP_DEFAULTS  35
#define YAW_SPEED_KI_DEFAULTS  0
#define YAW_SPEED_KD_DEFAULTS  0

// avoid bang --->  position:20.0  speed:19.0
//big bang   22.5 20.0
#define PITCH_POSITION_KP_DEFAULTS  20.0
#define PITCH_POSITION_KI_DEFAULTS  0
#define PITCH_POSITION_KD_DEFAULTS  0

#define PITCH_SPEED_KP_DEFAULTS  19.0
#define PITCH_SPEED_KI_DEFAULTS  0
#define PITCH_SPEED_KD_DEFAULTS  0
#define CM2_MOTOR_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	0.0f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	5000,\
	3500,\
	1500,\
	0,\
	8000,\
	&PID_Calc,\
	&PID_Reset,\
	0,\
	0,\
	0,\
}

#define CM2_MOTOR_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	0.0f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	5000,\
	3500,\
	1500,\
	0,\
	4900,\
	&PID_Calc,\
	&PID_Reset,\
	0,\
	0,\
	0,\
}

#define CHASSIS_MOTOR_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	25.0f,\
	0.5f,\
	20.0f,\
	0,\
	0,\
	0,\
	11000,\
	3500,\
	1500,\
	0,\
	16000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

#define CHASSIS_MOTOR_ROTATE_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	10.0f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	5000,\
	3500,\
	1500,\
	0,\
	8000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

#define GIMBAL_MOTOR7_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	20.0f,\
	1.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	5000,\
	3500,\
	1500,\
	0,\
	8000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

//gimbal position pid control
//20  19
#define GIMBAL_MOTOR_PITCH_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	PITCH_POSITION_KP_DEFAULTS,\
	PITCH_POSITION_KI_DEFAULTS,\
	PITCH_POSITION_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	5900,\
	1000,\
	1500,\
	0,\
	6000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

//gimbal speed pid control
#define GIMBAL_MOTOR_PITCH_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	PITCH_SPEED_KP_DEFAULTS,\
	PITCH_SPEED_KI_DEFAULTS,\
	PITCH_SPEED_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	4900,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

//gimbal yaw position pid control
#define GIMBAL_MOTOR_YAW_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	YAW_POSITION_KP_DEFAULTS,\
	YAW_POSITION_KI_DEFAULTS,\
	YAW_POSITION_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	8000,\
	2000,\
	1500,\
	0,\
	5000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

#define CHASSIS_SPEED_OFFSET_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	200,\
	0,\
	100,\
	0,\
	150,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

//gimbal yaw speed pid control
#define GIMBAL_MOTOR_YAW_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	YAW_SPEED_KP_DEFAULTS,\
	YAW_SPEED_KI_DEFAULTS,\
	YAW_SPEED_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	8000,\
	1000,\
	1500,\
	0,\
	4900,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

uint32_t time_tick_1ms = 0;

RampGen_t GMPitchRamp = RAMP_GEN_DAFAULT;
RampGen_t GMYawRamp = RAMP_GEN_DAFAULT;
RampGen_t MotorRotateRamp = RAMP_GEN_DAFAULT;

PID_Regulator_t GMPPositionPID = GIMBAL_MOTOR_PITCH_POSITION_PID_DEFAULT;     
PID_Regulator_t GMPSpeedPID 	 = GIMBAL_MOTOR_PITCH_SPEED_PID_DEFAULT;
PID_Regulator_t GMYPositionPID = GIMBAL_MOTOR_YAW_POSITION_PID_DEFAULT;			
PID_Regulator_t GMYSpeedPID 	 = GIMBAL_MOTOR_YAW_SPEED_PID_DEFAULT;

PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT; 
PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM3SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM4SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;

PID_Regulator_t ShootMotorPositionPID = GIMBAL_MOTOR_PITCH_POSITION_PID_DEFAULT;
PID_Regulator_t ShootMotorSpeedPID = GIMBAL_MOTOR7_SPEED_PID_DEFAULT;
PID_Regulator_t Speed_Offset = CHASSIS_SPEED_OFFSET_DEFAULT;
PID_Regulator_t CAN2_CM1SpeedPID      = CHASSIS_MOTOR_SPEED_PID_DEFAULT;//翻箱子的3510电机1
PID_Regulator_t CAN2_CM1PositionPID   = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CAN2_CM2SpeedPID      = CHASSIS_MOTOR_SPEED_PID_DEFAULT;//翻箱子的3510电机2
PID_Regulator_t CAN2_CM2PositionPID   = CHASSIS_MOTOR_SPEED_PID_DEFAULT;

/**
函数：Control_Task()
功能：以1000HZ执行该函数，更新各电机工作状态,无返回值
**/
void Control_Task(void)
{
	time_tick_1ms++;									//记录运行时间
	remote_error_time++;
	WorkStateFSM();										//工作状态选择
	WorkStateSwitchProcess();					//检测工作状态是否从 其他模式转到PREPARE_STATE，若是，重新初始化，否则跳过
  setdistancemode();//设置距离模式
//	float iii = KALMAN1.A.pData[1];
	//LLL = kalman_filter_calc(&KALMAN1, TEMP, 0);
	//KalmanPredict(&KALMAN1);
	//lll = KalmanCorrect(&KALMAN1, *TEMP);
  
//	 camera_yaw += V_SPEED/2;
	
	 //printf("%f\t",camera_yaw);
	GMYawControlLoop();								//计算Y轴电机输出量,0x205
//	GMPitchControlLoop();	

	//计算P轴电机输出量,0x206
	ShooterMControlLoop();      	//	控制拨轮  //发射机构控制任务,0x207
	{
		//将云台电机输出量传给电机
//		if((GetInputMode() == REMOTE_INPUT )|| (GetWorkState()== PREPARE_STATE))
//		{
			SetGimbalMotorOutput();
//		}
		
		wwdg_flag &=~0x0070;
	}
	

//	    frictionwheelmotor(800,800);
	
	//if((wwdg_flag & 0x000f) ==0x000f && time_tick_1ms%4==0)
  if(time_tick_1ms%4==0)	
	{
		CMControlLoop();								//底盘控制任务			 
		wwdg_flag &=~0x000f;
	}
	
	SuperiviseTask();									//监控任务
	//ShootSupervise();//防卡弹
		user_data_handle();//改变自定义数据，裁判系统交互
		data_pack_handle(0x0301,user_data,0x0013);//用户自定义数据打包(命令码+数据+长度)，裁判系统交互
	  
	if(time_tick_1ms % 200 == 0)
	{
		//UART3_PrintBlock(computer_tx_buf,28);
		UART3_PrintBlock1();
	}
	{
		
	Speed_Offset.kp = 10;
	Speed_Offset.kd = 2;
	Speed_Offset.outputMax = 100; 
	
	Speed_Offset.ref = 10;
  VAL_LIMIT(robotPowerHeat.ChassisPowerBuffer,0,20);
	Speed_Offset.fdb = robotPowerHeat.ChassisPowerBuffer;
	Speed_Offset.Calc(&Speed_Offset);
	Speed_Offset.output = -Speed_Offset.output;      //应该是负的关系
	VAL_LIMIT(Speed_Offset.output,-300,0);
	}
}

void ControtLoopTaskInit(void)
{
	//计数初始化
	time_tick_1ms = 0;   //中断中的计数清零
	//斜坡初始化
	GMPitchRamp.SetScale(&GMPitchRamp, MOTOR_TIME_TICK);
	GMYawRamp.SetScale(&GMYawRamp, MOTOR_TIME_TICK);
	MotorRotateRamp.SetScale(&MotorRotateRamp,MOTOR_TIME_TICK);
	 
	GMPitchRamp.ResetCounter(&GMPitchRamp);
	GMYawRamp.ResetCounter(&GMYawRamp);
	MotorRotateRamp.ResetCounter(&MotorRotateRamp);

	//PID初始化
	GMPPositionPID.Reset(&GMPPositionPID);
	GMPSpeedPID.Reset(&GMPSpeedPID);
	
	GMYPositionPID.Reset(&GMYPositionPID);
	GMYSpeedPID.Reset(&GMYSpeedPID);
	
	ShootMotorPositionPID.Reset(&ShootMotorSpeedPID);
	ShootMotorSpeedPID.Reset(&ShootMotorSpeedPID);
	
	CMRotatePID.Reset(&CMRotatePID);
	CM1SpeedPID.Reset(&CM1SpeedPID);
	CM2SpeedPID.Reset(&CM2SpeedPID);
	CM3SpeedPID.Reset(&CM3SpeedPID);
	CM4SpeedPID.Reset(&CM4SpeedPID);
}
void ChassisMotorInit(void)
{
	CM1SpeedPID.Reset(&CM1SpeedPID);
	CM2SpeedPID.Reset(&CM2SpeedPID);
	CM3SpeedPID.Reset(&CM3SpeedPID);
	CM4SpeedPID.Reset(&CM4SpeedPID);
}
//////////////A板上的可控电源输出口，通过配置PH?管脚，并且置高为输出setbits
void engineerpower_Init(void)
{		GPIO_InitTypeDef gpio;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE); 
		
		gpio.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_2;
		gpio.GPIO_Mode = GPIO_Mode_OUT;//??????
		gpio.GPIO_OType = GPIO_OType_PP;//????
		gpio.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
		gpio.GPIO_PuPd = GPIO_PuPd_UP;//??
//	GPIO_StructInit(&gpio);
		GPIO_Init(GPIOH, &gpio);
	
	GPIO_SetBits(GPIOH,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_2);
}

