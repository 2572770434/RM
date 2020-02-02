#include "main.h"

ChassisSpeed_Ref_t ChassisSpeedRef;
Gimbal_Ref_t GimbalRef;
RC_Ctl_t RC_CtrlData;
RC_GameInformation_t GameInformationData;
uint16_t Speed_Friction1 = 600;//375;//摩擦论速度,1000停止,2000最大   右边
uint16_t Speed_Friction2 = 600;//375;//摩擦论速度,1000停止,2000最大   左边

RampGen_t frictionRamp = RAMP_GEN_DAFAULT;   //摩擦轮斜坡
RampGen_t MouseXSpeedRamp = RAMP_GEN_DAFAULT; 
RampGen_t MouseYSpeedRamp = RAMP_GEN_DAFAULT; 
RampGen_t catGimbalCount = RAMP_GEN_DAFAULT; 
RampGen_t LRSpeedRamp  = RAMP_GEN_DAFAULT;   //mouse左右移动斜坡
RampGen_t FBSpeedRamp  = RAMP_GEN_DAFAULT;   //mouse前后移动斜坡
RampGen_t RoSpeedRamp  = RAMP_GEN_DAFAULT;	 //q e快速旋转

uint8_t IsRemoteBeingAction(void)
{
	return 	(fabs(GimbalRef.yaw_speed_ref)>=10 || \
		       fabs(ChassisSpeedRef.forward_back_ref)>=10 || \
	         fabs(ChassisSpeedRef.left_right_ref)>=10 || \
	         fabs(ChassisSpeedRef.rotate_ref)>=10);
}

/**
函数：RemoteDataPrcess(uint8_t *pData)
功能：对遥控器信号进行处理
**/
void RemoteDataPrcess(uint8_t *pData)
{
	remote_micrsecond.time_last =Get_Time_Micros();
	if(pData == NULL)
	{
			return;
	}
	//ch0~ch3:max=1684,min=364,|error|=660
	RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; //遥控器通道0，控制左右平移
	RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;//遥控器通道1，控制前进后退
	RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;//遥控器通道2，控制旋转
	RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;//遥控器通道3：控制俯仰
	
	RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;//遥控器左边开关，有3个挡位，遥控器控制模式下有用，详见 RemoteShootControl
	RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);//遥控器右边开关，有3个挡位，最下为强制停止，最上为遥控器控制，中间为键盘控制

	RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);//鼠标左右
	RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);//鼠标上下
	RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);    //没用到

	RC_CtrlData.mouse.press_l = pData[12];//鼠标左键
	RC_CtrlData.mouse.press_r = pData[13];//鼠标右键

	RC_CtrlData.key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);//每一位对应一个按键
	
/**
0x0001:w 
0x0002:s 
0x0004:a 
0x0008:d 

0x0010:shift 
0x0020:ctrl 
0x0040:q 
0x0080:e 

0x0100:r 
0x0200:f 
0x0400:g 
0x0800:z 

0x1000:x 
0x2000:c 
0x4000:v 
0x8000:b
**/
		
		SetInputMode(&RC_CtrlData.rc);//检测遥控器右边开关，选择模式
		SetS1InputMode(&RC_CtrlData.rc);//检测左边开关状态，选择自瞄

		switch(GetInputMode())
	{
		case REMOTE_INPUT:
		{
			//遥控器控制模式
			RemoteControlProcess(&(RC_CtrlData.rc));
		}break;
		case KEY_MOUSE_INPUT:
		{
			//键鼠控制模式
			MouseKeyControlProcess(&RC_CtrlData.mouse,&RC_CtrlData.key);
		}break;
		case STOP:
		{
			//紧急停车
			InitFrictionWheel();
			frictionwheelmotor( 0 , 0 );
			Set_CM_Speed(CAN1, 0,0,0,0);
			Set_Gimbal_Current(CAN1, 0, 0, 0);
		}break;
	}
	

//	wwdg_flag |=0x0040;
}


/*****************
函数:JudgementHandle(uint8_t *pData)
功能：裁判系统数据处理
*****************/
void JudgementHandle(uint8_t *pData)
{
  GameInformationData.CommandCodeID = (int16_t)pData[5];
	GameInformationData.TimeRemain = ((int32_t)pData[7]<<24)|((int32_t)pData[8]<<16)|((int32_t)pData[9]<<8)|((int32_t)pData[10]);
	GameInformationData.BloodRemain = ((int16_t)pData[12]<<8)|((int16_t)pData[11]);                                                                                                                                                                           
	GameInformationData.Voltage = ((int32_t)pData[16]<<24)|((int32_t)pData[15]<<16)|((int32_t)pData[14]<<8)|((int32_t)pData[13]);
	GameInformationData.Current = ((int32_t)pData[20]<<24)|((int32_t)pData[18]<<19)|((int32_t)pData[18]<<8)|((int32_t)pData[17]);
	GameInformationData.RemainPower = ((int32_t)pData[41]<<24)|((int32_t)pData[40]<<16)|((int32_t)pData[39]<<8)|((int32_t)pData[38]);
	GameInformationData.ecd_Voltage = Parameter_Transformation(GameInformationData.Voltage);
	GameInformationData.ecd_Current = Parameter_Transformation(GameInformationData.Current);
	GameInformationData.ChassisPower = GameInformationData.ecd_Voltage*GameInformationData.ecd_Current;
}
//遥控器数据初始化，斜坡函数等的初始化
void RemoteTaskInit(void)
{
	//斜坡初始化
	frictionRamp.SetScale(&frictionRamp, FRICTION_RAMP_TICK_COUNT);
	MouseXSpeedRamp.SetScale (&MouseXSpeedRamp , MOUSR_MOUSEX_RAMP_TICK_COUNT);
	MouseYSpeedRamp.SetScale (&MouseYSpeedRamp , MOUSR_MOUSEY_RAMP_TICK_COUNT);
	catGimbalCount.SetScale(&catGimbalCount, 150);
	LRSpeedRamp.SetScale (&LRSpeedRamp , 20);
	FBSpeedRamp.SetScale (&FBSpeedRamp , 30);
	RoSpeedRamp.SetScale (&RoSpeedRamp , MOUSE_Ro_RAMP_TICK_COUNT);
	
	frictionRamp.ResetCounter(&frictionRamp);
	MouseXSpeedRamp.ResetCounter(&MouseXSpeedRamp);
	MouseYSpeedRamp.ResetCounter(&MouseYSpeedRamp);
	catGimbalCount.ResetCounter(&catGimbalCount);
	LRSpeedRamp.ResetCounter(&LRSpeedRamp);
	FBSpeedRamp.ResetCounter(&FBSpeedRamp);
	RoSpeedRamp.ResetCounter(&RoSpeedRamp);
	
	//底盘云台给定值初始化
	GimbalRef.pitch_angle_dynamic_ref = 0.0f;
	GimbalRef.yaw_angle_dynamic_ref = 0.0f;
	ChassisSpeedRef.forward_back_ref = 0.0f;
	ChassisSpeedRef.left_right_ref = 0.0f;
	ChassisSpeedRef.rotate_ref = 0.0f;
	
	//摩擦轮运行状态初始化
	SetFrictionState(FRICTION_WHEEL_OFF);
	InitFrictionWheel();
}
