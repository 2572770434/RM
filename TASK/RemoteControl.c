#include "main.h"
#include "math.h"
static RemoteSwitch_t switch1;   //遥控器左侧拨杆
Key_State_R dafuKey_R = KEY_INIT;//大符键
void GetRemoteSwitchAction(RemoteSwitch_t *switch1, uint8_t s1)
{
	static uint32_t switch_cnt = 0;

	/* 最新状态值 */
	switch1->switch_value_raw = s1;
	switch1->switch_value_buf[switch1->buf_index] = switch1->switch_value_raw;

	/* 取最新值和上一次值 */
	switch1->switch_value1 = (switch1->switch_value_buf[switch1->buf_last_index] << 2)| (switch1->switch_value_buf[switch1->buf_index]);


	/* 最老的状态值的索引 */
	switch1->buf_end_index = (switch1->buf_index + 1) % REMOTE_SWITCH_VALUE_BUF_DEEP;

	/* 合并三个值 */
	switch1->switch_value2 = (switch1->switch_value_buf[switch1->buf_end_index]<<4) | switch1->switch_value1;	

	/* 长按判断 */
	if(switch1->switch_value_buf[switch1->buf_index] == switch1->switch_value_buf[switch1->buf_last_index])
	{
		switch_cnt++;	
	}
	else
	{
		switch_cnt = 0;
	}

	if(switch_cnt >= 40)
	{
		switch1->switch_long_value = switch1->switch_value_buf[switch1->buf_index]; 	
	}

	//索引循环
	switch1->buf_last_index = switch1->buf_index;
	switch1->buf_index++;		
	if(switch1->buf_index == REMOTE_SWITCH_VALUE_BUF_DEEP)
	{
		switch1->buf_index = 0;	
	}			
}
/**
函数：RemoteShootControl(RemoteSwitch_t *switch1, uint8_t s1) 
功能：检测遥控器左边开关的状态
大概流程：
开关从别的位置移动到最上面，第一次打开摩擦论，第二次关闭摩擦论
开关中间位置为过度状态
当摩擦论打开时，移动到最下面的话将会打开波轮，即发弹
**/
void RemoteShootControl(RemoteSwitch_t *switch1, uint8_t s1) 
{
	GetRemoteSwitchAction(switch1, s1);//更新按键状态，最后用到这一次和上一次的状态
	switch(friction_wheel_state)
	{
		case FRICTION_WHEEL_OFF:
		{	
			SetShootState(NOSHOOTING);
			frictionwheelmotor(0,0);
			if(switch1->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   //从关闭到start turning
			{
				friction_wheel_state = FRICTION_WHEEL_ON;	 
			}
		}break;
		case FRICTION_WHEEL_START_TURNNING:
		{
			if(switch1->switch_value1 == REMOTE_SWITCH_CHANGE_1TO3)   //刚启动就被关闭
			{
				SetShootState(NOSHOOTING);
				frictionwheelmotor(0,0);
				friction_wheel_state = FRICTION_WHEEL_OFF;
			}
			else
			{
				//摩擦轮加速
				//frictionwheelmotor(80*frictionRamp.Calc(&frictionRamp),80*frictionRamp.Calc(&frictionRamp));
				TIM3->CCR3 = 4000;
				//if(frictionRamp.IsOverflow(&frictionRamp))
				//{
					friction_wheel_state = FRICTION_WHEEL_ON; 	
				//}
			}
		}break;
		case FRICTION_WHEEL_ON:
		{
			if(switch1->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)//REMOTE_SWITCH_CHANGE_1TO3）   //关闭摩擦轮
			{
				friction_wheel_state = FRICTION_WHEEL_OFF;				  
				frictionwheelmotor(0,0);
				//TIM3->CCR3 = 0;
				SetShootState(NOSHOOTING);
			}
			else 
			   {
					 
					 frictionwheelmotor(-50,-50);
					 ChassisSpeedRef.rotate_ref=50;
					 CM1SpeedPID.ref =  -ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref;
		       CM2SpeedPID.ref =  ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref;
					 CM3SpeedPID.ref =  ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref;
					 CM4SpeedPID.ref =  -ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref;
		
			       if(switch1->switch_value_raw == 2)   //拨轮开始转
			       {
				
				        SetShootState(SHOOTING);
							  shot_frequency_limt.time_now = Get_Time_Micros();//获取单片机绝对时间     //TIM2的计数器值（32位不会溢出）
							  shot_frequency_limt.time_error = shot_frequency_limt.time_now - shot_frequency_limt.time_last;
							if(shot_frequency_limt.time_error > 1500000)
								{
					       count_temp ++;
					       shot_frequency_limt.time_last = Get_Time_Micros();  
				        }
			       }
//			else
//			{
//				SetShootState(NOSHOOTING);
//			}
		} break;
		default:break;
	}
/*			if(switch1->switch_value_raw == 2&&GetInputMode()==REMOTE_INPUT)
			{
				frictionwheelmotor(-250,-250);
				SetShootState(SHOOTING);
				shot_frequency_limt.time_now = Get_Time_Micros();     //TIM2的计数器值（32位不会溢出）
				shot_frequency_limt.time_error = shot_frequency_limt.time_now - shot_frequency_limt.time_last;
				if(shot_frequency_limt.time_error > 100000)
				{
					count_temp ++;
					shot_frequency_limt.time_last = Get_Time_Micros();  
				}
			}
			*/
////		 if(switch1->switch_value_raw == 2&&GetInputMode()==REMOTE_INPUT)   //进入大幅自瞄模式
////			{
////				 dafuKey_R.thisState =1;
////			}
////			else
////			{
////			   dafuKey_R.thisState =0;
////			}
	//		else 
//				if(GetInputMode()==REMOTE_INPUT)
//			{
//				SetShootState(NOSHOOTING);
//				frictionwheelmotor(0,0);
//			}
			DafuKeyCtrl_R();
			dafuKey_R.lastState = dafuKey_R.thisState;
    }
}

void DafuKeyCtrl_R(void)
 {
	if((dafuKey_R.thisState ==0)&&(dafuKey_R.lastState==1))
	{
		dafuKey_R.keyCnt ++;
		dafuKey_R.keyFlag = 1;
	}
	if(dafuKey_R.keyCnt%2==1)
	{		
		dafu_temp =1;
	}
	else
	{
		dafu_temp =0;
	}
}
/**
函数：RemoteControlProcess(Remote *rc)
功能：遥控器数据更新
**/
void RemoteControlProcess(Remote *rc)
{
	if(GetWorkState()!=PREPARE_STATE && GetWorkState()!=CAMERA_STATE)//分别为前后、左右、俯仰、旋转
	{
       if(friction_wheel_state == FRICTION_WHEEL_ON)
			 {
           ChassisSpeedRef.forward_back_ref   = -(rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET)*cos(GMYawEncoder.ecd_angle/180*PI) * STICK_TO_CHASSIS_SPEED_FB_REF_FACT
				                                        -(rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET)*sin(GMYawEncoder.ecd_angle/180*PI) * STICK_TO_CHASSIS_SPEED_LR_REF_FACT; 
		       ChassisSpeedRef.left_right_ref     = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET)*sin(GMYawEncoder.ecd_angle/180*PI) * STICK_TO_CHASSIS_SPEED_FB_REF_FACT
				                                        -(rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET)*cos(GMYawEncoder.ecd_angle/180*PI) * STICK_TO_CHASSIS_SPEED_LR_REF_FACT; 
			 }
			 else
			 {
		   ChassisSpeedRef.forward_back_ref   = -(rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_FB_REF_FACT;
//		if(ChassisSpeedRef.forward_back_ref>100)ChassisSpeedRef.forward_back_ref=100;
//		if(ChassisSpeedRef.forward_back_ref< -100)ChassisSpeedRef.forward_back_ref= -100;
				//改成限位函数VAL_LIMIT
		   ChassisSpeedRef.left_right_ref   = -(rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_LR_REF_FACT; 
       ChassisSpeedRef.rotate_ref       =  (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_FB_REF_FACT;	
			 }
			 GimbalRef.pitch_angle_dynamic_ref += -(rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;   
			 GimbalRef.yaw_angle_dynamic_ref   += -(rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_YAW_ANGLE_INC_FACT*2;
	  //大车摆脱陀螺仪用ChassisSpeedRef.rotate_ref = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_FB_REF_FACT;//为了摆脱陀螺仪，取消反馈控制
	}
	else
	{}
		/*  限位  */
		VAL_LIMIT(ChassisSpeedRef.forward_back_ref,-300,300);
		VAL_LIMIT(ChassisSpeedRef.left_right_ref,-170,170);//不知道要不要
		VAL_LIMIT(GimbalRef.pitch_angle_dynamic_ref, -30, 10);
	// VAL_LIMIT(GimbalRef.yaw_angle_dynamic_ref,  -30, 30);
	 //遥控器拨杆数据处理	
		RemoteShootControl(&switch1, rc->s1);
	    ShooterMControlLoop();
//			SetGimbalMotorOutput();
}
