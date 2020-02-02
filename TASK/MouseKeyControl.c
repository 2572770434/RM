#include  "main.h"
//Key_State MouseKey = KEY_INIT;
Key_State Rotate_Quater_L = KEY_INIT;
Key_State Rotate_Quater_R = KEY_INIT;
Key_State cameraKey = KEY_INIT;//zimiao
Key_State dafuKey = KEY_INIT;//大符
Key_State servoKey = KEY_INIT;//舵机键
Key_State frictionKey = KEY_INIT;//摩擦轮速度切换键
Key_State heatLoopKey = KEY_INIT;//热量闭环控制键
Key_State sendMainfoldDataKey = KEY_INIT;
Key_State xtl = KEY_INIT;//XTL
uint16_t MouseChangeCnt = 0;
uint16_t DelayCount = 0;
uint8_t mouseLeftPress = 0;
uint8_t continuous_shoot_Flag = 0;
uint8_t quickShootFlag = 0;
uint8_t servoPositionFlag = 0;
uint8_t sendCustomDataFlag = 0;
uint8_t predictflag = 0;
/*  速度控制   */
#define HIGH_FORWARD_BACK_SPEED 			430
#define HIGH_LEFT_RIGHT_SPEED   			300
#define HIGH_ROTATE_SPEED   			    80

#define LOW_FORWARD_BACK_SPEED 			  340
#define LOW_LEFT_RIGHT_SPEED   			  170
#define LOW_ROTATE_SPEED   			      50

#define MOUSE_TO_PITCH_ANGLE_INC_FACT 		0.080f
#define MOUSE_TO_YAW_ANGLE_INC_FACT 		0.080f

#define MOUSE_TO_PITCH_SPEED_REF 		5.0f
#define MOUSE_TO_YAW_SPEED_REF 			5.0f
float kp = 1.0;
/**
函数：MouseShootControl(Mouse *mouse)
功能：控制发弹模式
具体流程：
单击鼠标右启动摩擦轮
点击鼠标左键开始发弹
长按右键关闭摩擦轮
**/
void MouseShootControl(Mouse *mouse)
{
	switch(friction_wheel_state)
	{
		case FRICTION_WHEEL_OFF:
		{
			if((GetWorkState() == STOP_STATE)|| (GetWorkState() == PREPARE_STATE))
			{	
			}
			else
			{
			}
		}break;
		case FRICTION_WHEEL_START_TURNNING:
		{
		}break;
		case FRICTION_WHEEL_ON:
		{
			if(mouseLeftPress ==1)
				{
					if(robotPowerHeat.shooter_17_Heat - Heat17_Max< -3*robotShootData.bulletSpeed )//shooot heat loop
					{
						shot_frequency_limt.time_now = Get_Time_Micros();
						shot_frequency_limt.time_error = shot_frequency_limt.time_now - shot_frequency_limt.time_last;
						if(shot_frequency_limt.time_error > 64000)//0.15s的间隔
						{
							SetShootState(SHOOTING);
						  count_temp ++;
							shot_frequency_limt.time_last = Get_Time_Micros();
						}
					}
				}
		}break;	
	}	
	if(mouse->press_l ==1)//mouse->last_press_l ==0 && 
	{
		mouseLeftPress = 1;
	}
	else
	{mouseLeftPress = 0;}
	mouse->last_press_r = mouse->press_r;
	mouse->last_press_l = mouse->press_l;
	
//	ShooterMControlLoop();
//	SetGimbalMotorOutput();
}

/**
0x0001:w 
0x0002:s 
0x0004:a 
0x0008:d 

0x0010:shift 
0x0020:ctrl 
0x0040:q 
0x0080:e 
`
0x0100:r 
0x0200:f 
0x0400:g 
0x0800:z 

0x1000:x 
0x2000:c 
0x4000:v 
0x8000:b
**/

/**
函数：MouseKeyControlProcess(Mouse *mouse, Key *key)
功能：键盘控制模式
按键配置：WASD前后左右
z弹仓盖子舵机
q猫步
c大幅
v切换大小符程序(调试专用)
**/
void MouseKeyControlProcess(Mouse *mouse, Key *key)
{
	static uint16_t forward_back_speed = HIGH_FORWARD_BACK_SPEED;
	static uint16_t left_right_speed = HIGH_LEFT_RIGHT_SPEED;
//	static uint16_t rotate_speed = HIGH_ROTATE_SPEED;
	if(GetWorkState()!=PREPARE_STATE)
	{
		  
		   if(key->v &0x0010) // key: shift
		  {
			  sendMainfoldDataKey.thisState=1;
		  }
		  else
		  {
			  sendMainfoldDataKey.thisState=0;
		  }
		
	    if((sendMainfoldDataKey.thisState ==1)&&(sendMainfoldDataKey.lastState==0))
	    { 
		    sendMainfoldDataKey.keyCnt ++;
	    }
	    if(sendMainfoldDataKey.keyCnt%2==1)
	    {
					continuous_shoot_Flag = 0;
					forward_back_speed =  LOW_FORWARD_BACK_SPEED;
					left_right_speed   = LOW_LEFT_RIGHT_SPEED;
	    }		
			else
			{
				continuous_shoot_Flag = 1;
				forward_back_speed =  HIGH_FORWARD_BACK_SPEED;
				left_right_speed   = HIGH_LEFT_RIGHT_SPEED;
			}
				
				if(xtl_state == XTL_OFF)
			{
			//movement process
				if(key->v & 0x0001)//if w and V are pressed
			  {
					if(key->v & 0x4000)
					{
						GimbalRef.pitch_angle_dynamic_ref += 0.1;
					}
					else 
					{
						ChassisSpeedRef.forward_back_ref = -(forward_back_speed + Speed_Offset.output) * (FBSpeedRamp.Calc(&FBSpeedRamp));
					}
				}
				else if(key->v & 0x0002) //key: s 后
				{
					if(key->v & 0x4000)
					{
						GimbalRef.pitch_angle_dynamic_ref -= 0.1;
					}
					else
					{
						ChassisSpeedRef.forward_back_ref = (forward_back_speed-10 + Speed_Offset.output) * (FBSpeedRamp.Calc(&FBSpeedRamp));
					}
				}
				else
				{
					ChassisSpeedRef.forward_back_ref = 0;	
					FBSpeedRamp.ResetCounter(&FBSpeedRamp);
				}
				
				if(key->v & 0x0004)  // key: a 左平移
				{
					if(key->v & 0x4000)
					{
						GimbalRef.yaw_angle_dynamic_ref += 0.1;
					}
					else 
					{

						ChassisSpeedRef.left_right_ref = (left_right_speed + 0.3*Speed_Offset.output) * LRSpeedRamp.Calc(&LRSpeedRamp);
						}
				}
				else if(key->v & 0x0008) //key: d 右平移
				{
					if(key->v & 0x4000)
					{
						GimbalRef.yaw_angle_dynamic_ref -= 0.1;
					}
					else
					{
											ChassisSpeedRef.left_right_ref = -(left_right_speed + 0.3*Speed_Offset.output) * LRSpeedRamp.Calc(&LRSpeedRamp);
					}
				}
				else
				{
				
					ChassisSpeedRef.left_right_ref = 0;
					LRSpeedRamp.ResetCounter(&LRSpeedRamp);
				}
			}
			else if(xtl_state == XTL_ON)
			{
					if(key->v & 0x0001)//if w are pressed
				  {
						  if(key->v & 0x0004)//a
						  {  
								 ChassisSpeedRef.forward_back_ref = -(forward_back_speed ) * FBSpeedRamp.Calc(&FBSpeedRamp)*cos(GMYawEncoder.ecd_angle/180*PI)
								                                    +(forward_back_speed ) * LRSpeedRamp.Calc(&LRSpeedRamp)*sin(GMYawEncoder.ecd_angle/180*PI);
						     ChassisSpeedRef.left_right_ref = (forward_back_speed  ) * FBSpeedRamp.Calc(&FBSpeedRamp)*sin(GMYawEncoder.ecd_angle/180*PI)
								                                   +(forward_back_speed ) * LRSpeedRamp.Calc(&LRSpeedRamp)*cos(GMYawEncoder.ecd_angle/180*PI);
						  }
							else if(key->v & 0x0008)//d
							{
							   ChassisSpeedRef.forward_back_ref = -(forward_back_speed ) * FBSpeedRamp.Calc(&FBSpeedRamp)*cos(GMYawEncoder.ecd_angle/180*PI)
								                                    -(forward_back_speed ) * LRSpeedRamp.Calc(&LRSpeedRamp)*sin(GMYawEncoder.ecd_angle/180*PI);
						     ChassisSpeedRef.left_right_ref = (forward_back_speed  ) * FBSpeedRamp.Calc(&FBSpeedRamp)*sin(GMYawEncoder.ecd_angle/180*PI)
								                                   -(forward_back_speed  ) * LRSpeedRamp.Calc(&LRSpeedRamp)*cos(GMYawEncoder.ecd_angle/180*PI);
							}
							else//w
							{
						  ChassisSpeedRef.forward_back_ref = -(forward_back_speed ) * FBSpeedRamp.Calc(&FBSpeedRamp)*cos(GMYawEncoder.ecd_angle/180*PI);
						  ChassisSpeedRef.left_right_ref = (forward_back_speed  ) * FBSpeedRamp.Calc(&FBSpeedRamp)*sin(GMYawEncoder.ecd_angle/180*PI);
							}
					}
				  else if(key->v & 0x0002) //key: s 后
				  {
				 	   if(key->v & 0x4000)
					   {
						     GimbalRef.pitch_angle_dynamic_ref -= 0.1;
					   }
					   else if(key->v & 0x0004)//a
						  {  
								 ChassisSpeedRef.forward_back_ref = (forward_back_speed ) * FBSpeedRamp.Calc(&FBSpeedRamp)*cos(GMYawEncoder.ecd_angle/180*PI)
								                                    +(forward_back_speed ) * LRSpeedRamp.Calc(&LRSpeedRamp)*sin(GMYawEncoder.ecd_angle/180*PI);
						     ChassisSpeedRef.left_right_ref = -(forward_back_speed  ) * FBSpeedRamp.Calc(&FBSpeedRamp)*sin(GMYawEncoder.ecd_angle/180*PI)
								                                   +(forward_back_speed ) * LRSpeedRamp.Calc(&LRSpeedRamp)*cos(GMYawEncoder.ecd_angle/180*PI);
						  }
							else if(key->v & 0x0008)//d
							{
							   ChassisSpeedRef.forward_back_ref = (forward_back_speed ) * FBSpeedRamp.Calc(&FBSpeedRamp)*cos(GMYawEncoder.ecd_angle/180*PI)
								                                    -(forward_back_speed ) * LRSpeedRamp.Calc(&LRSpeedRamp)*sin(GMYawEncoder.ecd_angle/180*PI);
						     ChassisSpeedRef.left_right_ref = -(forward_back_speed  ) * FBSpeedRamp.Calc(&FBSpeedRamp)*sin(GMYawEncoder.ecd_angle/180*PI)
								                                   -(forward_back_speed  ) * LRSpeedRamp.Calc(&LRSpeedRamp)*cos(GMYawEncoder.ecd_angle/180*PI);
							}
							else//s
					   {
						     ChassisSpeedRef.forward_back_ref   = (forward_back_speed ) * FBSpeedRamp.Calc(&FBSpeedRamp)*cos(GMYawEncoder.ecd_angle/180*PI);
				         ChassisSpeedRef.left_right_ref = -(forward_back_speed  ) * FBSpeedRamp.Calc(&FBSpeedRamp)*sin(GMYawEncoder.ecd_angle/180*PI); 
						 }
				   }
				
				   else if(key->v & 0x0004)  // key: a 左平移
				   {
					   if(key->v & 0x4000)
					   {
						     GimbalRef.yaw_angle_dynamic_ref += 0.1;
					   }
					   else if(key->v & 0x0001)//w
						  {  
								 ChassisSpeedRef.forward_back_ref = -(left_right_speed ) * FBSpeedRamp.Calc(&FBSpeedRamp)*cos(GMYawEncoder.ecd_angle/180*PI)
								                                    +(left_right_speed ) * LRSpeedRamp.Calc(&LRSpeedRamp)*sin(GMYawEncoder.ecd_angle/180*PI);
						     ChassisSpeedRef.left_right_ref = (left_right_speed  ) * FBSpeedRamp.Calc(&FBSpeedRamp)*sin(GMYawEncoder.ecd_angle/180*PI)
								                                   +(left_right_speed ) * LRSpeedRamp.Calc(&LRSpeedRamp)*cos(GMYawEncoder.ecd_angle/180*PI);
						  }
							else if(key->v & 0x0002)//s
							{
							   ChassisSpeedRef.forward_back_ref = (left_right_speed ) * FBSpeedRamp.Calc(&FBSpeedRamp)*cos(GMYawEncoder.ecd_angle/180*PI)
								                                    +(left_right_speed ) * LRSpeedRamp.Calc(&LRSpeedRamp)*sin(GMYawEncoder.ecd_angle/180*PI);
						     ChassisSpeedRef.left_right_ref = -(left_right_speed  ) * FBSpeedRamp.Calc(&FBSpeedRamp)*sin(GMYawEncoder.ecd_angle/180*PI)
								                                   +(left_right_speed ) * LRSpeedRamp.Calc(&LRSpeedRamp)*cos(GMYawEncoder.ecd_angle/180*PI);
							}
					   else //a
					   {				     
						    ChassisSpeedRef.forward_back_ref   = (left_right_speed ) * LRSpeedRamp.Calc(&LRSpeedRamp)*sin(GMYawEncoder.ecd_angle/180*PI);
						    ChassisSpeedRef.left_right_ref = (left_right_speed  ) * LRSpeedRamp.Calc(&LRSpeedRamp)*cos(GMYawEncoder.ecd_angle/180*PI);
						 }
				   }
				   else if(key->v & 0x0008) //key: d 右平移
				   {
					   if(key->v & 0x4000)
					    {
						      GimbalRef.yaw_angle_dynamic_ref -= 0.1;
					    }
							else if(key->v & 0x0001)//w
						  {  
								 ChassisSpeedRef.forward_back_ref = -(left_right_speed) * FBSpeedRamp.Calc(&FBSpeedRamp)*cos(GMYawEncoder.ecd_angle/180*PI)
								                                    -(left_right_speed ) * LRSpeedRamp.Calc(&LRSpeedRamp)*sin(GMYawEncoder.ecd_angle/180*PI);
						     ChassisSpeedRef.left_right_ref = (left_right_speed ) * FBSpeedRamp.Calc(&FBSpeedRamp)*sin(GMYawEncoder.ecd_angle/180*PI)
								                                   -(left_right_speed ) * LRSpeedRamp.Calc(&LRSpeedRamp)*cos(GMYawEncoder.ecd_angle/180*PI);
						  }
							else if(key->v & 0x0002)//s
							{
							   ChassisSpeedRef.forward_back_ref = (left_right_speed) * FBSpeedRamp.Calc(&FBSpeedRamp)*cos(GMYawEncoder.ecd_angle/180*PI)
								                                    -(left_right_speed ) * LRSpeedRamp.Calc(&LRSpeedRamp)*sin(GMYawEncoder.ecd_angle/180*PI);
						     ChassisSpeedRef.left_right_ref = -(left_right_speed  ) * FBSpeedRamp.Calc(&FBSpeedRamp)*sin(GMYawEncoder.ecd_angle/180*PI)
								                                   -(left_right_speed  ) * LRSpeedRamp.Calc(&LRSpeedRamp)*cos(GMYawEncoder.ecd_angle/180*PI);
							}
					   else//d
					    {				
						     ChassisSpeedRef.forward_back_ref   = -(left_right_speed  ) * LRSpeedRamp.Calc(&LRSpeedRamp)*sin(GMYawEncoder.ecd_angle/180*PI);
						     ChassisSpeedRef.left_right_ref = -(left_right_speed  ) * LRSpeedRamp.Calc(&LRSpeedRamp)*cos(GMYawEncoder.ecd_angle/180*PI);
				      }
				   }
				   else
				   {
							ChassisSpeedRef.forward_back_ref = 0;
							ChassisSpeedRef.left_right_ref = 0;
							FBSpeedRamp.ResetCounter(&FBSpeedRamp);
							LRSpeedRamp.ResetCounter(&LRSpeedRamp);
				   }
			}
						
				if(key->v & 0x0040)  // key: q 左旋转 45°
				{
		//			GimbalRef.yaw_angle_dynamic_ref += 0.2;
					Key_State Rotate_Quater_L = KEY_INIT;
				}
				else if(key->v & 0x0080) //key: e 右旋转 45°
				{
		//			GimbalRef.yaw_angle_dynamic_ref += -0.2;
					Key_State Rotate_Quater_R = KEY_INIT;
				}
				else if(xtl.keyCnt)
				{ChassisSpeedRef.rotate_ref = 20;}
				else
				{
					ChassisSpeedRef.rotate_ref = 0;
					RoSpeedRamp.ResetCounter(&RoSpeedRamp);
				}
				
				if(key->v & 0x0800) // key: z，控制舵机
				{
					servoKey.thisState = 1;
				}
				else
				{
					servoKey.thisState = 0;
				}
				
//				if(mouse->press_r == 1)  // 鼠标右键： 猫步模式，左右乱扭
//				{
//					cat_temp = 1;
//				}
//				else
//				{
//					cat_temp = 0;
//				}
				
				if(key->v & 0x0100)//key:r
				{
					heatLoopKey.thisState =1;
				}
				else 
				{
					heatLoopKey.thisState = 0;
				}
				
				if(key->v & 0x0200)  // key: f
				{
					dafuKey.thisState =1;
				}
				else
				{
					dafuKey.thisState =0;
				}

				if(key->v & 0x2000)  // key: C
				{
					cameraKey.thisState =1;
				}
				else
				{
					cameraKey.thisState =0;			
				}
			
				if(key->v & 0x0020 )  // key: ctrl 
				{
					frictionKey.thisState = 1;
				}
				else
				{
					frictionKey.thisState = 0;			
				}			
				
				if(key->v & 0x1000)  // key: X
				{
					predictflag =1;
				}
				else
				{
					predictflag =0;			
				}
				
				if(key->v & 0x8000)  // key: b 
				{
					xtl.thisState = 1;
				} 		
				else
				{ xtl.thisState = 0;
				}
				if(key->v & 0x0200)
				{
				}
				else
				{
				
				if(mouse->y == 0)
				{
					MouseYSpeedRamp.ResetCounter(&MouseYSpeedRamp);
				}
				else
				{
					if(GimbalRef.pitch_angle_dynamic_ref>38)//|| GimbalRef.pitch_angle_dynamic_ref< -5
					{
						if(mouse->y < 0)
						{
							mouse->y = 0;
						}
					}
					else if(GimbalRef.pitch_angle_dynamic_ref< -27)
					{
						if(mouse->y > 0)
						{
							mouse->y = 0;
						}
					}
					GimbalRef.pitch_angle_dynamic_ref += -mouse->y* MOUSE_TO_PITCH_ANGLE_INC_FACT*MouseYSpeedRamp.Calc(&MouseYSpeedRamp);

				}
				if(mouse->x == 0)
				{
					MouseXSpeedRamp.ResetCounter(&MouseXSpeedRamp);
				}
				else
				{
//					if(GMYawEncoder.ecd_xtl_angle < 60 && GMYawEncoder.ecd_xtl_angle > -60)
					//{
					GimbalRef.yaw_angle_dynamic_ref  -= mouse->x* MOUSE_TO_YAW_ANGLE_INC_FACT*MouseXSpeedRamp.Calc(&MouseXSpeedRamp);
					//}
//					else
//					{
//						GimbalRef.yaw_angle_dynamic_ref -= 0;				
//					}
				}
				}
				MouseShootControl(mouse);
				ServoKeyCtrl();//舵机波轮
				CameraKeyCtrl();//小符程序
				DafuKeyCtrl();
				//SendManifoldDataControl();//大符程序
				FrictionKeyCtrl();//摩擦轮
				//SendCustomDataControl();//？？？
				HeatLoopControl();//热量环
				XTLKeyCtrl();
				servoKey.lastState = servoKey.thisState;
				cameraKey.lastState = cameraKey.thisState;	
				dafuKey.lastState = dafuKey.thisState;	
				frictionKey.lastState = frictionKey.thisState;//摩擦轮控制状态更新
				heatLoopKey.lastState = heatLoopKey.thisState;
				sendMainfoldDataKey.lastState = sendMainfoldDataKey.thisState;
				xtl.lastState = xtl.thisState;//xtl
			}
		//限制鼠标移动速度
		VAL_LIMIT(mouse->x, -150, 150); 
		VAL_LIMIT(mouse->y, -150, 150);
	/*  限位  */
		//VAL_LIMIT(GimbalRef.pitch_angle_dynamic_ref, PITCH_MIN, PITCH_MAX);
//   	VAL_LIMIT(CMRotatePID.ref,  -YAW_MAX, YAW_MAX);
}
void ServoKeyCtrl(void)  
{
	if((servoKey.thisState==1)&&(servoKey.lastState==0))
	{
		servoKey.keyCnt++;
	}
	if(servoKey.keyCnt%2==0)
	{
		TIM_SetCompare1(TIM4,1050);
		servoPositionFlag = 0;
	}
	else
	{
		TIM_SetCompare1(TIM4,500);//第一次按下
		servoPositionFlag = 1;
	}
}
void CameraKeyCtrl(void)
{
//	if((cameraKey.thisState ==0)&&(cameraKey.lastState==1))
//	{
//		cameraKey.keyCnt ++;
//		cameraKey.keyFlag = 1;
//	}
//	if(cameraKey.keyCnt%2==1)
//	{		
//		camera_temp =1;
//	}
//	else
//	{
//		camera_temp =0;
//}
}
void DafuKeyCtrl(void)
{
	if((dafuKey.thisState ==0)&&(dafuKey.lastState==1))
	{
		dafuKey.keyCnt ++;
		dafuKey.keyFlag = 1;
	}
	if(dafuKey.keyCnt%2==1)
	{		
		dafu_temp =1;
	}
	else
	{
		dafu_temp =0;
	}
}
void FrictionKeyCtrl(void)//摩擦轮
{
	if((frictionKey.thisState == 0) && (frictionKey.lastState == 1))
	{
		frictionKey.keyCnt ++;
	}
	if(frictionKey.keyCnt % 3 == 1)
	{
		quickShootFlag = 2;
		frictionwheelmotor(60,60);//高速射击
		friction_wheel_state=FRICTION_WHEEL_ON;
		
	}
	else if(frictionKey.keyCnt % 3 == 2)
	{
		frictionwheelmotor(30,30);//低速射击
		quickShootFlag = 1;
		friction_wheel_state=FRICTION_WHEEL_ON;
	}
	else if(dafu_temp == 1&&flag_Dafu_fromsight==1)		//大符模式超高速摩擦轮
	{
//		frictionwheelmotor(400,400);
		//frictionwheelmotor(0,0);
		//sendCustomDataFlag = 1;
		quickShootFlag = 3;					//大符射击
		friction_wheel_state=FRICTION_WHEEL_ON;
	}
	else
	{
		quickShootFlag = 0;
		frictionwheelmotor(0,0);
		friction_wheel_state=FRICTION_WHEEL_OFF;
	}
}
void XTLKeyCtrl(void)  //xtl
{
		if((xtl.thisState == 0) && (xtl.lastState == 1))
			{
				   xtl.keyCnt = !(xtl.keyCnt);
			}
			if(xtl.keyCnt)
			{
				   xtl_state = XTL_ON;
				   ChassisSpeedRef.rotate_ref=100;
				   CM1SpeedPID.ref =  -ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref;
		       CM2SpeedPID.ref =  ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref;
					 CM3SpeedPID.ref =  ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref;
					 CM4SpeedPID.ref =  -ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref;
			}
			else
				   xtl_state = XTL_OFF;
}


void SendManifoldDataControl(void)
{
	if((sendMainfoldDataKey.thisState ==0)&&(sendMainfoldDataKey.lastState==1))
	{
		sendMainfoldDataKey.keyCnt ++;
	}
	if(sendMainfoldDataKey.keyCnt%2==1)
	{
	}
	else
	{
	}
}
void SendCustomDataControl(void)
{
	//if(sendCustomDataFlag == 1)
	{
		user_data_handle();//改变自定义数据
		data_pack_handle(0x0301,user_data,19);//用户自定义数据打包(命令码+数据+长度)
		UART3_PrintBlock(computer_tx_buf,28);
		//sendCustomDataFlag = 1;
	}
}
void HeatLoopControl(void)
{
	if((heatLoopKey.thisState ==0)&&(heatLoopKey.lastState==1))
	{
		heatLoopKey.keyCnt ++;
	}
	if(heatLoopKey.keyCnt%2==1)
	{
		heatLoopKey.keyFlag = 1;
	}
	else
	{
		heatLoopKey.keyFlag = 0;
	}
}
