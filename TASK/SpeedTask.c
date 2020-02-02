#include "main.h"
#define PI 3.1415926
float pitchCameraOffset = 2.0;
float yawCameraOffset = 8.0;
float lastYawAngle = 0.0f;
float lastpitchangle = 0.0f;
float preparepitchangle = 0.0f;
S1MODE lasts1_state = NOINPUT;
distance_mode distance_choice = normal_distance;
uint16_t rotateCnt = 0;
float cnt1 = 0;
float cnt2 = 0;

int count1=1;
int no_count=0;
//xtlState_e xtl_state=XTL_OFF;
float now_eular =0;

void setdistancemode(void)
{
	  if(GetWorkState() == CAMERA_STATE)
			distance_choice = camera_distance;

		else if(GetWorkState() == CATWALK_STATE)
			distance_choice = catwalk;
		else if(GetS1InputMode() == ZIMIAO)
		{   
      if(recognize_flag ==1)
			{
			  distance_choice = near_distance;
			}
			else
			{
			  distance_choice = lost_distance;
			}
	  }
		else
			distance_choice = normal_distance;
}

distance_mode getdistancemode(void)
{
	return distance_choice;
}

int first_rec_count=0; 
void PIDparameterchange(void)
{
	switch (getdistancemode())
	{
		case near_distance:
		{
		  GMYPositionPID.kp =130;	
     if(fabs(camera_yaw)>25 || first_rec_count != 0)//)
	   {
				first_rec_count++;
				GMYPositionPID.kp = 40 ;
				if(fabs(camera_yaw)<1)first_rec_count = 0;
				
			}
			
			GMYPositionPID.ki = 0 ;
			GMYPositionPID.kd = 1.3 ;
			GMYPositionPID.outputMax=20000;

        GMYSpeedPID.kp = 500 ;//100
			if(fabs(camera_yaw)>25 || (first_rec_count != 0))//
	   {
				first_rec_count++;
				GMYSpeedPID.kp =300 ;//100
				if(fabs(camera_yaw)<1)first_rec_count = 0;
				
			}
				GMYSpeedPID.ki = 0;
        GMYSpeedPID.kd = 4;//50
			GMYSpeedPID.outputMax=25000;
			
	   
			
			GMPPositionPID.kp = 200;//100
	    GMPPositionPID.ki = 0;//0.15+pitemp01;
	    GMPPositionPID.kd = 0;
	
	    GMPSpeedPID.kp = 60;//40
	    GMPSpeedPID.ki = 0.0015;//0.001+sitemp01;
	    GMPSpeedPID.kd = 2;
//			if(GetWorkState() == CATWALK_STATE)
//			{
//				CMRotatePID.kp = 8;
//				CMRotatePID.ki = 0;
//				CMRotatePID.kd = 0;
//							
//				CM1SpeedPID.kp = CM2SpeedPID.kp = CM3SpeedPID.kp =CM4SpeedPID.kp = 5;
//				CM1SpeedPID.ki = CM2SpeedPID.ki = CM3SpeedPID.ki = CM4SpeedPID.ki = 0;
//				CM1SpeedPID.kd = CM2SpeedPID.kd = CM3SpeedPID.kd = CM4SpeedPID.kd = 5;
//			}
//			else
			{
				CMRotatePID.kp = 10;
				CMRotatePID.ki = 0;
				CMRotatePID.kd = 0.5;
							
				CM1SpeedPID.kp = CM2SpeedPID.kp = CM3SpeedPID.kp =CM4SpeedPID.kp = 25;
				CM1SpeedPID.ki = CM2SpeedPID.ki = CM3SpeedPID.ki = CM4SpeedPID.ki = 0;
				CM1SpeedPID.kd = CM2SpeedPID.kd = CM3SpeedPID.kd = CM4SpeedPID.kd = 15;
			}
		}break;
	
		case lost_distance:
		{GMYPositionPID.kp = 100;
	    GMYPositionPID.ki =	0;//0.02+pitemp00;
	    GMYPositionPID.kd = 0;
	
	    GMYSpeedPID.kp = 400;
	    GMYSpeedPID.ki = 0;//0.002+sitemp00;
	    GMYSpeedPID.kd = 3;
			
			GMPPositionPID.kp = 160;
	    GMPPositionPID.ki = 0;//0.15+pitemp01;
	    GMPPositionPID.kd = 0;
	
	    GMPSpeedPID.kp = 60;
	    GMPSpeedPID.ki = 0;//0.001+sitemp01;
	    GMPSpeedPID.kd = 0.4;
			
			CMRotatePID.kp = 10;
		  CMRotatePID.ki = 0;
		  CMRotatePID.kd = 3;
						
			CM1SpeedPID.kp = CM2SpeedPID.kp = CM3SpeedPID.kp =CM4SpeedPID.kp = 10;
		  CM1SpeedPID.ki = CM2SpeedPID.ki = CM3SpeedPID.ki = CM4SpeedPID.ki = 0;
		  CM1SpeedPID.kd = CM2SpeedPID.kd = CM3SpeedPID.kd = CM4SpeedPID.kd = 10;
		
		}break;
		
		case catwalk:
		{
			GMYPositionPID.kp = 130;
	    GMYPositionPID.ki =	0;//0.02+pitemp00;
	    GMYPositionPID.kd = 7;
	
	    GMYSpeedPID.kp = 400;
	    GMYSpeedPID.ki = 0.;//0.002+sitemp00;
	    GMYSpeedPID.kd = 10;
			
			GMPPositionPID.kp = 120;
	    GMPPositionPID.ki = 0;//0.15+pitemp01;
	    GMPPositionPID.kd = 1;
	
	    GMPSpeedPID.kp = 60;
	    GMPSpeedPID.ki = 0.015;//0.001+sitemp01;
	    GMPSpeedPID.kd = 1.3;
			
			CMRotatePID.kp = 8;
		  CMRotatePID.ki = 0;
		  CMRotatePID.kd = 0;
						
			CM1SpeedPID.kp = CM2SpeedPID.kp = CM3SpeedPID.kp =CM4SpeedPID.kp = 5;
		  CM1SpeedPID.ki = CM2SpeedPID.ki = CM3SpeedPID.ki = CM4SpeedPID.ki = 0;
		  CM1SpeedPID.kd = CM2SpeedPID.kd = CM3SpeedPID.kd = CM4SpeedPID.kd = 5;
		}break;
		
		case normal_distance:
		{
//			GMYPositionPID.kp = 200;
//	    GMYPositionPID.ki =	0.2;//0.02+pitemp00;
//	    GMYPositionPID.kd = 10;
			GMYPositionPID.outputMax=20000;
			
	
	    GMYSpeedPID.kp = 150;//160;
	    GMYSpeedPID.ki = 0;//0.002+sitemp00;
	    GMYSpeedPID.kd = 0;
			GMYSpeedPID.outputMax=22000;
			
			
//			GMPPositionPID.kp = 280;
//	    GMPPositionPID.ki = 0;//0.15+pitemp01;
//	    GMPPositionPID.kd = 10;
	
//	    GMPSpeedPID.kp = 80;
//	    GMPSpeedPID.ki = 0;//0.001+sitemp01;
//	    GMPSpeedPID.kd = 2;
			GMPSpeedPID.outputMax=4800;
			CMRotatePID.kp = 10;
		  CMRotatePID.ki = 0;
		  CMRotatePID.kd = 2;
						
			CM1SpeedPID.kp = CM2SpeedPID.kp = CM3SpeedPID.kp = CM4SpeedPID.kp = 20;
		  CM1SpeedPID.ki = CM2SpeedPID.ki = CM3SpeedPID.ki = CM4SpeedPID.ki = 0;
		  CM1SpeedPID.kd = CM2SpeedPID.kd = CM3SpeedPID.kd = CM4SpeedPID.kd = 6;
			
		}break;
		case camera_distance:
		{
			GMYPositionPID.kp = 13;//25;//70;
	    GMYPositionPID.ki =	0.1;//0.02+pitemp00;
	    GMYPositionPID.kd = 3;//3
			GMYPositionPID.outputMax=20000;
			
	    GMYSpeedPID.kp = 150;//300//400;
	    GMYSpeedPID.ki = 3;//3//0.002+sitemp00;
	    GMYSpeedPID.kd = 3;//3
			GMYSpeedPID.outputMax=25000;
			
			GMPPositionPID.kp = 25;//100;//200;//300;
	    GMPPositionPID.ki = 0.1;//0.15+pitemp01;
	    GMPPositionPID.kd = 1;
	
	    GMPSpeedPID.kp = 50;
	    GMPSpeedPID.ki = 0.01;//0.001+sitemp01;
	    GMPSpeedPID.kd = 1;
		}break;
	}
		
}

/**
函数：GimbalYawControlModeSwitch()
功能：云台模式控制
**/
float temp_yaw=0;
float temp_GYROYAW=0;
float temp_GYROPITCH=0;
void GimbalYawControlModeSwitch(void)
{temp_yaw=Eular[2];
	PIDparameterchange();
	switch(GetWorkState())
	{
		case PREPARE_STATE:                                                                                                                                                                                                                                                
		{ 
			GMYPositionPID.kp = 200;
	    GMYPositionPID.ki =	0.2;//0.02+pitemp00;
	    GMYPositionPID.kd = 10;
			GMYPositionPID.ref = 0;
			GMYPositionPID.fdb = GMYawEncoder.ecd_xtl_angle*GMYawRamp.Calc(&GMYawRamp);
			GMYPositionPID.Calc(&GMYPositionPID);
			GimbalRef.yaw_angle_dynamic_ref_add = Eular[2];//rm_imu_data.newyaw;//Eular[2];//remeber gyro yaw value now
			preparepitchangle = Eular[0];//remeber gyro pitch value now
			
			GMPPositionPID.ref = 0;
				
//	    VAL_LIMIT(GMPPositionPID.ref,-20,20);
		  GMPPositionPID.fdb = GMPitchEncoder.ecd_angle*GMPitchRamp.Calc(&GMPitchRamp);//GMPitchEncoder.ecd_angle * GMPitchRamp.Calc(&GMPitchRamp);    //加入斜坡函数
		  GMPPositionPID.kp = 280;
	    GMPPositionPID.ki = 0;//0.15+pitemp01;
	    GMPPositionPID.kd = 1;
			GMPPositionPID.Calc(&GMPPositionPID);
			
			GMPSpeedPID.kp = 50;
	    GMPSpeedPID.ki = 0;//0.001+sitemp01;
	    GMPSpeedPID.kd = 2;
			
		}break;
		
		case CAMERA_STATE:
		{ //	camera_yaw=-camera_yaw;
			//	camera_pitch=-camer   a_pitch;
				{//yaw轴
						VAL_LIMIT(camera_yaw,-30,30);			
						if((camera_yaw*8192/360)>=((int)(camera_yaw*8192/360)+0.5))
							GMYPositionPID.ref=(int)(camera_yaw*8192/360)+1;
						else
							GMYPositionPID.ref=(int)(camera_yaw*8192/360);
						//GMYPositionPID.ref = camera_yaw*8192/360//camera_yaw*4;
						GMYPositionPID.fdb = (GMYawEncoder.raw_value-4049)/**360/8192*/;//GMYawEncoder.ecd_angle*4;//3400，原没有360/8192
						GMYPositionPID.Calc(&GMYPositionPID);
			  }
						
				{//p轴
		  		VAL_LIMIT(camera_pitch,-27,27);		
						//GMPPositionPID.ref = camera_pitch*2;
						if((camera_pitch*8192/360)>=((int)(camera_pitch*8192/360)+0.5))
							GMPPositionPID.ref=(int)(camera_pitch*8192/360)+1;
						else
							GMPPositionPID.ref=(int)(camera_pitch*8192/360);
						GMPPositionPID.fdb = (GMPitchEncoder.raw_value-1815)*0.7105637/**360/8192*/;//4900,原没有360/8192//(GMPitchEncoder.ecd_angle-0.18)*0.7105637*2;//9/4;// -Eular[0]*6;//-GMPitchEncoder.ecd_angle;
						GMPPositionPID.Calc(&GMPPositionPID);
				}
		}break;
		case NORMAL_STATE:
		{   
			//no_count=0;
			if(GetS1InputMode() == ZIMIAO  )
			{
				if(recognize_flag == 1)
				{
					  no_count=0;
					{//yaw轴
					
					  GMYPositionPID.ref = 0 ;//Eular[2]-camera_yaw
					  GMYPositionPID.fdb = camera_yaw-0.5;//160 16 Eular[2]
					  GMYPositionPID.Calc(&GMYPositionPID);
					}
					
					{//p轴
						
				    GMPPositionPID.ref =  0;//(GMPitchEncoder.ecd_angle+camera_pitch)(Eular[0]-camera_pitch)
				    GMPPositionPID.fdb = -camera_pitch;//GMPitchEncoder.ecd_angle*GMPitchRamp.Calc(&GMPitchRamp)//Eular[0]
				    GMPPositionPID.Calc(&GMPPositionPID);
						
					}
					
					{//底盘
						CMRotatePID.ref = 0;
						CMRotatePID.fdb = GMYawEncoder.ecd_angle*GMYawRamp.Calc(&GMYawRamp);

		        CMRotatePID.Calc(&CMRotatePID);
	          ChassisSpeedRef.rotate_ref = CMRotatePID.output;
					}
					
				}
			 if(recognize_flag == 0)
				{
//					count1 = 0;
					no_count++;
					
					{//yaw
					  if(no_count==1)lastYawAngle = Eular[2];
					
//					  VAL_LIMIT(lastYawAngle,-30,30);
					  GMYPositionPID.ref = lastYawAngle;
				    GMYPositionPID.fdb = Eular[2];//(-Eular[2]-GimbalRef.yaw_angle_dynamic_ref_add)
					
				    GMYPositionPID.Calc(&GMYPositionPID);
					}
					
					{//pitch
//						VAL_LIMIT(GMPPositionPID.ref,-27,38);
						if(no_count==1)lastpitchangle=GMPitchEncoder.ecd_angle;
					  
					  GMPPositionPID.ref = lastpitchangle ;
				    GMPPositionPID.fdb =  GMPitchEncoder.ecd_angle;//(Eular[0]-preparepitchangle)
						
				    GMPPositionPID.Calc(&GMPPositionPID);
					}
					{//底盘
						
				    CMRotatePID.ref = 0;
	          CMRotatePID.fdb = GMYawEncoder.ecd_angle*GMYawRamp.Calc(&GMYawRamp);
		        CMRotatePID.Calc(&CMRotatePID);
	          ChassisSpeedRef.rotate_ref = CMRotatePID.output;
					}
					
				}
				GimbalRef.yaw_angle_dynamic_ref = Eular[2]-GimbalRef.yaw_angle_dynamic_ref_add;
				GimbalRef.pitch_angle_dynamic_ref = GMPitchEncoder.ecd_angle;
			}	
     else
			{	
         no_count = 0;  				
				{//yaw
					//if(friction_wheel_state == FRICTION_WHEEL_OFF)
					//{
          GMYPositionPID.ref = GimbalRef.yaw_angle_dynamic_ref + GimbalRef.yaw_angle_dynamic_ref_add;
					//GMYPositionPID.ref = 0;
					//GMYPositionPID.fdb = GMYawEncoder.ecd_angle ;
			    GMYPositionPID.fdb = Eular[2];//rm_imu_data.newyaw;//Eular[2];//rm_imu_data.newyaw
					//GMYPositionPID.error=GMYPositionPID.err[0];
					//}
//			VAL_LIMIT(GMYPositionPID.ref,-60,60);
//			GMYPositionPID.kp = 350;
//	    GMYPositionPID.ki =	0.2;//0.02+pitemp00;
//	    GMYPositionPID.kd = 10;

//	PID_realize(&GMYPositionPID,21,7,GMYP_rule_kp,200,150,GMYP_rule_ki,0.18,0.18,GMYP_rule_kd,12,12);
					//GMYPositionPID.ki =	0;
			    GMYPositionPID.Calc(&GMYPositionPID);
				}
				
				{//pitch
					GMPPositionPID.ref = GimbalRef.pitch_angle_dynamic_ref;// + last_camera_pitch;
				  GMPPositionPID.error=GMPPositionPID.err[0];
					GMPPositionPID.ec=GMPPositionPID.err[0]-GMPPositionPID.err[1];
	       //VAL_LIMIT(GMPPositionPID.ref,-8,45);
		      GMPPositionPID.fdb = GMPitchEncoder.ecd_angle*GMPitchRamp.Calc(&GMPitchRamp);//GMPitchEncoder.ecd_angle * GMPitchRamp.Calc(&GMPitchRamp);    //加入斜坡函数
		     
//				PID_realize(&GMPPositionPID,8.8,2.5,GMPP_rule_kp,280,80,GMPP_rule_kp,0.03,0.03,GMPP_rule_kp,0,0);
					GMPPositionPID.Calc(&GMPPositionPID);   //得到pitch轴位置环输出控制量
					//PID_realize(&GMPSpeedPID,7,2.5,GMPP_rule_kp,50,30,GMPP_rule_ki,0,0,GMPP_rule_kd,0,0);
				}
				{//底盘


//		      VAL_LIMIT(GMYawEncoder.ecd_angle,-30,30);
					
		      CMRotatePID.ref =0 ;//-GimbalRef.yaw_angle_dynamic_ref +Eular[2]- GimbalRef.yaw_angle_dynamic_ref_add

	        CMRotatePID.fdb = GMYawEncoder.ecd_xtl_angle*GMYawRamp.Calc(&GMPitchRamp);
		      CMRotatePID.Calc(&CMRotatePID);
					if(friction_wheel_state == FRICTION_WHEEL_OFF)
					{
						  if(xtl_state == XTL_OFF)
							{
						   VAL_LIMIT(CMRotatePID.output,-100,100);
	             ChassisSpeedRef.rotate_ref = CMRotatePID.output;
							}
					}
				}
			}			
		}break;
		case CATWALK_STATE:       //猫步模式
	  {
			if(GetS1InputMode() == ZIMIAO  )
			{
				if(recognize_flag == 1)
				{
					  no_count=0;
					{//yaw轴
					
					  GMYPositionPID.ref = 0 ;//Eular[2]-camera_yaw
					  GMYPositionPID.fdb = camera_yaw-0.5-0.3*sin(GMYawEncoder.ecd_angle*PI/180);//160 16 Eular[2]
					  GMYPositionPID.Calc(&GMYPositionPID);
					}
					
					{//p轴
						
				    GMPPositionPID.ref =  0;//(GMPitchEncoder.ecd_angle+camera_pitch)(Eular[0]-camera_pitch)
				    GMPPositionPID.fdb = -camera_pitch;//GMPitchEncoder.ecd_angle*GMPitchRamp.Calc(&GMPitchRamp)//Eular[0]
				    GMPPositionPID.Calc(&GMPPositionPID);
						
					}
					/*
					{//底盘
						CMRotatePID.ref = 0;
						CMRotatePID.fdb = GMYawEncoder.ecd_angle*GMYawRamp.Calc(&GMYawRamp);
//						VAL_LIMIT(CMRotatePID.fdb ,-30,30);

		            CMRotatePID.Calc(&CMRotatePID);
	              ChassisSpeedRef.rotate_ref = CMRotatePID.output;
					   }
					*/
				  }
			 if(recognize_flag == 0)
				{
//					count1 = 0;
					no_count++;
					
					{//yaw
					  if(no_count==1)lastYawAngle = Eular[2];
					
//					  VAL_LIMIT(lastYawAngle,-30,30);
					  GMYPositionPID.ref = lastYawAngle;
				    GMYPositionPID.fdb = Eular[2];//(-Eular[2]-GimbalRef.yaw_angle_dynamic_ref_add)
					
				    GMYPositionPID.Calc(&GMYPositionPID);
					}
					
					{//pitch
//						VAL_LIMIT(GMPPositionPID.ref,-27,38);
						if(no_count==1)lastpitchangle=GMPitchEncoder.ecd_angle;
					  
					  GMPPositionPID.ref = lastpitchangle ;
				    GMPPositionPID.fdb = GMPitchEncoder.ecd_angle;//(Eular[0]-preparepitchangle)
						
				    GMPPositionPID.Calc(&GMPPositionPID);
					}
					/*
					{//底盘
						
				    CMRotatePID.ref = 0;
	          CMRotatePID.fdb = GMYawEncoder.ecd_angle*GMYawRamp.Calc(&GMYawRamp);
		        CMRotatePID.Calc(&CMRotatePID);
	          ChassisSpeedRef.rotate_ref = CMRotatePID.output;
					}
					*/
				}
				GimbalRef.yaw_angle_dynamic_ref = Eular[2]-GimbalRef.yaw_angle_dynamic_ref_add;
				GimbalRef.pitch_angle_dynamic_ref = GMPitchEncoder.ecd_angle;
			}	
     else
			{				
				{//yaw
          GMYPositionPID.ref = GimbalRef.yaw_angle_dynamic_ref + GimbalRef.yaw_angle_dynamic_ref_add;
			    GMYPositionPID.fdb = Eular[2];
//			VAL_LIMIT(GMYPositionPID.ref,-60,60);
			    GMYPositionPID.Calc(&GMYPositionPID);
				}
				
				{//pitch
					GMPPositionPID.ref = GimbalRef.pitch_angle_dynamic_ref;// + last_camera_pitch;
	       //VAL_LIMIT(GMPPositionPID.ref,-8,45);
		      GMPPositionPID.fdb = GMPitchEncoder.ecd_angle*GMPitchRamp.Calc(&GMPitchRamp);//GMPitchEncoder.ecd_angle * GMPitchRamp.Calc(&GMPitchRamp);    //加入斜坡函数
		      GMPPositionPID.Calc(&GMPPositionPID);   //得到pitch轴位置环输出控制量
				}
		}
		default:
		{
		}break;
	}
}
}
/**
函数：GMYawControlLoop()
功能：利用PID计算出云台电机的输出量，Y轴
**/

float last_ref=0.0;
void GMYawControlLoop(void)
{
	GimbalYawControlModeSwitch();
	{
	  GMYSpeedPID.ref = GMYPositionPID.output/20;//* GMYawRamp.Calc(&GMPitchRamp)
	  GMYSpeedPID.fdb = Gyro[2]/20;//rm_imu_data.gyro_yaw/35;//Gyro[2]/20;//rm_imu_data.gyro_yaw
	  
		GMYSpeedPID.Calc(&GMYSpeedPID);
		//VAL_LIMIT(GMYSpeedPID.output,-20000,20000);
		temp_GYROYAW=Gyro[2];
	}
	{
	  GMPSpeedPID.ref = GMPPositionPID.output/30;
	  GMPSpeedPID.fdb = Gyro[1]/30;//rm_imu_data.gyro_pitch/30;//Gyro[1]/30;//rm_imu_data.gyro_pitch
	  GMPSpeedPID.Calc(&GMPSpeedPID);
	  temp_GYROPITCH=Gyro[1];
	}	
}

/**
函数：GMPitchControlLoop()
功能：利用PID计算出云台电机的输出量，P轴
**/
//云台pitch轴控制程序
void GMPitchControlLoop(void)
{
/***************************新版本*************************************/	
//	if(GetWorkState() == NORMAL_STATE)
//	{
//		

//		if(GetInputMode() == REMOTE_INPUT && GetS1InputMode() == ZIMIAO )
//			{
//				
//				frictionwheelmotor(200,200);
//				
//				shot_frequency_limt.time_now = Get_Time_Micros();     //TIM2的计数器值（32位不会溢出）
//				shot_frequency_limt.time_error = shot_frequency_limt.time_now - shot_frequency_limt.time_last;
//				if(shot_frequency_limt.time_error > 200000)
//				{
//					count_temp ++;
//					shot_frequency_limt.time_last = Get_Time_Micros();  
//				}		
//			}
//			else (frictionwheelmotor(0,0));
//	}
	
}


/**
函数：SetGimbalMotorOutput()
功能：通过can将云台电机的输出量传给电机
**/
void SetGimbalMotorOutput(void)
{
	//云台控制输出								
	if((GetWorkState() == STOP_STATE ) )//|| GetWorkState() == PREPARE_STATE)  || (remote_error_time>200) 
	{
		Set_Gimbal_Current(CAN1, 0, 0, 0);     //yaw + pitch	+M7		
	}
	else Set_Gimbal_Current(CAN1, /*0*/(int16_t)(GMYSpeedPID.output),/*0*/(int16_t)(GMPSpeedPID.output),(int16_t)(ShootMotorSpeedPID.output));
  
}
/** 
函数：CMControlLoop()
功能：底盘控制任务，输出底盘电机电流
**/
	int32_t round_count = 0;
	int8_t count_temp1 =0;
void CMControlLoop(void)
{  

	//底盘旋转量计算                  
	if(GetWorkState() == PREPARE_STATE || GetWorkState() == CAMERA_STATE || GetWorkState() == STOP_STATE) //底盘不动
	{
		CM1SpeedPID.ref = 0;
		CM2SpeedPID.ref = 0;
		CM3SpeedPID.ref = 0;
		CM4SpeedPID.ref = 0; 
		
	}
	else if((GetWorkState() == NORMAL_STATE))
	{
		cnt1=0;
		VAL_LIMIT(ChassisSpeedRef.rotate_ref,-800,800);
//		ChassisSpeedRef.rotate_ref = (ChassisSpeedRef.rotate_ref + 0.625*Speed_Offset.output);

//		ChassisSpeedRef.forward_back_ref = ChassisSpeedRef.forward_back_ref/10;
//		ChassisSpeedRef.left_right_ref = ChassisSpeedRef.left_right_ref/10;
//		ChassisSpeedRef.rotate_ref = ChassisSpeedRef.rotate_ref/10;
		
//		if(fabs(GMYawEncoder.ecd_angle)>35) 
//		VAL_LIMIT(ChassisSpeedRef.rotate_ref,-450,450);//350
		
		CM1SpeedPID.ref =  -ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref;
		CM2SpeedPID.ref =  ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref;
		CM3SpeedPID.ref =  ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref;
		CM4SpeedPID.ref =  -ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref;
		
		
		
	}
	else if(GetWorkState() == ADJUST_STATE)
	{
		CM1SpeedPID.ref =  0;
		CM2SpeedPID.ref = 0;
		CM3SpeedPID.ref = 0;
		CM4SpeedPID.ref =  0; 
	}
	else if(GetWorkState() == CATWALK_STATE)
	{
		
		
		if(cnt2 == 0 )cnt1+=0.005;
		if(cnt2 == 1)cnt1-=0.005;
		if(cnt2 == 2)cnt1+=0.005;
		if(cnt1 == 2 )cnt2 =1;
		if(cnt1 == 0)cnt2 =2;
		
		CMRotatePID.ref = 45+250*sin(PI*cnt1);//打哨兵时底盘和云台存在一定角度后扭腰
		CMRotatePID.fdb = GMYawEncoder.ecd_xtl_angle  ;
		CMRotatePID.Calc(&CMRotatePID);
		ChassisSpeedRef.rotate_ref = CMRotatePID.output;
		VAL_LIMIT(ChassisSpeedRef.rotate_ref,-800,800)
		
		if(GetS1InputMode() == ZIMIAO)
		{
		CM1SpeedPID.ref = -ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref +0.5*ChassisSpeedRef.rotate_ref;;//0.3*ChassisSpeedRef.rotate_ref;
		CM2SpeedPID.ref = ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref +0.5*ChassisSpeedRef.rotate_ref;//0.3*ChassisSpeedRef.rotate_ref;
		CM3SpeedPID.ref = ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref +0.5*ChassisSpeedRef.rotate_ref;//0.3*ChassisSpeedRef.rotate_ref;
		CM4SpeedPID.ref = -ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref +0.6*ChassisSpeedRef.rotate_ref;//0.35*ChassisSpeedRef.rotate_ref; 
		}
		else
		{
		CM1SpeedPID.ref = -0.8*ChassisSpeedRef.forward_back_ref - 0.8*ChassisSpeedRef.left_right_ref +0.5*ChassisSpeedRef.rotate_ref;
		CM2SpeedPID.ref = 0.8*ChassisSpeedRef.forward_back_ref - 0.8*ChassisSpeedRef.left_right_ref +0.5*ChassisSpeedRef.rotate_ref;
		CM3SpeedPID.ref = 0.8*ChassisSpeedRef.forward_back_ref + 0.8*ChassisSpeedRef.left_right_ref + 0.5*ChassisSpeedRef.rotate_ref;
		CM4SpeedPID.ref = -0.8*ChassisSpeedRef.forward_back_ref + 0.8*ChassisSpeedRef.left_right_ref +0.5*ChassisSpeedRef.rotate_ref;
		}
	}
	
	else
	{
	}
	
	
//    if(GetS1InputMode() == ZIMIAO && recognize_flag == 1)
//		{
//		  CM1SpeedPID.fdb = -CM1Encoder.filter_rate;
//		  CM2SpeedPID.fdb = CM2Encoder.filter_rate;
//		  CM3SpeedPID.fdb = CM3Encoder.filter_rate;
//		  CM4SpeedPID.fdb = CM4Encoder.filter_rate;
//		}
//		else
//		{
	
	
	CM1SpeedPID.fdb = CM1Encoder.filter_rate;
	CM2SpeedPID.fdb = CM2Encoder.filter_rate;
	CM3SpeedPID.fdb = CM3Encoder.filter_rate;
	CM4SpeedPID.fdb = CM4Encoder.filter_rate;
//		}
	if(continuous_shoot_Flag ==0 )
	{
    VAL_LIMIT(CM1SpeedPID.ref,-340,340);//全是340
		VAL_LIMIT(CM2SpeedPID.ref,-340,340);
		VAL_LIMIT(CM3SpeedPID.ref,-340,340);
		VAL_LIMIT(CM4SpeedPID.ref,-340,340);	
	}
		else 
		{
		VAL_LIMIT(CM1SpeedPID.ref,-430,430);//全是430
		VAL_LIMIT(CM2SpeedPID.ref,-430,430);
		VAL_LIMIT(CM3SpeedPID.ref,-430,430);
		VAL_LIMIT(CM4SpeedPID.ref,-430,430);
		}
	
		CM1SpeedPID.Calc(&CM1SpeedPID);
		CM2SpeedPID.Calc(&CM2SpeedPID);
		CM3SpeedPID.Calc(&CM3SpeedPID);
		CM4SpeedPID.Calc(&CM4SpeedPID);
	  
	
	 if((GetWorkState() == STOP_STATE)||  GetWorkState() == PREPARE_STATE )   //|| (remote_error_time>200)|| GetWorkState() == CAMERA_STATE|| dead_lock_flag == 1紧急停车，编码器校准，无控制输入时都会使底盘控制停止
	 {
		 Set_CM_Speed(CAN1, 0,0,0,0);
	 }
	 else if(GetS1InputMode() == ZIMIAO && recognize_flag == 1)
	 {
		 Set_CM_Speed(CAN1, 1 * CM1SpeedPID.output, 1 * CM2SpeedPID.output, 1 * CM3SpeedPID.output,1  * CM4SpeedPID.output);		  
	 }
	 else Set_CM_Speed(CAN1, CHASSIS_SPEED_ATTENUATION * CM1SpeedPID.output, CHASSIS_SPEED_ATTENUATION * CM2SpeedPID.output, CHASSIS_SPEED_ATTENUATION * CM3SpeedPID.output,CHASSIS_SPEED_ATTENUATION * CM4SpeedPID.output);		  


}

void frictionwheelmotor(float speed1,float speed2)
{
   
		CAN2_CM1SpeedPID.kp=25;
	  CAN2_CM1SpeedPID.ki=0;
	  CAN2_CM1SpeedPID.kd=0;
	  CAN2_CM1SpeedPID.ref = speed1;
	  CAN2_CM2SpeedPID.kp=25;
	  CAN2_CM2SpeedPID.ki=0;
	  CAN2_CM2SpeedPID.kd=0;
	  CAN2_CM2SpeedPID.ref = -speed2;
	  CAN2_CM1SpeedPID.fdb = CAN2_CM1Encoder.filter_rate;
		CAN2_CM2SpeedPID.fdb = CAN2_CM2Encoder.filter_rate;
		
		
		CAN2_CM1SpeedPID.Calc(&CAN2_CM1SpeedPID);
		CAN2_CM2SpeedPID.Calc(&CAN2_CM2SpeedPID);
		
    Set_CM_Speed(CAN2,CAN2_CM1SpeedPID.output, CAN2_CM2SpeedPID.output,0,0);	

}

/*
函数：ShooterMControlLoop()	
功能：发射机构射击电机任务
*/
int16_t count_temp = 0;
void ShooterMControlLoop(void)
{
	if(GetWorkState() != PREPARE_STATE)//GetS1InputMode() == ZIMIAO || GetS1InputMode() == shoot
	{
		/*--------- 2017-9-7 位置曲线较好,速度曲线可以再调整 --------------*/
		ShootMotorPositionPID.kp = 20.0;
		ShootMotorPositionPID.ki =	0.0;
		ShootMotorPositionPID.kd = 0;
		
		ShootMotorPositionPID.ref = 60*count_temp;//51.4285714*count_temp;
		ShootMotorPositionPID.fdb = CM7Encoder.ecd_angle;
		
		ShootMotorPositionPID.Calc(&ShootMotorPositionPID);
		ShootMotorSpeedPID.ref = ShootMotorPositionPID.output;
	}
	else
	{
		ShootMotorSpeedPID.ref = 0;
	}
	ShootMotorSpeedPID.kp = 10;
	ShootMotorSpeedPID.ki = 0;
	ShootMotorSpeedPID.kd = 0;
	
	ShootMotorSpeedPID.fdb = CM7Encoder.ecd_raw_rate;
	ShootMotorSpeedPID.Calc(&ShootMotorSpeedPID);
	
}
void SelectChassisMode(Time_Count *Encoder_Delay_Time)
{
	Encoder_Delay_Time->time_now = now_system_micrsecond;
	if((Encoder_Delay_Time->time_now -Encoder_Delay_Time->time_last) > 400000)
	{
		Encoder_Delay_Time->time_last = Encoder_Delay_Time->time_now;
		if(Encoder_Delay_Time->flag == 0)
		{	
			Encoder_Delay_Time->flag = 1;
			rotateCnt ++;
			catGimbalCount.ResetCounter(&catGimbalCount);
		}
		else if(Encoder_Delay_Time->flag == 1)
		{
			Encoder_Delay_Time->flag = 0;
			rotateCnt ++;
			catGimbalCount.ResetCounter(&catGimbalCount);
//			Encoder_Delay_Time->time_last = Encoder_Delay_Time->time_now;
		}
	}
	else
	{
////		Encoder_Delay_Time->flag = 0;
	}
}
float Jscope_A=0;
float Jscope_B=0;
float Jscope_C=0;
float Jscope_D=0;
float Jscope_E=0;
void Jscope_test(float Jscope_a,float Jscope_b,float Jscope_c,float Jscope_d,float Jscope_e)
{
	Jscope_A=Jscope_a;
	Jscope_B=Jscope_b;
	Jscope_C=Jscope_c;
	Jscope_D=Jscope_d;
	Jscope_E=Jscope_e;
}