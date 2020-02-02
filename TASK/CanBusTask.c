#include "main.h"

static uint32_t can_count = 0;
static uint32_t can2_count = 0;

int16_t  pitch_ecd_bias = 7000;
int16_t  yaw_ecd_bias   = 3400;

volatile Encoder CM1Encoder			= {0,0,0,0,0,0,0,{0,0,0,0,0,0},0,0,0};
volatile Encoder CM2Encoder			= {0,0,0,0,0,0,0,{0,0,0,0,0,0},0,0,0};
volatile Encoder CM3Encoder			= {0,0,0,0,0,0,0,{0,0,0,0,0,0},0,0,0};
volatile Encoder CM4Encoder		 	= {0,0,0,0,0,0,0,{0,0,0,0,0,0},0,0,0};
volatile Encoder GMYawEncoder 	= {0,0,0,0,0,0,0,{0,0,0,0,0,0},0,0,0};
volatile Encoder GMPitchEncoder = {0,0,0,0,0,0,0,{0,0,0,0,0,0},0,0,0};
volatile Encoder CM7Encoder 		= {0,0,0,0,0,0,0,{0,0,0,0,0,0},0,0,0};
volatile Encoder CAN2_CM1Encoder			= {0,0,0,0,0,0,0,{0,0,0,0,0,0},0,0,0};
volatile Encoder CAN2_CM2Encoder			= {0,0,0,0,0,0,0,{0,0,0,0,0,0},0,0,0};
volatile Encoder CAN2_CM5Encoder      = {0,0,0,0,0,0,0,{0,0,0,0,0,0},0,0,0};
volatile Encoder CAN2_CM7Encoder      = {0,0,0,0,0,0,0,{0,0,0,0,0,0},0,0,0};

rm_imu_data_t rm_imu_data;



void CanReceiveMsgProcess(CanRxMsg * msg)
{
    can_count++;
		switch(msg->StdId)
		{
			case CAN_BUS1_MOTOR1_FEEDBACK_MSG_ID:
			{
				(can_count<=50) ? GetEncoderBias(&CM1Encoder ,msg):Motor_3510_EncoderProcess(&CM1Encoder ,msg);       //获取到编码器的初始偏差值    
				wwdg_flag |=0x0001;
			}break;
			case CAN_BUS1_MOTOR2_FEEDBACK_MSG_ID:
			{
				(can_count<=50) ? GetEncoderBias(&CM2Encoder ,msg):Motor_3510_EncoderProcess(&CM2Encoder ,msg);
				wwdg_flag |=0x0002;
			}break;
			case CAN_BUS1_MOTOR3_FEEDBACK_MSG_ID:
			{
				(can_count<=50) ? GetEncoderBias(&CM3Encoder ,msg):Motor_3510_EncoderProcess(&CM3Encoder ,msg); 
				wwdg_flag |=0x0004;
			}break;
			case CAN_BUS1_MOTOR4_FEEDBACK_MSG_ID:
			{
				(can_count<=50) ? GetEncoderBias(&CM4Encoder ,msg):Motor_3510_EncoderProcess(&CM4Encoder ,msg);
				wwdg_flag |=0x0008;
			}break;

			case CAN_BUS1_MOTOR5_FEEDBACK_MSG_ID:
			{
				//Set_Gimbal_CALI_STATE(CAN1);
				if(can_count<50)
				{
					GetEncoderBias(&GMYawEncoder ,msg);
					if(GMYawEncoder.ecd_bias-yaw_ecd_bias>4096)
						yaw_ecd_bias+=8192;
					else if(GMYawEncoder.ecd_bias-yaw_ecd_bias<-4096)
						yaw_ecd_bias-=8192;
				}
				else 
				{
					GMYawEncoder.ecd_bias = yaw_ecd_bias; 
					Motor_6020_EncoderProcess(&GMYawEncoder ,msg);
				}
				wwdg_flag |=0x0010;
			}break;
			
			case CAN_BUS1_MOTOR6_FEEDBACK_MSG_ID:
			{
				if(can_count<50)
				{
					GetEncoderBias(&GMPitchEncoder ,msg);
					if(GMPitchEncoder.ecd_bias-pitch_ecd_bias>4096)
						pitch_ecd_bias+=8192;
					else if(GMPitchEncoder.ecd_bias-pitch_ecd_bias<-4096)
						pitch_ecd_bias-=8192;
				}	
				else
				{
					GMPitchEncoder.ecd_bias = pitch_ecd_bias;
					Motor_6623_EncoderProcess(&GMPitchEncoder ,msg);
				}
				wwdg_flag |=0x0020;
			}break;
			case CAN_BUS1_MOTOR7_FEEDBACK_MSG_ID:
			{
				(can_count<=50) ? GetEncoderBias(&CM7Encoder ,msg):Motor_2310_EncoderProcess(&CM7Encoder ,msg);
				wwdg_flag |=0x0040;
			}break;
			default:
			{
			}
	}
}
void Can2_ReceiveMsgProcess(CanRxMsg * msg)
{
	  can2_count++;
		switch(msg->StdId)
		{
			case CAN_BUS2_MOTOR1_FEEDBACK_MSG_ID:  //伸出机构电机 0x201
			{
				(can2_count<=50) ? GetEncoderBias(&CAN2_CM1Encoder ,msg):Motor_3510_EncoderProcess(&CAN2_CM1Encoder ,msg);       //获取到编码器的初始偏差值    
			}break;
			
			case CAN_BUS2_MOTOR2_FEEDBACK_MSG_ID:  //爪子电机 0x202
			{
				(can2_count<=50) ? GetEncoderBias(&CAN2_CM2Encoder ,msg):Motor_3510_EncoderProcess(&CAN2_CM2Encoder ,msg);
			}break;
			
			case RM_IMU_PARAM_ID:
			{
			rm_imu_data.accel_rangle = msg->Data[0] &0x0F;
			rm_imu_data.gyro_rangle = (msg->Data[0] &0xF0) >> 4;
			rm_imu_data.sensor_control_temperature = msg->Data[2];
			rm_imu_data.imu_sensor_rotation = msg->Data[3] & 0x1F;
			rm_imu_data.ahrs_rotation_sequence = (msg->Data[3] & 0xE0) >> 5;
			rm_imu_data.quat_euler = msg->Data[4] & 0x01;//收到1时说明为欧拉角模式
			switch(rm_imu_data.gyro_rangle)
			{case 0: rm_imu_data.gyro_sen = GYRO_2000_SEN; break;
			case 1: rm_imu_data.gyro_sen = GYRO_1000_SEN; break;
			case 2: rm_imu_data.gyro_sen = GYRO_500_SEN; break;
			case 3: rm_imu_data.gyro_sen = GYRO_250_SEN; break;
			case 4: rm_imu_data.gyro_sen = GYRO_125_SEN; break;
			}
			switch(rm_imu_data.accel_rangle)
			{
			case 0: rm_imu_data.accel_sen = ACCEL_3G_SEN; break;
			case 1: rm_imu_data.accel_sen = ACCEL_6G_SEN; break;
			case 2: rm_imu_data.accel_sen = ACCEL_12G_SEN; break;
			case 3: rm_imu_data.accel_sen = ACCEL_24G_SEN; break;
			}
			break;
			}
			case RM_IMU_QUAT_ID:
			{
			if(rm_imu_data.quat_euler && msg->DLC == 6)//欧拉角模式时使用，接受欧拉角
			{
			memcpy(rm_imu_data.euler_angle, msg->Data, 6);//这里将CAN接收到的数据存到rm_imu_data.euler_angle数组里
			rm_imu_data.euler_angle_float[0] = rm_imu_data.euler_angle[0] * 0.0001f/PI*180;//YAW,,都是先乘以10000后才传输过来的，所以结算需要除以10000
			rm_imu_data.euler_angle_float[1] = rm_imu_data.euler_angle[1] * 0.0001f/PI*180;//PITCH
			rm_imu_data.euler_angle_float[2] = rm_imu_data.euler_angle[2] * 0.0001f/PI*180;//ROLL
			
			}
			else if(rm_imu_data.quat_euler == 0 && msg->DLC == 8)
			{
			memcpy(rm_imu_data.quat, msg->Data, 8);
			rm_imu_data.quat_float[0] = rm_imu_data.quat[0] * 0.0001f;
			rm_imu_data.quat_float[1] = rm_imu_data.quat[1] * 0.0001f;
			rm_imu_data.quat_float[2] = rm_imu_data.quat[2] * 0.0001f;
			rm_imu_data.quat_float[3] = rm_imu_data.quat[3] * 0.0001f;
			}
			break;
			}
			case RM_IMU_GYRO_ID:
			{
			memcpy(rm_imu_data.gyro_int16, msg->Data,6);
			rm_imu_data.gyro_float[0] = rm_imu_data.gyro_int16[0] * rm_imu_data.gyro_sen;
			rm_imu_data.gyro_float[1] = rm_imu_data.gyro_int16[1] * rm_imu_data.gyro_sen;
			rm_imu_data.gyro_float[2] = rm_imu_data.gyro_int16[2] * rm_imu_data.gyro_sen;
			rm_imu_data.sensor_temperature = (int16_t)((msg->Data[6] << 3) | (msg->Data[7] >>
			5));
			if (rm_imu_data.sensor_temperature > 1023)
			{
			rm_imu_data.sensor_temperature -= 2048;
			}
			break;
			}
			case RM_IMU_ACCEL_ID:
			{
			memcpy(rm_imu_data.accel_int16, msg->Data,6);
			rm_imu_data.accel_float[0] = rm_imu_data.accel_int16[0] * rm_imu_data.accel_sen;
			rm_imu_data.accel_float[1] = rm_imu_data.accel_int16[1] * rm_imu_data.accel_sen;
			rm_imu_data.accel_float[2] = rm_imu_data.accel_int16[2] * rm_imu_data.accel_sen;
			memcpy(&rm_imu_data.sensor_time, (msg->Data + 6), 2);
			break;
			}
			case RM_IMU_MAG_ID:
			{
			memcpy(rm_imu_data.mag_int16, msg->Data,6);
			break;
			}
			
			default:
			{
			}
	 }
		include_round_engle();
}
int count=0;
void Set_CM_Speed(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (uint8_t)(cm1_iq >> 8);
    tx_message.Data[1] = (uint8_t)cm1_iq;
    tx_message.Data[2] = (uint8_t)(cm2_iq >> 8);
    tx_message.Data[3] = (uint8_t)cm2_iq;
    tx_message.Data[4] = (uint8_t)(cm3_iq >> 8);
    tx_message.Data[5] = (uint8_t)cm3_iq;
    tx_message.Data[6] = (uint8_t)(cm4_iq >> 8);
    tx_message.Data[7] = (uint8_t)cm4_iq;
    CAN_Transmit(CANx,&tx_message);
if(CANx==CAN2)
{count++;
}
}

void Set_Gimbal_Current(CAN_TypeDef *CANx, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq, int16_t cm7_iq)
{
    CanTxMsg tx_message;    
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (unsigned char)(gimbal_yaw_iq >> 8);
    tx_message.Data[1] = (unsigned char)gimbal_yaw_iq;
    tx_message.Data[2] = (unsigned char)(gimbal_pitch_iq >> 8);
    tx_message.Data[3] = (unsigned char)gimbal_pitch_iq;
    tx_message.Data[4] = (unsigned char)(cm7_iq >> 8);
    tx_message.Data[5] = (unsigned char)cm7_iq;
    tx_message.Data[6] = 0xff;
    tx_message.Data[7] = 0xff;
    CAN_Transmit(CANx,&tx_message);
}

void Set_Gimbal_CALI_STATE(CAN_TypeDef *CANx)
{
    CanTxMsg tx_message;    
    tx_message.StdId = 0x3f0;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;

    tx_message.Data[0] = 'c';
    tx_message.Data[2] = 0x00;
    tx_message.Data[1] = 0x00;
    tx_message.Data[3] = 0x00;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
}

void include_round_engle()
{
		if((rm_imu_data.euler_angle_float[0]-rm_imu_data.last_euler_angle_float[0])>300)
			rm_imu_data.round_cnt_yaw--;
		if((rm_imu_data.euler_angle_float[0]-rm_imu_data.last_euler_angle_float[0])<-300)
			rm_imu_data.round_cnt_yaw++;
		rm_imu_data.last_euler_angle_float[0]=rm_imu_data.euler_angle_float[0];
		rm_imu_data.newyaw=rm_imu_data.euler_angle_float[0]+rm_imu_data.round_cnt_yaw*360;//
		rm_imu_data.newpitch=rm_imu_data.euler_angle_float[1];
		rm_imu_data.newroll=rm_imu_data.euler_angle_float[2];
		rm_imu_data.gyro_yaw=rm_imu_data.gyro_int16[0];
		rm_imu_data.gyro_pitch=rm_imu_data.gyro_int16[1];
		rm_imu_data.gyro_roll=rm_imu_data.gyro_int16[2];
}
