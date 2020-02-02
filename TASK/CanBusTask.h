#ifndef __CANBUSTASK_H__
#define __CANBUSTASK_H__
#include <stm32f4xx.h>

/* Chassis Motor */
#define CAN_BUS1_MOTOR1_FEEDBACK_MSG_ID           0x201
#define CAN_BUS1_MOTOR2_FEEDBACK_MSG_ID           0x202 
#define CAN_BUS1_MOTOR3_FEEDBACK_MSG_ID           0x203
#define CAN_BUS1_MOTOR4_FEEDBACK_MSG_ID           0x204
/* Gimbal Motor  */
#define CAN_BUS1_MOTOR5_FEEDBACK_MSG_ID           0x205
#define CAN_BUS1_MOTOR6_FEEDBACK_MSG_ID           0x206

#define CAN_BUS1_MOTOR7_FEEDBACK_MSG_ID           0x207
#define RATE_BUF_SIZE 6
/*****************CAN2_Motor****************/
#define CAN_BUS2_MOTOR1_FEEDBACK_MSG_ID                  0x201
#define CAN_BUS2_MOTOR2_FEEDBACK_MSG_ID                  0x202 
#define CAN_BUS2_MOTOR5_FEEDBACK_MSG_ID                  0x205
#define CAN_BUS2_MOTOR7_FEEDBACK_MSG_ID                  0x207


typedef struct{
	int32_t raw_value;   									//编码器不经处理的原始值
	int32_t last_raw_value;								//上一次的编码器原始值
	int32_t ecd_value;                       //经过处理后连续的编码器值
	int32_t diff;													//两次编码器之间的差值,可以理解为速度
	uint8_t buf_count;								//滤波更新buf用
	int32_t ecd_bias;											//初始编码器值	
	int32_t ecd_raw_rate;									//通过编码器计算得到的速度原始值
	int32_t rate_buf[RATE_BUF_SIZE];	//保存RATE_BUF_SIZE次的diff
	int32_t round_cnt;										//记录圈数
	float filter_rate;											//速度,对diff識ATE_BUF_SIZE次平均
	double ecd_angle;											//角度
	double ecd_xtl_angle;
  float real_torque_current;          //实际转矩电流
}Encoder;

extern volatile Encoder CM1Encoder;
extern volatile Encoder CM2Encoder;
extern volatile Encoder CM3Encoder;
extern volatile Encoder CM4Encoder;
extern volatile Encoder GMYawEncoder;
extern volatile Encoder GMPitchEncoder;
extern volatile Encoder CM7Encoder;
extern float ZGyroModuleAngle;
extern volatile Encoder CAN2_CM1Encoder;
extern volatile Encoder CAN2_CM2Encoder;
extern volatile Encoder CAN2_CM5Encoder;
extern volatile Encoder CAN2_CM7Encoder;

void CanReceiveMsgProcess(CanRxMsg * msg);
void Set_CM_Speed(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
void Can2_ReceiveMsgProcess(CanRxMsg * msg);
void Set_Gimbal_Current(CAN_TypeDef *CANx, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq, int16_t cm7_iq);
void Set_Gimbal_CALI_STATE(CAN_TypeDef *CANx);
#endif
