#ifndef __DATAPROCESS_H__
#define __DATAPROCESS_H__

#include "main.h"
#include "arm_math.h"

#define BUFFER_MAX   100

#define mat         arm_matrix_instance_f32 
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32

typedef __packed struct
{
	uint8_t sof;             //数据帧头，固定0xA5
	uint16_t data_length;    //每帧内数据data的长度 
	uint8_t seq;             //包序号
	uint8_t crc8;            //帧头的crc校验结果
}frame_header_t;

typedef __packed struct
{
	uint8_t robot_id;
	uint16_t stageRemainTime; //当前阶段剩余时间
	uint8_t gameProgress;     //当前比赛处于哪个阶段
	uint8_t robotLevel;       //机器人当前等级
	uint16_t remainHP;        //机器人当前血量
	uint16_t maxHP;           //机器人满血量
}extGameRobotState_t;  //比赛机器人状态

typedef __packed struct
{
	uint8_t armorType; //因装甲被击打掉血时，标示受到伤害的装甲ID
	uint8_t hurtType;  //血量变化类型
}extRobotHurt_t; //伤害数据

typedef __packed struct
{
	uint16_t ChassisVolt;        //底盘输出电压
	uint16_t ChassisCurrent;     //底盘输出电流
	float ChassisPower;       //底盘输出功率
	uint16_t ChassisPowerBuffer; //底盘功率缓冲
	uint16_t shooter_17_Heat;   //17mm枪口热量
	uint16_t shooter_42_Heat;   //42mm枪口热量
}extPowerHeatData_t;  //实时功率热量数据

typedef __packed struct
{
	float data1;
	float data2;
	float data3;
	uint8_t mask;
}extShowData_t;  //参赛队自定义数据

typedef __packed struct
{
	int headPosition;
	int tailPosition;
	uint8_t ringBuf[BUFFER_MAX];
}ringBuffer_t;

typedef __packed struct
{
	uint8_t bulletType; //弹丸类型  1：17mm弹丸   2：42mm弹丸
	uint8_t bulletFreq; //弹丸射频
	float bulletSpeed;  //弹丸射速
}extShootData_t;  //实时射击信息

typedef struct
{
  float raw_value;
  float filtered_value[1];
  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman_filter_t;

typedef struct
{
  float raw_value;
	float filtered_value[1];
	float xhat_data[2], xhatminus_data[2], z_data[1], Pminus_data[4], K_data[2];
	float P_data[4];
	float AT_data[4], HT_data[2];
	float A_data[4];
	float H_data[2];
	float Q_data[4];
	float R_data[1];
} kalman_filter_init_t;



void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
float kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2);

extern ringBuffer_t buffer;
extern uint16_t Heat17_Max;
extern extPowerHeatData_t  robotPowerHeat;
extern extGameRobotState_t robotState;
extern uint8_t computer_tx_buf[28];
extern uint8_t user_data[19];
extern uint16_t bulletCounter;
extern extShootData_t robotShootData;
extern float chassisPowerError;
extern float chassisPowerBuffer;
void getRobotState(uint8_t *stateData);
void getRobotPowerHeat(uint8_t *powerHeartData);
void getRobotShootData(uint8_t *shootData);

float Parameter_Transformation(int32_t data);
void RingBuffer_Write(uint8_t data);
void user_data_handle(void);
u8   RingBuffer_Read(uint8_t *pdata);

#endif
