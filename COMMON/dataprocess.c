#include "main.h"
#include "math.h"
#include "arm_math.h"  


#define SpeedMax  3700

//kalman_filter_t KALMAN1 = KALMAN1INIT;
//kalman_filter_init_t KALMAN2 = KALMAN2INIT;


extGameRobotState_t robotState;
extRobotHurt_t      robotHurt;
extPowerHeatData_t  robotPowerHeat;
extShootData_t      robotShootData;
ringBuffer_t buffer;
float chassisPowerError = 0;  //前后两次的功率差
float lastChassisPower = 0;   //上一次的功率
float chassisPowerBuffer = 0;//功率缓冲
uint16_t Heat17_Max = 65;//假设步兵的子弹射速为25，所以保证发出去不超热量
extPowerHeatData_t  robotPowerHeat;
uint16_t bulletCounter = 0;
float user_float1=0; 
float user_float2=0; 
float user_float3=0; 
uint8_t user_data[19]={0};
/***
函数：void getRobotState(uint8_t *stateData)
功能：从裁判系统读取机器人状态(当前血量)
备注：ID：0x0001
      共17个数据，下标11，12为当前血量数据
***/
void getRobotState(uint8_t *stateData)
{
	robotState.robot_id= stateData[7];
//	robotState.stageRemainTime = |stateData[8]<<8;
//	robotState.gameProgress = stateData[9];
	robotState.robotLevel = stateData[8];//机器人等级
	if(robotState.robotLevel == 1)
	{
		Heat17_Max = 240;
	}
	else if(robotState.robotLevel == 2)
	{
		Heat17_Max = 360;
	}
	else if(robotState.robotLevel == 3)
	{
		Heat17_Max = 480;
	}
	if(heatLoopKey.keyFlag == 1 )
	{
		Heat17_Max = 10000;
	}
}
/***
函数：void getRobotPowerHeat(uint8_t *powerHeartData)
功能：从裁判系统读取实时功率热量数据
备注：ID：0x0004   50Hz频率周期发送
      共29个数据，下标23，24为17mm弹丸枪口热量数据
***/
char test[14]={0};
int icnt=0;
void getRobotPowerHeat(uint8_t *powerHeartData)
{
	uint32_t ChassisPower_temp;
	uint16_t ChassisPower_buffer;
	for(icnt=0;icnt<14;icnt++)
	{
		test[icnt]=powerHeartData[5+icnt];
	}
	ChassisPower_temp = powerHeartData[11]|(powerHeartData[12]<<8)|(powerHeartData[13]<<16)|(powerHeartData[14]<<24);
	robotPowerHeat.ChassisPower = Parameter_Transformation(ChassisPower_temp);
	
	robotPowerHeat.ChassisPowerBuffer = powerHeartData[15]|(powerHeartData[16]<<8);
	//chassisPowerBuffer = Parameter_Transformation(ChassisPower_buffer);
	
	robotPowerHeat.shooter_17_Heat= powerHeartData[17]|(powerHeartData[18]<<8);
//	chassisPowerError = robotPowerHeat.ChassisPower - lastChassisPower;
	
//	Speed_Offset.kp = 20;
//	Speed_Offset.kd = 2;
//	Speed_Offset.outputMax = 300; 
//	
//	Speed_Offset.ref = 20;
//	Speed_Offset.fdb = robotPowerHeat.ChassisPowerBuffer;
//	Speed_Offset.Calc(&Speed_Offset);
//	Speed_Offset.output = -Speed_Offset.output;      //应该是负的关系
//	VAL_LIMIT(Speed_Offset.output,-300,0);
}
/***
函数：void getRobotShootData(uint8_t *shootData)
功能：从裁判系统读取实时射击信息
备注：ID：0x0003
      共15个数据，下标9，10，11，12为弹丸射速数据
***/
void getRobotShootData(uint8_t *shootData)
{
	uint32_t Speed17mm_temp;
	static float last_Small_bulletSpeed;
	if(shootData[7]==1)
	{
		Speed17mm_temp = shootData[9]|(shootData[10]<<8)|(shootData[11]<<16)|(shootData[12]<<24);
		robotShootData.bulletSpeed = Parameter_Transformation(Speed17mm_temp);
		robotShootData.bulletFreq = shootData[8];//射频
		if(robotShootData.bulletSpeed != last_Small_bulletSpeed)
		bulletCounter ++;
	}
	last_Small_bulletSpeed = robotShootData.bulletSpeed;
}
/*********************************************
函数：Parameter_Transformation
功能：四字节二进制数转换为浮点数
**********************************************/
float Parameter_Transformation(int32_t data)
{
  int temp1,temp4;
	long temp2;
	float temp3;
	//temp1是阶码
	//temp2是尾数
	//temp3是最后算好的数
	//temp4是电流尾数的每一位
	temp1=((data&0X7F800000)>>23)-127; 
	temp2= data&0X007FFFFF;
	for(int j=0;j<24;j++)
	{
		if(j==0)
		{ 
			temp3=(float)ldexp(1.0,temp1);
		}
		else
		{
		temp4=(temp2&(0x00400000>>(j-1)))>>(23-j);
		temp3=temp3+temp4*(float)ldexp(1.0,temp1-j);
		}
	}
	return temp3;
}
/***
函数：void RingBuffer_Write(uint8_t data)
功能：将数据data写入环形队列buffer.ringBuf中
备注：无
***/
void RingBuffer_Write(uint8_t data)
{
	buffer.ringBuf[buffer.tailPosition] = data;     //从尾部追加
	if(++buffer.tailPosition>=BUFFER_MAX)           //尾节点偏移
		buffer.tailPosition = 0;                      //大于数组最大长度，归零，形成环形队列
	if(buffer.tailPosition == buffer.headPosition)  //如果尾部节点追到头部节点，则修改头结点偏移位置丢弃早期数据
		if(++buffer.headPosition>=BUFFER_MAX)
		buffer.headPosition = 0;
}

/***
函数：u8 RingBuffer_Read(uint8_t *pdata)
功能：从环形队列buffer.ringBuf中读取数据到地址pdata中
备注：无
***/
u8 RingBuffer_Read(uint8_t *pdata)
{
	if(buffer.headPosition == buffer.tailPosition)  //如果头尾接触表示缓冲区为空
	{
		return 1;  //返回1，环形缓冲区是空的
	}
	else
	{
		*pdata = buffer.ringBuf[buffer.headPosition];  //如果缓冲区非空则取头节点值并偏移头节点
		if(++buffer.headPosition>=BUFFER_MAX)
			buffer.headPosition = 0;
		return 0;   //返回0，表示读取数据成功
	}
}

void user_data_handle(void)//0-3，4-7，8-11，12
{
	 uint8_t i;	
	 user_float1 = servoPositionFlag;//bulletCounter; //发射子弹的数量
	 user_float2 = quickShootFlag;//quickShootFlag;//摩擦轮的速度 默认高速
	 user_float3 = sendCustomDataFlag;//servoPositionFlag;//舵机的位置  默认关闭
	
	 unsigned char * a = (unsigned char*)&user_float1;//进制转化
	 unsigned char * b = (unsigned char*)&user_float2;
	 unsigned char * c = (unsigned char*)&user_float3;
	
	//数据内容ID
	user_data[0]=0X80;
	user_data[1]=0xD1;
	
	//发送者ID
	user_data[2]=robotState.robot_id;
	user_data[3]=0x00;
	
  //客户端ID
  user_data[4]=robotState.robot_id;
	user_data[5]=0x01;
	
	
	
	 for(i = 6; i<10; i++)
	 {
		user_data[i]  =a[i-6]; 
	  user_data[i+4]=b[i-6];
		user_data[i+8]=c[i-6];
	 }
	 if(continuous_shoot_Flag == 1)//continuous_shoot_Flag 是车移动的高低速标志，不是射击标志
	 {
		 if(camera_temp == 1)
		 {
			 	 if(heatLoopKey.keyFlag == 0)user_data[18] = 0x23;
			   else user_data[18] = 0x03;
		
		 }
		 else 
		 {
			 if(heatLoopKey.keyFlag == 0)user_data[18] = 0x21;
			 else user_data[18] = 0x01;
		 }
	 }
	 else 
	 {
		 if(camera_temp == 1)
		 {
			 if(heatLoopKey.keyFlag == 0)user_data[18] = 0x22;
			 else user_data[18] = 0x02;
		 }
		 else 
		 {
			 if(heatLoopKey.keyFlag == 0)user_data[18] = 0x20;
			 else user_data[18] = 0x00;
		 }
	 }
	 
	 
}





void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{
	float Adata[4]={1,4,0,1};
	float Hdata[2]={1,0};
	float Qdata[4]={0.001,0,0,0.001};
	float Rdata[1]={2};
	float Pdata[4]={1,0,0,1};
  mat_init(&F->xhat,2,1,(float *)I->xhat_data);
	mat_init(&F->xhatminus,2,2,(float *)I->xhatminus_data);
	mat_init(&F->Pminus,2,2,(float *)I->Pminus_data);
	mat_init(&F->A,2,2,(float *)Adata);
	mat_init(&F->AT,2,2,(float *)I->AT_data);
	mat_init(&F->H,1,2,(float *)Hdata);
	mat_init(&F->Q,2,2,(float *)Qdata);
	mat_init(&F->K,2,1,(float *)I->K_data);
	mat_init(&F->z,1,1,(float *)I->z_data);
	mat_init(&F->R,1,1,(float *)Rdata);
	mat_init(&F->P,2,2,(float *)Pdata);
  mat_init(&F->HT,2,1,(float *)I->HT_data);
  mat_trans(&F->H, &F->HT);
	mat_trans(&F->A, &F->AT);
}

float kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2)
{
  float TEMP_data[4] = {0, 0, 0, 0};
  float TEMP_data21[2] = {0, 0};
	float idata[4]={1,0,0,1};
  mat TEMP,TEMP21,i;

  mat_init(&TEMP,2,2,(float *)TEMP_data);
  mat_init(&TEMP21,2,1,(float *)TEMP_data21);
  mat_init(&i,2,2,(float *)idata);
  F->z.pData[0] = signal1;

  //1. xhat'(k)= A xhat(k-1)
  mat_mult(&F->A, &F->xhat, &F->xhatminus);
	

	
  //2. P'(k) = A P(k-1) AT + Q
  mat_mult(&F->A, &F->P, &F->Pminus);
  mat_mult(&F->Pminus, &F->AT, &TEMP);
  mat_add(&TEMP, &F->Q, &F->Pminus);

  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
  mat_mult(&F->H, &F->Pminus, &F->K);
  mat_mult(&F->K, &F->HT, &TEMP);
  mat_add(&TEMP, &F->R, &F->K);
  mat_inv(&F->K, &F->P);
  mat_mult(&F->Pminus, &F->HT, &TEMP);
  mat_mult(&TEMP, &F->P, &F->K);

  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
  mat_mult(&F->H, &F->xhatminus, &TEMP21);
  mat_sub(&F->z, &TEMP21, &F->xhat);
  mat_mult(&F->K, &F->xhat, &TEMP21);
  mat_add(&F->xhatminus, &TEMP21, &F->xhat);

  //5. P(k) = (1-K(k)H)P'(k)
  mat_mult(&F->K, &F->H, &F->P);
  mat_sub(&i, &F->P, &TEMP);
  mat_mult(&TEMP, &F->Pminus, &F->P);

  F->filtered_value[0] = F->xhat.pData[0];

  return F->filtered_value[0];
}


