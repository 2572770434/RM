#ifndef __USART2_H__
#define __USART2_H__
#include "main.h"
#define USART2_MAX_RECV_LEN		400					//最大接收缓存字节数
#define USART2_MAX_SEND_LEN		400					//最大发送缓存字节数
	  	
#define BUFFER_SIZE 5
typedef struct CRringBuffer_t
{
	float ringBuf[BUFFER_SIZE];
	int16_t tailPosition;
	float lineBuf[BUFFER_SIZE-1];
	float errBuf[BUFFER_SIZE-2];
	float err_Average;
	float predict_Val;
	int16_t lost_COUNT;
	int16_t out_Point;
}CRringBuffer_t;	
			
extern u8  USART2_RX_BUF[USART2_MAX_RECV_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART2_RX_STA;

extern float camera_pitch;
extern float camera_yaw;
extern float V_SPEED;

extern float CM_camera_yaw;
extern float CM_camera_pitch;
extern u8 recognize_flag;
extern uint8_t stopSendFlag;
void usart2_Init(u32 bound);
void Camera_Mes_Process(uint8_t *p);
void Clear_Data(uint8_t *p);
extern float  distance;

extern float last_camera_yaw;
//void SendManifoldData(void);
void sendbytesinfoproc(u8*psendinfo,u16 nsendcount);
#endif
