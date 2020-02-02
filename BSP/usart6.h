#ifndef __USART6_H__
#define __USART6_H__

#include <stm32f4xx.h>
#define IMU_DMA_RX_BUF_LEN 72u
#define IMU_FRAME_LENGTH                            41u

extern int sendflag ;
extern int16_t Gyro[3];
extern float Eular[3];
extern  uint8_t angle[12];
void USART6_Configuration(void);
void IMUDataProcess(uint8_t *pData);
void AngleAdd(volatile float * Angles,volatile float * angles);
void SenddatatoNUC(void);

#endif
