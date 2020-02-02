#ifndef __USART7_H__
#define __USART7_H__

#include <stm32f4xx.h>
#define IMU_DMA_RX_BUF_LEN 72u
#define IMU_FRAME_LENGTH                            41u

extern int16_t Gyro[3];
extern float Eular[3];

void USART7_Configuration(void);


#endif
