#ifndef __TIMER_H__
#define __TIMER_H__
#include <stm32f4xx.h>

#define TIME_COUNT_INIT \
{\
	0,\
	0,\
	0,\
	0,\
}\


typedef __packed struct
{
	uint32_t time_last;
	uint32_t time_now;
	int32_t time_error;
	uint8_t flag;
}Time_Count;

extern Time_Count system_micrsecond;
extern Time_Count shot_frequency_limt;
extern Time_Count remote_micrsecond;
extern Time_Count mpu6050_micrsecond;
extern Time_Count SendData_Delay_Time;
extern Time_Count Encoder_Delay_Time;
extern uint32_t last_system_micrsecond;   //系统时间 单位ms
extern uint32_t now_system_micrsecond;   //系统时间 单位ms

void TIM2_Configuration(void);
uint32_t Get_Time_Micros(void);
void TIM6_Configuration(void);
void TIM6_Start(void);

#endif
