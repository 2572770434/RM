#ifndef __ENCODER_H__
#define __EMCODER_H__
#include <stm32f4xx.h>
#include "CanBusTask.h"

void GetEncoderBias(volatile Encoder *v, CanRxMsg * msg);
void Motor_3510_EncoderProcess(volatile Encoder *v, CanRxMsg * msg);
void Motor_6623_EncoderProcess(volatile Encoder *v, CanRxMsg * msg);
void Motor_6020_EncoderProcess(volatile Encoder *v, CanRxMsg * msg);
void Motor_2310_EncoderProcess(volatile Encoder *v, CanRxMsg * msg);
#endif
