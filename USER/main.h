#ifndef __MAIN_H__
#define __MAIN_H__

/*********SYSTEM********/
#include "stm32f4xx.h"
#include "stdarg.h"	 	 
#include "stdio.h"	 	 
#include "string.h"	 
#include "stdlib.h"
#include "math.h"
#include "Kalman.h"
#include "KALMANTEST.h"

/*********BSP********/
#include "bsp.h"
#include "can1.h"
#include "can2.h"
#include "timer.h"
#include "usart1.h"
#include "usart2.h"
#include "usart3.h"
#include "usart6.h"
#include "usart7.h"
#include "gun.h"
#include "new_official_imu.h"

/********TASK*********/
#include "CanBusTask.h"
#include "encoder.h"

#include "ControlTask.h"
#include "SpeedTask.h"


#include "RemoteTask.h"
#include "RemoteControl.h"
#include "MousekeyControl.h"
#include "InputState.h"

#include "SuperviseTask.h"

/********COMMON*********/
#include "dataprocess.h" 
#include "delay.h"
#include "crc.h"
#include "WorkState.h"
/********RMLib*********/
#include "common.h"
//#include "pid.h"
#include "ramp.h"
#include "pid_regulator.h"
#include "LostCounter.h"
#include "FIFO.h"
#include "pid_fuzzy.h"
#endif

