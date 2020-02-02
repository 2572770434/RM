#ifndef __WORKSTATE_H__
#define __WORKSTATE_H__
#include <stm32f4xx.h>

typedef enum
{
    PREPARE_STATE,     		//上电后初始化状态 4s钟左右
//    STANDBY_STATE,				//无输入状态
    NORMAL_STATE,					//操作手控制状态
		CAMERA_STATE,    				//大符状态
	  STOP_STATE,        		//停止运动状态
    ADJUST_STATE,    				//调整状态，用于瞄准 
	  IDENTIFY_STATE,         //战车识别模式
	  CATWALK_STATE,        //猫步模式
}WorkState_e;


extern uint8_t camera_temp;
extern uint8_t cat_temp;
extern uint8_t dafu_temp;

extern int flag_prepare_ready;
extern int flag_Dafu_fromsight;

void WorkStateFSM(void);
void WorkStateSwitchProcess(void);
void SetWorkState(WorkState_e state);
WorkState_e GetWorkState(void);
WorkState_e GetlastWorkState(void);
void AllDataInit(void);

#endif
