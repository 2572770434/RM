#ifndef __REMOTECONTROL_H__
#define __REMOTECONTROL_H__
#include <stm32f4xx.h>
#include "RemoteTask.h"

//remote control parameters
#define REMOTE_CONTROLLER_STICK_OFFSET      1024u   
#define PITCH_MAX 30.0f
#define PITCH_MIN -15.0f
#define YAW_MAX 60.0f
#define YAW_MIN -60.0f

#define STICK_TO_CHASSIS_SPEED_FB_REF_FACT     0.075f//0.5
#define STICK_TO_CHASSIS_SPEED_LR_REF_FACT     0.10f
#define STICK_TO_PITCH_ANGLE_INC_FACT       0.004f
#define STICK_TO_YAW_ANGLE_INC_FACT         0.0022f
#define STICK_TO_ROTATE_ANGLE_INC_FACT         0.96f

#define REMOTE_SWITCH_VALUE_UP         		0x01u  
#define REMOTE_SWITCH_VALUE_DOWN			0x02u
#define REMOTE_SWITCH_VALUE_CENTRAL			0x03u

#define REMOTE_SWITCH_CHANGE_1TO3      (uint8_t)((REMOTE_SWITCH_VALUE_UP << 2) | REMOTE_SWITCH_VALUE_CENTRAL)   
#define REMOTE_SWITCH_CHANGE_2TO3      (uint8_t)((REMOTE_SWITCH_VALUE_DOWN << 2) | REMOTE_SWITCH_VALUE_CENTRAL)  
#define REMOTE_SWITCH_CHANGE_3TO1      (uint8_t)((REMOTE_SWITCH_VALUE_CENTRAL << 2) | REMOTE_SWITCH_VALUE_UP)
#define REMOTE_SWITCH_CHANGE_3TO2      (uint8_t)((REMOTE_SWITCH_VALUE_CENTRAL << 2) | REMOTE_SWITCH_VALUE_DOWN)

#define REMOTE_SWITCH_VALUE_BUF_DEEP   16u
//to detect the action of the switch
typedef struct RemoteSwitch_t
{
	 uint8_t switch_value_raw;            // the current switch value
	 uint8_t switch_value1;				  //  last value << 2 | value
	 uint8_t switch_value2;				  //
	 uint8_t switch_long_value; 		  //keep still if no switching
	 uint8_t switch_value_buf[REMOTE_SWITCH_VALUE_BUF_DEEP]; 
	 uint8_t buf_index;
	 uint8_t buf_last_index;
	 uint8_t buf_end_index;
}RemoteSwitch_t;

typedef struct Key_State_R
{
	uint16_t lastState;
	uint16_t thisState;
	uint16_t keyFlag;
	uint16_t keyCnt;
}Key_State_R;
void RemoteControlProcess(Remote *rc);
void DafuKeyCtrl_R(void);
#endif
