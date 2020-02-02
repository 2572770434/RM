#ifndef __REMOTETASK_H__
#define __REMOTETASK_H__
#include <stm32f4xx.h>
#include "ramp.h"

#define FRICTION_RAMP_TICK_COUNT			100
#define MOUSE_Ro_RAMP_TICK_COUNT			50
#define MOUSE_LR_RAMP_TICK_COUNT			30
#define MOUSR_FB_RAMP_TICK_COUNT			30
#define MOUSR_MOUSEX_RAMP_TICK_COUNT			35//yaw
#define MOUSR_MOUSEY_RAMP_TICK_COUNT			20//pitch

//remote data process
typedef struct
{
    float pitch_angle_dynamic_ref;
    float yaw_angle_dynamic_ref;       //不含累加
    float yaw_angle_dynamic_ref_add;
	  float pitch_angle_static_ref;
    float yaw_angle_static_ref;
    float pitch_speed_ref;
    float yaw_speed_ref;
//	  float yaw_angle_dynamic_ref_add;   //含累加
}Gimbal_Ref_t;

typedef __packed struct
{
    int16_t forward_back_ref;
    int16_t left_right_ref;
    int16_t rotate_ref;
}ChassisSpeed_Ref_t;

/*  遥控器输入  */
typedef __packed struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int8_t s1;
	int8_t s2;
}Remote;

typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
}Mouse;	

typedef	__packed struct
{
	uint16_t v;
}Key;

typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
}RC_Ctl_t;

typedef __packed struct           //用__packed省地儿
{
	
	int CommandCodeID;              //命令码ID
  long TimeRemain;                //猜测是四字节二进制数表示浮点数
	int BloodRemain;                //猜测直接是十六位的二进制表示整数
	long Voltage;                    //四字节二进制数表示浮点数
	long Current;                    //四字节二进制数表示浮点数
	long RemainPower;                //猜测是四字节二进制数表示浮点数
	
	float ecd_TimeRemain;
	float ecd_Voltage;              
	float ecd_Current;
	float ecd_RemainPower;
	float ChassisPower;
	
}RC_GameInformation_t;


typedef enum
{
	FRICTION_WHEEL_OFF = 0,
	FRICTION_WHEEL_START_TURNNING = 1,
	FRICTION_WHEEL_ON = 2,
}FrictionWheelState_e;
typedef enum
{
	XTL_OFF = 0,
	XTL_ON = 1,
}xtlState_e;
extern ChassisSpeed_Ref_t ChassisSpeedRef;
extern Gimbal_Ref_t GimbalRef;
extern RampGen_t frictionRamp;   //摩擦轮斜坡
extern FrictionWheelState_e friction_wheel_state;
extern RampGen_t MouseXSpeedRamp;
extern RampGen_t MouseYSpeedRamp;
extern uint16_t Speed_Friction1;
extern uint16_t Speed_Friction2;
extern RampGen_t LRSpeedRamp;   //mouse左右移动斜坡
extern RampGen_t FBSpeedRamp;   //mouse前后移动斜坡
extern RampGen_t RoSpeedRamp;	 //q e快速旋转
extern RampGen_t catGimbalCount;
extern RC_GameInformation_t GameInformationData;
extern xtlState_e xtl_state;
void RemoteDataPrcess(uint8_t *pData);
void JudgementHandle(uint8_t *pData);
uint8_t IsRemoteBeingAction(void);
void RemoteTaskInit(void);
void isremote(void);
#endif
