#include "main.h"

static InputMode_e inputmode = REMOTE_INPUT;   //输入模式设定
Shoot_State_e shootState = NOSHOOTING;
static S1MODE S1inputmode = NOINPUT;
FrictionWheelState_e friction_wheel_state = FRICTION_WHEEL_OFF;
xtlState_e xtl_state=XTL_OFF;
void SetInputMode(Remote *rc)
{
	if(rc->s2 == 1)
	{
		inputmode = REMOTE_INPUT;
	}
	else if(rc->s2 == 3)
	{
		inputmode = KEY_MOUSE_INPUT;
	}
	else if(rc->s2 == 2)
	{
		inputmode = STOP;
	}	
}

InputMode_e GetInputMode(void)
{
	return inputmode;
}



void SetS1InputMode(Remote *rc)
{
//	if (rc->s1 == 2 )
//	{
//		S1inputmode = ZIMIAO;
//	}
//	else if(rc->s1 == 1)
//	{
//		S1inputmode = shoot;
//	}
//	else if(rc->s1 == 3 && camera_temp == 1)  S1inputmode = ZIMIAO;
//	else S1inputmode = NOINPUT;
		if( camera_temp == 1)  S1inputmode = ZIMIAO;
}

S1MODE GetS1InputMode(void)
{
	return S1inputmode;
}

void SetShootState(Shoot_State_e v)
{
	shootState = v;
}

Shoot_State_e GetShootState(void)
{
	return shootState;
}


void SetFrictionState(FrictionWheelState_e v)
{
	friction_wheel_state = v;
}

FrictionWheelState_e GetFrictionState()
{
	return friction_wheel_state;
}

xtlState_e GetXTLState()
{
	return xtl_state;
}
