#ifndef __INPUTSTATE_H__
#define __INPUTSTATE_H__
#include "RemoteTask.h"

typedef enum
{
	REMOTE_INPUT = 1,
	KEY_MOUSE_INPUT = 3,
	STOP = 2,
}InputMode_e;

typedef enum
{
	ZIMIAO =1,
	NOINPUT =2,
	shoot=3,
}S1MODE;


typedef enum
{
	NOSHOOTING = 0,
	SHOOTING = 1,
}Shoot_State_e;

void SetInputMode(Remote *rc);
InputMode_e GetInputMode(void);
void SetShootState(Shoot_State_e v);
Shoot_State_e GetShootState(void);
void SetFrictionState(FrictionWheelState_e v);

S1MODE GetS1InputMode(void);

#endif
