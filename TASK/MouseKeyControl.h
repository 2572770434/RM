#ifndef __MOUSEKEYCONTROL_H__
#define __MOUSEKEYCONTROL_H__
#include "RemoteTask.h"

typedef struct Key_State
{
	uint16_t lastState;
	uint16_t thisState;
	uint16_t keyFlag;
	uint16_t keyCnt;
}Key_State;

#define KEY_INIT \
{\
	0,\
	0,\
	0,\
	0,\
}
void ServoKeyCtrl(void);
void CameraKeyCtrl(void);
void FrictionKeyCtrl(void);
void DafuKeyCtrl(void);
void MouseKeyControlProcess(Mouse *mouse, Key *key);
void SendManifoldDataControl(void);
void SendCustomDataControl(void);
void HeatLoopControl(void);
extern uint8_t continuous_shoot_Flag;
extern uint8_t mouseLeftPress;
extern uint8_t quickShootFlag;
extern uint8_t servoPositionFlag;
extern uint8_t sendCustomDataFlag;
extern uint8_t predictflag;
extern Key_State heatLoopKey;
void XTLKeyCtrl(void);
#endif
