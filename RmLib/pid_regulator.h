#ifndef _PID_REGULATOR_H_
#define _PID_REGULATOR_H_
#include "stm32f4xx.h"
typedef struct PID_Regulator_t
{
	float ref;
	float fdb;
	float err[2];
	float kp;
	float ki;
	float kd;
	float componentKp;
	float componentKi;
	float componentKd;
	float componentKpMax;
	float componentKiMax;
	float componentKdMax;
	float output;
	float outputMax;
	float kp_offset;
	float ki_offset;
	float kd_offset;
	void (*Calc)(struct PID_Regulator_t *pid);//º¯ÊýÖ¸Õë
	void (*Reset)(struct PID_Regulator_t *pid);
	float ec;
	float detkp;
	float detki;
	float detkd;
	float maxdetkp;
	float maxdetki;
	float maxdetkd;
	float error;
  int rule_kp[7][7];
	int rule_ki[7][7];
	int rule_kd[7][7];
}PID_Regulator_t;
void PID_Reset(PID_Regulator_t *pid);
void PID_Calc(PID_Regulator_t *pid);
#endif

