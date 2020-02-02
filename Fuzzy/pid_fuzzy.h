#ifndef PID_H_
#define PID_H_
#include "stm32f4xx.h"
#include "main.h"
typedef struct PID 
{
	float Kp; // 增量式积分系数
	float Ki; 
	float Kd;
	float T;
	
	float K1; // 增量式积分系数
	float K2; 
	float K3; 
	float LastError; //Error[-1]
	float PrevError; // Error[-2]
	float pwm_out;
	
	uint16_t flag;//温度状态标志位
}PID;

//void PID_init(PID *structpid);
void PID_Set(PID_Regulator_t *structpid,float Kp,float Ki,float Kd,float T);
void PID_realize(PID_Regulator_t *structpid,float max_e,float max_ec,int rule_kp[7][7] ,float init_kp,float max_deltakp,int rule_ki[7][7],float init_ki,float max_deltaki,int rule_kd[7][7],float init_kd,float max_deltakd);
void PID_Init(PID_Regulator_t *structpid);
////
extern int GMYP_rule_kp[7][7];
extern int GMYP_rule_ki[7][7];
extern int GMYP_rule_kd[7][7];

extern int GMPP_rule_kp[7][7];
extern int GMPP_rule_ki[7][7];
extern int GMPP_rule_kd[7][7];
////
#endif /* PID_H_ */
