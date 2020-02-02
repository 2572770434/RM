#include <string.h>     
#include <stdio.h> 
#include <math.h>
#include "pid_fuzzy.h"

//注1：自适应模糊pid最重要的就是论域的选择，要和你应该控制的对象相切合
//注2：以下各阀值、限幅值、输出值均需要根据具体的使用情况进行更改
//注3：因为我的控制对象惯性比较大，所以以下各部分取值较小
//论域e:[-5,5]  ec:[-0.5,0.5]

//误差的阀值，小于这个数值的时候，不做PID调整，避免误差较小时频繁调节引起震荡
#define Emin 0.0
#define Emid 0.08
#define Emax 0.6
//调整值限幅，防止积分饱和
#define Umax 5
#define Umin -5

//输出值限幅
#define Pmax 7200
#define Pmin 0

#define NB 0
#define NM 1
#define NS 2
#define ZO 3
#define PS 4
#define PM 5
#define PB 6

int GMYP_rule_kp[7][7]=
			{	{PB,PB,PM,PM,PS,ZO,ZO},
				{PB,PB,PM,PS,ZO,NM,NS},
				{PB,PM,PM,PS,NB,NS,NS},
				{PM,PM,ZO,ZO,ZO,PM,PM},
				{NS,NS,NB,PS,PM,PM,PB},
				{NS,NM,ZO,PS,PM,PB,PB},
				{ZO,ZO,PS,PM,PM,PB,PB}    };

int GMYP_rule_kd[7][7]=
			{	{PM,PB,PB,ZO,NB,NB,NM},
				{PM,PB,PB,NB,NB,NM,ZO},
				{PM,PM,PM,NB,PS,PM,PB},
				{NB,NB,NB,NB,NB,NB,NB},
				{PB,PM,PS,NB,PM,PM,PM},
				{ZO,NM,NB,NB,PB,PB,PM},
				{NM,NB,NB,ZO,PB,PB,PM}    };

int GMYP_rule_ki[7][7]=
			{	{NB,NB,NB,NB,NB,NB,NB},
				{NB,NB,NB,NB,NB,NB,NB},
				{NS,NS,PB,PB,PB,NS,NS},
				{ZO,ZO,PB,PB,PB,ZO,ZO},
				{NS,NS,PB,PB,PB,NS,NS},
				{NB,NB,NB,NB,NB,NB,NB},
				{NB,NB,NB,NB,NB,NB,NB}    };
/////////////////////////////////////
int GMPP_rule_kp[7][7]=
			{	{PB,PB,PM,PM,PS,ZO,ZO},
				{PB,PB,PM,PS,ZO,NM,NS},
				{PB,PM,PM,ZO,NB,NS,NS},
				{PM,PM,ZO,NS,ZO,PM,PM},
				{NS,NS,NB,ZO,PM,PM,PB},
				{NS,NM,ZO,PS,PM,PB,PB},
				{ZO,ZO,PS,PM,PM,PB,PB}    };

int GMPP_rule_kd[7][7]=
			{	{PM,PB,PB,ZO,NB,NB,NM},
				{PM,PB,PB,NB,NB,NM,ZO},
				{PM,PM,PM,NB,PS,PM,PB},
				{NB,NB,NB,NB,NB,NB,NB},
				{PB,PM,PS,NB,PM,PM,PM},
				{ZO,NM,NB,NB,PB,PB,PM},
				{NM,NB,NB,ZO,PB,PB,PM}    };

int GMPP_rule_ki[7][7]=
			{	{NB,NB,NB,NB,NB,NB,NB},
				{NB,NB,NB,NB,NB,NB,NB},
				{NS,NS,PB,PB,PB,NS,NS},
				{ZO,ZO,PB,PB,PB,ZO,ZO},
				{NS,NS,PB,PB,PB,NS,NS},
				{NB,NB,NB,NB,NB,NB,NB},
				{NB,NB,NB,NB,NB,NB,NB}    };
//////////////////////////////////////				
//在此进行模糊规则的个性化定制，				
		
				
/**************求隶属度（三角形）***************/
float FTri(float x,float a,float b,float c)//FuzzyTriangle
{
	if(x<=a)
		return 0;
	else if((a<x)&&(x<=b))
		return (x-a)/(b-a);
	else if((b<x)&&(x<=c))
		return (c-x)/(c-b);
	else if(x>c)
		return 0;
	else
		return 0;
}
/*****************求隶属度（梯形左）*******************/
float FTraL(float x,float a,float b)//FuzzyTrapezoidLeft
{
	if(x<=a)  
		return 1;
	else if((a<x)&&(x<=b))
		return (b-x)/(b-a);
	else if(x>b)
		return 0;
	else
		return 0;
}
/*****************求隶属度（梯形右）*******************/
float FTraR(float x,float a,float b)//FuzzyTrapezoidRight
{
	if(x<=a)
		return 0;
	if((a<x)&&(x<b))
		return (x-a)/(b-a);
	if(x>=b)
		return 1;
	else
		return 1;
}
/****************三角形反模糊化处理**********************/
float uFTri(float x,float a,float b,float c)
{ 
	float y,z;
	z=(b-a)*x+a;
	y=c-(c-b)*x;
	return (y+z)/2;
}
/*******************梯形（左）反模糊化***********************/
float uFTraL(float x,float a,float b)
{
	return b-(b-a)*x;
}
/*******************梯形（右）反模糊化***********************/
float uFTraR(float x,float a,float b)
{
	return (b-a)*x +a;
}
/**************************求交集****************************/
float fand(float a,float b)
{
	return (a<b)?a:b;
}
/**************************求并集****************************/
float forr(float a,float b)
{
	return (a<b)?b:a;
}

/*==========   PID计算部分   ======================*/   
void PID_realize(PID_Regulator_t *structpid,float max_e,float max_ec,int rule_kp[7][7] ,float init_kp,float max_deltakp,int rule_ki[7][7],float init_ki,float max_deltaki,int rule_kd[7][7],float init_kd,float max_deltakd)
{
	float pwm_var;//pwm调整量
	float iError;//当前误差
	float ref,fdb;
	float e,ec;
	//计算隶属度表
	float es[7],ecs[7];
	float form[7][7];
	int i=0,j=0;
	int MaxX=0,MaxY=0;
	
	//记录隶属度最大项及相应推理表的p、i、d值
	float lsd;
	int temp_p,temp_d,temp_i;
	float detkp,detkd,detki;//推理后的结果
	
	//输入格式的转化及偏差计算

	structpid->error=structpid->err[0] = structpid->ref - structpid->fdb; // 偏差
	
	structpid->error=structpid->err[0];
	e=structpid->error/(max_e/3);
	structpid->ec=structpid->err[0]-structpid->err[1];
	ec=structpid->ec/(max_ec/3);
	structpid->err[1]=structpid->err[0];
	//当温度差的绝对值小于Emax时，对pid的参数进行调整
	//if(fabs(e)<=Emax)
	{
	//计算iError在es与ecs中各项的隶属度
	es[NB]=FTraL(e,-3,-1);  //e 
	es[NM]=FTri(e,-3,-2,0);
	es[NS]=FTri(e,-3,-1,1);
	es[ZO]=FTri(e,-2,0,2);
	es[PS]=FTri(e,-1,1,3);
	es[PM]=FTri(e,0,2,3);
	es[PB]=FTraR(e,1,3);

	ecs[NB]=FTraL(ec,-3,-1);//ec
	ecs[NM]=FTri(ec,-3,-2,0);
	ecs[NS]=FTri(ec,-3,-1,1);
	ecs[ZO]=FTri(ec,-2,0,2);
	ecs[PS]=FTri(ec,-1,1,3);
	ecs[PM]=FTri(ec,0,2,3);
	ecs[PB]=FTraR(ec,1,3);
	}
	
	//计算隶属度表，确定e和ec相关联后表格各项隶属度的值
	for(i=0;i<7;i++)
	{
		for(j=0;j<7;j++)
		{
			form[i][j]=fand(es[i],ecs[j]);
		}
	}
	
	//取出具有最大隶属度的那一项
	for(i=0;i<7;i++)
	{
		for(j=0;j<7;j++)
		{
			if(form[MaxX][MaxY]<form[i][j]) 
			{
				MaxX=i;
				MaxY=j;
			}
		}
	}
	//进行模糊推理，并去模糊
	lsd=form[MaxX][MaxY];
	temp_p=rule_kp[MaxX][MaxY];
	temp_d=rule_kd[MaxX][MaxY];   
	temp_i=rule_ki[MaxX][MaxY];
	
	if(temp_p==NB)
		detkp=uFTraL(lsd,-0.3,-0.1);
	else if(temp_p==NM)
		detkp=uFTri(lsd,-0.3,-0.2,0);
	else if(temp_p==NS)
		detkp=uFTri(lsd,-0.3,-0.1,0.1);
	else if(temp_p==ZO)
		detkp=uFTri(lsd,-0.2,0,0.2);
	else if(temp_p==PS)
		detkp=uFTri(lsd,-0.1,0.1,0.3);
	else if(temp_p==PM)
		detkp=uFTri(lsd,0,0.2,0.3);
	else if(temp_p==PB)
		detkp=uFTraR(lsd,0.1,0.3);

	if(temp_d==NB)
		detkd=uFTraL(lsd,-3,-1);
	else if(temp_d==NM)
		detkd=uFTri(lsd,-3,-2,0);
	else if(temp_d==NS)
		detkd=uFTri(lsd,-3,1,1);
	else if(temp_d==ZO)
		detkd=uFTri(lsd,-2,0,2);
	else if(temp_d==PS)
		detkd=uFTri(lsd,-1,1,3);
	else if(temp_d==PM)
		detkd=uFTri(lsd,0,2,3);
	else if(temp_d==PB)
		detkd=uFTraR(lsd,1,3);

	if(temp_i==NB)
		detki=uFTraL(lsd,-0.06,-0.02);
	else if(temp_i==NM)
		detki=uFTri(lsd,-0.06,-0.04,0);
	else if(temp_i==NS)
		detki=uFTri(lsd,-0.06,-0.02,0.02);
	else if(temp_i==ZO)
		detki=uFTri(lsd,-0.04,0,0.04);
	else if(temp_i==PS)
		detki=uFTri(lsd,-0.02,0.02,0.06);
	else if(temp_i==PM)
		detki=uFTri(lsd,0,0.04,0.06);
	else if (temp_i==PB)
		detki=uFTraR(lsd,0.04,0.096);

	structpid->detkp=detkp*(max_deltakp/0.3);
	structpid->detki=detki*(max_deltaki/0.06);
	structpid->detkd=detkd*(max_deltakd/3);
	
	//pid三项系数的修改
	structpid->kp=init_kp+structpid->detkp;
	structpid->ki=init_ki+structpid->detki;
	structpid->kd=init_kd+structpid->detkd;
	//structpid->kd=0;//取消微分作用
	
	//对Kp,Ki进行限幅
	if(structpid->kp<0)
	{structpid->kp=0;}
		if(structpid->ki<0)
	{structpid->ki=0;}
	if(structpid->ki<0)
	{structpid->ki=0;}
	

}

void PID_Set(PID_Regulator_t *structpid,float Kp,float Ki,float Kd,float T)
{
	(*structpid).kp=Kp;//Kp*(1+(Td/T));
	(*structpid).ki=Ki;
	(*structpid).kd=Kd;
	
}

//void PID_Init(PID_Regulator_t *structpid)
//{
//	PID_Set(structpid,8.3,1.2,0,1);
//	structpid->flag=0;
//	structpid->pwm_out=0;
//}