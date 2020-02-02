#include "main.h"
#include "KALMANTEST.h"

float A[2][2]={1,1,0,1};
float H[1][2]={1,0};
float Q[2][2]={1,0,0,1};
float M[2][2]={0};
float K[2][1]={0};
float Z[1][1]={0};
float R[1][1]={2};
float P[2][2]={1,0,0,1};
float AT[2][2]={0};
float HT[1][2]={0};
float XHAT[2][1]={0};
float XHAT_NEXT[2][1]={0};
float P_NEXT[2][2]={0};
float S[1][1]={0};
float O[2][2]={0};
float I[2][2]={1,0,0,1};
float T[2][2]={0};

float L[2][1]={0};
float N[1][2]={0};
float D[1][2]={0};
float DT[1][1]={0};

void FIRST_STEP(void)//xhat'(k)= A xhat(k-1)
{
	XHAT_NEXT[0][0]= A[0][0]*XHAT[0][0]+A[0][1]*XHAT[1][0];
	XHAT_NEXT[1][0]= A[1][0]*XHAT[0][0]+A[1][1]*XHAT[1][0];
}

void SECONND_STEP(void)
{
	//A×ªÖÃ
	AT[0][0]=A[0][0];
	AT[0][1]=A[1][0];
	AT[1][0]=A[0][1];
	AT[1][1]=A[1][1];
	
	 //2. P'(k) = A P(k-1) AT + Q
	M[0][0]=A[0][0]*P[0][0]+A[0][1]*P[1][0];
	M[0][1]=A[0][0]*P[0][1]+A[0][1]*P[1][1];
	M[1][0]=A[1][0]*P[0][0]+A[1][1]*P[1][0];
	M[1][1]=A[1][0]*P[0][1]+A[1][1]*P[1][1];
	
	P_NEXT[0][0]=(M[0][0]*AT[0][0]+M[0][1]*AT[1][0]) + Q[0][0];
	P_NEXT[0][1]=(M[0][0]*AT[0][1]+M[0][1]*AT[1][1]) + Q[0][1];
	P_NEXT[1][0]=(M[1][0]*AT[0][0]+M[1][1]*AT[1][0]) + Q[1][0];
	P_NEXT[1][1]=(M[1][0]*AT[0][1]+M[1][1]*AT[1][1]) + Q[1][1];
}
void third_step(void)
{

	
	HT[0][0]=H[0][0];
	HT[1][0]=H[0][1];
	
	L[0][0]= P_NEXT[0][0]*HT[0][0]+P_NEXT[0][1]*HT[1][0];
	L[1][0]= P_NEXT[1][0]*HT[0][0]+P_NEXT[1][1]*HT[1][0];
	
	N[0][0]=H[0][0]*P_NEXT[0][0]+H[0][1]*P_NEXT[1][0];
	N[0][1]=H[0][0]*P_NEXT[1][0]+H[0][1]*P_NEXT[1][1];
	
	D[0][0]=(N[0][0]*HT[0][0]+N[0][1]*HT[1][0])+R[0][0];
	
	DT[0][0]=1/D[0][0];
	
	K[0][0]=L[0][0]*DT[0][0];
	K[1][0]=L[1][0]*DT[0][0];
	
}
void FORTH_STEP(float Z)
{
	
	
	S[0][0]=Z-(H[0][0]*XHAT_NEXT[0][0]+H[0][1]*XHAT_NEXT[1][0]);
	
	XHAT[0][0]=XHAT_NEXT[0][0]+ K[0][0]*S[0][0];
	XHAT[1][0]=XHAT_NEXT[1][0]+ K[1][0]*S[0][0];
	
}
void FIFTH_STEP(void)
{
	
	O[0][0]=T[0][0]-K[0][0]*H[0][0];
	O[0][1]=T[0][1]-K[0][0]*H[0][1];
	O[1][0]=T[1][0]-K[1][0]*H[0][0];
	O[1][1]=T[1][1]-K[1][0]*H[0][1];
	
	P[0][0]=O[0][0]*P_NEXT[0][0]+O[0][1]*P_NEXT[1][0];
	P[0][1]=O[0][0]*P_NEXT[0][1]+O[0][1]*P_NEXT[1][1];
	P[1][0]=O[1][0]*P_NEXT[0][0]+O[1][1]*P_NEXT[1][0];
	P[1][1]=O[1][0]*P_NEXT[0][1]+O[1][1]*P_NEXT[1][1];
	
	camera_yaw=XHAT[0][0];
	V_SPEED=XHAT[1][0];
}	