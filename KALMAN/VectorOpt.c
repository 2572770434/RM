#include<math.h>

#include"VectorOpt.h"

//---------------------------------------------------------------------------
/*求一个n维矢量的模*/
double Norm(double *v, int n)
{
	int i;

	double sum = 0.0;

	for(i = 0; i < n; i++)
	{
		sum += v[i]*v[i];
	}
	
	return sqrt(sum);
}
//---------------------------------------------------------------------------
/*矢量的"点积"--Vector dot product*/
double dot(double *a, double *b, int n)
{
	int i;
	double sum = 0.0;
	
	for(i = 0; i < n; i++)
	{
		sum += a[i]*b[i];
	}
	
	return sum;
}
//---------------------------------------------------------------------------
/*矢量的"叉乘"--Vector cross product*/
/*这里只考虑了3维数矢量的叉乘运算*/
double* cross(double *a, double *b, double *c)
{
	c[0] = a[1]*b[2] - a[2]*b[1];
	c[1] = a[2]*b[0] - a[0]*b[2];
	c[2] = a[0]*b[1] - a[1]*b[0];

	return c;
}
//---------------------------------------------------------------------------
/*3维矢量对应的斜对称矩阵.*/
double* skewsymm(double *vect, double *S)
{
	double sigma_x, sigma_y, sigma_z;

	sigma_x = vect[0];
	sigma_y = vect[1];
	sigma_z = vect[2];

	S[0] = 0.0; 		S[1] =-sigma_z;  S[2] = sigma_y;
	S[3] = sigma_z; 	S[4] = 0.0; 	 S[5] =-sigma_x;
	S[6] =-sigma_y;     S[7] = sigma_x;  S[8] = 0.0;

	return S;
}
//---------------------------------------------------------------------------
/*
void VectorAdd(double alpha, double *a, double *b, double *c)
{
	c[0] = alpha*a[0]+b[0];
	c[1] = alpha*a[1]+b[1];
	c[2] = alpha*a[2]+b[2];
}
*/
//---------------------------------------------------------------------------
/*
void VectorAdd(double *a, double *b, double alpha, double *c)
{
	c[0] = alpha*(a[0]+b[0]);
	c[1] = alpha*(a[1]+b[1]);
	c[2] = alpha*(a[2]+b[2]);
}
*/
//---------------------------------------------------------------------------
// c = alpha*a + beta*b
double* GeneralVectorAdd(double alpha, double *a, double beta, double *b, double *c)
{
	c[0] = alpha*a[0] + beta*b[0];
	c[1] = alpha*a[1] + beta*b[1];
	c[2] = alpha*a[2] + beta*b[2];

	return c;
}
//---------------------------------------------------------------------------
double* VectorAdd(double *a, double *b, double *c)
{
    c[0] = a[0]+b[0];
    c[1] = a[1]+b[1];
    c[2] = a[2]+b[2];

	return c;
}
//---------------------------------------------------------------------------
//完成c=a-b矢量运算
double* VectorSub(double *a, double *b, double *c)
{
	c[0] = a[0]-b[0];
	c[1] = a[1]-b[1];
	c[2] = a[2]-b[2];

	return c;
}
//---------------------------------------------------------------------------
//完成b=alpha*a
double* VectorTimes(double alpha, double *a, double *b)
{
	b[0]=alpha*a[0];
	b[1]=alpha*a[1];
	b[2]=alpha*a[2];

	return b;
}
//---------------------------------------------------------------------------
/*
double* VectorCross(double *a, double *b, double *c)
{
	c[0] = a[1]*b[2] - a[2]*b[1];
	c[1] = a[2]*b[0] - a[0]*b[2];
	c[2] = a[0]*b[1] - a[1]*b[0];

	return c;
}
*/
//---------------------------------------------------------------------------
/*
double VectorDot(double *a, double *b)
{
	return (a[0]*b[0]+a[1]*b[1]+a[2]*b[2]);
}
*/
//---------------------------------------------------------------------------
