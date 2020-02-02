#include"Kalman.h"
#include"MatrixOpt.h"

#define r1 2
#define c1 2
#define r2 1
#define c2 1

//double A[r1][c1]={{1,0,1,0},{0,1,0,1},{0,0,1,0},{0,0,0,1}};
//double H[r2][c1]={{1,0,0,0},{0,1,0,0}};
//double Q[r1][c1]={{0.0000001,0,0,0},{0,0.0000001,0,0},{0,0,0.0000001,0},{0,0,0,0.0000001}};
//double R[r2][c2]={{0.05},{0.05}};
//double P[r1][c1]={{100,0,0,0},{0,100,0,0},{0,0,100,0},{0,0,0,100}};


//---------------------------------------------------------------------------

Kalman KALMAN1={0};

void CreateKalman(Kalman *kalman)
{
	//状态向量维数
	kalman->DP = 2;
	//测量状态维数
	kalman->MP = 1;
	kalman->CP = 0;
}
//---------------------------------------------------------------------------
void InitKalman(Kalman *kalman)
{
	double A[r1][c1]={{1,4},{0,1}};
	double H[r2][c1]={{1,0}};
	double Q[r1][c1]={{0.0000001,0},{0,0.0000001}};
	double R[r2][c2]={{0.05}};
	double P[r1][c1]={{100,0},{0,100}};
	double xhat_data[2][1]={0.0}, xhatminus_data[2][1]={0.0}, Pminus_data[2][2]={0.0}, K_data[2][1]={0.0};
	//状态转移矩阵A
	kalman->transition_matrix = *A;
	
	
	//预测状态x_pre 和估计状态x
	kalman->state_pre = *xhatminus_data;
	kalman->state_post = *xhat_data;

	//量测矩阵
	kalman->measurement_matrix = *H;

	//增益矩阵K
	kalman->gain = *K_data;

	/*后验错误估计协方差矩阵P(k)*/
	kalman->error_cov_post = *P;
	/*先验误差计协方差矩阵P'(k)*/
  kalman->error_cov_pre = *Pminus_data;
	
	//过程(系统)噪声和测量噪声
	kalman->process_noise_cov = *Q;
	kalman->measurement_noise_cov = *R;

}
//---------------------------------------------------------------------------
void KalmanPredict(Kalman *kalman)  //, double *control
{
	/*update the state*/
    /*x'(k) = A*x(k)*/
	MatrixMulti(kalman->transition_matrix, kalman->state_post, kalman->DP, kalman->DP, 1, kalman->state_pre);

	/*x'(k) = x'(k) + B*u(k)*/
//	if(control && kalman->CP > 0)
//	{
//		MatrixMulAdd(kalman->control_matrix, control, kalman->state_pre, kalman->DP, kalman->CP, 1, kalman->state_pre);
//	}
	
	/* update error covariance matrices */
    /* temp1 = A*P(k) */
	MatrixMulti(kalman->transition_matrix, kalman->error_cov_post, kalman->DP, kalman->DP, kalman->DP, kalman->temp1);

    MatrixTrans(kalman->transition_matrix, kalman->DP, kalman->DP, kalman->transition_matrix_T);

    /*这里应该是计算Pkk_1*/
    MatrixMulAdd(kalman->temp1, kalman->transition_matrix_T, kalman->process_noise_cov, kalman->DP, kalman->DP, kalman->DP, kalman->error_cov_pre);
}
//---------------------------------------------------------------------------
double KalmanCorrect(Kalman* kalman, double* measurement)
{
	/*temp2 = H*P'(k)*/
	//MatrixMul(kalman->measurement_matrix, kalman->error_cov_pre, kalman->temp2);

	/*temp2 = P'(k)*HT*/
	//MatrixMulti(kalman->error_cov_pre, kalman->measurement_matrix, kalman->DP, kalman->DP, kalman->MP, kalman->temp2);

    /*上述语句有错误, kalman->measurement_matrix -> kalman->measurement_matrix_T*/
    MatrixTrans(kalman->measurement_matrix, kalman->MP, kalman->DP, kalman->measurement_matrix_T);
    
	
    MatrixMulti(kalman->error_cov_pre, kalman->measurement_matrix_T, kalman->DP, kalman->DP, kalman->MP, kalman->temp2);

	/*temp3 = H*Pkk_1*HT + R*/
	MatrixMulAdd(kalman->measurement_matrix, kalman->temp2, kalman->measurement_noise_cov, kalman->MP, kalman->DP, kalman->MP, kalman->temp3);

	/*temp4 = inv(H*Pkk_1*HT+R)*/
	MatrixInvert(kalman->temp3, kalman->MP, kalman->temp4);
	
	/*K(k) = P'(k)*HT*(H*Pkk_1*HT+R)^-1*/
	MatrixMulti(kalman->temp2, kalman->temp4, kalman->DP, kalman->MP, kalman->MP, kalman->gain);


	/*temp5 = z(k) - H*xkk_1, temp5维数：M×1*/
	//GEMM(-1.0, kalman->measurement_matrix, kalman->state_post, 1.0, measurement, kalman->MP, kalman->DP, 1, kalman->temp5);
    GEMM(-1.0, kalman->measurement_matrix, kalman->state_pre, 1.0, measurement, kalman->MP, kalman->DP, 1, kalman->temp5);

	/*xk = xkk_1 + K(k)*(z(k)-H*xkk_1)*/
	MatrixMulAdd(kalman->gain, kalman->temp5, kalman->state_pre, kalman->DP, kalman->MP, 1, kalman->state_post);

    /*temp6 = H*P'(k)*/
	MatrixMulti(kalman->measurement_matrix, kalman->error_cov_pre, kalman->MP, kalman->DP, kalman->DP, kalman->temp6);

    GEMM(-1.0, kalman->gain, kalman->temp6, 1.0, kalman->error_cov_pre, kalman->DP, kalman->MP, kalman->DP, kalman->error_cov_post);
		
		return kalman->state_post[0];
}
//---------------------------------------------------------------------------
//double* MatrixAdd(double *A, double *B, int m, int n, double *C)
//{
//	int i,j,k;
//	float c[m][n];
//	for(i=0;i<m;i++)
//	{
//		for(j=0;j<n;j++)
//		{
//			*(C++)=*(A++)+*(B++);				
//		}
//	}
//	return C;
//}

//double* MatrixSub(double *A, double *B, int m, int n, double *C)
//{
//	int i,j,k;
//	float c[m][n];
//	for(i=0;i<m;i++)
//	{
//		for(j=0;j<n;j++)
//		{
//			*(C++)=*(A++)-*(B++);					
//		}
//	}
//	return C;
//}


	
