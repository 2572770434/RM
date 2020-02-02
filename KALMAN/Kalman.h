#ifndef Kalman_H
#define Kalman_H


typedef struct 
{
	/*测量向量维数*/
	int MP;
	/*状态向量维数*/
	int DP;
	/*控制向量维数*/
	int CP;

	/*预测状态x'(k): x(k)=A*x(k-1)+B*u(k)*/
	double *state_pre;
	/*估计状态x(k): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))*/
	double *state_post;

	/*状态传递矩阵*/
	double *transition_matrix;
	/*状态传递矩阵的转置*/
	double transition_matrix_T[15*15];

	/*控制矩阵, 如果没有控制，则不使用它*/
	double *control_matrix;
	/*测量矩阵H*/
	double *measurement_matrix;
	
	/*测量矩阵H的转置HT*/
	double measurement_matrix_T[15*3];

	/*过程噪声协方差矩阵Q*/
	double *process_noise_cov;
	/*测量噪声协方差矩阵R*/
	double *measurement_noise_cov;
	/*先验误差计协方差矩阵P'(k)*/
	double *error_cov_pre;

	/*Kalman 增益矩阵K(k)*/
	double *gain;
	
	/*后验错误估计协方差矩阵P(k)*/
	double *error_cov_post;

	/*临时矩阵*/
	double temp1[15*15];
	double temp2[15*3];
	double temp3[3*3];
	double temp4[3*3];
	double temp5[3];
	double temp6[3*15];
}Kalman;

extern Kalman KALMAN1;

void InitKalman(Kalman *kalman);
void CreateKalman(Kalman *kalman);
double KalmanCorrect(Kalman* kalman, double* measurement);
void KalmanPredict(Kalman *kalman);
#endif
