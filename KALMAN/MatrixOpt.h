#ifndef MatrixOptH
#define MatrixOptH

//void ones(int n, double *Y);
double* ones(int m, int n, double *Y);
void eye(int n, double *I);

double trace(double *src, int m, int n);

double* MatrixTrans(double *A, int m, int n, double *B);
double* transpose(double *scr, int m, int n, double *dst);

double* GEMA(double alpha, double *A, double beta, double *B, int m, int n, double *C);
double* MatrixAdd(double *A, double *B, int m, int n, double *C);
double* MatrixSub(double *A, double *B, int m, int n, double *C);

double* GEMM(double alpha, double *A, double *B, double beta, double *C, int m, int n, int p, double *dst);
double* MatrixMulAdd(double *A, double *B, double *C, int m, int n, int p, double *dst);
double* MatrixMulti(double *A, double *B, int m, int n, int p, double *C);

void MatrixZero(double *A, int m, int n);
void MatrixOnes(double *A,int n);
void MatrixDiag(double *A, double *V, int n);
double* MatrixTimes(double cnst, double *A, int m, int n, double *B);
void MatrixEval(double *dest, double *src, int m, int n);

//通用方阵求逆
void MatrixInvert(double *A, int n, double *B);
void Mat2Inv(double *src, double *dst);
void Mat3Inv(double *src, double *dst);

#endif
