#ifndef VectorOpt_H
#define VectorOpt_H

double Norm(double *v, int n);
double dot(double *a, double *b, int n);
double* cross(double *a, double *b, double *c);
double* skewsymm(double *vect, double *S);

//void VectorAdd(double alpha, double *a, double *b, double *c);
//void VectorAdd(double *a, double *b, double alpha, double *c);
double* VectorAdd(double *a, double *b, double *c);
double* GeneralVectorAdd(double alpha, double *a, double beta, double *b, double *c);

double* VectorSub(double *a, double *b, double *c);
double* VectorTimes(double alpha, double *a, double *b);
double* VectorCross(double *a, double *b, double *c);
double VectorDot(double *a, double *b);

#endif
