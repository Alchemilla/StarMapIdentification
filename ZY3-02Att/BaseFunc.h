#ifndef BASEFUNC_H
#define BASEFUNC_H
#include "SateBase.h"
#include <string>
#include <vector>
typedef unsigned char byte;
float ReverseQ (long Qtemp);
float Reverse2 (long Qtemp);
double tran2double(unsigned char *pData);
double RevDouble (unsigned char a[]);
void mult(double *m1,double *m2,double *result,int a,int b,int c);
void quat2matrix(double q1,double q2,double q3,double q0,double *R);
void matrix2quat(double *R,double &q1,double &q2,double &q3,double &q0);
void crossmultnorm(double *x,double *y,double *z);
void normalvect(double *x,double *y);
int invers_matrix(double *m1,int n);
void rotationZXY(double x,double y,double z,double *r);
void quatmult(double *q1,double *q2,double *q3);//四元数顺序为0123，其中0为标量
void QuatInterpolation(Quat *Att, int AttNum, double *UTC, int interNum,Quat *&m_att);
void LagrangianInterpolation(Orbit_Ep *Eph, long EphNum, double UTC, Orbit_Ep &m_point, int order);
#endif