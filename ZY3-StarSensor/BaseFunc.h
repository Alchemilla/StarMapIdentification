#pragma once
#ifndef NULL 0
#define NULL 0
#endif // !1
#include "SateBase.h"
typedef unsigned char byte;
#include <string>

class BaseFunc
{
public:
	BaseFunc();
	~BaseFunc();
	// ����
	void pNormal(double *a, int n, double b, double *aa, double *ab, double p);
	// ��˹���
	int Gauss(double *ATA, double *ATL, int n);
	// ��������,A����Ϊ[m,p],B����Ϊ[p,n],CΪ[m,n] 
	void Multi(double *A, double *B, double *C, int m, int p, int n);
	// ��A�������C 
	void Inv(double *A, double *C, int m);
	int invers_matrix(double *m1, int n);
	// ���������ʽ
	double Det(double *A, int m);
	// �����ת�ã��β�mΪ�У�nΪ��,Aת�ú��ΪB 
	void Transpose(double *A, double *B, int m, int n);
	//��Ԫ��ת��ת����
	void quat2matrix(double q1, double q2, double q3, double q0, double *R);
	//��Ԫ���ڲ壬��Ԫ��˳��Ϊ0123������0Ϊ����
	void QuatInterpolation(Quat *Att, int AttNum, double *UTC, int interNum, Quat *&m_att);
	//����STGҪ�õ�������С����
	double RevDouble(unsigned char a[]);
	float ReverseQ(long Qtemp);
	float Reverse2(long Qtemp);
};

