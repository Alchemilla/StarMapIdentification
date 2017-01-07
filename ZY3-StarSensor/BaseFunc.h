#pragma once
#ifndef NULL 0
#define NULL 0
#endif // !1
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
	// ���������ʽ
	double Det(double *A, int m);
	// �����ת�ã��β�mΪ�У�nΪ��,Aת�ú��ΪB 
	void Transpose(double *A, double *B, int m, int n);
};

