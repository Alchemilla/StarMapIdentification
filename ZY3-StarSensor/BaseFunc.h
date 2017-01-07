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
	// 法化
	void pNormal(double *a, int n, double b, double *aa, double *ab, double p);
	// 高斯求解
	int Gauss(double *ATA, double *ATL, int n);
	// 求矩阵相乘,A矩阵为[m,p],B矩阵为[p,n],C为[m,n] 
	void Multi(double *A, double *B, double *C, int m, int p, int n);
	// 求A的逆矩阵C 
	void Inv(double *A, double *C, int m);
	// 求矩阵行列式
	double Det(double *A, int m);
	// 求矩阵转置，形参m为行，n为列,A转置后存为B 
	void Transpose(double *A, double *B, int m, int n);
};

