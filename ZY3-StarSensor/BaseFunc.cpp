#include "BaseFunc.h"



BaseFunc::BaseFunc()
{
}


BaseFunc::~BaseFunc()
{
}

// 法化
void BaseFunc::pNormal(double *a, int n, double b, double *aa, double *ab, double p)
{
	int i, j;
	for (i = 0; i<n; i++)
	{
		for (j = 0; j<n; j++)
			*aa++ += p*a[i] * a[j];
		*ab++ += p*a[i] * b;
	}
}

int BaseFunc::Gauss(double *ATA, double *ATL, int n)
{
	double *ATAinv = new double[n*n];
	double *temp = new double[n];
	Inv(ATA, ATAinv, n);
	Multi(ATAinv, ATL, temp, n, n, 1);
	memcpy(ATL, temp, sizeof(double)*n);
	delete[]ATAinv;	ATAinv = NULL;
	delete[]temp;		temp = NULL;
	return 1;
}

// 求矩阵相乘,A矩阵为[m,p],B矩阵为[p,n],C为[m,n] 
void BaseFunc::Multi(double *A, double *B, double *C, int m, int p, int n)
{
	for (int i = 0; i<m; i++)
		for (int j = 0; j<n; j++)
		{
			double sum = 0;
			for (int k = 0; k<p; k++)
				sum = sum + A[i*p + k] * B[k*n + j];
			C[i*n + j] = sum;
		}
}

double BaseFunc::Det(double *A, int m)
{
	int i = 0, ii = 0, j = 0, jj = 0, k = 0, t = 0, tt = 1;
	double det = 1, mk = 0;
	double *pA = new double[m*m];
	double *pB = new double[m];
	for (i = 0; i<m; i++)
	{
		pB[i] = 0;
		for (j = 0; j<m; j++)
			pA[i*m + j] = A[i*m + j];
	}
	for (k = 0; k<m; k++)
	{
		for (j = k; j<m; j++)
			if (pA[k*m + j])
			{
				for (i = 0; i<m; i++)
				{
					pB[i] = pA[i*m + k];
					pA[i*m + k] = pA[i*m + j];
					pA[i*m + j] = pB[i];
				}
				if (j - k)
					tt = tt*(-1);
				t = t + 1;
				break;
			}
		if (t)
		{
			for (ii = k + 1; ii<m; ii++)
			{
				mk = (-1)*pA[ii*m + k] / pA[k*m + k];
				pA[ii*m + k] = 0;
				for (jj = k + 1; jj<m; jj++)
					pA[ii*m + jj] = pA[ii*m + jj] + mk*pA[k*m + jj];
			}
			det = det*pA[k*m + k];
			t = 0;
		}
		else
		{
			det = 0;
			break;
		}
	}
	det = det*tt;
	delete pA;	pA = NULL;
	delete pB;	pB = NULL;
	return det;
}

// 求A的逆矩阵C 
void BaseFunc::Inv(double *A, double *C, int m)
{
	int i, j, x0, y0;
	double M = 0;
	double *SP = new double[m*m];
	double *AB = new double[m*m];
	double *B = new double[m*m];
	M = Det(A, m);
	if (M == 0.0)
		return;
	M = 1 / M;
	for (i = 0; i<m; i++)
	{
		for (j = 0; j<m; j++)
		{
			for (x0 = 0; x0<m; x0++)
				for (y0 = 0; y0<m; y0++)
					B[x0*m + y0] = A[x0*m + y0];
			for (x0 = 0; x0<m; x0++)
				B[x0*m + j] = 0;
			for (y0 = 0; y0<m; y0++)
				B[i*m + y0] = 0;
			B[i*m + j] = 1;
			SP[i*m + j] = Det(B, m);
			SP[i*m + j] = SP[i*m + j];
			AB[i*m + j] = M*SP[i*m + j];
		}
	}
	Transpose(AB, C, m, m);
	delete SP;		SP = NULL;
	delete AB;		AB = NULL;
	delete B;		B = NULL;
}

// 求矩阵转置，形参m为行，n为列,A转置后存为B 
void BaseFunc::Transpose(double *A, double *B, int m, int n)
{
	for (int i = 0; i<n; i++)
		for (int j = 0; j<m; j++)
			B[i*m + j] = A[j*n + i];
}