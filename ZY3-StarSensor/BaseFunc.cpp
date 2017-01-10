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

// 求m1的逆矩阵C 
int BaseFunc::invers_matrix(double *m1, int n)
{
	int *is, *js;

	int i, j, k, l, u, v;

	double temp, max_v;

	is = (int *)malloc(n * sizeof(int));

	js = (int *)malloc(n * sizeof(int));

	if (is == NULL || js == NULL)
	{

		printf("out of memory!\n");

		return(0);

	}

	for (k = 0; k<n; k++) {
		max_v = 0.0;
		for (i = k; i<n; i++)
			for (j = k; j<n; j++) {
				temp = fabs(m1[i*n + j]);
				if (temp>max_v) {
					max_v = temp; is[k] = i; js[k] = j;
				}
			}
		if (max_v == 0.0) {
			free(is); free(js);
			printf("invers is not availble!\n");
			return(0);
		}
		if (is[k] != k)
			for (j = 0; j<n; j++) {
				u = k*n + j; v = is[k] * n + j;
				temp = m1[u]; m1[u] = m1[v]; m1[v] = temp;
			}
		if (js[k] != k)
			for (i = 0; i<n; i++) {
				u = i*n + k; v = i*n + js[k];
				temp = m1[u]; m1[u] = m1[v]; m1[v] = temp;
			}
		l = k*n + k;
		m1[l] = 1.0 / m1[l];
		for (j = 0; j<n; j++)
			if (j != k) {
				u = k*n + j;
				m1[u] *= m1[l];
			}
		for (i = 0; i<n; i++)
			if (i != k)
				for (j = 0; j<n; j++)
					if (j != k) {
						u = i*n + j;
						m1[u] -= m1[i*n + k] * m1[k*n + j];
					}
		for (i = 0; i<n; i++)
			if (i != k) {
				u = i*n + k;
				m1[u] *= -m1[l];
			}
	}
	for (k = n - 1; k >= 0; k--) {
		if (js[k] != k)
			for (j = 0; j<n; j++) {
				u = k*n + j; v = js[k] * n + j;
				temp = m1[u]; m1[u] = m1[v]; m1[v] = temp;
			}
		if (is[k] != k)
			for (i = 0; i<n; i++) {
				u = i*n + k; v = i*n + is[k];
				temp = m1[u]; m1[u] = m1[v]; m1[v] = temp;
			}
	}
	free(is); free(js);
	return(1);
}

// 求矩阵转置，形参m为行，n为列,A转置后存为B 
void BaseFunc::Transpose(double *A, double *B, int m, int n)
{
	for (int i = 0; i<n; i++)
		for (int j = 0; j<m; j++)
			B[i*m + j] = A[j*n + i];
}

double BaseFunc::RevDouble(unsigned char a[])
{
	union Tpacket
	{
		unsigned char a[8];
		double b;
	}p;
	for (int i = 0; i<8; i++)
	{
		p.a[i] = a[7 - i];
	}
	double y = p.b;
	return y;
}

float BaseFunc::ReverseQ(long Qtemp)
{
	float Q;
	long Qt;
	Qt = (((Qtemp&(0xff000000)) >> 24) | ((Qtemp&(0x00ff0000)) >> 8) |
		((Qtemp&(0x0000ff00)) << 8) | ((Qtemp&(0x000000ff)) << 24));
	Q = Qt / 1073741824.0;
	return Q;
}

float BaseFunc::Reverse2(long Qtemp)
{
	float Q;
	long Qt;
	Qt = (((Qtemp&(0xff000000)) >> 24) | ((Qtemp&(0x00ff0000)) >> 8) |
		((Qtemp&(0x0000ff00)) << 8) | ((Qtemp&(0x000000ff)) << 24));
	Q = (float&)Qt;
	return Q;
}

void BaseFunc::quat2matrix(double q1, double q2, double q3, double q0, double *R)
{
	R[0] = q1*q1 - q2*q2 - q3*q3 + q0*q0;
	R[1] = 2 * (q1*q2 + q3*q0);
	R[2] = 2 * (q1*q3 - q2*q0);

	R[3] = 2 * (q1*q2 - q3*q0);
	R[4] = -q1*q1 + q2*q2 - q3*q3 + q0*q0;
	R[5] = 2 * (q2*q3 + q1*q0);

	R[6] = 2 * (q1*q3 + q2*q0);
	R[7] = 2 * (q2*q3 - q1*q0);
	R[8] = -q1*q1 - q2*q2 + q3*q3 + q0*q0;

	return;
}

void BaseFunc::QuatInterpolation(Quat *Att, int AttNum, double *UTC, int interNum, Quat *&m_att)
{
	if (AttNum<2) { printf("QuatInterpolation Error：AttNum is less than 2, can't interpolation!\n");	return; }
	// 寻找临近的两个点(对分查找)
	Quat attleft, attright, att;
	long posstart, posend, pos;
	for (int i = 0; i<interNum; i++)
	{
		posstart = 0, posend = AttNum - 1, pos = 0;
		while (posstart<posend)
		{
			pos = (posstart + posend) / 2;
			if (pos == posstart) break;	// 记得加上这句判断,否则会陷入死循环
			if ((Att[pos].UTC <= UTC[i]) && (Att[pos + 1].UTC>UTC[i]))
				break;
			if (Att[pos].UTC <= UTC[i])
				posstart = pos;
			else
				posend = pos;
		}
		if (pos < 0)	pos = 0;
		if (pos >= AttNum - 1)		pos = AttNum - 2;
		attleft = Att[pos];		attright = Att[pos + 1];

		// 进行内插
		double sp, sq;
		double t = (UTC[i] - attleft.UTC) / (attright.UTC - attleft.UTC);
		double cosa = attleft.Q0*attright.Q0 + attleft.Q1*attright.Q1 + attleft.Q2*attright.Q2 + attleft.Q3*attright.Q3;
		// 这个错误需要注意了,防止邻近两个值互为反号的情况,需要确保length>0
		if (cosa<0)
		{
			cosa = -cosa;
			attright.Q0 = -attright.Q0;	attright.Q1 = -attright.Q1;	attright.Q2 = -attright.Q2;	attright.Q3 = -attright.Q3;
		}
		if (cosa>0.9999f)
		{
			sp = 1.0 - t;	sq = t;
		}
		else
		{
			double sina = sqrt(1.0 - pow(cosa, 2));	double a = atan2(sina, cosa);	double invSina = 1.0 / sina;
			sp = sin((1.0 - t)*a)*invSina;			sq = sin(t*a)*invSina;
		}
		m_att[i].Q0 = sp*attleft.Q0 + sq*attright.Q0;	m_att[i].Q1 = sp*attleft.Q1 + sq*attright.Q1;
		m_att[i].Q2 = sp*attleft.Q2 + sq*attright.Q2;	m_att[i].Q3 = sp*attleft.Q3 + sq*attright.Q3;
		m_att[i].UTC = UTC[i];
	}
}