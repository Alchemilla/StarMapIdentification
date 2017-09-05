#include "BaseFunc.h"



BaseFunc::BaseFunc()
{
}


BaseFunc::~BaseFunc()
{
}

// ����
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

//�����޵ȣ������������������ڲ������ݴ����е�Ӧ��
void BaseFunc::GaussExt(double *ATA, double *ATL, double *x, int n)
{
	double *ATAinv = new double[n*n];
	long i;
	int num = 0;
	for (int i = 0; i<n; i++)
	{
		ATA[i*n + i] += 1;
	}
	Inv(ATA, ATAinv, n);
	double *temp = new double[n];
	double *temp1 = new double[n];
	double dx0 = 1e10, dx = 1e10, dxx = 0;
	do
	{
		dx0 = dx;
		memcpy(temp1, x, sizeof(double)*n);
		for (i = 0; i<n; i++)
			temp[i] = ATL[i] + x[i];
		Multi(ATAinv, temp, x, n, n, 1);
		dx = 0;
		for (i = 0; i<n; i++)
		{
			dx += (x[i] - temp1[i])*(x[i] - temp1[i]);
		}
		num++;
	} while (num<10000 && dx<dx0);;
	delete[]temp;		temp = NULL;
	delete[]temp1;		temp1 = NULL;
	delete[]ATAinv;	ATAinv = NULL;
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

// ��������,A����Ϊ[m,p],B����Ϊ[p,n],CΪ[m,n] 
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

// ��A�������C 
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

// ��m1�������C 
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

// ��ȡ������ģ
double BaseFunc::Norm(double *R, int num)
{
	double retVal = 0.0;
	for (int i = 0; i<num; i++)
		retVal += pow(R[i], 2);
	return sqrt(retVal);
}

// ���������й�һ��
void BaseFunc::NormVector(double *R, int num)
{
	double retVal = 0.0;
	for (int i = 0; i < num; i++)
		retVal += pow(R[i], 2);
	retVal = sqrt(retVal);
	for (int i = 0; i < num; i++)
		R[i] /= retVal;
}
void BaseFunc::normalvect(double *x, double *y)
{
	double t;
	t = sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
	if (fabs(t)<10e-8)
	{
		memcpy(y, x, sizeof(double) * 3);
		return;
	}
	y[0] = x[0] / t; y[1] = x[1] / t; y[2] = x[2] / t;
}
//��������������
void BaseFunc::crossmultnorm(double *x, double *y, double *z)
{
	z[0] = x[1] * y[2] - x[2] * y[1];
	z[1] = x[2] * y[0] - x[0] * y[2];
	z[2] = x[0] * y[1] - x[1] * y[0];

	double t;
	t = sqrt(z[0] * z[0] + z[1] * z[1] + z[2] * z[2]);
	if (fabs(t)<10e-8)
		return;
	z[0] /= t; z[1] /= t; z[2] /= t;
}

// �����ת�ã��β�mΪ�У�nΪ��,Aת�ú��ΪB 
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

void BaseFunc::quatMult(double *q1, double *q2, double *q3)//��Ԫ��˳��Ϊ1234������4Ϊ������������תq1
{
	q3[0] = q1[3] * q2[0] - q1[2] * q2[1] + q1[1] * q2[2] + q1[0] * q2[3];
	q3[1] = q1[2] * q2[0] + q1[3] * q2[1] - q1[0] * q2[2] + q1[1] * q2[3];
	q3[2] = -q1[1] * q2[0] + q1[0] * q2[1] + q1[3] * q2[2] + q1[2] * q2[3];
	q3[3] = -q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] + q1[3] * q2[3];
}

void BaseFunc::quatMult3(double *q1, double *q2, double *q3)//��Ԫ��˳��Ϊ1234������4Ϊ������������תq2
{
	q3[0] = q1[3] * q2[0] + q1[2] * q2[1] - q1[1] * q2[2] + q1[0] * q2[3];
	q3[1] = -q1[2] * q2[0] + q1[3] * q2[1] + q1[0] * q2[2] + q1[1] * q2[3];
	q3[2] = q1[1] * q2[0] - q1[0] * q2[1] + q1[3] * q2[2] + q1[2] * q2[3];
	q3[3] = -q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] + q1[3] * q2[3];
}

void BaseFunc::quatMult2(double *q1, double *q2, double *q3)//��Ԫ��˳��Ϊ0123������0Ϊ����
{
	q3[1] = q1[0] * q2[1] - q1[3] * q2[2] + q1[2] * q2[3] + q1[1] * q2[0];
	q3[2] = q1[3] * q2[1] + q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0];
	q3[3] = -q1[2] * q2[1] + q1[1] * q2[2] + q1[0] * q2[3] + q1[3] * q2[0];
	q3[0] = -q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3] + q1[0] * q2[0];
}
////////////////////////////////////////////////////////
// ��ת������ŷ���ǵ��໥�任
////////////////////////////////////////////////////////
//////////////////////////////////////
// ���ܣ�����ת������ŷ����
// ����:
//		double *R��		��ת����3*3��������	
//		int ratateOrder:ŷ����ת��
// �����
//		double &eulor1:	ŷ����1
//		double &eulor2:	ŷ����2
//		double &eulor3:	ŷ����3
// ����ֵ��
//		void
///////////////////////////////////////
void BaseFunc::Matrix2Eulor(double *R, int rotateOrder, double &eulor1, double &eulor2, double &eulor3)
{
	switch (rotateOrder)
	{
		// ��һ��:��һ�κ͵�����ת������ͬ����������е�,�ڶ���ת�������������е�һ����е�
	case 121:  // 1
	{	// eulor1ֵ��-pi��pi��eulor2ֵ��0��pi��eulor3ֵ��-pi��pi
		eulor2 = acos(R[0]);					double temp = sin(eulor2);
		eulor1 = atan2(R[1] * temp, -R[2] * temp);	eulor3 = atan2(R[3] * temp, R[6] * temp);	break;
	}
	case 131:	// 2
	{	// eulor1ֵ��-pi��pi��eulor2ֵ��0��pi��eulor3ֵ��-pi��pi
		eulor2 = acos(R[0]);					double temp = sin(eulor2);
		eulor1 = atan2(R[2] * temp, R[1] * temp);	eulor3 = atan2(R[6] * temp, -R[3] * temp);	break;
	}
	case 212:	// 3
	{	// eulor1ֵ��-pi��pi��eulor2ֵ��0��pi��eulor3ֵ��-pi��pi
		eulor2 = acos(R[4]);					double temp = sin(eulor2);
		eulor1 = atan2(R[3] * temp, R[5] * temp);	eulor3 = atan2(R[1] * temp, -R[7] * temp);	break;
	}
	case 232:	// 4
	{	// eulor1ֵ��-pi��pi��eulor2ֵ��0��pi��eulor3ֵ��-pi��pi
		eulor2 = acos(R[4]);					double temp = sin(eulor2);
		eulor1 = atan2(R[5] * temp, -R[3] * temp);	eulor3 = atan2(R[7] * temp, R[1] * temp);	break;
	}
	case 313:	// 5
	{	// eulor1ֵ��-pi��pi��eulor2ֵ��0��pi��eulor3ֵ��-pi��pi
		eulor2 = acos(R[8]);					double temp = sin(eulor2);
		eulor1 = atan2(R[6] * temp, -R[7] * temp);	eulor3 = atan2(R[2] * temp, R[5] * temp);	break;
	}
	case 323:	// 6
	{	// eulor1ֵ��-pi��pi��eulor2ֵ��0��pi��eulor3ֵ��-pi��pi
		eulor2 = acos(R[8]);					double temp = sin(eulor2);
		eulor1 = atan2(R[7] * temp, R[6] * temp);	eulor3 = atan2(R[5] * temp, -R[2] * temp);	break;
	}
	// �ڶ���:ÿ��ת�����Ʋ�ͬ������������е�
	case 123:	// 7
	{	// eulor1ֵ��-pi��pi��eulor2ֵ��-pi/2��pi/2��eulor3ֵ��-pi��pi
		eulor2 = asin(R[6]);					double temp = cos(eulor2);
		eulor1 = atan2(-R[7] * temp, R[8] * temp);	eulor3 = atan2(-R[3] * temp, R[0] * temp);	break;
	}
	case 132:	// 8
	{	// eulor1ֵ��-pi��pi��eulor2ֵ��-pi/2��pi/2��eulor3ֵ��-pi��pi
		eulor2 = asin(-R[3]);					double temp = cos(eulor2);
		eulor1 = atan2(R[5] * temp, R[4] * temp);	eulor3 = atan2(R[6] * temp, R[0] * temp);	break;
	}
	case 213:	// 9
	{	// eulor1ֵ��-pi��pi��eulor2ֵ��-pi/2��pi/2��eulor3ֵ��-pi��pi
		eulor2 = asin(-R[7]);					double temp = cos(eulor2);
		eulor1 = atan2(R[6] * temp, R[8] * temp);	eulor3 = atan2(R[1] * temp, R[4] * temp);	break;
	}
	case 231:	// 10
	{	// eulor1ֵ��-pi��pi��eulor2ֵ��-pi/2��pi/2��eulor3ֵ��-pi��pi
		eulor2 = asin(R[1]);					double temp = cos(eulor2);
		eulor1 = atan2(-R[2] * temp, R[0] * temp);	eulor3 = atan2(-R[7] * temp, R[4] * temp);	break;
	}
	case 312:	// 11
	{	// eulor1ֵ��-pi��pi��eulor2ֵ��-pi/2��pi/2��eulor3ֵ��-pi��pi
		eulor2 = asin(R[5]);					double temp = cos(eulor2);
		eulor1 = atan2(-R[3] * temp, R[4] * temp);	eulor3 = atan2(-R[2] * temp, R[8] * temp);	break;
	}
	case 321:	// 12
	{	// eulor1ֵ��-pi��pi��eulor2ֵ��-pi/2��pi/2��eulor3ֵ��-pi��pi
		eulor2 = asin(-R[2]);					double temp = cos(eulor2);
		eulor1 = atan2(R[1] * temp, R[0] * temp);	eulor3 = atan2(R[5] * temp, R[8] * temp);	break;
	}
	}
}
//////////////////////////////////////
// ���ܣ���ŷ���ǻ����ת����
// ����:
//		double &eulor1:	ŷ����1
//		double &eulor2:	ŷ����2
//		double &eulor3:	ŷ����3
// �����
//		double *R��		��ת����3*3��������	
//		int ratateOrder:ŷ����ת��	
// ����ֵ��
//		void
///////////////////////////////////////
void BaseFunc::Eulor2Matrix(double eulor1, double eulor2, double eulor3, int rotateOrder, double *R)
{
	double R1[9], R2[9], R3[9], Rtemp[9];
	switch (rotateOrder)
	{
		// ��һ��:��һ�κ͵�����ת������ͬ����������е�,�ڶ���ת�������������е�һ����е�
	case 121:	// 1
	{	// eulor1ֵ��-pi��pi��eulor2ֵ��0��pi��eulor3ֵ��-pi��pi
		RotationX(eulor1, R1);	RotationY(eulor2, R2);	RotationX(eulor3, R3);
		Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
	}
	case 131:	// 2
	{	// eulor1ֵ��-pi��pi��eulor2ֵ��0��pi��eulor3ֵ��-pi��pi
		RotationX(eulor1, R1);	RotationZ(eulor2, R2);	RotationX(eulor3, R3);
		Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
	}
	case 212:	// 3
	{	// eulor1ֵ��-pi��pi��eulor2ֵ��0��pi��eulor3ֵ��-pi��pi
		RotationY(eulor1, R1);	RotationX(eulor2, R2);	RotationY(eulor3, R3);
		Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
	}
	case 232:	// 4
	{	// eulor1ֵ��-pi��pi��eulor2ֵ��0��pi��eulor3ֵ��-pi��pi
		RotationY(eulor1, R1);	RotationZ(eulor2, R2);	RotationY(eulor3, R3);
		Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
	}
	case 313:	// 5
	{	// eulor1ֵ��-pi��pi��eulor2ֵ��0��pi��eulor3ֵ��-pi��pi
		RotationZ(eulor1, R1);	RotationX(eulor2, R2);	RotationZ(eulor3, R3);
		Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
	}
	case 323:	// 6
	{	// eulor1ֵ��-pi��pi��eulor2ֵ��0��pi��eulor3ֵ��-pi��pi
		RotationZ(eulor1, R1);	RotationY(eulor2, R2);	RotationZ(eulor3, R3);
		Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
	}
	// �ڶ���:ÿ��ת�����Ʋ�ͬ������������е�
	case 123:	// 7
	{	// eulor1ֵ��-pi��pi��eulor2ֵ��-pi/2��pi/2��eulor3ֵ��-pi��pi
		RotationX(eulor1, R1);	RotationY(eulor2, R2);	RotationZ(eulor3, R3);
		Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
	}
	case 132:	// 8
	{	// eulor1ֵ��-pi��pi��eulor2ֵ��-pi/2��pi/2��eulor3ֵ��-pi��pi
		RotationX(eulor1, R1);	RotationZ(eulor2, R2);	RotationY(eulor3, R3);
		Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
	}
	case 213:	// 9
	{	// eulor1ֵ��-pi��pi��eulor2ֵ��-pi/2��pi/2��eulor3ֵ��-pi��pi
		RotationY(eulor1, R1);	RotationX(eulor2, R2);	RotationZ(eulor3, R3);
		Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
	}
	case 231:	// 10
	{	// eulor1ֵ��-pi��pi��eulor2ֵ��-pi/2��pi/2��eulor3ֵ��-pi��pi
		RotationY(eulor1, R1);	RotationZ(eulor2, R2);	RotationX(eulor3, R3);
		Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
	}
	case 312:	// 11
	{	// eulor1ֵ��-pi��pi��eulor2ֵ��-pi/2��pi/2��eulor3ֵ��-pi��pi
		RotationZ(eulor1, R1);	RotationX(eulor2, R2);	RotationY(eulor3, R3);
		Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
	}
	case 321:	// 12
	{	// eulor1ֵ��-pi��pi��eulor2ֵ��-pi/2��pi/2��eulor3ֵ��-pi��pi
		RotationZ(eulor1, R1);	RotationY(eulor2, R2);	RotationX(eulor3, R3);
		Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
	}
	default:
	{
		printf("Eulor2Matrix Error!\n");	break;	// û�д���ת�����!
	}
	}
}
//////////////////////////////////////////////////////////////////////////
// �������ת
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
// ���ܣ���X��ת��angle����ת����
// [ 1          0         0     ]
// [ 0    cos(angle)  sin(angle)]
// [ 0   -sin(angle)  cos(angle)]
// ����:
//		double angle:	ת���Ľ�(����)
// �����
//		double *R��		��ת����3*3��������	
// ����ֵ��
//		void
//////////////////////////////////////
void BaseFunc::RotationX(double angle, double *R)
{
	memset(R, 0, sizeof(double) * 9);
	R[0] = 1.0;
	R[8] = R[4] = cos(angle);
	R[5] = sin(angle);
	R[7] = -R[5];
}
//////////////////////////////////////
// ���ܣ���Y��ת��angle����ת����
// [cos(angle)  0    -sin(angle)]
// [     0      1         0     ]
// [sin(angle)  0     cos(angle)]
// ����:
//		double angle:	ת���Ľ�(����)
// �����
//		double *R��		��ת����3*3��������	
// ����ֵ��
//		void
//////////////////////////////////////
void BaseFunc::RotationY(double angle, double *R)
{
	memset(R, 0, sizeof(double) * 9);
	R[8] = R[0] = cos(angle);
	R[4] = 1.0;
	R[2] = -sin(angle);
	R[6] = -R[2];
}
//////////////////////////////////////
// ���ܣ���Z��ת��angle����ת����
// [ cos(angle)  sin(angle)  0]
// [-sin(angle)  cos(angle)  0]
// [     0            0      1]
// ����:
//		double angle:	ת���Ľ�(����)
// �����
//		double *R��		��ת����3*3��������	
// ����ֵ��
//		void
//////////////////////////////////////
void BaseFunc::RotationZ(double angle, double *R)
{
	memset(R, 0, sizeof(double) * 9);
	R[4] = R[0] = cos(angle);
	R[8] = 1.0;
	R[1] = sin(angle);
	R[3] = -R[1];
}
// 3*3�ĸ�˹���
bool BaseFunc::solve33(double *A, double *al)
{
	double calresult[3];
	if (fabs(A[0]) < 1e-30)
		return false;
	double L10 = A[3] / A[0];
	double L20 = A[6] / A[0];
	double U11 = A[4] - L10*A[1];
	if (fabs(U11) < 1e-30)
		return false;
	double L21 = (A[7] - (A[1] * L20)) / U11;
	double U12 = A[5] - L10*A[2];
	double U22 = A[8] - L20*A[2] - L21*U12;
	double b0 = al[0];
	double b1 = al[1] - b0*L10;
	double b2 = al[2] - b0*L20 - b1*L21;
	if (fabs(U22) < 1e-30)
		return false;
	calresult[2] = b2 / U22;
	calresult[1] = (b1 - U12*calresult[2]) / U11;
	calresult[0] = (b0 - A[1] * calresult[1] - A[2] * calresult[2]) / A[0];
	memcpy(al, calresult, sizeof(double) * 3);
	return true;
}
void BaseFunc::matrix2quat(double *R, double &q1, double &q2, double &q3, double &q4)
{
	int i, j;	double tq[4];
	tq[0] = 1 + R[0] + R[4] + R[8];	tq[1] = 1 + R[0] - R[4] - R[8];
	tq[2] = 1 - R[0] + R[4] - R[8];	tq[3] = 1 - R[0] - R[4] + R[8];
	// Ѱ�����ֵ
	j = 0;
	for (i = 1; i < 4; i++) j = (tq[i] > tq[j]) ? i : j;
	// ���Խ���
	switch (j)
	{
	case 0: { q4 = tq[0];	q1 = R[5] - R[7]; q2 = R[6] - R[2];	q3 = R[1] - R[3];	break;	}
	case 1: { q4 = R[5] - R[7];	q1 = tq[1];	q2 = R[1] + R[3];	q3 = R[6] + R[2];	break;	}
	case 2: { q4 = R[6] - R[2];	q1 = R[1] + R[3];	q2 = tq[2];	q3 = R[5] + R[7];	break;	}
	case 3: { q4 = R[1] - R[3];	q1 = R[6] + R[2];	q2 = R[5] + R[7]; q3 = tq[3];	break;	}
	default: {	printf("Matrix2Quat Error\n");	return;	}
	}
	double s = sqrt(0.25 / tq[j]);	q1 *= s;	q2 *= s;	q3 *= s;	q4 *= s;
	//double q1q2q3 = sqrt((R[1] + R[3])*(R[2] + R[6])*(R[5] + R[7]));
	//q1 = q1q2q3 / (R[5] + R[7]) / 2;
	//q2 = q1q2q3 / (R[2] + R[6]) / 2;
	//q3 = q1q2q3 / (R[1] + R[3]) / 2;
	//q0 = sqrt((R[0] + R[4] + R[8] + 1) / 4);

	//if (fabs(2 * (q1*q2 + q3*q0) - R[1])>0.0001)
	//	q0 = -q0;
}

void BaseFunc::QuatInterpolation(Quat *Att, int AttNum, double *UTC, int interNum, Quat *&m_att)
{
	if (AttNum<2) { printf("QuatInterpolation Error��AttNum is less than 2, can't interpolation!\n");	return; }
	// Ѱ���ٽ���������(�Էֲ���)
	Quat attleft, attright, att;
	long posstart, posend, pos;
	for (int i = 0; i<interNum; i++)
	{
		posstart = 0, posend = AttNum - 1, pos = 0;
		while (posstart<posend)
		{
			pos = (posstart + posend) / 2;
			if (pos == posstart) break;	// �ǵü�������ж�,�����������ѭ��
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

		// �����ڲ�
		double sp, sq;
		double t = (UTC[i] - attleft.UTC) / (attright.UTC - attleft.UTC);
		double cosa = attleft.Q0*attright.Q0 + attleft.Q1*attright.Q1 + attleft.Q2*attright.Q2 + attleft.Q3*attright.Q3;
		// ���������Ҫע����,��ֹ�ڽ�����ֵ��Ϊ���ŵ����,��Ҫȷ��length>0
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

//////////////////////////////////////////////////////////////////////////
//���ܣ���Ԫ���ڲ�
//���룺Att��ԭʼ��Ԫ����UTC���ڲ�ʱ��㣻interNum��ʱ�����
//�����m_att���ڲ����Ԫ��
//ע�⣺��Ԫ��ʱ�䷶ΧҪ��������ʱ��UTC��Χ
//���ߣ�GZC
//���ڣ�2017.03.23
//////////////////////////////////////////////////////////////////////////
void BaseFunc::QuatInterpolation(vector<Quat>Att, double *UTC, int interNum, Quat *&m_att)
{
	int AttNum = Att.size();
	if (AttNum<2) { printf("QuatInterpolation Error��AttNum is less than 2, can't interpolation!\n");	return; }
	// Ѱ���ٽ���������(�Էֲ���)
	Quat attleft, attright, att;
	long posstart, posend, pos;
	for (int i = 0; i<interNum; i++)
	{
		posstart = 0, posend = AttNum - 1, pos = 0;
		while (posstart<posend)
		{
			pos = (posstart + posend) / 2;
			if (pos == posstart) break;	// �ǵü�������ж�,�����������ѭ��
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

		// �����ڲ�
		double sp, sq;
		double t = (UTC[i] - attleft.UTC) / (attright.UTC - attleft.UTC);
		double cosa = attleft.Q0*attright.Q0 + attleft.Q1*attright.Q1 + attleft.Q2*attright.Q2 + attleft.Q3*attright.Q3;
		// ���������Ҫע����,��ֹ�ڽ�����ֵ��Ϊ���ŵ����,��Ҫȷ��length>0
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

//////////////////////////////////////////////////////////////////////////
//���ܣ�����һ��������̬�ֲ��������
//���룺mean:��ֵ��sigma:����
//����������ֲ�ͼ
//ע�⣺
//���ڣ�2016.10.12 by HWC
//////////////////////////////////////////////////////////////////////////
double BaseFunc::GaussRand(double mean, double sigma, int &phase)
{
	static double v1, v2, s;
	double x;
	if (0 == phase)
	{
		do
		{
			double u1 = (double)rand() / RAND_MAX;
			double u2 = (double)rand() / RAND_MAX;
			v1 = 2 * u1 - 1;
			v2 = 2 * u2 - 1;
			s = v1 * v1 + v2 * v2;
		} while (1 <= s || 0 == s);
		x = v1 * sqrt(-2 * log(s) / s);
	}
	else
	{
		x = v2 * sqrt(-2 * log(s) / s);
	}
	phase = 1 - phase;
	return (x*sigma + mean);    // ע��Ҫ������,���򷵻�ֵ����sigma��
}

//////////////////////////////////////////////////////////////////////////
//���ܣ�����һ�������̬�ֲ��������
//���룺mean:��ֵ��sigma:���n:���������randCount:����0ÿ���������һ��
//�����a:����n���������a
//ע�⣺randCountһ��Ϊ0
//���ڣ�2016.10.12 by HWC
//////////////////////////////////////////////////////////////////////////
double BaseFunc::RandomDistribution(double mean, double sigma, int n, long randCount, double *a)
{
	if (n <= 0)
		return !0;
	static int phase1 = 0;
	int phase2 = 0;
	static unsigned int timetemp = 0;
	// ����ÿ�ζ���ͬ�������
	if (randCount == 0)
	{
		double min = mean - 3 * sigma;
		double max = mean + 3 * sigma;
		// ע��Ҫʹ��time(NULL),������GetTickcount
		// GetTickcount�����������شӲ���ϵͳ��������ǰ�������ĺ�������
		// ���������ж�ĳ������ִ�е�ʱ�䣬�亯��ԭ����DWORD GetTickCount(void)��
		// ����ֵ��32λ��˫������DWORD�洢����˿��Դ洢�����ֵ��2^32 msԼΪ49.71�죬
		// �����ϵͳ����ʱ�䳬��49.71��ʱ��������ͻ��0��MSDN��Ҳ��ȷ���ᵽ��:
		// "Retrieves the number of milliseconds that have elapsed since the 
		// system was started, up to 49.7 days."����ˣ�����Ǳ�д�������˳���
		// �˴�һ��Ҫ���ע�⣬�������������״����
		srand(time(NULL) + timetemp);
		// ���������
		for (int i = 0; i<n; i++)
		{
			a[i] = GaussRand(mean, sigma, phase1);
			if ((a[i]<min) || (a[i]>max))
				i--;
		}
		timetemp = (unsigned int)fabs(a[0]);
	}
	// ����ÿ�ζ���ͬ�������,�����ȶ��Ⱦ���Ҫ�õ�
	else
	{
		double min = mean - sigma;
		double max = mean + sigma;
		srand(randCount);
		// ���������
		for (int i = 0; i<n; i++)
		{
			a[i] = GaussRand(mean, sigma, phase2);
			if ((a[i]<min) || (a[i]>max))
				i--;
		}
	}
	// ����̫�پͲ�����
	if (n>3)
	{
		// ��ȡƽ��ֵ
		double meantemp = 0;
		for (int i = 0; i<n; i++)
			meantemp += a[i];
		meantemp /= n;
		// ��ƽ��ֵ��������
		double div = meantemp - mean;
		for (int i = 0; i<n; i++)
			a[i] -= div;
	}
	return 0;
}

//////////////////////////////////////////////////////////////////////////
//���ܣ�����һ����Χ�ڵ������
//���룺min����Χ���ޣ�max����Χ���ޣ�num���������
//�����randnum��������ɵ�ϵ����ֵ
//ע�⣺����ʱ��������������ͬ�����
//���ߣ�GZC
//���ڣ�2017.02.28
//////////////////////////////////////////////////////////////////////////
double BaseFunc::AverageRand(double min, double max, int num, double * randnum)
{
	srand((int)time(0));
	for (int i = 0; i < num; i++)
	{
		int minus = max - min;
		randnum[i] = random(minus);
	}
	return 0.0;
}

