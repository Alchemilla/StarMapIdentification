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

//王新洲等，谱修正迭代法及其在测量数据处理中的应用
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

// 求取向量的模
double BaseFunc::Norm(double *R, int num)
{
	double retVal = 0.0;
	for (int i = 0; i<num; i++)
		retVal += pow(R[i], 2);
	return sqrt(retVal);
}

// 对向量进行归一化
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
//对两个向量求叉积
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

void BaseFunc::quatMult(double *q1, double *q2, double *q3)//四元数顺序为1234，其中4为标量，首先旋转q1
{
	q3[0] = q1[3] * q2[0] - q1[2] * q2[1] + q1[1] * q2[2] + q1[0] * q2[3];
	q3[1] = q1[2] * q2[0] + q1[3] * q2[1] - q1[0] * q2[2] + q1[1] * q2[3];
	q3[2] = -q1[1] * q2[0] + q1[0] * q2[1] + q1[3] * q2[2] + q1[2] * q2[3];
	q3[3] = -q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] + q1[3] * q2[3];
}

void BaseFunc::quatMult3(double *q1, double *q2, double *q3)//四元数顺序为1234，其中4为标量，首先旋转q2
{
	q3[0] = q1[3] * q2[0] + q1[2] * q2[1] - q1[1] * q2[2] + q1[0] * q2[3];
	q3[1] = -q1[2] * q2[0] + q1[3] * q2[1] + q1[0] * q2[2] + q1[1] * q2[3];
	q3[2] = q1[1] * q2[0] - q1[0] * q2[1] + q1[3] * q2[2] + q1[2] * q2[3];
	q3[3] = -q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] + q1[3] * q2[3];
}

void BaseFunc::quatMult2(double *q1, double *q2, double *q3)//四元数顺序为0123，其中0为标量
{
	q3[1] = q1[0] * q2[1] - q1[3] * q2[2] + q1[2] * q2[3] + q1[1] * q2[0];
	q3[2] = q1[3] * q2[1] + q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0];
	q3[3] = -q1[2] * q2[1] + q1[1] * q2[2] + q1[0] * q2[3] + q1[3] * q2[0];
	q3[0] = -q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3] + q1[0] * q2[0];
}
////////////////////////////////////////////////////////
// 旋转矩阵与欧拉角的相互变换
////////////////////////////////////////////////////////
//////////////////////////////////////
// 功能：从旋转矩阵获得欧拉角
// 输入:
//		double *R：		旋转矩阵3*3，行优先	
//		int ratateOrder:欧拉角转序
// 输出：
//		double &eulor1:	欧拉角1
//		double &eulor2:	欧拉角2
//		double &eulor3:	欧拉角3
// 返回值：
//		void
///////////////////////////////////////
void BaseFunc::Matrix2Eulor(double *R, int rotateOrder, double &eulor1, double &eulor2, double &eulor3)
{
	switch (rotateOrder)
	{
		// 第一类:第一次和第三次转动是绕同类坐标轴进行的,第二次转动是绕另两轴中的一类进行的
	case 121:  // 1
	{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
		eulor2 = acos(R[0]);					double temp = sin(eulor2);
		eulor1 = atan2(R[1] * temp, -R[2] * temp);	eulor3 = atan2(R[3] * temp, R[6] * temp);	break;
	}
	case 131:	// 2
	{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
		eulor2 = acos(R[0]);					double temp = sin(eulor2);
		eulor1 = atan2(R[2] * temp, R[1] * temp);	eulor3 = atan2(R[6] * temp, -R[3] * temp);	break;
	}
	case 212:	// 3
	{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
		eulor2 = acos(R[4]);					double temp = sin(eulor2);
		eulor1 = atan2(R[3] * temp, R[5] * temp);	eulor3 = atan2(R[1] * temp, -R[7] * temp);	break;
	}
	case 232:	// 4
	{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
		eulor2 = acos(R[4]);					double temp = sin(eulor2);
		eulor1 = atan2(R[5] * temp, -R[3] * temp);	eulor3 = atan2(R[7] * temp, R[1] * temp);	break;
	}
	case 313:	// 5
	{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
		eulor2 = acos(R[8]);					double temp = sin(eulor2);
		eulor1 = atan2(R[6] * temp, -R[7] * temp);	eulor3 = atan2(R[2] * temp, R[5] * temp);	break;
	}
	case 323:	// 6
	{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
		eulor2 = acos(R[8]);					double temp = sin(eulor2);
		eulor1 = atan2(R[7] * temp, R[6] * temp);	eulor3 = atan2(R[5] * temp, -R[2] * temp);	break;
	}
	// 第二类:每次转动是绕不同类别的坐标轴进行的
	case 123:	// 7
	{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
		eulor2 = asin(R[6]);					double temp = cos(eulor2);
		eulor1 = atan2(-R[7] * temp, R[8] * temp);	eulor3 = atan2(-R[3] * temp, R[0] * temp);	break;
	}
	case 132:	// 8
	{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
		eulor2 = asin(-R[3]);					double temp = cos(eulor2);
		eulor1 = atan2(R[5] * temp, R[4] * temp);	eulor3 = atan2(R[6] * temp, R[0] * temp);	break;
	}
	case 213:	// 9
	{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
		eulor2 = asin(-R[7]);					double temp = cos(eulor2);
		eulor1 = atan2(R[6] * temp, R[8] * temp);	eulor3 = atan2(R[1] * temp, R[4] * temp);	break;
	}
	case 231:	// 10
	{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
		eulor2 = asin(R[1]);					double temp = cos(eulor2);
		eulor1 = atan2(-R[2] * temp, R[0] * temp);	eulor3 = atan2(-R[7] * temp, R[4] * temp);	break;
	}
	case 312:	// 11
	{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
		eulor2 = asin(R[5]);					double temp = cos(eulor2);
		eulor1 = atan2(-R[3] * temp, R[4] * temp);	eulor3 = atan2(-R[2] * temp, R[8] * temp);	break;
	}
	case 321:	// 12
	{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
		eulor2 = asin(-R[2]);					double temp = cos(eulor2);
		eulor1 = atan2(R[1] * temp, R[0] * temp);	eulor3 = atan2(R[5] * temp, R[8] * temp);	break;
	}
	}
}
//////////////////////////////////////
// 功能：从欧拉角获得旋转矩阵
// 输入:
//		double &eulor1:	欧拉角1
//		double &eulor2:	欧拉角2
//		double &eulor3:	欧拉角3
// 输出：
//		double *R：		旋转矩阵3*3，行优先	
//		int ratateOrder:欧拉角转序	
// 返回值：
//		void
///////////////////////////////////////
void BaseFunc::Eulor2Matrix(double eulor1, double eulor2, double eulor3, int rotateOrder, double *R)
{
	double R1[9], R2[9], R3[9], Rtemp[9];
	switch (rotateOrder)
	{
		// 第一类:第一次和第三次转动是绕同类坐标轴进行的,第二次转动是绕另两轴中的一类进行的
	case 121:	// 1
	{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
		RotationX(eulor1, R1);	RotationY(eulor2, R2);	RotationX(eulor3, R3);
		Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
	}
	case 131:	// 2
	{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
		RotationX(eulor1, R1);	RotationZ(eulor2, R2);	RotationX(eulor3, R3);
		Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
	}
	case 212:	// 3
	{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
		RotationY(eulor1, R1);	RotationX(eulor2, R2);	RotationY(eulor3, R3);
		Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
	}
	case 232:	// 4
	{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
		RotationY(eulor1, R1);	RotationZ(eulor2, R2);	RotationY(eulor3, R3);
		Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
	}
	case 313:	// 5
	{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
		RotationZ(eulor1, R1);	RotationX(eulor2, R2);	RotationZ(eulor3, R3);
		Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
	}
	case 323:	// 6
	{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
		RotationZ(eulor1, R1);	RotationY(eulor2, R2);	RotationZ(eulor3, R3);
		Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
	}
	// 第二类:每次转动是绕不同类别的坐标轴进行的
	case 123:	// 7
	{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
		RotationX(eulor1, R1);	RotationY(eulor2, R2);	RotationZ(eulor3, R3);
		Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
	}
	case 132:	// 8
	{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
		RotationX(eulor1, R1);	RotationZ(eulor2, R2);	RotationY(eulor3, R3);
		Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
	}
	case 213:	// 9
	{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
		RotationY(eulor1, R1);	RotationX(eulor2, R2);	RotationZ(eulor3, R3);
		Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
	}
	case 231:	// 10
	{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
		RotationY(eulor1, R1);	RotationZ(eulor2, R2);	RotationX(eulor3, R3);
		Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
	}
	case 312:	// 11
	{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
		RotationZ(eulor1, R1);	RotationX(eulor2, R2);	RotationY(eulor3, R3);
		Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
	}
	case 321:	// 12
	{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
		RotationZ(eulor1, R1);	RotationY(eulor2, R2);	RotationX(eulor3, R3);
		Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
	}
	default:
	{
		printf("Eulor2Matrix Error!\n");	break;	// 没有此种转序存在!
	}
	}
}
//////////////////////////////////////////////////////////////////////////
// 绕轴的旋转
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
// 功能：绕X轴转角angle的旋转矩阵
// [ 1          0         0     ]
// [ 0    cos(angle)  sin(angle)]
// [ 0   -sin(angle)  cos(angle)]
// 输入:
//		double angle:	转过的角(弧度)
// 输出：
//		double *R：		旋转矩阵3*3，行优先	
// 返回值：
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
// 功能：绕Y轴转角angle的旋转矩阵
// [cos(angle)  0    -sin(angle)]
// [     0      1         0     ]
// [sin(angle)  0     cos(angle)]
// 输入:
//		double angle:	转过的角(弧度)
// 输出：
//		double *R：		旋转矩阵3*3，行优先	
// 返回值：
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
// 功能：绕Z轴转角angle的旋转矩阵
// [ cos(angle)  sin(angle)  0]
// [-sin(angle)  cos(angle)  0]
// [     0            0      1]
// 输入:
//		double angle:	转过的角(弧度)
// 输出：
//		double *R：		旋转矩阵3*3，行优先	
// 返回值：
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
// 3*3的高斯求解
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
	// 寻找最大值
	j = 0;
	for (i = 1; i < 4; i++) j = (tq[i] > tq[j]) ? i : j;
	// 检查对角线
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

//////////////////////////////////////////////////////////////////////////
//功能：四元数内插
//输入：Att：原始四元数；UTC：内插时间点；interNum：时间个数
//输出：m_att：内插后四元数
//注意：四元数时间范围要大于陀螺时间UTC范围
//作者：GZC
//日期：2017.03.23
//////////////////////////////////////////////////////////////////////////
void BaseFunc::QuatInterpolation(vector<Quat>Att, double *UTC, int interNum, Quat *&m_att)
{
	int AttNum = Att.size();
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

//////////////////////////////////////////////////////////////////////////
//功能：产生一个符合正态分布的随机数
//输入：mean:均值，sigma:方差
//输出：函数分布图
//注意：
//日期：2016.10.12 by HWC
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
	return (x*sigma + mean);    // 注意要加括号,否则返回值就是sigma了
}

//////////////////////////////////////////////////////////////////////////
//功能：产生一组符合正态分布的随机数
//输入：mean:均值，sigma:方差，n:随机数量，randCount:输入0每组随机数不一样
//输出：a:生成n组随机数组a
//注意：randCount一般为0
//日期：2016.10.12 by HWC
//////////////////////////////////////////////////////////////////////////
double BaseFunc::RandomDistribution(double mean, double sigma, int n, long randCount, double *a)
{
	if (n <= 0)
		return !0;
	static int phase1 = 0;
	int phase2 = 0;
	static unsigned int timetemp = 0;
	// 产生每次都不同的随机数
	if (randCount == 0)
	{
		double min = mean - 3 * sigma;
		double max = mean + 3 * sigma;
		// 注意要使用time(NULL),而不是GetTickcount
		// GetTickcount函数：它返回从操作系统启动到当前所经过的毫秒数，
		// 常常用来判断某个方法执行的时间，其函数原型是DWORD GetTickCount(void)，
		// 返回值以32位的双字类型DWORD存储，因此可以存储的最大值是2^32 ms约为49.71天，
		// 因此若系统运行时间超过49.71天时，这个数就会归0，MSDN中也明确的提到了:
		// "Retrieves the number of milliseconds that have elapsed since the 
		// system was started, up to 49.7 days."。因此，如果是编写服务器端程序，
		// 此处一定要万分注意，避免引起意外的状况。
		srand(time(NULL) + timetemp);
		// 产生随机数
		for (int i = 0; i<n; i++)
		{
			a[i] = GaussRand(mean, sigma, phase1);
			if ((a[i]<min) || (a[i]>max))
				i--;
		}
		timetemp = (unsigned int)fabs(a[0]);
	}
	// 产生每次都相同的随机数,比如稳定度就需要用到
	else
	{
		double min = mean - sigma;
		double max = mean + sigma;
		srand(randCount);
		// 产生随机数
		for (int i = 0; i<n; i++)
		{
			a[i] = GaussRand(mean, sigma, phase2);
			if ((a[i]<min) || (a[i]>max))
				i--;
		}
	}
	// 个数太少就不进行
	if (n>3)
	{
		// 求取平均值
		double meantemp = 0;
		for (int i = 0; i<n; i++)
			meantemp += a[i];
		meantemp /= n;
		// 对平均值进行修正
		double div = meantemp - mean;
		for (int i = 0; i<n; i++)
			a[i] -= div;
	}
	return 0;
}

//////////////////////////////////////////////////////////////////////////
//功能：产生一定范围内的随机数
//输入：min，范围下限，max，范围上限，num，随机数量
//输出：randnum，随机生成的系列数值
//注意：根据时间种子来产生不同随机数
//作者：GZC
//日期：2017.02.28
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

