#pragma once
#ifndef NULL 0
#define NULL 0
#endif // !1
#include "SateBase.h"
#include<Windows.h>
//����Ϊ����������趨
#include<stdlib.h>
#include<time.h>
typedef unsigned char byte;
#include <string>
#define random(x) (rand()%x)

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
	//�����޵ȣ������������������ڲ������ݴ����е�Ӧ��
	void GaussExt(double *ATA, double *ATL, double *x, int n);
	// ��A�������C 
	void Inv(double *A, double *C, int m);
	int invers_matrix(double *m1, int n);
	// ��ȡ������ģ
	double Norm(double *R, int num);
	// ���������й�һ��
	void NormVector(double *R, int num);
	void normalvect(double *x, double *y);
	//�����������Ĳ��
	void crossmultnorm(double *x, double *y, double *z);
	// ���������ʽ
	double Det(double *A, int m);
	// �����ת�ã��β�mΪ�У�nΪ��,Aת�ú��ΪB 
	void Transpose(double *A, double *B, int m, int n);
	//��Ԫ��ת��ת����
	void quat2matrix(double q1, double q2, double q3, double q0, double *R);
	//��Ԫ���˷���˳��Ϊ1234������4Ϊ����
	void quatMult(double *q1, double *q2, double *q3);
	void quatMult(Quat q1, Quat q2, Quat& q3);
	void quatMult2(double *q1, double *q2, double *q3);
	void quatMult3(double *q1, double *q2, double *q3);
	void Matrix2Eulor(double *R, int rotateOrder, double &eulor1, double &eulor2, double &eulor3);
	// ��ŷ���ǻ����ת����
	void Eulor2Matrix(double eulor1, double eulor2, double eulor3, int rotateOrder, double *R);
	//////////////////////////////////////////////////////////////////////////
	// �������ת
	//////////////////////////////////////////////////////////////////////////
	// ��X��ת��angle����ת����
	void RotationX(double angle, double *R);
	// ��Y��ת��angle����ת����
	void RotationY(double angle, double *R);
	// ��Z��ת��angle����ת����
	void RotationZ(double angle, double *R);
	//��ת
	void rot(double phi, double omg, double kap, double *R);
	void rot123(double omg, double phi, double kap, double* R);
	// 3*3�ĸ�˹���
	bool solve33(double *A, double *al);

	//��ת����ת��Ԫ��
	void matrix2quat(double *R, double &q1, double &q2, double &q3, double &q0);
	//��Ԫ���ڲ壬��Ԫ��˳��Ϊ0123������0Ϊ����
	void QuatInterpolation(Quat *Att, int AttNum, double *UTC, int interNum, Quat *&m_att);
	void QuatInterpolation(vector<Quat>Att, double *UTC, int interNum, Quat *&m_att);
	void QuatInterpolation(vector<Quat>Att, double UTC, Quat& m_att);
	//������������ڲ�
	void LagrangianInterpolation(Orbit_Ep *Eph, long EphNum, double UTC, Orbit_Ep &m_point, int order);
	//����STGҪ�õ�������С����
	double RevDouble(unsigned char a[]);
	float ReverseQ(long Qtemp);
	float Reverse2(long Qtemp);
	//���������(�ֱ����һ���������һ�������)
	double GaussRand(double mean, double sigma, int &phase);
	double RandomDistribution(double mean, double sigma, int n, long randCount, double *a);
	double AverageRand(double min, double max, int num, double *randnum);

	//����ʽ���
	double getValue_poly(double* px, int order, double x, double y);
	//����ָ����׺�ļ�
	void search_directory(const string& caseFileName, char *File_ext, vector<string>& ResPath);
};

