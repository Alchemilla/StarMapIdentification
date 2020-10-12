#ifndef SATEBASE_H
#define SATEBASE_H
#define PI 3.1415926535897932384626433832795
#include <vector>
#include <Eigen/Dense>
#include <iostream>
using namespace Eigen;
using namespace std;
typedef Matrix<double, Dynamic, Dynamic, RowMajor>rMatrixXd;
//the struct for orbit;
struct Orbit_Ep
{
	double X;
	double Y;
	double Z;
	double Xv;
	double Yv;
	double Zv;
	double UTC;
} ;

//the struct for attitude;
struct Attitude
{
	double UTC;
	double Yaw,Pitch,Roll,vYaw,vPitch,vRoll;
	double Q1,Q2,Q3,Q0;
	bool isEuler;
} ;

//the struct for quaternion
struct Quat
{
	double Q1,Q2,Q3,Q0;
	double UTC;
};

//the struct for Euler
struct SateEuler
{
	double R,P,Y;
	double UTC;
	SateEuler()
	{
		R = P = Y = UTC = 0;
	}
};

//struct for scan time;
struct LineScanTime
{
	int lineNumber;	
	double lineTimeUT;
	double integralTime;
};

struct clockInfo
{
	int timeIndex;
	int intUTC;
	int clockFreq;
};

struct YMD
{
	int year;
	int mon;
	int day;
	int hour;
	int min;
	double sec;
};
//struct for frame data of ZY302
struct FrameData_ZY302
{
	//vector<Orbit_Ep> frame_Ep;
	//vector<Attitude> frame_Att;
	//clockInfo frame_clock;
	//vector<int> frame_timeIndex;
	//vector<int> frame_lineIndex;
	Orbit_Ep frame_Ep;
	Attitude frame_Att[2];
	clockInfo frame_clock;
	int frame_timeIndex[15];
	int frame_lineIndex[15];
	bool isgpsValid;

};

//����һ��֡��ˮ�ź�ʱ��
struct img
{
	int id;
	double time;
};

//struct for STG data of ZY302
struct STGData
{
	Quat StarA,StarB,StarC;
	double utgyro,g1,g2,g3,g4,g5,g6,g7,g8,g9,g10,g11,g12;
	double bx,by,bz;
};

//struct for Gyro data
struct Gyro
{
	double UTC;
	double x,y,z;
};

//�����Ǳ�ṹ��
struct Star
{
	int ID, DN;//���Ǳ�ź����Ȼ���DNֵ
	double phiX, phiY, mag;//�ྭ��γ������
	double V[3];//J2000ϵ�µ�XYZ����
};

//�洢�ǵ�Ľṹ��
//��xy����ʱ��ʾ��������
//��xy������ʱ��ʾ�ྭ��γ
struct StarPoint
{
	double x, y;
	double Mag;
};

//�ǵ���Ƶ�
struct StarGCP
{
	double UTC;
	double x, y;
	double V[3];
};

//���������������
struct StarCaliParam
{
	double x0, y0;
	double f, k1,k2;
	StarCaliParam()
	{
		x0 = y0 = f = k1 = k2 = 0;
	}
};

//�ǵ���Ƶ�ת��Ϊ ����ʸ���ͱ���ʸ��
struct BmImStar
{
	double UT;
	double Im[3], Bm[3];

};

struct FrameDistortion
{
	double px[80]; //(n+1)*(n+2)/2; n less than 10
	double py[80];
	int xOrder;
	int yOrder;

	FrameDistortion()
	{
		xOrder = yOrder = 0;
		memset(px, 0, sizeof(int) * 80);
		memset(py, 0, sizeof(int) * 80);
	};

};
#endif