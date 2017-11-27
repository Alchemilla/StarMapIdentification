#ifndef _GEODEFINE
#define _GEODEFINE

#include <stdio.h>
#include <vector>
#include <deque>
#include <string>
using namespace std;

#ifndef max
#define max(a,b)   (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b)   (((a) < (b)) ? (a) : (b))
#endif

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

#ifndef GM
#define GM 3.986004415E+14
#endif

#ifndef GM0
#define GM0 3.986004415e+5
#endif

#ifndef byte
typedef unsigned char byte;
#endif

#ifndef POLYORDER
#define POLYORDER 7
#endif


#ifndef min
#define min(a,b)   (((a) < (b)) ? (a) : (b))
#endif

extern "C" int _fseeki64(FILE *, __int64, int);
extern "C" __int64 _ftelli64(FILE *);

//////////////////////
// ZY3�ṹ��
//////////////////////
//the struct for orbit;
struct Orbit_Ep
{
	double X,Y,Z,Xv,Yv,Zv;
	double UTC;
};
//the struct for attitude;
struct Attitude
{
	double UTC;
	double Q1,Q2,Q3,Q0;
};
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
//struct for frame data of ZY302
struct FrameData_ZY3
{
	Orbit_Ep frame_Ep;
	Attitude frame_Att[2];
	clockInfo frame_clock;
	int frame_timeIndex[15];
	int frame_lineIndex[15];
	bool isgpsValid;
};


//�˽ṹ���ڲ�ʹ��,�洢Ӱ�������
struct ImageRect
{
	bool isread;
	string filepath;
	double sample[4];       // ������Ӱ���е�sampleλ��
	double line[4];         // ������Ӱ���е�lineλ��
	double sampleorigin[4]; // ������Ӱ���е�sampleλ��
	double lineorigin[4];   // ������Ӱ���е�lineλ��
	double lat[4];
	double lon[4];
	double height[4];
};

//////////////////////
// ��׼�������ο�����
//////////////////////
struct StrDATUM
{
	string DatumName;	// �ο���������
	double a;			// �볤��(��λ��m)
	double f;			// ���ʵĵ���
	double b;			// �̰���
	double a2;			// �볤���ƽ��
	double b2;			// ������ƽ��
	double a2_b2;		// ���̰���ȵ�ƽ��
	double e2;			// ƫ���ʵ�ƽ��
	double m_Off[3];	// ����ƫ�����ĵľ���(��λ��m)
	// ���ع��캯��,Ĭ����WGS84
	StrDATUM()
	{
		DatumName = "";
		a = 6378137.0;	f = 298.257223563;	memset(m_Off, 0, sizeof(double)*3);
		e2 = (2*f-1)/pow(f, 2);	// e = (2*f-1)/f
		b = (f-1)/f*a;
		a2 = a*a;	b2 = b*b;
		a2_b2 = pow(a/b, 2);
	}
	// ���ز�����=
	StrDATUM& operator=(const StrDATUM &s)
	{
		this->DatumName = s.DatumName;
		this->a = s.a;	this->f = s.f;
		this->b = s.b;	this->a2 = s.a2;
		this->b2 = s.b2;	this->a2_b2 = s.a2_b2;
		this->e2 = s.e2;
		memcpy(this->m_Off, s.m_Off, sizeof(double)*3);
		return *this;
	}
};


//////////////////////////////////////////////////////////////////////////
// �������
//////////////////////////////////////////////////////////////////////////
////////////////////////
// ���������Ϣ�ṹ��
////////////////////////
struct StrOrbitParamInput
{
	string DatumName;		// ��׼����
	string m_EOP;			// �洢EOP�ļ���·����Ϣ
	string m_JPL;			// �洢JPL�ļ���·����Ϣ
	int m_InterOrder;		// ����ڲ�Ľ���
	int m_PolyOrder;		// �����ϵĽ���
	int m_adjustOrder;		// ���ƽ��Ľ״�
	double refMJD;			// �ο��ۼ�����ʼ��Ԫ��Լ��������
	double m_Off[3];		// �����λ������Ա������ĵ�ƫ�ľ�(����-����)

	// ���ع��캯��
	StrOrbitParamInput()
	{
		DatumName = "WGS84";
		m_EOP = m_JPL = "";
		m_InterOrder = 7;			// Ĭ��Ϊ7
		m_PolyOrder = 5;			// Ĭ��Ϊ5
		m_adjustOrder = 2;			// Ĭ��Ϊ2
		refMJD = 0;
		memset(m_Off, 0, sizeof(double)*3);
	}
	// ���ز�����=
	StrOrbitParamInput& operator=(const StrOrbitParamInput &s)
	{
		this->DatumName = s.DatumName;
		this->m_EOP = s.m_EOP;	this->m_JPL = s.m_JPL;
		this->m_InterOrder = s.m_InterOrder;
		this->m_PolyOrder = s.m_PolyOrder;
		this->m_adjustOrder = s.m_adjustOrder;
		this->refMJD = s.refMJD;	
		memcpy(this->m_Off, s.m_Off, sizeof(double)*3);
		return *this;
	}
};

////////////////////////
// �����ɢ����Ϣ
////////////////////////
struct StrOrbitPoint
{
	double X[6];					// ֱ�����꣺����X,Y,Z,Vx,Vy,Vz
	double UT;						// ʱ��ϵͳ���ۼ���
	int year,month,day,hour,minute; // ʱ��ϵͳ������ʱ
	double second;                  // ʱ��ϵͳ������ʱ
	// ���ع��캯��
	StrOrbitPoint()
	{
		memset(X, 0, sizeof(double)*6);
		UT = second = 0.0;
		year = month = day = hour = minute = 0;
	}
};

////////////////////////
// �������ʽģ��
////////////////////////
struct StrOrbitPolyModel 
{
	// ��1��7�ֱ�ΪX,Y,Z,Vx,Vy,Vz,T
	double T[7][POLYORDER];
	double Toff[7], Tscale[7];
	// ������
	double Tdelta[7][POLYORDER];
	// �����Ͻ���
	int PolyOrder;	
	// ���ƽ��״�
	int m_adjustOrder;
	// ���ع��캯��
	StrOrbitPolyModel()
	{
		PolyOrder = 3;
		m_adjustOrder = 2;			// Ĭ��Ϊ2
		memset(Toff, 0, sizeof(double)*7);
		memset(Tscale, 0, sizeof(double)*7);
		for(int j=0; j<7; j++)
		{
			memset(&(T[j][0]), 0, sizeof(double)*POLYORDER);
			memset(&(Tdelta[j][0]), 0, sizeof(double)*POLYORDER);
		}
	}
	// ���ز�����=
	StrOrbitPolyModel& operator=(const StrOrbitPolyModel &s)
	{
		this->PolyOrder = s.PolyOrder;
		this->m_adjustOrder = s.m_adjustOrder;
		memcpy(this->Toff, s.Toff, sizeof(double)*7);
		memcpy(this->Tscale, s.Tscale, sizeof(double)*7);
		for(int j=0; j<7; j++)
		{
			memcpy(&(this->T[j][0]), &(s.T[j][0]), sizeof(double)*POLYORDER);
			memcpy(&(this->Tdelta[j][0]), &(s.Tdelta[j][0]), sizeof(double)*POLYORDER);
		}
		return *this;
	}
};


//////////////////////////////////////////////////////////////////////////
// ��̬����
//////////////////////////////////////////////////////////////////////////
////////////////////////
// ��̬������Ϣ�ṹ��
////////////////////////
struct StrAttParamInput
{
	string DatumName;			// ��׼����
	int m_PolyOrder;			// ��̬��ϵĽ���
	int m_adjustOrder;			// ��̬ƽ��״�
	double refMJD;				// �ο��ۼ�����ʼ��Ԫ��Լ��������
	double ROff[9];				// �Ӳ�������ϵ����������ϵ����ת����
	// ���ع��캯��
	StrAttParamInput()
	{
		DatumName = "WGS84";
		m_PolyOrder = 3;		// Ĭ��Ϊ3
		m_adjustOrder = 2;			// Ĭ��Ϊ2
		refMJD = 0;				// 0
		memset(ROff, 0, sizeof(double)*9);
		ROff[0] = ROff[4] = ROff[8] = 1.0;
	}
	// ���ز�����=
	StrAttParamInput& operator=(const StrAttParamInput &s)
	{
		this->DatumName = s.DatumName;
		this->m_PolyOrder = s.m_PolyOrder;
		this->m_adjustOrder = s.m_adjustOrder;
		this->refMJD = s.refMJD;
		memcpy(this->ROff, s.ROff, sizeof(double)*9);
		return *this;
	}
};


////////////////////////
// ��̬��ɢ����Ϣ
////////////////////////
struct StrAttPoint
{
	double R[9];                    // ��ת����
	double Eulor[6];				// ����ŷ���Ǽ�����ٶ�
	int EulorOrder;					// ŷ����ת��
	double q[4];					// ��λ��Ԫ��,q[0],q[1],q[2]Ϊʸ����q[3]Ϊ����
	double UT;						// ʱ��ϵͳ������ʱ
	int year,month,day,hour,minute; // ʱ��ϵͳ������ʱ
	double second;                  // ʱ��ϵͳ������ʱ
	// ���ع��캯��
	StrAttPoint()
	{
		memset(R, 0, sizeof(double)*9);
		memset(Eulor, 0, sizeof(double)*6);
		memset(q, 0, sizeof(double)*4);
		EulorOrder = 213;
		UT = second = 0.0;
		year = month = day = hour = minute = 0;
	}
};


////////////////////////
// ��̬����ʽģ��
////////////////////////
struct StrAttPolyModel 
{
	// ��1��4�ֱ�ΪEulor1,Eulor2,Eulor3,T
	double T[4][POLYORDER];
	double Toff[4], Tscale[4];
	// ������
	double Tdelta[4][POLYORDER];
	int PolyOrder;					// ��̬��Ͻ���
	int m_adjustOrder;				// ��̬ƽ��״�
	int Order;						// ת��
	// ���ع��캯��
	StrAttPolyModel()
	{
		PolyOrder = 3;
		m_adjustOrder = 2;			// Ĭ��Ϊ2
		Order = 123;
		memset(Toff, 0, sizeof(double)*4);
		memset(Tscale, 0, sizeof(double)*4);
		for(int j=0; j<4; j++)
		{
			memset(&(T[j][0]), 0, sizeof(double)*POLYORDER);
			memset(&(Tdelta[j][0]), 0, sizeof(double)*POLYORDER);
		}
	}
	// ���ز�����=
	StrAttPolyModel& operator=(const StrAttPolyModel &s)
	{
		this->PolyOrder = s.PolyOrder;
		this->m_adjustOrder = s.m_adjustOrder;
		memcpy(this->Toff, s.Toff, sizeof(double)*4);
		memcpy(this->Tscale, s.Tscale, sizeof(double)*4);
		for(int j=0; j<4; j++)
		{
			memcpy(&(this->T[j][0]), &(s.T[j][0]), sizeof(double)*POLYORDER);
			memcpy(&(this->Tdelta[j][0]), &(s.Tdelta[j][0]), sizeof(double)*POLYORDER);
		}
		return *this;
	}
};


//////////////////////////////////////////////////////////////////////////
// �������
//////////////////////////////////////////////////////////////////////////
////////////////////////
// �洢�ڷ�λԪ�ػ�����Ϣ
////////////////////////
struct StrDistortion
{
	// ����ƫ�ơ�����仯
	double sysx0, sysy0, sysf;
	// �������
	double k1, k2;
	// ƫ�Ļ���
	double p1, p2;
	//���λ���
	double a1,a2,b1,b2;
};

////////////////////////
// ���������Ϣ�ṹ��
////////////////////////
struct StrCamParamInput
{
	double f;			// ����
	double Xsize;		// ̽Ԫ�ع����С
	double Ysize;		// ̽Ԫ�������С
	double sita;		// CCD���е���ת��
	long Xnum;			// �ع������ظ���
	long Ynum;			// ���������ظ���
	double Xstart;		// ���������ϵ��CCD���е���ʼ���ع�����
	double Ystart;		// ���������ϵ��CCD���е���ʼ�㴹������
	double ROff[9];		// ���������ϵ����������ϵ����ת����
	double m_Off[3];	// �������������Ա������ĵ�ƫ�ľ�(���-����)
	// �����õ�
	StrDistortion m_Distor;	// �ڷ�λԪ�ػ���
	// ���ع��캯��
	StrCamParamInput()
	{
		f = Xsize = Ysize = sita = Xnum = Ynum = Xstart= Ystart = 0;
		memset(m_Off, 0, sizeof(double)*3);
		memset(ROff, 0, sizeof(double)*9);
		ROff[0] = ROff[4] = ROff[8] = 1.0;
		// �����õ�
		memset(&m_Distor, 0, sizeof(StrDistortion));
	}
	// ���ز�����=
	StrCamParamInput& operator=(const StrCamParamInput &s)
	{
		this->f = s.f;
		this->Xsize = s.Xsize;	this->Ysize = s.Ysize;
		this->Xnum = s.Xnum; this->Ynum = s.Ynum;
		this->Xstart = s.Xstart; this->Ystart = s.Ystart;
		return *this;
	}
};

////////////////////////
// ������������Ϣ�ṹ��
////////////////////////
struct StrRangeParamInput
{
	double ROff[9];		// ���������ϵ����������ϵ����ת����
	double m_Off[3];	// �������������Ա������ĵ�ƫ�ľ�(���-����)
	// ���ع��캯��
	StrRangeParamInput()
	{
		memset(m_Off, 0, sizeof(double)*3);
		memset(ROff, 0, sizeof(double)*9);
		ROff[0] = ROff[4] = ROff[8] = 1.0;
	}
};


//////////////////////////////////////////////////////////////////////////
// ģ�Ͳ���
//////////////////////////////////////////////////////////////////////////
////////////////////////
// ģ��������Ϣ�ṹ��
////////////////////////
struct StrModelParamInput
{
	bool isOrbitPoly;			// �Ƿ�ʹ�ù������ʽģ��,Ĭ��Ϊfalse
	bool isAttPoly;				// �Ƿ�ʹ����̬����ʽģ��,Ĭ��Ϊfalse
	double timeExtend;			// �˹��������ʱ��(���ߣ���λ��)
};


////////////////////////
// RFMģ���õ��Ľṹ��
////////////////////////
struct StrRFMData		
{
	// ��һ��ϵ��
	double LINE_OFF;
	double SAMP_OFF;
	double LAT_OFF;
	double LONG_OFF;
	double HEIGHT_OFF;
	double LINE_SCALE;
	double SAMP_SCALE;
	double LAT_SCALE;
	double LONG_SCALE;
	double HEIGHT_SCALE;
	// RPC����
	double LNUM[20];
	double LDEN[20];
	double SNUM[20];
	double SDEN[20];
	// ���ع��캯��
	StrRFMData()
	{
		LINE_OFF = SAMP_OFF = LAT_OFF = LONG_OFF = HEIGHT_OFF = 0.0;
		LINE_SCALE = SAMP_SCALE = LAT_SCALE = LONG_SCALE = HEIGHT_SCALE = 0.0;
		memset(LNUM, 0, sizeof(double)*20);
		memset(LDEN, 0, sizeof(double)*20);
		memset(SNUM, 0, sizeof(double)*20);
		memset(SDEN, 0, sizeof(double)*20);
	}
};


////////////////////////
// �洢RPCģ�������ж�Ӧ�����Ϳ��Ƶ���Ϣ
////////////////////////
struct StrRFMGCP
{
	long m_nGCPNum;
	double *m_X;
	double *m_Y;
	double *m_P;
	double *m_L;
	double *m_H;
};


////////////////////////
// ƥ����
////////////////////////
struct StrCornerPoint
{
	double xl;	// ����
	double yl;	// �ع�
	double xr;
	double yr;
	double XYZ[3];
	bool operator < (const StrCornerPoint &m)const 
	{
		return yl < m.yl;
	}
};


////////////////////////
// ����ƥ����
////////////////////////
struct Str3MatchPoint
{
	double x[3];	// ����
	double y[3];	// �ع�
	double XYZ[3];
	// ���ز�����=
	Str3MatchPoint& operator=(const Str3MatchPoint &s)
	{
		memcpy(this->x, s.x, sizeof(double)*3);
		memcpy(this->y, s.y, sizeof(double)*3);
		memcpy(this->XYZ, s.XYZ, sizeof(double)*3);
		return *this;
	}
};


////////////////////////
// ���Ƶ���
////////////////////////
struct StrGCP
{
	double x;	// ������������(����)
	double y;	// �ع���������(����)
	double lat;	// ����γ������(����)
	double lon;	// ���澭������(����)
	double h;	// ����߳�ֵ
};


////////////////////////
// �񷽹۲��ṹ��
////////////////////////
struct StrOBRPoint
{
	double ID;			// ��ʾ��Ψһֵ��ID
	double x;			// ����������
	double y;			// �ع�������
	long index_gcp;		// ��Ӧ���Ƶ�������
	long index_img;		// ��ӦӰ��������
};


////////////////////////
// �﷽�۲��ṹ��
////////////////////////
struct StrGCPPoint
{
	long type;
	double X[3];	// γ��/����/�߳�
	byte mark;		// ��־�����ӵ�(0),���Ƶ�(1),����(2)
	bool isInvalid;	// �Ƿ���Ч,Ĭ��Ϊ��
	double R[9];	// �洢BTBinv��
	// ���ع��캯��
	StrGCPPoint()
	{
		memset(R, 0, sizeof(double)*9);
		X[0] = X[1] = X[2] = -99999;
	}
};


////////////////////////
// �����������ṹ��_CE5
////////////////////////
struct StrCamPos
{
	double f;		// ����
	double t[3];	// ƽ��
	double R[9];	// ��ת
	// ���ع��캯��
	StrCamPos()
	{
		memset(R, 0, sizeof(double)*9);
		R[0] = R[4] = R[8] = 1.0;
		t[0] = t[1] = t[2] = 0.0;
		f = 31250;
	}
};


////////////////////////
// CE5ʹ�õĿ��Ƶ�ṹ��_CE5
////////////////////////
struct StrGCP_CE5
{
	double GCP[3];		// �������ά����
	double meanx;		// x��RMS
	double meany;		// y��RMS
	double meanxy;		// xy��RMS
	bool isuse;			// �Ƿ�ͳ�Ʊ��
	// ���ع��캯��
	StrGCP_CE5()
	{
		memset(GCP, 0, sizeof(double)*3);
		meanx = meany = meanxy = 0.0;
		isuse = false;
	}
};


//////////////////////////////////////////////////////////////////////////
// ƽ������ýṹ��
//////////////////////////////////////////////////////////////////////////
////////////////////////
// the information of the corresponding image point
////////////////////////
struct StrPoint_SBA
{
	int imgid;					// The identity of the photo
	double p;					// The weight of the image point
	double x[2];				// The coordinates of the image point,��line��sample
	// ���캯��
	StrPoint_SBA()
	{
		imgid = -1;
		p = 1.0;
		x[0] = x[1] = 0.0;
	}
};


////////////////////////
// the information of the object point
////////////////////////
struct StrGCP_SBA
{
	char gcpid[10];				// The identity of the object point
	double X[3];				// The coordinate of the object point,Ҳ���Ա�ʾγ�Ⱦ��ȸ߳�(��γ���پ���)
	int mark;					// The type of the point,
								// 0Ϊ���ӵ㣬1Ϊƽ�߿��Ƶ㣬2Ϊ�߳̿��Ƶ㣬3Ϊƽ����Ƶ㣬4Ϊ����
	deque<StrPoint_SBA> link;	// The link to the coordinates of the image points
	// ���캯��
	StrGCP_SBA()
	{
		X[0] = X[1] = X[2] = 0.0;
		mark = 0;
	}
	// ��������
	~StrGCP_SBA()
	{
		link.clear();
	}
};


////////////////////////
// the information of Affine
////////////////////////
struct StrAffine_SBA
{
	double e[3];
	double f[3];
	// ���캯��
	StrAffine_SBA()
	{
		memset(e, 0, sizeof(double)*3);
		memset(f, 0, sizeof(double)*3);
		e[1] = f[2] = 1.0;
	}
};
//ƥ���
struct MatchPoint
{
	double lx, ly, rx, ry;
};
//ƫ�þ���
struct OffsetAngle
{
	//double RuStartPoint;
	double RuPhi;
	double RuOmega;
	double RuKappa;
	//double RuVPhi;
	//double RuVOmega;
	//double RuVKappa;
	// ���ز�����=
	OffsetAngle& operator=(const OffsetAngle &s)
	{
		this->RuPhi = s.RuPhi;
		this->RuOmega = s.RuOmega;
		this->RuKappa = s.RuKappa;
		return *this;
	}
	// ���ز�����+
	OffsetAngle& operator+(const OffsetAngle &s)
	{
		this->RuPhi += s.RuPhi;
		this->RuOmega += s.RuOmega;
		this->RuKappa += s.RuKappa;
		return *this;
	}
	// ���ز�����/
	OffsetAngle& operator/(const double s)
	{
		this->RuPhi /= s;
		this->RuOmega /= s;
		this->RuKappa /= s;
		return *this;
	}
};

//�в�
struct strRMS
{
	double rmsx, rmsy, rmsall;
};

//�洢WGS84-J2000��ת����
struct structEOP
{
	double R[9];
};
//�洢���ݽṹ��
struct Gyro
{
	double UT;
	double wx, wy, wz;
};
//����ϡ�����Ŀ��Ƶ�
struct conjugatePoints
{
	float xl;
	float yl;
	float xr;
	float yr;
	float X;
	float Y;
	float Z;
	int nIndex[2];  //record the where are the conjugate points from�� 
};
//���徫�ȿ��Ƶ�ṹ��
struct Str3DAccuracyData
{
	double lat;
	double lon;
	double ex;
	double ey;
	double exy;
	double min;
	double max;
	double eh;
	double minh;
	double maxh;
};
#endif
