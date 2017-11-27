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
// ZY3结构体
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


//此结构体内部使用,存储影像的区域
struct ImageRect
{
	bool isread;
	string filepath;
	double sample[4];       // 在整体影像中的sample位置
	double line[4];         // 在整体影像中的line位置
	double sampleorigin[4]; // 在整体影像中的sample位置
	double lineorigin[4];   // 在整体影像中的line位置
	double lat[4];
	double lon[4];
	double height[4];
};

//////////////////////
// 基准：建立参考椭球
//////////////////////
struct StrDATUM
{
	string DatumName;	// 参考椭球名称
	double a;			// 半长轴(单位：m)
	double f;			// 扁率的倒数
	double b;			// 短半轴
	double a2;			// 半长轴的平方
	double b2;			// 半短轴的平方
	double a2_b2;		// 长短半轴比的平方
	double e2;			// 偏心率的平方
	double m_Off[3];	// 中心偏离质心的距离(单位：m)
	// 重载构造函数,默认用WGS84
	StrDATUM()
	{
		DatumName = "";
		a = 6378137.0;	f = 298.257223563;	memset(m_Off, 0, sizeof(double)*3);
		e2 = (2*f-1)/pow(f, 2);	// e = (2*f-1)/f
		b = (f-1)/f*a;
		a2 = a*a;	b2 = b*b;
		a2_b2 = pow(a/b, 2);
	}
	// 重载操作符=
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
// 轨道部分
//////////////////////////////////////////////////////////////////////////
////////////////////////
// 轨道输入信息结构体
////////////////////////
struct StrOrbitParamInput
{
	string DatumName;		// 基准名称
	string m_EOP;			// 存储EOP文件的路径信息
	string m_JPL;			// 存储JPL文件的路径信息
	int m_InterOrder;		// 轨道内插的阶数
	int m_PolyOrder;		// 轨道拟合的阶数
	int m_adjustOrder;		// 轨道平差的阶次
	double refMJD;			// 参考累计秒起始历元的约化儒略日
	double m_Off[3];		// 轨道相位中心相对本体中心的偏心距(定轨-本体)

	// 重载构造函数
	StrOrbitParamInput()
	{
		DatumName = "WGS84";
		m_EOP = m_JPL = "";
		m_InterOrder = 7;			// 默认为7
		m_PolyOrder = 5;			// 默认为5
		m_adjustOrder = 2;			// 默认为2
		refMJD = 0;
		memset(m_Off, 0, sizeof(double)*3);
	}
	// 重载操作符=
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
// 轨道离散点信息
////////////////////////
struct StrOrbitPoint
{
	double X[6];					// 直角坐标：坐标X,Y,Z,Vx,Vy,Vz
	double UT;						// 时间系统：累计秒
	int year,month,day,hour,minute; // 时间系统：历书时
	double second;                  // 时间系统：历书时
	// 重载构造函数
	StrOrbitPoint()
	{
		memset(X, 0, sizeof(double)*6);
		UT = second = 0.0;
		year = month = day = hour = minute = 0;
	}
};

////////////////////////
// 轨道多项式模型
////////////////////////
struct StrOrbitPolyModel 
{
	// 从1到7分别为X,Y,Z,Vx,Vy,Vz,T
	double T[7][POLYORDER];
	double Toff[7], Tscale[7];
	// 改正数
	double Tdelta[7][POLYORDER];
	// 轨道拟合阶数
	int PolyOrder;	
	// 轨道平差阶次
	int m_adjustOrder;
	// 重载构造函数
	StrOrbitPolyModel()
	{
		PolyOrder = 3;
		m_adjustOrder = 2;			// 默认为2
		memset(Toff, 0, sizeof(double)*7);
		memset(Tscale, 0, sizeof(double)*7);
		for(int j=0; j<7; j++)
		{
			memset(&(T[j][0]), 0, sizeof(double)*POLYORDER);
			memset(&(Tdelta[j][0]), 0, sizeof(double)*POLYORDER);
		}
	}
	// 重载操作符=
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
// 姿态部分
//////////////////////////////////////////////////////////////////////////
////////////////////////
// 姿态输入信息结构体
////////////////////////
struct StrAttParamInput
{
	string DatumName;			// 基准名称
	int m_PolyOrder;			// 姿态拟合的阶数
	int m_adjustOrder;			// 姿态平差阶次
	double refMJD;				// 参考累计秒起始历元的约化儒略日
	double ROff[9];				// 从测姿坐标系到本体坐标系的旋转矩阵
	// 重载构造函数
	StrAttParamInput()
	{
		DatumName = "WGS84";
		m_PolyOrder = 3;		// 默认为3
		m_adjustOrder = 2;			// 默认为2
		refMJD = 0;				// 0
		memset(ROff, 0, sizeof(double)*9);
		ROff[0] = ROff[4] = ROff[8] = 1.0;
	}
	// 重载操作符=
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
// 姿态离散点信息
////////////////////////
struct StrAttPoint
{
	double R[9];                    // 旋转矩阵
	double Eulor[6];				// 三个欧拉角及其角速度
	int EulorOrder;					// 欧拉角转序
	double q[4];					// 单位四元数,q[0],q[1],q[2]为矢量，q[3]为标量
	double UT;						// 时间系统：世界时
	int year,month,day,hour,minute; // 时间系统：历书时
	double second;                  // 时间系统：历书时
	// 重载构造函数
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
// 姿态多项式模型
////////////////////////
struct StrAttPolyModel 
{
	// 从1到4分别为Eulor1,Eulor2,Eulor3,T
	double T[4][POLYORDER];
	double Toff[4], Tscale[4];
	// 改正数
	double Tdelta[4][POLYORDER];
	int PolyOrder;					// 姿态拟合阶数
	int m_adjustOrder;				// 姿态平差阶次
	int Order;						// 转序
	// 重载构造函数
	StrAttPolyModel()
	{
		PolyOrder = 3;
		m_adjustOrder = 2;			// 默认为2
		Order = 123;
		memset(Toff, 0, sizeof(double)*4);
		memset(Tscale, 0, sizeof(double)*4);
		for(int j=0; j<4; j++)
		{
			memset(&(T[j][0]), 0, sizeof(double)*POLYORDER);
			memset(&(Tdelta[j][0]), 0, sizeof(double)*POLYORDER);
		}
	}
	// 重载操作符=
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
// 相机部分
//////////////////////////////////////////////////////////////////////////
////////////////////////
// 存储内方位元素畸变信息
////////////////////////
struct StrDistortion
{
	// 主点偏移、主距变化
	double sysx0, sysy0, sysf;
	// 径向畸变
	double k1, k2;
	// 偏心畸变
	double p1, p2;
	//变形畸变
	double a1,a2,b1,b2;
};

////////////////////////
// 相机输入信息结构体
////////////////////////
struct StrCamParamInput
{
	double f;			// 焦距
	double Xsize;		// 探元沿轨向大小
	double Ysize;		// 探元垂轨向大小
	double sita;		// CCD阵列的旋转角
	long Xnum;			// 沿轨向像素个数
	long Ynum;			// 垂轨向像素个数
	double Xstart;		// 在相机坐标系中CCD阵列的起始点沿轨坐标
	double Ystart;		// 在相机坐标系中CCD阵列的起始点垂轨坐标
	double ROff[9];		// 从相机坐标系到本体坐标系的旋转矩阵
	double m_Off[3];	// 相机光轴中心相对本体中心的偏心距(相机-本体)
	// 测试用的
	StrDistortion m_Distor;	// 内方位元素畸变
	// 重载构造函数
	StrCamParamInput()
	{
		f = Xsize = Ysize = sita = Xnum = Ynum = Xstart= Ystart = 0;
		memset(m_Off, 0, sizeof(double)*3);
		memset(ROff, 0, sizeof(double)*9);
		ROff[0] = ROff[4] = ROff[8] = 1.0;
		// 测试用的
		memset(&m_Distor, 0, sizeof(StrDistortion));
	}
	// 重载操作符=
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
// 激光器输入信息结构体
////////////////////////
struct StrRangeParamInput
{
	double ROff[9];		// 从相机坐标系到本体坐标系的旋转矩阵
	double m_Off[3];	// 相机光轴中心相对本体中心的偏心距(相机-本体)
	// 重载构造函数
	StrRangeParamInput()
	{
		memset(m_Off, 0, sizeof(double)*3);
		memset(ROff, 0, sizeof(double)*9);
		ROff[0] = ROff[4] = ROff[8] = 1.0;
	}
};


//////////////////////////////////////////////////////////////////////////
// 模型部分
//////////////////////////////////////////////////////////////////////////
////////////////////////
// 模型输入信息结构体
////////////////////////
struct StrModelParamInput
{
	bool isOrbitPoly;			// 是否使用轨道多项式模型,默认为false
	bool isAttPoly;				// 是否使用姿态多项式模型,默认为false
	double timeExtend;			// 姿轨拟合外扩时间(单边，单位秒)
};


////////////////////////
// RFM模型用到的结构体
////////////////////////
struct StrRFMData		
{
	// 归一化系数
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
	// RPC参数
	double LNUM[20];
	double LDEN[20];
	double SNUM[20];
	double SDEN[20];
	// 重载构造函数
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
// 存储RPC模型生成中对应的像点和控制点信息
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
// 匹配结果
////////////////////////
struct StrCornerPoint
{
	double xl;	// 垂轨
	double yl;	// 沿轨
	double xr;
	double yr;
	double XYZ[3];
	bool operator < (const StrCornerPoint &m)const 
	{
		return yl < m.yl;
	}
};


////////////////////////
// 三视匹配结果
////////////////////////
struct Str3MatchPoint
{
	double x[3];	// 垂轨
	double y[3];	// 沿轨
	double XYZ[3];
	// 重载操作符=
	Str3MatchPoint& operator=(const Str3MatchPoint &s)
	{
		memcpy(this->x, s.x, sizeof(double)*3);
		memcpy(this->y, s.y, sizeof(double)*3);
		memcpy(this->XYZ, s.XYZ, sizeof(double)*3);
		return *this;
	}
};


////////////////////////
// 控制点结果
////////////////////////
struct StrGCP
{
	double x;	// 垂轨像素坐标(像素)
	double y;	// 沿轨像素坐标(像素)
	double lat;	// 地面纬度坐标(弧度)
	double lon;	// 地面经度坐标(弧度)
	double h;	// 地面高程值
};


////////////////////////
// 像方观测点结构体
////////////////////////
struct StrOBRPoint
{
	double ID;			// 标示其唯一值的ID
	double x;			// 垂轨向坐标
	double y;			// 沿轨向坐标
	long index_gcp;		// 对应控制点索引号
	long index_img;		// 对应影像索引号
};


////////////////////////
// 物方观测点结构体
////////////////////////
struct StrGCPPoint
{
	long type;
	double X[3];	// 纬度/经度/高程
	byte mark;		// 标志是连接点(0),控制点(1),检查点(2)
	bool isInvalid;	// 是否无效,默认为否
	double R[9];	// 存储BTBinv阵
	// 重载构造函数
	StrGCPPoint()
	{
		memset(R, 0, sizeof(double)*9);
		X[0] = X[1] = X[2] = -99999;
	}
};


////////////////////////
// 相机内外参数结构体_CE5
////////////////////////
struct StrCamPos
{
	double f;		// 焦距
	double t[3];	// 平移
	double R[9];	// 旋转
	// 重载构造函数
	StrCamPos()
	{
		memset(R, 0, sizeof(double)*9);
		R[0] = R[4] = R[8] = 1.0;
		t[0] = t[1] = t[2] = 0.0;
		f = 31250;
	}
};


////////////////////////
// CE5使用的控制点结构体_CE5
////////////////////////
struct StrGCP_CE5
{
	double GCP[3];		// 地面点三维坐标
	double meanx;		// x向RMS
	double meany;		// y向RMS
	double meanxy;		// xy向RMS
	bool isuse;			// 是否统计标记
	// 重载构造函数
	StrGCP_CE5()
	{
		memset(GCP, 0, sizeof(double)*3);
		meanx = meany = meanxy = 0.0;
		isuse = false;
	}
};


//////////////////////////////////////////////////////////////////////////
// 平差部分所用结构体
//////////////////////////////////////////////////////////////////////////
////////////////////////
// the information of the corresponding image point
////////////////////////
struct StrPoint_SBA
{
	int imgid;					// The identity of the photo
	double p;					// The weight of the image point
	double x[2];				// The coordinates of the image point,先line再sample
	// 构造函数
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
	double X[3];				// The coordinate of the object point,也可以表示纬度经度高程(先纬度再经度)
	int mark;					// The type of the point,
								// 0为连接点，1为平高控制点，2为高程控制点，3为平面控制点，4为检查点
	deque<StrPoint_SBA> link;	// The link to the coordinates of the image points
	// 构造函数
	StrGCP_SBA()
	{
		X[0] = X[1] = X[2] = 0.0;
		mark = 0;
	}
	// 析构函数
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
	// 构造函数
	StrAffine_SBA()
	{
		memset(e, 0, sizeof(double)*3);
		memset(f, 0, sizeof(double)*3);
		e[1] = f[2] = 1.0;
	}
};
//匹配点
struct MatchPoint
{
	double lx, ly, rx, ry;
};
//偏置矩阵
struct OffsetAngle
{
	//double RuStartPoint;
	double RuPhi;
	double RuOmega;
	double RuKappa;
	//double RuVPhi;
	//double RuVOmega;
	//double RuVKappa;
	// 重载操作符=
	OffsetAngle& operator=(const OffsetAngle &s)
	{
		this->RuPhi = s.RuPhi;
		this->RuOmega = s.RuOmega;
		this->RuKappa = s.RuKappa;
		return *this;
	}
	// 重载操作符+
	OffsetAngle& operator+(const OffsetAngle &s)
	{
		this->RuPhi += s.RuPhi;
		this->RuOmega += s.RuOmega;
		this->RuKappa += s.RuKappa;
		return *this;
	}
	// 重载操作符/
	OffsetAngle& operator/(const double s)
	{
		this->RuPhi /= s;
		this->RuOmega /= s;
		this->RuKappa /= s;
		return *this;
	}
};

//残差
struct strRMS
{
	double rmsx, rmsy, rmsall;
};

//存储WGS84-J2000旋转矩阵
struct structEOP
{
	double R[9];
};
//存储陀螺结构体
struct Gyro
{
	double UT;
	double wx, wy, wz;
};
//用于稀疏矩阵的控制点
struct conjugatePoints
{
	float xl;
	float yl;
	float xr;
	float yr;
	float X;
	float Y;
	float Z;
	int nIndex[2];  //record the where are the conjugate points from； 
};
//立体精度控制点结构体
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
