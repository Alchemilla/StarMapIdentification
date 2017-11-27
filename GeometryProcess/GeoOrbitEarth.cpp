#include "GeoOrbitEarth.h"

//////////////////////////////////////////////////////////////////////////
// 构造/析构函数
//////////////////////////////////////////////////////////////////////////
GeoOrbitEarth::GeoOrbitEarth(void){}
GeoOrbitEarth::~GeoOrbitEarth(void){}

// 读取轨道文件
void GeoOrbitEarth::ReadEphFile(string filepath, StrOrbitParamInput input){}
// 读取轨道
void GeoOrbitEarth::ReadEphNoFile(int num, StrOrbitPoint *point, StrOrbitParamInput input){}

void GeoOrbitEarth::ReadZY3EphFile(vector<Orbit_Ep> allEp, StrOrbitParamInput input){}


//////////////////////////////////////
// 功能：根据指定时间获取从J2000到WGS84的旋转矩阵
// 输入:
//		double UT：		从起始历元开始的累计秒
// 输出：
//		double *R：		旋转矩阵3*3，行优先	
// 返回值：
//		StrOrbitPoint：	返回的轨道点
////////////////////////////////////// 
void GeoOrbitEarth::GetJ20002WGS84Rotation(double UT, double *R)
{
	int year,month,day,hour,minute;
	double second;
	m_Base.FromSecondtoYMD(m_Input.refMJD, UT, year, month, day, hour, minute, second);
	IAU2000ABaseCIOCelToTer(year, month, day, hour, minute, second, (char*)m_Input.m_EOP.c_str(), 2, R);
}


//////////////////////////////////////
// 功能：根据指定时间获取从WGS84到J2000的旋转矩阵
// 输入:
//		double UT：		从起始历元开始的累计秒
// 输出：
//		double *R：		旋转矩阵3*3，行优先	
// 返回值：
//		StrOrbitPoint：	返回的轨道点
////////////////////////////////////// 
void GeoOrbitEarth::GetWGS842J2000Rotation(double UT, double *R)
{
	int year,month,day,hour,minute;
	double second;
	m_Base.FromSecondtoYMD(m_Input.refMJD, UT, year, month, day, hour, minute, second);
	IAU2000ABaseCIOTerToCel(year, month, day, hour, minute, second, (char*)m_Input.m_EOP.c_str(), 2, R);
}


//////////////////////////////////////
// 功能：根据指定时间将位置速度从J2000转化到WGS84
// 输入:
//		double UT：		从起始历元开始的累计秒
// 输出：
//		double &XYZ:	位置X、Y、Z
//		double &VxVyVz：速度Vx、Vy、Vz
// 返回值：
//		void
////////////////////////////////////// 
void GeoOrbitEarth::State_J20002WGS84(double UT, double &X, double &Y, double &Z, double &Vx, double &Vy, double &Vz)
{
	int year,month,day,hour,minute;
	double second,R[9];
	double Pos[3],Vel[3];
	Pos[0] = X;    Pos[1] = Y;    Pos[2] = Z;
	Vel[0] = Vx;   Vel[1] = Vy;   Vel[2] = Vz;
	m_Base.FromSecondtoYMD(m_Input.refMJD, UT, year, month, day, hour, minute, second);
	IAU2000ABaseCIOCelToTer(year,month,day,hour,minute,second,(char*)m_Input.m_EOP.c_str(),2,R,Pos,Vel);
	X = Pos[0];    Y = Pos[1];    Z = Pos[2];
	Vx= Vel[0];    Vy= Vel[1];    Vz= Vel[2];
}


//////////////////////////////////////
// 功能：根据指定时间将位置速度从WGS84转化到J2000
// 输入:
//		double UT：		从起始历元开始的累计秒
// 输出：
//		double &XYZ:	位置X、Y、Z
//		double &VxVyVz：速度Vx、Vy、Vz
// 返回值：
//		void
////////////////////////////////////// 
void GeoOrbitEarth::State_WGS842J2000(double UT, double &X, double &Y, double &Z, double &Vx, double &Vy, double &Vz)
{
	int year,month,day,hour,minute;
	double second,R[9];
	double Pos[3],Vel[3];
	Pos[0] = X;    Pos[1] = Y;    Pos[2] = Z;
	Vel[0] = Vx;   Vel[1] = Vy;   Vel[2] = Vz;
	m_Base.FromSecondtoYMD(m_Input.refMJD, UT, year, month, day, hour, minute, second);
	IAU2000ABaseCIOTerToCel(year, month, day, hour, minute, second, (char*)m_Input.m_EOP.c_str(), 2, R, Pos, Vel);
	X = Pos[0];    Y = Pos[1];    Z = Pos[2];
	Vx= Vel[0];    Vy= Vel[1];    Vz= Vel[2];
}


