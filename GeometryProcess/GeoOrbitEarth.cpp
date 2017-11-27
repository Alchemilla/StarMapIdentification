#include "GeoOrbitEarth.h"

//////////////////////////////////////////////////////////////////////////
// ����/��������
//////////////////////////////////////////////////////////////////////////
GeoOrbitEarth::GeoOrbitEarth(void){}
GeoOrbitEarth::~GeoOrbitEarth(void){}

// ��ȡ����ļ�
void GeoOrbitEarth::ReadEphFile(string filepath, StrOrbitParamInput input){}
// ��ȡ���
void GeoOrbitEarth::ReadEphNoFile(int num, StrOrbitPoint *point, StrOrbitParamInput input){}

void GeoOrbitEarth::ReadZY3EphFile(vector<Orbit_Ep> allEp, StrOrbitParamInput input){}


//////////////////////////////////////
// ���ܣ�����ָ��ʱ���ȡ��J2000��WGS84����ת����
// ����:
//		double UT��		����ʼ��Ԫ��ʼ���ۼ���
// �����
//		double *R��		��ת����3*3��������	
// ����ֵ��
//		StrOrbitPoint��	���صĹ����
////////////////////////////////////// 
void GeoOrbitEarth::GetJ20002WGS84Rotation(double UT, double *R)
{
	int year,month,day,hour,minute;
	double second;
	m_Base.FromSecondtoYMD(m_Input.refMJD, UT, year, month, day, hour, minute, second);
	IAU2000ABaseCIOCelToTer(year, month, day, hour, minute, second, (char*)m_Input.m_EOP.c_str(), 2, R);
}


//////////////////////////////////////
// ���ܣ�����ָ��ʱ���ȡ��WGS84��J2000����ת����
// ����:
//		double UT��		����ʼ��Ԫ��ʼ���ۼ���
// �����
//		double *R��		��ת����3*3��������	
// ����ֵ��
//		StrOrbitPoint��	���صĹ����
////////////////////////////////////// 
void GeoOrbitEarth::GetWGS842J2000Rotation(double UT, double *R)
{
	int year,month,day,hour,minute;
	double second;
	m_Base.FromSecondtoYMD(m_Input.refMJD, UT, year, month, day, hour, minute, second);
	IAU2000ABaseCIOTerToCel(year, month, day, hour, minute, second, (char*)m_Input.m_EOP.c_str(), 2, R);
}


//////////////////////////////////////
// ���ܣ�����ָ��ʱ�佫λ���ٶȴ�J2000ת����WGS84
// ����:
//		double UT��		����ʼ��Ԫ��ʼ���ۼ���
// �����
//		double &XYZ:	λ��X��Y��Z
//		double &VxVyVz���ٶ�Vx��Vy��Vz
// ����ֵ��
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
// ���ܣ�����ָ��ʱ�佫λ���ٶȴ�WGS84ת����J2000
// ����:
//		double UT��		����ʼ��Ԫ��ʼ���ۼ���
// �����
//		double &XYZ:	λ��X��Y��Z
//		double &VxVyVz���ٶ�Vx��Vy��Vz
// ����ֵ��
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


