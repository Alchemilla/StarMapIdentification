#ifndef _GEOORBITEARTH
#define	_GEOORBITEARTH

#include "GeoOrbit.h"

////////////////////////////////////////////////////////
// ���������
////////////////////////////////////////////////////////
class GeoOrbitEarth : public GeoOrbit
{
public:
	GeoOrbitEarth(void);
	virtual ~GeoOrbitEarth(void);

	// ��ȡ����ļ�
	virtual void ReadEphFile(string filepath, StrOrbitParamInput input);	
	// ��ȡ���
	virtual void ReadEphNoFile(int num, StrOrbitPoint *point, StrOrbitParamInput input);
	//��ȡZY3���
	virtual void ReadZY3EphFile(vector<Orbit_Ep> allEp, StrOrbitParamInput input);
	// ����ָ��ʱ���ȡ��J2000��WGS84����ת����
	void GetJ20002WGS84Rotation(double UT, double *R);
	// ����ָ��ʱ���ȡ��WGS84��J2000����ת����
	void GetWGS842J2000Rotation(double UT, double *R);
	// ����ָ��ʱ�佫λ���ٶȴ�J2000ת����WGS84
	void State_J20002WGS84(double UT, double &X, double &Y, double &Z, double &Vx, double &Vy, double &Vz);	
	// ����ָ��ʱ�佫λ���ٶȴ�WGS84ת����J2000
	void State_WGS842J2000(double UT, double &X, double &Y, double &Z, double &Vx, double &Vy, double &Vz);	
};

#endif
