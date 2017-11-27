#ifndef _GEOORBITEARTH
#define	_GEOORBITEARTH

#include "GeoOrbit.h"

////////////////////////////////////////////////////////
// 轨道地球类
////////////////////////////////////////////////////////
class GeoOrbitEarth : public GeoOrbit
{
public:
	GeoOrbitEarth(void);
	virtual ~GeoOrbitEarth(void);

	// 读取轨道文件
	virtual void ReadEphFile(string filepath, StrOrbitParamInput input);	
	// 读取轨道
	virtual void ReadEphNoFile(int num, StrOrbitPoint *point, StrOrbitParamInput input);
	//读取ZY3轨道
	virtual void ReadZY3EphFile(vector<Orbit_Ep> allEp, StrOrbitParamInput input);
	// 根据指定时间获取从J2000到WGS84的旋转矩阵
	void GetJ20002WGS84Rotation(double UT, double *R);
	// 根据指定时间获取从WGS84到J2000的旋转矩阵
	void GetWGS842J2000Rotation(double UT, double *R);
	// 根据指定时间将位置速度从J2000转化到WGS84
	void State_J20002WGS84(double UT, double &X, double &Y, double &Z, double &Vx, double &Vy, double &Vz);	
	// 根据指定时间将位置速度从WGS84转化到J2000
	void State_WGS842J2000(double UT, double &X, double &Y, double &Z, double &Vx, double &Vy, double &Vz);	
};

#endif
