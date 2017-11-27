#ifndef _GEOORBIT
#define	_GEOORBIT

#include "GeoDefine.h"
#include "GeoBase.h"

////////////////////////////////////////////////////////
// 轨道基类
////////////////////////////////////////////////////////
class GeoOrbit
{
public:
	GeoOrbit(void);
	virtual ~GeoOrbit(void);

protected:
	StrOrbitParamInput m_Input;			// 轨道参数输入结构体,这个不能被类外部改变,通过get_OffX、get_OffY、get_OffZ获取
	long m_num;							// 轨道点个数
	StrOrbitPoint *m_EpWGS84;			// 存储WGS84系下轨道点信息(注意在外行星这个指的是月固坐标系)
	StrOrbitPoint *m_EpJ2000;			// 存储J2000系下轨道点信息(注意在外行星这个指的是月心天球坐标系)
	StrOrbitPolyModel m_modelWGS84;		// 存储WGS84系下轨道拟合模型
	StrOrbitPolyModel m_modelJ2000;		// 存储J2000系下轨道拟合模型
	GeoBase m_Base;						// 底层通用算法类
	// 清空数据
	void ClearData();

public:
	StrDATUM get_datum();					// 获取所用的参考椭球参数
	long get_num();							// 获取轨道点个数
	double get_OffX();						// 轨道相位中心相对本体中心的偏心距X(定轨-本体)
	double get_OffY();						// 轨道相位中心相对本体中心的偏心距Y(定轨-本体)
	double get_OffZ();						// 轨道相位中心相对本体中心的偏心距Z(定轨-本体)
	double get_RefUT();						// 获得参考历元
	StrOrbitPolyModel *GetPolyModel();				// 获取轨道拟合模型
	void SetPolyModel(StrOrbitPolyModel orbModel);	// 设置轨道拟合模型
	void ModifyPolyModelPara(double *para);			// 修正轨道拟合模型系数

	// 内插轨道
	StrOrbitPoint GetEpWGS84(double UT);	// 获取指定时间的WGS84轨道点信息
	StrOrbitPoint GetEpJ2000(double UT);	// 获取指定时间的J2000轨道点信息
	// 轨道坐标系的变换
	void GetJ20002OrbitRotation(double UT, double *R, bool isJ2000=true);		// 获取从J2000到轨道坐标系的旋转矩阵(轨道坐标系定义在J2000系下)
	void GetOrbit2J2000Rotation(double UT, double *R, bool isJ2000=true);		// 获取从轨道坐标系到J2000的旋转矩阵(轨道坐标系定义在J2000系下)
	// 返回索引处WGS84系下轨道点信息
	struct StrOrbitPoint get_m_EpWGS84(long index);
	// 返回索引处J2000系下轨道点信息
	struct StrOrbitPoint get_m_EpJ2000(long index);
	// 获得轨道的仿真的起始时间
	double get_m_startTime();
	// 获得轨道的仿真的结束时间
	double get_m_endTime();
	// 轨道多项式拟合
	void GenPolyModel(double startUT, double endUT, bool isJ2000=false);		// 根据时间段获得对应轨道的多项式模型
	StrOrbitPoint PolyValue(double UT, bool isJ2000 = false);					// 提供时间根据多项式模型获得对应的轨道点信息
	// 轨道根据时间排序
	void OrderOrbit(StrOrbitPoint *m_point, int num);

	// 以下几个函数跟星球有关系,地球和外行星是不同的,需要分别实现,但是注意不要用纯虚函数,否则无法实例化
	virtual void ReadEphFile(string filepath, StrOrbitParamInput input);					// 读取轨道文件
	virtual void ReadEphNoFile(int num, StrOrbitPoint *point, StrOrbitParamInput input);	// 读取轨道
	virtual void ReadZY3EphFile(vector<Orbit_Ep> allEp, StrOrbitParamInput input);			// 读取ZY3轨道
	virtual void WriteEphFile(string filepath, bool iserror=true);							// 写出轨道文件
	virtual void GetJ20002WGS84Rotation(double UT, double *R);					// 获取从J2000到WGS84的旋转矩阵
	virtual void GetWGS842J2000Rotation(double UT, double *R);					// 获取从WGS84到J2000的旋转矩阵
	virtual void State_J20002WGS84(double UT, double &X, double &Y, double &Z, double &Vx, double &Vy, double &Vz);	// 根据指定时间将位置速度从J2000转化到WGS84
	virtual void State_WGS842J2000(double UT, double &X, double &Y, double &Z, double &Vx, double &Vy, double &Vz);	// 根据指定时间将位置速度从WGS84转化到J2000
};

#endif

