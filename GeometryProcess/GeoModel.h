#ifndef _GEOMODEL
#define	_GEOMODEL

#include "GeoOrbit.h"
#include "GeoAttitude.h"
#include "GeoTime.h"
#include "GeoCamera.h"
#include "GeoTranslation.h"
#include "GeoReadImage.h"

////////////////////////////////////////////////////////
// 成像模型基类
////////////////////////////////////////////////////////
class GeoModel
{
public:
	GeoModel(void);
	virtual ~GeoModel(void);

public:
	struct StrRFMData m_rfm;			// RFM对象变量
	double m_affine[6];					// 仿射变换六参数
	GeoBase m_base;						// 底层操作函数
	long m_xnum, m_ynum;				// 影像大小
	StrModelParamInput m_input;			// 模型输入信息
	GeoOrbit *m_orbit;					// 指向轨道对象
	GeoAttitude *m_att;					// 指向姿态对象
	GeoTime *m_time;					// 指向时间对象
	GeoCamera *m_cam;					// 指向相机对象
	//GeoRange *m_range;					// 指向测距对象
	StrDATUM m_datum;					// 获取处理所用的基准
	string sDEM;								//全球DEM路径
	struct StrOrbitPoint m_GPSSet;		// GPS在本体坐标系下的偏心距
	struct StrOrbitPoint m_camSet;		// 相机在本体坐标系下的偏心距
	struct StrOrbitPoint m_cam2GPSSet;	// 将相机从相机系移到GPS系的偏心距(本体坐标系下)
	struct StrAttPoint m_cam2body;		// 从相机坐标系到本体坐标系的旋转矩阵
	struct StrAttPoint m_body2att;		// 从本体坐标系到测姿坐标系的旋转矩阵

public:
	// 计算姿轨多项式模型以及整数位的姿轨以及常见的安装
	virtual void ReCalPosAndAtt();
	// 获得轨道对象
	GeoOrbit* GetOrbitObject();
	// 获得姿态对象
	GeoAttitude* GetAttitudeObject();
	// 获得时间对象
	GeoTime* GetTimeObject();
	// 获得相机对象
	GeoCamera* GetCameraObject();
	//// 获得测距对象
	//GeoRange* GetRangeObject();
	// 获取轨道拟合模型
	StrOrbitPolyModel *GetOrbPolyModel();
	// 设置轨道拟合模型
	void SetOrbPolyModel(StrOrbitPolyModel orbModel);
	// 修正轨道拟合模型系数
	void ModifyOrbitPolyModelPara(double *para);			
	// 获取姿态拟合模型
	StrAttPolyModel *GetAttPolyModel();
	// 设置姿态拟合模型
	void SetAttPolyModel(StrAttPolyModel attModel);
	// 修正姿态拟合模型系数
	void ModifyAttPolyModelPara(double *para);		
	// 获取仿射模型参数
	double *GetModelAffinePara();
	// 设置仿射模型参数
	void SetModelAffinePara(double *para);

protected:
	//一像素大小在地面的投影,是一个比例系数:经纬度/像素
	double DistanceOnePixel(double x, double y, double H, double dx, double dy);

public:
	/////////////////////////////
	// 以下函数由各个子类实现
	/////////////////////////////
	// 遥感从卫星到地面
	virtual bool FromXY2LatLon(double x, double y, double &H, double &lat, double &lon);
	// 遥感从地面到卫星
	virtual void FromLatLon2XY(double lat, double lon, double H, double &x, double &y);
	// 获取缩放比例
	virtual double GetScale();
	// 获得影像的宽度和高度
	virtual long get_m_height();
	virtual long get_m_width();
	// 根据xy像素坐标得到轨道姿态和内方位元素
	virtual void GetOrbitAttitudeInnerBaseXY(double x, double y, double *eph, double *R, double *inner);
	// 根据物点和像点坐标得到对应的偏导数
	virtual void GetPartialDerivative(double lat, double lon, double H, double scale, double &x, double &y, 
				double &x_lat, double &y_lat, double &x_lon, double &y_lon, double &x_h, double &y_h);
	// 获取RFM模型的模型参数
	virtual struct StrRFMData GetRFMModel();
	// 获取全局变换参数以及经纬度/像素
    virtual void ComputerGlobalParam();

public:
	// 严密模型的前方交会
	virtual void RigorousForwardIntersection(long num, GeoModel **model, double *x, double *y, double &X, double &Y, double &Z);
	
public:
	/////////////////////////////
	// 各类检校参数设置
	/////////////////////////////
	// 地形匹配进行激光器检校
	void TopoMatchForRangeCalibration3(long num, GeoModel *model, long *index, double *X, 
						double *Y, double *Z, double &deltaT, double &deltaPhi, double &deltaOmega);
	// 测距检校参数设置
	void SetRangeCalPara(double dis, double phi, double omega);

protected:
	/////////////////////////////
	// 各类检校参数存储
	/////////////////////////////
	double Ru[9];		// 偏置修正
	double delDis;		// 距离修正
};

#endif

