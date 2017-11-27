// GeoModelBaseArray.h : Declaration of the CGeoModelBaseArray
#pragma once
#include "GeoModel.h"

// CGeoModelBaseArray

class GeoModelArray : public GeoModel
{

	// 针对面阵相机特意设计的数据,主要为了避免重复的姿轨计算
protected:
	struct StrAttPoint m_attcam2wgs84;			// 相机到wgs84的姿态
	struct StrAttPoint m_attbody2wgs84;			//本体到84姿态
	double RCamPos[3];                          // 相机在WGS84下的位置
	Attitude qCam2wgs84;//四元数

public:
	GeoModelArray(void);
	virtual ~GeoModelArray(void);
public:
	// 面阵成像模型初始化
	void InitModelArray(GeoOrbit *orbit, GeoAttitude *att, GeoCamera *inner, StrModelParamInput param, double UT);
	// 重新计算姿态和轨道
	void ReCalPosAndAtt (double UT);

	////////////////////////////////////////////////////////
	// 成像模型正算
	////////////////////////////////////////////////////////
	// 成像模型真实值正算
	bool FromXY2LatLon(double x, double y, double &H, double &lat, double &lon);
	bool Fromxyh2XYZ(double x, double y, double H, double &X, double &Y, double &Z);
	void FromLatLon2XYBaseAffine(double lat, double lon, double H, double &x, double &y);
	void PreciseBasedAffine(double &x, double &y, double lat, double lon, double H, double dx, double dy);

	////////////////////////////////////////////
	// 仿射反算
	////////////////////////////////////////////
	// 反算使用的变量
	double m_pixelsize, m_pixelsize2;		// 影像一像素大小在地面大小以及其平方项(经纬度/像素)
	GeoTranslation m_trans;					// 全局仿射变换对象
	// 获取全局变换参数以及经纬度/像素
	void ComputerGlobalParam();
	
	////////////////////////////////////////////////////////
	// 成像模型反算
	////////////////////////////////////////////////////////
	void FromLatLon2XY(double lat, double lon, double H, double &x, double &y);
	
	////////////////////////////////////////////////////////
	// 更新Ru
	////////////////////////////////////////////////////////
	void updateOffsetmatrix(OffsetAngle &angle);

	//////////////////////////////////////////////////////////////////////////
	//从DEM得到交会出高程
	//////////////////////////////////////////////////////////////////////////
	double CalcHeihtFromDEM(double lx, double ly, double &Lat, double &Lon);

	//////////////////////////////////////////////////////////////////////////
	//前方交会过程
	//////////////////////////////////////////////////////////////////////////
	void Intersection(GeoModelArray *model, MatchPoint pts, double * XYZ);

	////////////////////////////////////////////////////////
	// 得到各种参数
	////////////////////////////////////////////////////////
	// 根据行列号获得位置、姿态、内方位元素-真实姿态内方位元素
	void GetPosAndInner(double x, double y, struct StrOrbitPoint *pos, struct StrAttPoint *att, double *innerx, double *innery);
	void GetCam2WGS84(double *Rcam2WGS84);
	void GetCamPos(double *PosCam);
	void GetInner(double x, double y, double &innerx, double &innery);
	void GetCamInstall(double *Rcam);
	void SetDEMpath(string strDEM);
	Attitude GetQuatCam2wgs84();
};

