#ifndef _GEOMODELRFM
#define	_GEOMODELRFM

#include "GeoModel.h"
#include "GeoTranslation.h"

class GeoModelRFM : public GeoModel
{
public:
	GeoModelRFM(void);
	virtual ~GeoModelRFM(void);

private:
	// 格网划分
	int m_nx, m_ny, m_nz;
	// 最大,最小高程
	double m_MaxHeight, m_MinHeight, m_AveHeight;
	// 行列向最小值
	double m_Minx;
	double m_Miny;
	// 得到间隔高度
	long ControlHeight;
	long ControlBlockHeight;
	long ControlBlockWidth;
	// 控制点及对应的像点信息
	struct StrRFMGCP m_RFMgcp;
	// 反算RPC用
	double m_pixelsize, m_pixelsize2;
	GeoTranslation m_trans;
	// 恢复的严密模型
	GeoModel *rigorousmodel;

public:
	// 写出RPC文件
	bool WriteRFMFile(string filepath);
	// 读取RPC文件
	bool ReadRFMFile(string filepath, bool isrpc=true);
	// 生成RPC模型
	bool GenRPCFile(GeoModel *model, double minH, double maxH, int nx, int ny, int nz, 
					string errFile = "", int order = 3);
	
public:
	// 从影像到地面
	bool FromXY2LatLon(double x, double y, double &H, double &lat, double &lon);
	// 从地面到影像
	void FromLatLon2XY(double lat, double lon, double H, double &x, double &y);
	// 获取缩放比例
	double GetScale();
	//得到平均高程
	double getAveHeight();
	// 根据像素坐标及其定位位置求出其所对应的卫星高度角和方位角
	void GetPosBaseXY(double x, double y, double lat, double lon, double H, double &SatAzimuth, double &SatZenith);

private:
	// 读取RPC文件
	bool ReadRPCFile(string filepath);
	// 读取RPB文件
	bool ReadRPBFile(string filepath);
	// 根据像素坐标得到地面点及光线指向
	void GetPosBaseXY(double x, double y, double *GCP, double *direct);
	// 释放内存空间
	void Destroy();
	// 全局仿射变换参数
    void ComputerGlobalAffine();
	// 局部仿射变换预测
	void PreciseBasedAffine(double &x,double &y,double lat,double lon,double H,double dx,double dy);
	//一像素大小在地面的投影,是一个比例系数:经纬度/像素
	double DistanceOnePixel(double x,double y,double H,double dx,double dy);
	// 输出精度报告文件
	void WriteAccuracyReport(string filepath, GeoModel *model);

private:
	////////////////////////////////////////////////////////
	// 求解RPC的各种方法
	////////////////////////////////////////////////////////
	// 阶数为1的求解
	void Cal_Order1();
	// 阶数为2的求解
	void Cal_Order2();

	// 求解RPC的直接最小二乘方法
	void Cal_DirectLM();
	// 求解RPC的直接谱修正方法
	void Cal_DirectLME();
	// 求解RPC的直接岭估计方法(L曲线法)
	void Cal_DirectLCurve();
	// 求解RPC的直接正交匹配方法
	void Cal_DirectOMP();

	// 求解RPC的间接最小二乘方法
	void Cal_IndirectLM();
	// 求解RPC的间接谱修正方法
	void Cal_IndirectLME();
	// 求解RPC的间接岭估计方法(L曲线法)
	void Cal_IndirectLCurve();
	// 求解RPC的间接正交匹配方法
	void Cal_IndirectOMP();
};

#endif

