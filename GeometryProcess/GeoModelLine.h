#ifndef _GEOMODELLINE
#define	_GEOMODELLINE

#include "GeoModel.h"

////////////////////////////////////////////////////////
// 成像模型线阵类
////////////////////////////////////////////////////////
class GeoModelLine : public GeoModel
{
public:
	GeoModelLine(void);
	virtual ~GeoModelLine(void);

public:
	////////////////////////////////////////////
	// 仿射反算
	////////////////////////////////////////////
	// 反算使用的变量
	double m_pixelsize, m_pixelsize2;		// 影像一像素大小在地面大小以及其平方项(经纬度/像素)
	GeoTranslation m_trans;					// 全局仿射变换对象
	// 获取全局变换参数以及经纬度/像素
    void ComputerGlobalParam();
	// 局部仿射变换预测
	void PreciseBasedAffine(double &x,double &y,double lat,double lon,double H,double dx,double dy);
	//////////////////////////////////////////////////////////////////////////
	//前方交会过程
	//////////////////////////////////////////////////////////////////////////
	void Intersection(GeoModelLine *model, MatchPoint pts, double * XYZ);

	////////////////////////////////////////////
	// 物方约束反算
	////////////////////////////////////////////
	// DouglasPeucker算法使用的结构体
	struct StrDP
	{
		double x;
		double y;
		bool flag;		// 是否保留
		long index;		// 所对应的面片号
	};
	//为了避免递归调用而设计的结构体
	struct StrStartEnd
	{
		int Start;   //起点索引
		int End;     //终点索引
	};
	// 存储面参数信息
	struct StrPlane
	{
		double para[3];	// 面方程的三个参数
		int isPos;		// 沿轨向上正面代入面方程是正数还是负数的修正,是的话为+1,不是的话为-1
	};
	StrDP *m_dppoint;				// DouglasPeucker存储变量
	StrPlane **m_Plane;				// 面方程的变量
	vector<StrStartEnd> m_StartEnd;	// 分片CCD的首尾索引号
	int m_CCDnum;					// 垂轨向通过压缩后的分片CCD个数
	double m_ccdDis;				// 相邻两行间的面距离
	// 获得DouglasPeucker最大高度
	double GetMaxHeight(StrDP *pt, int nStart, int nEnd, int &PtIndex);
	// 根据空间三个点求取空间平面参数
	void GetPlanePara(StrOrbitPoint *point, StrPlane &para);
	// 求取空间点到空间面的距离
	double GetPoint2PlaneDis(StrPlane para, StrOrbitPoint point);
	// 根据三点坐标求取高度
	double GetDistance(StrDP pt, StrDP pt1, StrDP pt2);
	// 创建每个扫描行的投影面(DouglasPeucker算法)
	void CreateProjectPlane(double tolerance = 0.0001);

protected:
	// 初始化参数
	void Destroy();
	// 计算姿轨多项式模型以及整数位的姿轨以及常见的安装
	void ReCalPosAndAtt();
	// 从地面到影像(基于仿射)
	void FromLatLon2XYBaseAffine(double lat, double lon, double H, double &x, double &y);
	// 从地面到影像(基于像方约束)
	void FromLatLon2XYBaseImageConst(double lat, double lon, double H, double &x, double &y);
	// 从地面到影像(基于物方约束)
	void FromLatLon2XYBaseObjectConst(double lat, double lon, double H, double &x, double &y);

protected:
	// 针对线阵相机特意设计的数据,主要为了避免重复的姿轨计算
	double *m_timeUT;						// 存储UT时间,按照扫描行来存储
	struct StrOrbitPoint *m_wgs84;			// GPS相位中心在wgs84下位置,按照扫描行来存储
	struct StrAttPoint *m_body2wgs84;		// 本体到wgs84的姿态,按照扫描行来存储

public:
	// 线阵模型初始化函数
	void InitModelLine(GeoOrbit *orbit, GeoAttitude *att, GeoTime *time, GeoCamera *cam, StrModelParamInput input);
	// 初始化外方位元素
	bool InitExtFile(string fpath);
	// 从影像到地面
	bool FromXY2LatLon(double x, double y, double &H, double &lat, double &lon);
	// 从地面到影像
	void FromLatLon2XY(double lat, double lon, double H, double &x, double &y);
	// 根据xy像素坐标得到轨道姿态和内方位元素
	void GetOrbitAttitudeInnerBaseXY(double x, double y, double *eph, double *R, double *inner);
};

#endif

