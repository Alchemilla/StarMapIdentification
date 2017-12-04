#pragma once
#include "geomodelline.h"
#include "GeoModelArray.h"
#include "GeoBase.h"

class GeoCalibration :
	public GeoModelLine
{
public:
	GeoCalibration(void);
	~GeoCalibration(void);
	GeoBase m_base;
	// 外方位元素检校
	void ExtOrientCali(GeoOrbit *orbit, GeoAttitude *att, GeoTime *time, GeoCamera *cam, StrModelParamInput input,vector<StrGCP>ZY3_GCP);
	//计算前后两帧偏置补偿矩阵
	bool calcOffsetMatrix(GeoModelArray* pModel, StrGCP* pGCP, int numGCP, OffsetAngle &angle);
	//根据修正后Ru计算角速度
	void CalcRealOmega(GeoModelArray *pModel, OffsetAngle Ru, structEOP* Eop, Gyro & omega);
	//计算残差
	void calcRMS(GeoModelArray* pModel, string workpath, StrGCP *pGCP, int numGCP);
	//根据真实控制点计算残差
	void calcGCPerr(GeoModelArray* pModel, string strImg, string out, vector<strRMS>&acc, bool isPlus1);
	//输出外定标参数
	void OutputRu(double phi, double omg, double kap);
public:
	// 添加成像模型
	void AddModel (GeoModelArray *model);
	// 删除成像模型
	void DelModel();
	// 立体定位精度分析接口
	void Cal3DAccuracy(long step, long times, long ctlnum, string outpath);
	// 定位精度分析：输出分析结果
	void Write3DAccuracyResult(string outpath);
	// 对单独一个点进行法化和前方交会
	bool CalPointAccuracy(double lat, double lon, double h, int ctlnum, struct Str3DAccuracyData &data,
		vector<GeoTranslation> m_trans);

public:
	long m_step;                                // 分析步距
	long m_times;                               // 分析次数
	vector<GeoModelArray*> m_model;                 // 存储成像模型
	struct Str3DAccuracyData *pStatis;          // 存储统计信息的结构体
	double cornerlat[4], cornerlon[4];			// 存储四个角点信息
	double RMS, RMSx, RMSy, RMSh;               // 平面、X、Y、高程的RMS
};

