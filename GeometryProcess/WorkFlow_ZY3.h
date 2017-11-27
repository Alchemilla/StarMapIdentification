#ifndef _WORKFLOW_ZY3
#define	_WORKFLOW_ZY3

#include "WorkFlow.h"
#include "GeoOrbitEarth_ZY3.h"
#include "GeoAttitude_ZY3.h"
#include "AttDeter.h"
#include "GeoTime_ZY3.h"
#include "GeoCameraLine.h"		// 线阵相机类
#include "GeoCameraArray.h"		// 面阵相机类
#include "GeoModelLine.h"		// 线阵相机成像模型类
#include "GeoModelArray.h"		// 面阵相机成像类
#include "GeoModelRFM.h"		//
#include "ParseZY3Aux.h"
#include "GeoCalibration.h"
#include "GeoReadImage.h"
#include "GeoImage.h"
#include "GeoHarris.h"
#include "LSMatching.h"
#include "PhaseCorrelation.h"
#include <iomanip>

class WorkFlow_ZY3 : public WorkFlow
{
public:
	WorkFlow_ZY3(void);
	virtual ~WorkFlow_ZY3(void);
	
	void GetEOP(string eoppath);
	void GetDEM(string dempath);
	void GetPitch(double FWD,double BWD);
	// 严密模型构建
	void GenerateRigorousModel(string auxpath, string eoppath);
	//检校模型构建
	void CalibrationModel(string auxpath, string eoppath);
	void CalibrationModel2(string auxpath, string eoppath);
	//精度验证
	void AccuracyVerify(string auxpath);
	//读取控制点
	void GetGCP(string auxpath, vector<StrGCP> &ZY3_GCP);

	//////////////////////////////////////////////////////////////////////////
	//小面阵相关函数
	//////////////////////////////////////////////////////////////////////////
	//设置面阵参数
	void SetCamInput(StrCamParamInput caminput);
	//模型建立验证
	void ModelVerify();
	//线阵相机模型
	void NADCamera(string auxpath);
	//获得前后视真实模型
	GeoModelLine FwdBwdModel(string workpath,double omg, bool isReal);
	GeoModelLine FwdBwdModelVerify(string workpath, double omg, bool isReal);
	//得到推扫相机真实控制点
	void CalcFwdBwdRealMatchPoint(char* argv[]);	
	//前后视相机前方交会
	void CalcFwdBwdIntersection(char* argv[]);
	//将前后视路径下四元数改过来
	void ChangeAttPath(char* argv[]);
	//调用立体平差
	void Calc3DAccuracyByAdjustment(char* argv[]);
	//输出匹配点和控制点
	void OutputMatchAndGCP(string filePath, vector<MatchPoint>pMatch, vector<StrGCP>pGCP);
	void OutputPxyAndGCP(string filePath, vector<MatchPoint>pMatch, vector<StrGCP>pGCP);
	//生产前后视的rpc文件
	void CalcFwdBwdRPC(char* argv[]);

	//小面阵阵相机模型
	void LittleArrayCamera(string auxpath);
	//基于RPC匹配
	int Image_registration_rpc(string tifPath);
	//匹配过程
	void MatchBasedGeoModel(MatchPoint *gcp, int num, string imgL, string imgR);
	//计算真实匹配控制点
	void CalcRealMatchPoint(string workpath);
	//计算恢复的姿态（计算并比较）
	void CalcRealAttitude(string workpath);
	//计算恢复的姿态（仅计算）
	void CalcModifyAttitude(string workpath);
	//融合两次RU
	void RuFusion(vector<OffsetAngle>&RuForward, vector<OffsetAngle>RuBackward);
	//根据模型计算残差
	void CalcRMS(string out,StrGCP *pGCP,int num);
	//对比
	void CompareMeasModifyAndReal(vector<Attitude>Meas, vector<Attitude>Modify, string realAttPath);
	//对比RMS
	void OutputRMS(string outFile, vector<strRMS>accuracy1, vector<strRMS>accuracy2);
	//输出四元数结果
	void OutputQuat(string path,vector<Attitude>att);

	//根据匹配计算角速度，然后结合真实姿态做卡尔曼滤波
	void CalcOmegaKalman(string workpath);
	//根据稀疏矩阵约束姿态
	void CalcRealAttitude_sparse(string workpath);
	//稀疏矩阵求解
	bool autoGeoCalibration_sparse(GeoModelArray *pGeoModel, int num, 
		conjugatePoints* pMatch,int nMatch, int xOrder, int yOrder, double *&pRes, string sRes);
private:
	string sEOP,sDEM;
	static int outCount;
	StrCamParamInput CameraInput;
	double mFWD, mBWD;
	//GeoModelArray *pModelArray;
};

#endif


