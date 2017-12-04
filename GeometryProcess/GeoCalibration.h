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
	// �ⷽλԪ�ؼ�У
	void ExtOrientCali(GeoOrbit *orbit, GeoAttitude *att, GeoTime *time, GeoCamera *cam, StrModelParamInput input,vector<StrGCP>ZY3_GCP);
	//����ǰ����֡ƫ�ò�������
	bool calcOffsetMatrix(GeoModelArray* pModel, StrGCP* pGCP, int numGCP, OffsetAngle &angle);
	//����������Ru������ٶ�
	void CalcRealOmega(GeoModelArray *pModel, OffsetAngle Ru, structEOP* Eop, Gyro & omega);
	//����в�
	void calcRMS(GeoModelArray* pModel, string workpath, StrGCP *pGCP, int numGCP);
	//������ʵ���Ƶ����в�
	void calcGCPerr(GeoModelArray* pModel, string strImg, string out, vector<strRMS>&acc, bool isPlus1);
	//����ⶨ�����
	void OutputRu(double phi, double omg, double kap);
public:
	// ��ӳ���ģ��
	void AddModel (GeoModelArray *model);
	// ɾ������ģ��
	void DelModel();
	// ���嶨λ���ȷ����ӿ�
	void Cal3DAccuracy(long step, long times, long ctlnum, string outpath);
	// ��λ���ȷ���������������
	void Write3DAccuracyResult(string outpath);
	// �Ե���һ������з�����ǰ������
	bool CalPointAccuracy(double lat, double lon, double h, int ctlnum, struct Str3DAccuracyData &data,
		vector<GeoTranslation> m_trans);

public:
	long m_step;                                // ��������
	long m_times;                               // ��������
	vector<GeoModelArray*> m_model;                 // �洢����ģ��
	struct Str3DAccuracyData *pStatis;          // �洢ͳ����Ϣ�Ľṹ��
	double cornerlat[4], cornerlon[4];			// �洢�ĸ��ǵ���Ϣ
	double RMS, RMSx, RMSy, RMSh;               // ƽ�桢X��Y���̵߳�RMS
};

