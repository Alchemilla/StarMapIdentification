#ifndef _GEOMODELLINE
#define	_GEOMODELLINE

#include "GeoModel.h"

////////////////////////////////////////////////////////
// ����ģ��������
////////////////////////////////////////////////////////
class GeoModelLine : public GeoModel
{
public:
	GeoModelLine(void);
	virtual ~GeoModelLine(void);

public:
	////////////////////////////////////////////
	// ���䷴��
	////////////////////////////////////////////
	// ����ʹ�õı���
	double m_pixelsize, m_pixelsize2;		// Ӱ��һ���ش�С�ڵ����С�Լ���ƽ����(��γ��/����)
	GeoTranslation m_trans;					// ȫ�ַ���任����
	// ��ȡȫ�ֱ任�����Լ���γ��/����
    void ComputerGlobalParam();
	// �ֲ�����任Ԥ��
	void PreciseBasedAffine(double &x,double &y,double lat,double lon,double H,double dx,double dy);
	//////////////////////////////////////////////////////////////////////////
	//ǰ���������
	//////////////////////////////////////////////////////////////////////////
	void Intersection(GeoModelLine *model, MatchPoint pts, double * XYZ);

	////////////////////////////////////////////
	// �﷽Լ������
	////////////////////////////////////////////
	// DouglasPeucker�㷨ʹ�õĽṹ��
	struct StrDP
	{
		double x;
		double y;
		bool flag;		// �Ƿ���
		long index;		// ����Ӧ����Ƭ��
	};
	//Ϊ�˱���ݹ���ö���ƵĽṹ��
	struct StrStartEnd
	{
		int Start;   //�������
		int End;     //�յ�����
	};
	// �洢�������Ϣ
	struct StrPlane
	{
		double para[3];	// �淽�̵���������
		int isPos;		// �ع�������������淽�����������Ǹ���������,�ǵĻ�Ϊ+1,���ǵĻ�Ϊ-1
	};
	StrDP *m_dppoint;				// DouglasPeucker�洢����
	StrPlane **m_Plane;				// �淽�̵ı���
	vector<StrStartEnd> m_StartEnd;	// ��ƬCCD����β������
	int m_CCDnum;					// ������ͨ��ѹ����ķ�ƬCCD����
	double m_ccdDis;				// �������м�������
	// ���DouglasPeucker���߶�
	double GetMaxHeight(StrDP *pt, int nStart, int nEnd, int &PtIndex);
	// ���ݿռ���������ȡ�ռ�ƽ�����
	void GetPlanePara(StrOrbitPoint *point, StrPlane &para);
	// ��ȡ�ռ�㵽�ռ���ľ���
	double GetPoint2PlaneDis(StrPlane para, StrOrbitPoint point);
	// ��������������ȡ�߶�
	double GetDistance(StrDP pt, StrDP pt1, StrDP pt2);
	// ����ÿ��ɨ���е�ͶӰ��(DouglasPeucker�㷨)
	void CreateProjectPlane(double tolerance = 0.0001);

protected:
	// ��ʼ������
	void Destroy();
	// �����˹����ʽģ���Լ�����λ���˹��Լ������İ�װ
	void ReCalPosAndAtt();
	// �ӵ��浽Ӱ��(���ڷ���)
	void FromLatLon2XYBaseAffine(double lat, double lon, double H, double &x, double &y);
	// �ӵ��浽Ӱ��(������Լ��)
	void FromLatLon2XYBaseImageConst(double lat, double lon, double H, double &x, double &y);
	// �ӵ��浽Ӱ��(�����﷽Լ��)
	void FromLatLon2XYBaseObjectConst(double lat, double lon, double H, double &x, double &y);

protected:
	// ����������������Ƶ�����,��ҪΪ�˱����ظ����˹����
	double *m_timeUT;						// �洢UTʱ��,����ɨ�������洢
	struct StrOrbitPoint *m_wgs84;			// GPS��λ������wgs84��λ��,����ɨ�������洢
	struct StrAttPoint *m_body2wgs84;		// ���嵽wgs84����̬,����ɨ�������洢

public:
	// ����ģ�ͳ�ʼ������
	void InitModelLine(GeoOrbit *orbit, GeoAttitude *att, GeoTime *time, GeoCamera *cam, StrModelParamInput input);
	// ��ʼ���ⷽλԪ��
	bool InitExtFile(string fpath);
	// ��Ӱ�񵽵���
	bool FromXY2LatLon(double x, double y, double &H, double &lat, double &lon);
	// �ӵ��浽Ӱ��
	void FromLatLon2XY(double lat, double lon, double H, double &x, double &y);
	// ����xy��������õ������̬���ڷ�λԪ��
	void GetOrbitAttitudeInnerBaseXY(double x, double y, double *eph, double *R, double *inner);
};

#endif

