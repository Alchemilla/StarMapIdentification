#ifndef _GEOMODEL
#define	_GEOMODEL

#include "GeoOrbit.h"
#include "GeoAttitude.h"
#include "GeoTime.h"
#include "GeoCamera.h"
#include "GeoTranslation.h"
#include "GeoReadImage.h"

////////////////////////////////////////////////////////
// ����ģ�ͻ���
////////////////////////////////////////////////////////
class GeoModel
{
public:
	GeoModel(void);
	virtual ~GeoModel(void);

public:
	struct StrRFMData m_rfm;			// RFM�������
	double m_affine[6];					// ����任������
	GeoBase m_base;						// �ײ��������
	long m_xnum, m_ynum;				// Ӱ���С
	StrModelParamInput m_input;			// ģ��������Ϣ
	GeoOrbit *m_orbit;					// ָ��������
	GeoAttitude *m_att;					// ָ����̬����
	GeoTime *m_time;					// ָ��ʱ�����
	GeoCamera *m_cam;					// ָ���������
	//GeoRange *m_range;					// ָ�������
	StrDATUM m_datum;					// ��ȡ�������õĻ�׼
	string sDEM;								//ȫ��DEM·��
	struct StrOrbitPoint m_GPSSet;		// GPS�ڱ�������ϵ�µ�ƫ�ľ�
	struct StrOrbitPoint m_camSet;		// ����ڱ�������ϵ�µ�ƫ�ľ�
	struct StrOrbitPoint m_cam2GPSSet;	// ����������ϵ�Ƶ�GPSϵ��ƫ�ľ�(��������ϵ��)
	struct StrAttPoint m_cam2body;		// ���������ϵ����������ϵ����ת����
	struct StrAttPoint m_body2att;		// �ӱ�������ϵ����������ϵ����ת����

public:
	// �����˹����ʽģ���Լ�����λ���˹��Լ������İ�װ
	virtual void ReCalPosAndAtt();
	// ��ù������
	GeoOrbit* GetOrbitObject();
	// �����̬����
	GeoAttitude* GetAttitudeObject();
	// ���ʱ�����
	GeoTime* GetTimeObject();
	// ����������
	GeoCamera* GetCameraObject();
	//// ��ò�����
	//GeoRange* GetRangeObject();
	// ��ȡ������ģ��
	StrOrbitPolyModel *GetOrbPolyModel();
	// ���ù�����ģ��
	void SetOrbPolyModel(StrOrbitPolyModel orbModel);
	// ����������ģ��ϵ��
	void ModifyOrbitPolyModelPara(double *para);			
	// ��ȡ��̬���ģ��
	StrAttPolyModel *GetAttPolyModel();
	// ������̬���ģ��
	void SetAttPolyModel(StrAttPolyModel attModel);
	// ������̬���ģ��ϵ��
	void ModifyAttPolyModelPara(double *para);		
	// ��ȡ����ģ�Ͳ���
	double *GetModelAffinePara();
	// ���÷���ģ�Ͳ���
	void SetModelAffinePara(double *para);

protected:
	//һ���ش�С�ڵ����ͶӰ,��һ������ϵ��:��γ��/����
	double DistanceOnePixel(double x, double y, double H, double dx, double dy);

public:
	/////////////////////////////
	// ���º����ɸ�������ʵ��
	/////////////////////////////
	// ң�д����ǵ�����
	virtual bool FromXY2LatLon(double x, double y, double &H, double &lat, double &lon);
	// ң�дӵ��浽����
	virtual void FromLatLon2XY(double lat, double lon, double H, double &x, double &y);
	// ��ȡ���ű���
	virtual double GetScale();
	// ���Ӱ��Ŀ�Ⱥ͸߶�
	virtual long get_m_height();
	virtual long get_m_width();
	// ����xy��������õ������̬���ڷ�λԪ��
	virtual void GetOrbitAttitudeInnerBaseXY(double x, double y, double *eph, double *R, double *inner);
	// ���������������õ���Ӧ��ƫ����
	virtual void GetPartialDerivative(double lat, double lon, double H, double scale, double &x, double &y, 
				double &x_lat, double &y_lat, double &x_lon, double &y_lon, double &x_h, double &y_h);
	// ��ȡRFMģ�͵�ģ�Ͳ���
	virtual struct StrRFMData GetRFMModel();
	// ��ȡȫ�ֱ任�����Լ���γ��/����
    virtual void ComputerGlobalParam();

public:
	// ����ģ�͵�ǰ������
	virtual void RigorousForwardIntersection(long num, GeoModel **model, double *x, double *y, double &X, double &Y, double &Z);
	
public:
	/////////////////////////////
	// �����У��������
	/////////////////////////////
	// ����ƥ����м�������У
	void TopoMatchForRangeCalibration3(long num, GeoModel *model, long *index, double *X, 
						double *Y, double *Z, double &deltaT, double &deltaPhi, double &deltaOmega);
	// ����У��������
	void SetRangeCalPara(double dis, double phi, double omega);

protected:
	/////////////////////////////
	// �����У�����洢
	/////////////////////////////
	double Ru[9];		// ƫ������
	double delDis;		// ��������
};

#endif

