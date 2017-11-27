// GeoModelBaseArray.h : Declaration of the CGeoModelBaseArray
#pragma once
#include "GeoModel.h"

// CGeoModelBaseArray

class GeoModelArray : public GeoModel
{

	// ����������������Ƶ�����,��ҪΪ�˱����ظ����˹����
protected:
	struct StrAttPoint m_attcam2wgs84;			// �����wgs84����̬
	struct StrAttPoint m_attbody2wgs84;			//���嵽84��̬
	double RCamPos[3];                          // �����WGS84�µ�λ��
	Attitude qCam2wgs84;//��Ԫ��

public:
	GeoModelArray(void);
	virtual ~GeoModelArray(void);
public:
	// �������ģ�ͳ�ʼ��
	void InitModelArray(GeoOrbit *orbit, GeoAttitude *att, GeoCamera *inner, StrModelParamInput param, double UT);
	// ���¼�����̬�͹��
	void ReCalPosAndAtt (double UT);

	////////////////////////////////////////////////////////
	// ����ģ������
	////////////////////////////////////////////////////////
	// ����ģ����ʵֵ����
	bool FromXY2LatLon(double x, double y, double &H, double &lat, double &lon);
	bool Fromxyh2XYZ(double x, double y, double H, double &X, double &Y, double &Z);
	void FromLatLon2XYBaseAffine(double lat, double lon, double H, double &x, double &y);
	void PreciseBasedAffine(double &x, double &y, double lat, double lon, double H, double dx, double dy);

	////////////////////////////////////////////
	// ���䷴��
	////////////////////////////////////////////
	// ����ʹ�õı���
	double m_pixelsize, m_pixelsize2;		// Ӱ��һ���ش�С�ڵ����С�Լ���ƽ����(��γ��/����)
	GeoTranslation m_trans;					// ȫ�ַ���任����
	// ��ȡȫ�ֱ任�����Լ���γ��/����
	void ComputerGlobalParam();
	
	////////////////////////////////////////////////////////
	// ����ģ�ͷ���
	////////////////////////////////////////////////////////
	void FromLatLon2XY(double lat, double lon, double H, double &x, double &y);
	
	////////////////////////////////////////////////////////
	// ����Ru
	////////////////////////////////////////////////////////
	void updateOffsetmatrix(OffsetAngle &angle);

	//////////////////////////////////////////////////////////////////////////
	//��DEM�õ�������߳�
	//////////////////////////////////////////////////////////////////////////
	double CalcHeihtFromDEM(double lx, double ly, double &Lat, double &Lon);

	//////////////////////////////////////////////////////////////////////////
	//ǰ���������
	//////////////////////////////////////////////////////////////////////////
	void Intersection(GeoModelArray *model, MatchPoint pts, double * XYZ);

	////////////////////////////////////////////////////////
	// �õ����ֲ���
	////////////////////////////////////////////////////////
	// �������кŻ��λ�á���̬���ڷ�λԪ��-��ʵ��̬�ڷ�λԪ��
	void GetPosAndInner(double x, double y, struct StrOrbitPoint *pos, struct StrAttPoint *att, double *innerx, double *innery);
	void GetCam2WGS84(double *Rcam2WGS84);
	void GetCamPos(double *PosCam);
	void GetInner(double x, double y, double &innerx, double &innery);
	void GetCamInstall(double *Rcam);
	void SetDEMpath(string strDEM);
	Attitude GetQuatCam2wgs84();
};

