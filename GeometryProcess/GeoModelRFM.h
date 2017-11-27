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
	// ��������
	int m_nx, m_ny, m_nz;
	// ���,��С�߳�
	double m_MaxHeight, m_MinHeight, m_AveHeight;
	// ��������Сֵ
	double m_Minx;
	double m_Miny;
	// �õ�����߶�
	long ControlHeight;
	long ControlBlockHeight;
	long ControlBlockWidth;
	// ���Ƶ㼰��Ӧ�������Ϣ
	struct StrRFMGCP m_RFMgcp;
	// ����RPC��
	double m_pixelsize, m_pixelsize2;
	GeoTranslation m_trans;
	// �ָ�������ģ��
	GeoModel *rigorousmodel;

public:
	// д��RPC�ļ�
	bool WriteRFMFile(string filepath);
	// ��ȡRPC�ļ�
	bool ReadRFMFile(string filepath, bool isrpc=true);
	// ����RPCģ��
	bool GenRPCFile(GeoModel *model, double minH, double maxH, int nx, int ny, int nz, 
					string errFile = "", int order = 3);
	
public:
	// ��Ӱ�񵽵���
	bool FromXY2LatLon(double x, double y, double &H, double &lat, double &lon);
	// �ӵ��浽Ӱ��
	void FromLatLon2XY(double lat, double lon, double H, double &x, double &y);
	// ��ȡ���ű���
	double GetScale();
	//�õ�ƽ���߳�
	double getAveHeight();
	// �����������꼰�䶨λλ�����������Ӧ�����Ǹ߶ȽǺͷ�λ��
	void GetPosBaseXY(double x, double y, double lat, double lon, double H, double &SatAzimuth, double &SatZenith);

private:
	// ��ȡRPC�ļ�
	bool ReadRPCFile(string filepath);
	// ��ȡRPB�ļ�
	bool ReadRPBFile(string filepath);
	// ������������õ�����㼰����ָ��
	void GetPosBaseXY(double x, double y, double *GCP, double *direct);
	// �ͷ��ڴ�ռ�
	void Destroy();
	// ȫ�ַ���任����
    void ComputerGlobalAffine();
	// �ֲ�����任Ԥ��
	void PreciseBasedAffine(double &x,double &y,double lat,double lon,double H,double dx,double dy);
	//һ���ش�С�ڵ����ͶӰ,��һ������ϵ��:��γ��/����
	double DistanceOnePixel(double x,double y,double H,double dx,double dy);
	// ������ȱ����ļ�
	void WriteAccuracyReport(string filepath, GeoModel *model);

private:
	////////////////////////////////////////////////////////
	// ���RPC�ĸ��ַ���
	////////////////////////////////////////////////////////
	// ����Ϊ1�����
	void Cal_Order1();
	// ����Ϊ2�����
	void Cal_Order2();

	// ���RPC��ֱ����С���˷���
	void Cal_DirectLM();
	// ���RPC��ֱ������������
	void Cal_DirectLME();
	// ���RPC��ֱ������Ʒ���(L���߷�)
	void Cal_DirectLCurve();
	// ���RPC��ֱ������ƥ�䷽��
	void Cal_DirectOMP();

	// ���RPC�ļ����С���˷���
	void Cal_IndirectLM();
	// ���RPC�ļ������������
	void Cal_IndirectLME();
	// ���RPC�ļ������Ʒ���(L���߷�)
	void Cal_IndirectLCurve();
	// ���RPC�ļ������ƥ�䷽��
	void Cal_IndirectOMP();
};

#endif

