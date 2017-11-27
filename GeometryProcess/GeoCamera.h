#ifndef _GEOCAMERA
#define	_GEOCAMERA

#include "GeoDefine.h"
#include "GeoBase.h"

////////////////////////////////////////////////////////
// �������
////////////////////////////////////////////////////////
class GeoCamera
{
public:
	GeoCamera(void);
	virtual ~GeoCamera(void);

public:
	StrCamParamInput m_Input;	// �����������ṹ��,������ܱ����ⲿ�ı�

public:
	long get_xnum();			// ��ȡ�ع������ظ���
	long get_ynum();			// ��ȡ���������ظ���
	double get_OffX();			// ��������Ա���ƫ�ľ�X
	double get_OffY();			// ��������Ա���ƫ�ľ�Y
	double get_OffZ();			// ��������Ա���ƫ�ľ�Z
	void get_ROff(double *R);	// ��ȡ���������ϵ����������ϵ����ת����
	void set_ROff(double *R);	// ��ȡ���������ϵ����������ϵ����ת����

	/////////////////////////////////////////////////////
	// ���¼���������Ҫ�ֱ�ʵ��,����ע�ⲻҪ�ô��麯��,�����޷�ʵ����
	/////////////////////////////////////////////////////
	// ��ȡ�ڷ�λԪ��
	virtual void ReadCamFile(string filepath, StrCamParamInput input);
	// ����ڷ�λԪ���ļ�
	virtual void WriteCamFile(string filepath);
	// ����CCD���л�ȡ��ʼ�����ڷ�λԪ���ļ�
	virtual void InitInnerFile(string outpath, StrCamParamInput input);
	// ��ȡ�ڷ�λԪ��(̽Ԫָ�����ʽ)
	virtual void GetInner(double x, double y, double &phiX, double &phiY);
	// ����̽Ԫָ��ǵ�ֵȡ����������
	virtual void GetIndexBaseInner(double phiY, double phiX, double &y, double &x);
};

#endif

