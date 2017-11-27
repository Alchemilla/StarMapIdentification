#ifndef _GEOCAMERALINE
#define	_GEOCAMERALINE

#include "GeoCamera.h"

////////////////////////////////////////////////////////
// ���������
////////////////////////////////////////////////////////
class GeoCameraLine : public GeoCamera
{
public:
	GeoCameraLine(void);
	virtual ~GeoCameraLine(void);

protected:
	double *m_X, *m_Y;		// �ڷ�λԪ��tan��ʾ
	int mark;				// ��Ǵ�����ָ����ǴӸ��������Ǵ�������
							// ����ǴӸ�������Ϊ1,����Ǵ���������Ϊ-1
	// �������
	void ClearData();

public:
	// ��ȡ�ڷ�λԪ��
	void ReadCamFile(string filepath, StrCamParamInput input);
	// ��ȡ�ڷ�λԪ��
	void ReadCamNoFile(int num, double *phi0, double *phi1, StrCamParamInput input);
	// ����ڷ�λԪ���ļ�
	void WriteCamFile(string filepath);
	// ����CCD���л�ȡ��ʼ�����ڷ�λԪ���ļ�
	void InitInnerFile(string outpath, StrCamParamInput input);
	// ��ȡ�ڷ�λԪ��(̽Ԫָ�����ʽ)
	void GetInner(double x, double y, double &phiX, double &phiY);
	// ����̽Ԫָ��ǵ�ֵȡ����������
	void GetIndexBaseInner(double phiY, double phiX, double &y, double &x);
};

#endif

