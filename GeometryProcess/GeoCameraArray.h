#ifndef _GEOCAMERA_ARRAY
#define	_GEOCAMERA_ARRAY
#include "GeoCamera.h"

class GeoCameraArray :	public GeoCamera
{
public:
	GeoCameraArray(void);
	virtual ~GeoCameraArray(void);

	// ����CCD���л�ȡ��ʼ�����ڷ�λԪ���ļ�
	void InitInnerFile(string outpath, StrCamParamInput input);
	// ��ȡ�ڷ�λԪ��(̽Ԫָ�����ʽ)
	void GetInner(double x, double y, double &phiX, double &phiY);
	// ����̽Ԫָ��ǵ�ֵȡ����������
	void GetIndexBaseInner(double phiY, double phiX, double &y, double &x);
};

#endif
