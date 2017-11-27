#ifndef _GEOCAMERA_ARRAY
#define	_GEOCAMERA_ARRAY
#include "GeoCamera.h"

class GeoCameraArray :	public GeoCamera
{
public:
	GeoCameraArray(void);
	virtual ~GeoCameraArray(void);

	// 根据CCD排列获取初始化的内方位元素文件
	void InitInnerFile(string outpath, StrCamParamInput input);
	// 获取内方位元素(探元指向角形式)
	void GetInner(double x, double y, double &phiX, double &phiY);
	// 根据探元指向角的值取反求索引号
	void GetIndexBaseInner(double phiY, double phiX, double &y, double &x);
};

#endif
