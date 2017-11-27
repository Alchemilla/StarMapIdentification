#ifndef _GEOCAMERALINE
#define	_GEOCAMERALINE

#include "GeoCamera.h"

////////////////////////////////////////////////////////
// 线阵相机类
////////////////////////////////////////////////////////
class GeoCameraLine : public GeoCamera
{
public:
	GeoCameraLine(void);
	virtual ~GeoCameraLine(void);

protected:
	double *m_X, *m_Y;		// 内方位元素tan表示
	int mark;				// 标记垂轨向指向角是从负到正还是从正到负
							// 如果是从负到正则为1,如果是从正到负则为-1
	// 清空数据
	void ClearData();

public:
	// 读取内方位元素
	void ReadCamFile(string filepath, StrCamParamInput input);
	// 读取内方位元素
	void ReadCamNoFile(int num, double *phi0, double *phi1, StrCamParamInput input);
	// 输出内方位元素文件
	void WriteCamFile(string filepath);
	// 根据CCD排列获取初始化的内方位元素文件
	void InitInnerFile(string outpath, StrCamParamInput input);
	// 获取内方位元素(探元指向角形式)
	void GetInner(double x, double y, double &phiX, double &phiY);
	// 根据探元指向角的值取反求索引号
	void GetIndexBaseInner(double phiY, double phiX, double &y, double &x);
};

#endif

