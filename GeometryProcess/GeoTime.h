#ifndef _GEOTIME
#define	_GEOTIME

#include "GeoDefine.h"
#include "GeoBase.h"

////////////////////////////////////////////////////////
// 时间基类
////////////////////////////////////////////////////////
class GeoTime
{
public:
	GeoTime(void);
	virtual ~GeoTime(void);

protected:
	long m_num;						// 行成像时间个数
	double *m_Time;					// 存储每条扫描行对应的实际成像时刻
	double *m_DelTime;				// 存储每行的积分时间
	GeoBase m_Base;					// 底层通用算法类
	double refMJD;					// 参考累计秒起始历元的约化儒略日
	// 清空数据
	void ClearData();

public:
	long get_num();					// 获取行成像时间个数
	double get_time(double line);	// 根据索引号获得对应的行成像时间
	double get_refMJD();			// 参考累计秒起始历元的约化儒略日
	// 以下函数由各个子类实现
	virtual void ReadTimeFile(string filepath);	// 读取行时文件
	virtual void ReadZY3TimeFile(vector<LineScanTime> allTime);//读取ZY3行时文件
};

#endif

