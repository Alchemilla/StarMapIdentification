#include "GeoTime.h"

//////////////////////////////////////////////////////////////////////////
// 构造/析构函数
//////////////////////////////////////////////////////////////////////////
GeoTime::GeoTime(void)
{
	m_Time = NULL;
	m_DelTime = NULL;
}


GeoTime::~GeoTime(void)
{
	ClearData();
}

// 清空数据
void GeoTime::ClearData()
{
	m_num  = 0;
	if(m_Time!=NULL)	{ delete []m_Time;		m_Time = NULL; }
	if(m_DelTime!=NULL) { delete []m_DelTime;	m_DelTime = NULL; }
}


// 获取行成像时间个数
long GeoTime::get_num()
{
	return m_num;
}


// 参考累计秒起始历元的约化儒略日
double GeoTime::get_refMJD()
{
	return refMJD;
}


//////////////////////////////////////
// 功能：根据索引号获得对应的行成像时间
// 输入:
//		double line:	行成像积分时间索引号
// 返回值：
//		double：		返回行成像积分时间
//////////////////////////////////////
double GeoTime::get_time(double line)
{
	if(m_num>1)
	{
		long left, right;
		if(line<=0||line!=line)	// line!=line的意思是line值为无效值
		{
			left = 0;
			right = 1;
		}
		else if(line>=m_num-1)
		{
			left = (long)(m_num-2);
			right = (long)(m_num-1);
		}
		else
		{
			left = (long)line;
			right = (long)(line+1);
		}
		return (right-line)*m_Time[left] + (line-left)*m_Time[right];
	}
	else
		return 0;
}


// 以下函数由各个子类实现
void GeoTime::ReadTimeFile(string filepath){}
//读取ZY3行时文件
void GeoTime::ReadZY3TimeFile(vector<LineScanTime> allTime){}
