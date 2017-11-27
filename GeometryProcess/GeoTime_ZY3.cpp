#include "GeoTime_ZY3.h"


GeoTime_ZY3::GeoTime_ZY3(void)
{
}


GeoTime_ZY3::~GeoTime_ZY3(void)
{
}

//////////////////////////////////////
// 功能：读取行时文件
// 输入:
//		string filepath:	行成像时间文件路径
// 返回值：
//		void
//////////////////////////////////////
void GeoTime_ZY3::ReadZY3TimeFile(vector<LineScanTime> allTime)
{
	
	// 开始读取
	try
	{
		// 清空数据
		ClearData();
		double jd0, refMJD;
		long startLine, endLine;
		long timeSize = allTime.size();

		startLine = allTime[0].lineNumber;
		endLine = allTime[timeSize-1].lineNumber;
		m_num = endLine - startLine;
		m_Time = new double[timeSize];
		m_DelTime = new double[timeSize];
		//ZY301行时计算公式
		for (long i = 0;i < timeSize; i++)
		{
			m_Time[i] = allTime.at(i).lineTimeUT - 28800;
			//m_Time[i] = allTime.at(i).lineTimeUT;
		}
		//ZY302行时计算公式
		//LineScanTime preUTC;
		//preUTC.lineTimeUT = allTime[0].lineTimeUT - 28800;
		//preUTC.lineNumber = allTime[0].lineNumber - 1;	
		//for (long i = 0;i < timeSize; i++)
		//{
		//	m_Time[i] = allTime.at(i).lineTimeUT - 28800;
		//	//m_DelTime[i] = (allTime[i].lineTimeUT - preUTC.lineTimeUT)/(allTime[i].lineNumber - preUTC.lineNumber);
		//	preUTC = allTime[i];
		//}
	}
	catch(...)
	{
		printf("GeoTime_XX11HSI::ReadTimeFile Error1!\n");	
		return;
	}
}

