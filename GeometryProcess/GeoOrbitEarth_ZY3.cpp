#include "GeoOrbitEarth_ZY3.h"


GeoOrbitEarth_ZY3::GeoOrbitEarth_ZY3(void)
{
}


GeoOrbitEarth_ZY3::~GeoOrbitEarth_ZY3(void)
{
}

void GeoOrbitEarth_ZY3::ReadZY3EphFile(vector<Orbit_Ep> allEp, StrOrbitParamInput input)
{
	StrDATUM datum;
	m_Base.GetRefEllipsoid(input.DatumName, datum);
	// 清空数据
	ClearData();
	// 获得参数
	m_Input = input;
	// 累计秒从2008年1月1日0时开始计数
	double jd0;
	Cal2JD(2009, 1, 1, 0, &jd0, &m_Input.refMJD);
	// 读取轨道文件
	int m = allEp.size();
	int i;
	vector<StrOrbitPoint> m_EpWGS84tmp;
	vector<StrOrbitPoint> m_EpJ2000tmp;	
	for(i = 0;i < m; i++)
	{		
		// 开始读取
		StrOrbitPoint tmp;
		double R[9];
		tmp.UT = allEp.at(i).UTC - 28800;//从北京时算到格林尼治时间
		//tmp.UT = allEp.at(i).UTC;//仿真所用的时间
		tmp.X[0] = allEp.at(i).X, tmp.X[1] = allEp.at(i).Y, tmp.X[2] = allEp.at(i).Z;
		tmp.X[3] = allEp.at(i).Xv, tmp.X[4] = allEp.at(i).Yv, tmp.X[5] = allEp.at(i).Zv;

		m_Base.FromSecondtoYMD(m_Input.refMJD, tmp.UT, tmp.year, tmp.month, tmp.day, tmp.hour, tmp.minute, tmp.second);
		m_EpWGS84tmp.push_back(tmp);
			// 转到J2000系下
		IAU2000ABaseCIOTerToCel(tmp.year, tmp.month, tmp.day, tmp.hour, tmp.minute, tmp.second, (char*)m_Input.m_EOP.c_str(), 2, R, &(tmp.X[0]), &(tmp.X[3]));
		m_EpJ2000tmp.push_back(tmp);
			
	}
		m_num = m_EpWGS84tmp.size();
		// 存储
		m_EpWGS84 = new StrOrbitPoint[m_num];
		m_EpJ2000 = new StrOrbitPoint[m_num];
		for(int i=0; i<m_num; i++)
		{
			m_EpWGS84[i] = m_EpWGS84tmp[i];
			m_EpJ2000[i] = m_EpJ2000tmp[i];
		}
		m_EpWGS84tmp.clear();
		m_EpJ2000tmp.clear();
		// 排序
		OrderOrbit(m_EpWGS84, m_num);
		OrderOrbit(m_EpJ2000, m_num);
	
}