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
	// �������
	ClearData();
	// ��ò���
	m_Input = input;
	// �ۼ����2008��1��1��0ʱ��ʼ����
	double jd0;
	Cal2JD(2009, 1, 1, 0, &jd0, &m_Input.refMJD);
	// ��ȡ����ļ�
	int m = allEp.size();
	int i;
	vector<StrOrbitPoint> m_EpWGS84tmp;
	vector<StrOrbitPoint> m_EpJ2000tmp;	
	for(i = 0;i < m; i++)
	{		
		// ��ʼ��ȡ
		StrOrbitPoint tmp;
		double R[9];
		tmp.UT = allEp.at(i).UTC - 28800;//�ӱ���ʱ�㵽��������ʱ��
		//tmp.UT = allEp.at(i).UTC;//�������õ�ʱ��
		tmp.X[0] = allEp.at(i).X, tmp.X[1] = allEp.at(i).Y, tmp.X[2] = allEp.at(i).Z;
		tmp.X[3] = allEp.at(i).Xv, tmp.X[4] = allEp.at(i).Yv, tmp.X[5] = allEp.at(i).Zv;

		m_Base.FromSecondtoYMD(m_Input.refMJD, tmp.UT, tmp.year, tmp.month, tmp.day, tmp.hour, tmp.minute, tmp.second);
		m_EpWGS84tmp.push_back(tmp);
			// ת��J2000ϵ��
		IAU2000ABaseCIOTerToCel(tmp.year, tmp.month, tmp.day, tmp.hour, tmp.minute, tmp.second, (char*)m_Input.m_EOP.c_str(), 2, R, &(tmp.X[0]), &(tmp.X[3]));
		m_EpJ2000tmp.push_back(tmp);
			
	}
		m_num = m_EpWGS84tmp.size();
		// �洢
		m_EpWGS84 = new StrOrbitPoint[m_num];
		m_EpJ2000 = new StrOrbitPoint[m_num];
		for(int i=0; i<m_num; i++)
		{
			m_EpWGS84[i] = m_EpWGS84tmp[i];
			m_EpJ2000[i] = m_EpJ2000tmp[i];
		}
		m_EpWGS84tmp.clear();
		m_EpJ2000tmp.clear();
		// ����
		OrderOrbit(m_EpWGS84, m_num);
		OrderOrbit(m_EpJ2000, m_num);
	
}