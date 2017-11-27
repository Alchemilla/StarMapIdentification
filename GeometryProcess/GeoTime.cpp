#include "GeoTime.h"

//////////////////////////////////////////////////////////////////////////
// ����/��������
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

// �������
void GeoTime::ClearData()
{
	m_num  = 0;
	if(m_Time!=NULL)	{ delete []m_Time;		m_Time = NULL; }
	if(m_DelTime!=NULL) { delete []m_DelTime;	m_DelTime = NULL; }
}


// ��ȡ�г���ʱ�����
long GeoTime::get_num()
{
	return m_num;
}


// �ο��ۼ�����ʼ��Ԫ��Լ��������
double GeoTime::get_refMJD()
{
	return refMJD;
}


//////////////////////////////////////
// ���ܣ����������Ż�ö�Ӧ���г���ʱ��
// ����:
//		double line:	�г������ʱ��������
// ����ֵ��
//		double��		�����г������ʱ��
//////////////////////////////////////
double GeoTime::get_time(double line)
{
	if(m_num>1)
	{
		long left, right;
		if(line<=0||line!=line)	// line!=line����˼��lineֵΪ��Чֵ
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


// ���º����ɸ�������ʵ��
void GeoTime::ReadTimeFile(string filepath){}
//��ȡZY3��ʱ�ļ�
void GeoTime::ReadZY3TimeFile(vector<LineScanTime> allTime){}
