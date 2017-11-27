#ifndef _GEOTIME
#define	_GEOTIME

#include "GeoDefine.h"
#include "GeoBase.h"

////////////////////////////////////////////////////////
// ʱ�����
////////////////////////////////////////////////////////
class GeoTime
{
public:
	GeoTime(void);
	virtual ~GeoTime(void);

protected:
	long m_num;						// �г���ʱ�����
	double *m_Time;					// �洢ÿ��ɨ���ж�Ӧ��ʵ�ʳ���ʱ��
	double *m_DelTime;				// �洢ÿ�еĻ���ʱ��
	GeoBase m_Base;					// �ײ�ͨ���㷨��
	double refMJD;					// �ο��ۼ�����ʼ��Ԫ��Լ��������
	// �������
	void ClearData();

public:
	long get_num();					// ��ȡ�г���ʱ�����
	double get_time(double line);	// ���������Ż�ö�Ӧ���г���ʱ��
	double get_refMJD();			// �ο��ۼ�����ʼ��Ԫ��Լ��������
	// ���º����ɸ�������ʵ��
	virtual void ReadTimeFile(string filepath);	// ��ȡ��ʱ�ļ�
	virtual void ReadZY3TimeFile(vector<LineScanTime> allTime);//��ȡZY3��ʱ�ļ�
};

#endif

