#include "stdafx.h"
#include "DateTime.h"

int Cal2JD(int year, int month, int day, double fracday, double *jd0, double *mjd)
{
	int j, ly, my;
	long iypmy;
	// �㷨���������ʱ��(4800BC)
	const int minyear = -4799;
	// ÿ���µ�������������
	static const int MonthTable[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	// Ԥ��״̬Ϊ0 
	j = 0;
	// �������µ���Ч��
	if(year<minyear)
		return -1;
	if (month < 1 || month > 12)
		return -2;
	// ����������2����Ϊ1,����Ϊ0
	ly = ((month==2)&&!(year%4)&&(year%100||!(year%400)));
	// ���������ȥ����¼����յ���Ч��
	if((day<1)||(day>(MonthTable[month-1]+ly)))
		j = -3;
	// �����յ�С�����ֵ���Ч��
	if(!((fracday>=0)&&(fracday<1.0)))
		return -4;
	// ���ؽ��
	my = (month-14)/12;
	iypmy = (long)(year+my);
	*jd0 = 2400000.5;
	*mjd = (double)((1461*(iypmy + 4800))/4 + (367*(long)(month-2-12*my))/12
			-(3*((iypmy+4900)/100))/4 + (long)day - 2432076) + fracday;
	// ����״̬
	return j;
}
////////////////////////////////////////////////////////////////
// ��Լ��������(Julian Day)�����������(Gregorian Calendar)��ת��
// (�ꡢ�¡��ա��յ�С������)
// ����:
//     double jd0��     Լ�������յ�Լ������
//     double mjd��     Լ�������ղ���
// ���:
//     int *year��      ���������-��
//     int *month��     ���������-��
//     int *day��       ���������-��
//     double *fracday�����������-�յ�С������
//  ����ֵ��
//     int��      0    �ɹ� 
//               -1    ����������Υ��
////////////////////////////////////////////////////////////////
int JD2Cal(double jd0, double mjd,int *year, int *month, int *day, double *fracday)
{
	// �������������������Сֵ
	static const double MinDay = -68569.5;
	static const double MaxDay = 1e9;
	long jd, l, n, i, k;
	double dj, d1, d2, f1, f2, f, d;
	// ����������Ƿ�Խ��
	dj = jd0 + mjd;
	if(dj<MinDay||dj>MaxDay)
		return -1;
	// ����������,�ȴ��С
	if(jd0 >= mjd) { d1 = jd0;	d2 = mjd;	} 
	else		   { d1 = mjd;  d2 = jd0;	}
	d2 -= 0.5;
	// �������ڵ�С������
	f1 = fmod(d1, 1.0);
	f2 = fmod(d2, 1.0);
	f = fmod(f1 + f2, 1.0);
	if(f<0.0) 
		f += 1.0;
	d = floor(d1-f1) + floor(d2-f2) + floor(f1+f2-f);
	jd = (long) floor(d) + 1;
	// ʹ�ø���������������
	l = jd + 68569;
	n = (4*l)/146097;
	l -= (146097*n+3)/4;
	i = (4000*(l+1))/1461001;
	l -= (1461*i)/4 - 31;
	k = (80*l)/2447;
	*day = (int)(l-(2447*k)/80);
	l = k/11;
	*month = (int)(k+2-12*l);
	*year = (int)(100*(n-49)+i+l);
	*fracday = f;
	return 0;
}
////////////////////////////////////////////////////
// ���ܣ����û��涨���ۼ���ת��Ϊ�����������UTʱ
// ����:
//		double refMJD��		�ο���Ԫ��Լ��������
//		double refsecond��	��ǰ��Ԫ����ο���Ԫ���ۼ���
// �����
//		int &year:			��ת����Ԫ����
//		int &month��		��ת����Ԫ����
//		int &day��			��ת����Ԫ����
//		int &hour��			��ת����Ԫ��ʱ
//		int &minute��		��ת����Ԫ�ķ�
//		double &second��	��ת����Ԫ����
// ����ֵ��
//     void:			
////////////////////////////////////////////////////
void FromSecondtoYMD(double refMJD, double refsecond, int& year, int& month, int& day, int& hour, int& minute, double& second)
{
	double fracday, jd0;
	double mjd = refMJD + refsecond/86400;
	JD2Cal(2400000.5, mjd, &year, &month, &day, &fracday);
	Cal2JD(year, month, day, 0, &jd0, &mjd);
	fracday = refsecond - (mjd-refMJD)*86400;
	hour = (int)(fracday/3600);
	minute = (int)((fracday-hour*3600)/60.0);
	second = fracday - hour*3600.0 - minute*60.0;
}
//////////////////////////////////////////////////////////////////////////
// ʱ��ת������
//////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////
// ���ܣ��������������UTʱת��Ϊ�û��涨���ۼ���
// ����:
//		double refMJD��		�ο���Ԫ��Լ��������
//		int year:			��ת����Ԫ����
//		int month��			��ת����Ԫ����
//		int day��			��ת����Ԫ����
//		int hour��			��ת����Ԫ��ʱ
//		int minute��		��ת����Ԫ�ķ�
//		double second��		��ת����Ԫ����
// �����
//		double &refsecond��	��ǰ��Ԫ����ο���Ԫ���ۼ���
// ����ֵ��
//     void:			
////////////////////////////////////////////////////
void FromYMDtoSecond(double refMJD, int year, int month, int day, int hour, int minute, double second, double& refsecond)
{
	double jd0, mjd;
	Cal2JD(year, month, day, 0, &jd0, &mjd);
	refsecond = (mjd-refMJD)*86400 + hour*3600 + minute*60 + second;
}