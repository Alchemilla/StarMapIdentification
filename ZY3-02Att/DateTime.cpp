#include "stdafx.h"
#include "DateTime.h"

int Cal2JD(int year, int month, int day, double fracday, double *jd0, double *mjd)
{
	int j, ly, my;
	long iypmy;
	// 算法允许的最早时间(4800BC)
	const int minyear = -4799;
	// 每个月的日期数索引表
	static const int MonthTable[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	// 预设状态为0 
	j = 0;
	// 检验年月的有效性
	if(year<minyear)
		return -1;
	if (month < 1 || month > 12)
		return -2;
	// 如果是闰年的2月则为1,否则为0
	ly = ((month==2)&&!(year%4)&&(year%100||!(year%400)));
	// 考虑闰年进去情况下检验日的有效性
	if((day<1)||(day>(MonthTable[month-1]+ly)))
		j = -3;
	// 检验日的小数部分的有效性
	if(!((fracday>=0)&&(fracday<1.0)))
		return -4;
	// 返回结果
	my = (month-14)/12;
	iypmy = (long)(year+my);
	*jd0 = 2400000.5;
	*mjd = (double)((1461*(iypmy + 4800))/4 + (367*(long)(month-2-12*my))/12
			-(3*((iypmy+4900)/100))/4 + (long)day - 2432076) + fracday;
	// 返回状态
	return j;
}
////////////////////////////////////////////////////////////////
// 从约化儒略日(Julian Day)到格里高利历(Gregorian Calendar)的转化
// (年、月、日、日的小数部分)
// 输入:
//     double jd0：     约化儒略日的约化部分
//     double mjd：     约化儒略日部分
// 输出:
//     int *year：      格里高利历-年
//     int *month：     格里高利历-月
//     int *day：       格里高利历-日
//     double *fracday：格里高利历-日的小数部分
//  返回值：
//     int：      0    成功 
//               -1    输入儒略日违法
////////////////////////////////////////////////////////////////
int JD2Cal(double jd0, double mjd,int *year, int *month, int *day, double *fracday)
{
	// 儒略日所允许的最大和最小值
	static const double MinDay = -68569.5;
	static const double MaxDay = 1e9;
	long jd, l, n, i, k;
	double dj, d1, d2, f1, f2, f, d;
	// 检查儒略日是否越界
	dj = jd0 + mjd;
	if(dj<MinDay||dj>MaxDay)
		return -1;
	// 拷贝儒略日,先大后小
	if(jd0 >= mjd) { d1 = jd0;	d2 = mjd;	} 
	else		   { d1 = mjd;  d2 = jd0;	}
	d2 -= 0.5;
	// 分离日期的小数部分
	f1 = fmod(d1, 1.0);
	f2 = fmod(d2, 1.0);
	f = fmod(f1 + f2, 1.0);
	if(f<0.0) 
		f += 1.0;
	d = floor(d1-f1) + floor(d2-f2) + floor(f1+f2-f);
	jd = (long) floor(d) + 1;
	// 使用格里高利历表达日期
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
// 功能：将用户规定的累计秒转化为格里高利历和UT时
// 输入:
//		double refMJD：		参考历元的约化儒略日
//		double refsecond：	当前历元距离参考历元的累计秒
// 输出：
//		int &year:			待转化历元的年
//		int &month：		待转化历元的月
//		int &day：			待转化历元的日
//		int &hour：			待转化历元的时
//		int &minute：		待转化历元的分
//		double &second：	待转化历元的秒
// 返回值：
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
// 时间转化函数
//////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////
// 功能：将格里高利历与UT时转化为用户规定的累计秒
// 输入:
//		double refMJD：		参考历元的约化儒略日
//		int year:			待转化历元的年
//		int month：			待转化历元的月
//		int day：			待转化历元的日
//		int hour：			待转化历元的时
//		int minute：		待转化历元的分
//		double second：		待转化历元的秒
// 输出：
//		double &refsecond：	当前历元距离参考历元的累计秒
// 返回值：
//     void:			
////////////////////////////////////////////////////
void FromYMDtoSecond(double refMJD, int year, int month, int day, int hour, int minute, double second, double& refsecond)
{
	double jd0, mjd;
	Cal2JD(year, month, day, 0, &jd0, &mjd);
	refsecond = (mjd-refMJD)*86400 + hour*3600 + minute*60 + second;
}