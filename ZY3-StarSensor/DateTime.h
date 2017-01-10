#ifndef DATE_TIME
#define DATE_TIME
#include <math.h>
using namespace std;

int Cal2JD(int year, int month, int day, double fracday, double *jd0, double *mjd);

int JD2Cal(double jd0, double mjd,int *year, int *month, int *day, double *fracday);

void FromSecondtoYMD(double refMJD, double refsecond, int& year, int& month, int& day, int& hour, int& minute, double& second);

void FromYMDtoSecond(double refMJD, int year, int month, int day, int hour, int minute, double second, double& refsecond);
#endif