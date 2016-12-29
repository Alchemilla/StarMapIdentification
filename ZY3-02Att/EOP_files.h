
//#pragma comment(lib, "Need\\Win32\\CEOPDLL.lib")
//#pragma comment(lib, "C:\\Users\\wcsgz\\Documents\\2-CProject\\9-ZY3\\Need\\x64\\CEOPDLL.lib")

#include <string>
using namespace std;

#ifndef EOP_FILES_H
#define EOP_FILES_H
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 外部封的函数库,主要用于J2000到WGS84的相互转化
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 设置EOP文件路径
	extern "C" int  _stdcall SetEopPath(char* EopPath);

	// IAU 2000A, 基于CIO方式,从惯性系转化到非惯性系
	extern "C" void _stdcall IAU2000ABaseCIOCelToTer(int year,int month,int day, int hour,int minute,double second,
		char* EOP_path, int order, double *R=NULL,double *Pos=NULL,double *Vel=NULL);
	// IAU 2000A, 基于CIO方式,从非惯性系转化到惯性系
	extern "C" void _stdcall IAU2000ABaseCIOTerToCel(int year,int month,int day, int hour,int minute,double second,
		char* EOP_path, int order, double *R=NULL,double *Pos=NULL,double *Vel=NULL);
	// IAU 2000A, 基于春分点方式,从惯性系转化到非惯性系
	extern "C" void _stdcall IAU2000ABaseEquinoxCelToTer(int year,int month,int day, int hour,int minute,double second,
		char* EOP_path, int order, double *R=NULL,double *Pos=NULL,double *Vel=NULL);
	// IAU 2000A, 基于春分点方式,从非惯性系转化到惯性系
	extern "C" void _stdcall IAU2000ABaseEquinoxTerToCel(int year,int month,int day, int hour,int minute,double second,
		char* EOP_path, int order, double *R=NULL,double *Pos=NULL,double *Vel=NULL);
	// IAU 2006A, 基于CIO方式,从非惯性系转化到惯性系
	extern "C" void _stdcall IAU2006ABaseCIOTerToCel(int year,int month,int day, int hour,int minute,double second,
		char* EOP_path, int order, double *R=NULL,double *Pos=NULL,double *Vel=NULL);

//////////////////////////////////////////////////////////////////
// 行星位置的一些计算
//////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////
// 各个行星间相互位置
// 输入：
//    year：     输入的需要得到的内插状态(位置、速度)的年
//    month：    输入的需要得到的内插状态(位置、速度)的月
//    day：      输入的需要得到的内插状态(位置、速度)的日
//    hour：     输入的需要得到的内插状态(位置、速度)的时
//    minute：   输入的需要得到的内插状态(位置、速度)的分
//    second：   输入的需要得到的内插状态(位置、速度)的秒
//	  JPL_path:	 JPL行星星历路径
//	  EOPPath:	 EOP路径
//    targ：     目标星体
//    cent：     中心星体
//               0 = MERCURY	水星    7 = NEPTUNE 海王星
//               1 = VENUS	 金星    8 = PLUTO	 冥王星
//               2 = EARTH	 地球    9 = MOON	 月球
//               3 = MARS	 火星    10 = SUN	 太阳
//               4 = JUPITER	木星    11 = SOLAR-SYSTEM BARYCENTER	太阳系质心
//               5 = SATURN	 土星    12 = EARTH-MOON BARYCENTER	 地月系执行
//               6 = URANUS	 天王星  13 = NUTATIONS (intITUDE AND OBLIQ)	章动
//              14 = LIBRATIONS, IF ON EPH FILE	 天平动
// 输出：
//	 Pos：内插状态(位置、速度)值
// 返回值：
//	 void:
/////////////////////////////////////////////////////
extern "C" void __declspec(dllexport) __stdcall PlanetEph(int year, int month, int day, int hour, int minute, 
double second, char* JPL_path, char* EOPPath, int targ, int cent, double *Pos);
#endif