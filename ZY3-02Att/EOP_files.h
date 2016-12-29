
//#pragma comment(lib, "Need\\Win32\\CEOPDLL.lib")
//#pragma comment(lib, "C:\\Users\\wcsgz\\Documents\\2-CProject\\9-ZY3\\Need\\x64\\CEOPDLL.lib")

#include <string>
using namespace std;

#ifndef EOP_FILES_H
#define EOP_FILES_H
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// �ⲿ��ĺ�����,��Ҫ����J2000��WGS84���໥ת��
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// ����EOP�ļ�·��
	extern "C" int  _stdcall SetEopPath(char* EopPath);

	// IAU 2000A, ����CIO��ʽ,�ӹ���ϵת�����ǹ���ϵ
	extern "C" void _stdcall IAU2000ABaseCIOCelToTer(int year,int month,int day, int hour,int minute,double second,
		char* EOP_path, int order, double *R=NULL,double *Pos=NULL,double *Vel=NULL);
	// IAU 2000A, ����CIO��ʽ,�ӷǹ���ϵת��������ϵ
	extern "C" void _stdcall IAU2000ABaseCIOTerToCel(int year,int month,int day, int hour,int minute,double second,
		char* EOP_path, int order, double *R=NULL,double *Pos=NULL,double *Vel=NULL);
	// IAU 2000A, ���ڴ��ֵ㷽ʽ,�ӹ���ϵת�����ǹ���ϵ
	extern "C" void _stdcall IAU2000ABaseEquinoxCelToTer(int year,int month,int day, int hour,int minute,double second,
		char* EOP_path, int order, double *R=NULL,double *Pos=NULL,double *Vel=NULL);
	// IAU 2000A, ���ڴ��ֵ㷽ʽ,�ӷǹ���ϵת��������ϵ
	extern "C" void _stdcall IAU2000ABaseEquinoxTerToCel(int year,int month,int day, int hour,int minute,double second,
		char* EOP_path, int order, double *R=NULL,double *Pos=NULL,double *Vel=NULL);
	// IAU 2006A, ����CIO��ʽ,�ӷǹ���ϵת��������ϵ
	extern "C" void _stdcall IAU2006ABaseCIOTerToCel(int year,int month,int day, int hour,int minute,double second,
		char* EOP_path, int order, double *R=NULL,double *Pos=NULL,double *Vel=NULL);

//////////////////////////////////////////////////////////////////
// ����λ�õ�һЩ����
//////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////
// �������Ǽ��໥λ��
// ���룺
//    year��     �������Ҫ�õ����ڲ�״̬(λ�á��ٶ�)����
//    month��    �������Ҫ�õ����ڲ�״̬(λ�á��ٶ�)����
//    day��      �������Ҫ�õ����ڲ�״̬(λ�á��ٶ�)����
//    hour��     �������Ҫ�õ����ڲ�״̬(λ�á��ٶ�)��ʱ
//    minute��   �������Ҫ�õ����ڲ�״̬(λ�á��ٶ�)�ķ�
//    second��   �������Ҫ�õ����ڲ�״̬(λ�á��ٶ�)����
//	  JPL_path:	 JPL��������·��
//	  EOPPath:	 EOP·��
//    targ��     Ŀ������
//    cent��     ��������
//               0 = MERCURY	ˮ��    7 = NEPTUNE ������
//               1 = VENUS	 ����    8 = PLUTO	 ڤ����
//               2 = EARTH	 ����    9 = MOON	 ����
//               3 = MARS	 ����    10 = SUN	 ̫��
//               4 = JUPITER	ľ��    11 = SOLAR-SYSTEM BARYCENTER	̫��ϵ����
//               5 = SATURN	 ����    12 = EARTH-MOON BARYCENTER	 ����ϵִ��
//               6 = URANUS	 ������  13 = NUTATIONS (intITUDE AND OBLIQ)	�¶�
//              14 = LIBRATIONS, IF ON EPH FILE	 ��ƽ��
// �����
//	 Pos���ڲ�״̬(λ�á��ٶ�)ֵ
// ����ֵ��
//	 void:
/////////////////////////////////////////////////////
extern "C" void __declspec(dllexport) __stdcall PlanetEph(int year, int month, int day, int hour, int minute, 
double second, char* JPL_path, char* EOPPath, int targ, int cent, double *Pos);
#endif