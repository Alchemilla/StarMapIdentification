#ifndef _GEOBASE
#define	_GEOBASE

#include "GeoDefine.h"
#include <Eigen\Dense>
#include <Eigen\Sparse>
#include <Eigen\SparseQR>
#include <Eigen\OrderingMethods>
using namespace Eigen;
typedef SparseMatrix<double> SparseMatrixType;
///////////////////////////////////////////////////////////////
// ͨ�ú�����
///////////////////////////////////////////////////////////////
class GeoBase
{
public:
	GeoBase(void);
	virtual ~GeoBase(void);

public:
	//////////////////////////////////////////////////////////////////////////
	// ͨ�ú���
	//////////////////////////////////////////////////////////////////////////
	// ��һ���ļ����а���ͨ���ȥ�������������������ļ�(����)
	void DiretorySearch(string dirPath, vector<string> &outPath, string dirCode);
	// ��һ���ļ����а���ͨ���ȥ�������������������ļ�(���)
	void DiretorySearchAll(string dirPath, vector<string> &outPath, string dirCode);
	// ��һ���ļ����������������ļ���(����)
	void DiretorySearchFolder(string dirPath, vector<string> &folder);
	// ����ǰ׺��׺��(ȫ��)
	void ChangeAndSaveExt(string outFolder, string ext, vector<string> &list1, vector<string> &list2);
	// ��ȡ�õ�·������
	void GetDiretory(vector<string> &list1, vector<string> &list2);
	// ����ǰ׺��׺��(����)
	void ChangeAndSaveExt(string outFolder, string ext, string &list1, string &list2);

public:
	//////////////////////////////////////////////////////////////////////////
	// ʱ��ת������
	//////////////////////////////////////////////////////////////////////////
	// �������������UTʱת��Ϊ�û��涨���ۼ���
	void FromYMDtoSecond(double refMJD, int year, int month, int day, int hour, 
						 int minute, double second, double& refsecond);
	// ���û��涨���ۼ���ת��Ϊ�����������UTʱ
	void FromSecondtoYMD(double refMJD, double refsecond, int& year, int& month, 
						 int& day, int& hour, int& minute, double& second);

public:
	//////////////////////////////////////////////////////////////////////////
	// ����ת������
	//////////////////////////////////////////////////////////////////////////
	// ��òο�����ϵ��
	int GetRefEllipsoid(string datumname, StrDATUM &datum);
	// �������תֱ������
	void Geograph2Rect(StrDATUM datum, double B, double L, double H, double &X, double &Y, double &Z);
	// �������ת��γ������(�˷������ڵ������ĸ������������)
	void Rect2Geograph(StrDATUM datum, double X, double Y, double Z,double &B, double &L, double &H);

public:
	//////////////////////////////////////////////////////////////////////////
	// �ڲ庯��
	//////////////////////////////////////////////////////////////////////////
	// ������������ڲ�
	void LagrangianInterpolation(StrOrbitPoint *Eph, long EphNum, double UT, StrOrbitPoint &m_point, int order=7);
	// ��Ԫ���ڲ�
	void QuatInterpolation(StrAttPoint *Att, long AttNum, double UT, StrAttPoint &m_att);
	void QuatInterpolation(vector<Attitude>Att, double UT, Attitude &m_att);
	//��Ԫ���˷�
	void quatMult(double *q1, double *q2, double *q3);

public:
	//////////////////////////////////////////////////////////////////////////
	// ��ת��������Ԫ�����໥�任
	//////////////////////////////////////////////////////////////////////////
	// ����ת��������Ԫ��
	void Matrix2Quat(double *R, double &q1, double &q2, double &q3, double &q4);
	// ����Ԫ�������ת����
	void Quat2Matrix(double q1, double q2, double q3, double q4, double *R);

public:
	//////////////////////////////////////////////////////////////////////////
	// ��ת������ŷ���ǵ��໥�任
	//////////////////////////////////////////////////////////////////////////
	// ����ת������ŷ����
	void Matrix2Eulor(double *R, int rotateOrder, double &eulor1, double &eulor2, double &eulor3);
	// ��ŷ���ǻ����ת����
	void Eulor2Matrix(double eulor1, double eulor2, double eulor3, int rotateOrder, double *R);

public:
	//////////////////////////////////////////////////////////////////////////
	// �������ת
	//////////////////////////////////////////////////////////////////////////
	// ��X��ת��angle����ת����
	void RotationX(double angle, double *R);
	// ��Y��ת��angle����ת����
	void RotationY(double angle, double *R);
	// ��Z��ת��angle����ת����
	void RotationZ(double angle, double *R);
	//��ת
	void rot(double phi,double omg,double kap,double *R);
	
public:
	//////////////////////////////////////////////////////////////////////////
	// �������������
	//////////////////////////////////////////////////////////////////////////
	// ����
	void pNormal(double *a,int n,double b,double *aa, double *ab,double p);
	// ��˹���
	int Gauss(double *ATA,double *ATL,int n);
	// 3*3�ĸ�˹���
	bool solve33(double *A, double *al);
	//�����޵ȣ������������������ڲ������ݴ����е�Ӧ��
	void GaussExt(double *ATA,double *ATL,double *x,int n);
	// �����ת�ã��β�mΪ�У�nΪ��,Aת�ú��ΪB 
	void Transpose(double *A,double *B, int m,int n);
	// �����ת�ã��β�mΪ�У�nΪ��,Aת�ú��ΪA
	void Transpose(double *A,int m,int n);
	// ��������,A����Ϊ[m,p],B����Ϊ[p,n],CΪ[m,n] 
	void Multi(double *A,double *B,double *C ,int m,int p,int n);
	// �����ͳ������,A����Ϊ[m,n] 
	void Multi(double *A, int m, int n, double p);
	// ��������,A����Ϊ[m,n],B����Ϊ[m,n],��B����ֵ�ӵ�A������
	void Add(double *A,double *B, int m,int n);
	// ��������,A����Ϊ[m,n],B����Ϊ[m,n],C����Ϊ[m,n],��AB����ֵ�ӵ�C������
	void Add(double *A,double *B, double *C, int m,int n);
	// ��������,A����Ϊ[m,n],B����Ϊ[l, k],��B����ֵ�ӵ�A������,��ӵ����Ͻ�Ϊ(p, q)
	void Add(double *A,double *B, int m,int n, int l, int k, int p, int q);
	// ��������,A����Ϊ[m,n],B����Ϊ[m,n],��B����ֵ�ӵ�A������
	void Minus(double *A,double *B, int m,int n);
	// ��������,A����Ϊ[m,n],B����Ϊ[m,n],C����Ϊ[m,n],��AB����ֵ�ӵ�C������
	void Minus(double *A,double *B, double *C, int m,int n);
	// ��������,A����Ϊ[m,n],B����Ϊ[l, k],��B����ֵ�ӵ�A������,��ӵ����Ͻ�Ϊ(p, q)
	void Minus(double *A,double *B, int m,int n, int l, int k, int p, int q);
	// ���󷴺�,A����Ϊ[m,n],B����Ϊ[m,n]
	void RevSig(double *A,double *B, int m,int n);
	// ���������ʽ
	double Det(double *A,int m);
	// ��A�������C 
	void Inv(double *A,double *C,int m);
	// ��A�������,ֱ�Ӹı�Aֵ
	void Inv33(double *A,double *C,int m);
	// ���������й�һ��
	void NormVector(double *R, int num);
	// ��ȡ������ģ
	double Norm(double *R, int num);
	// �����������
	double Dot(double *A, double *B, int num);
	// �����������
	void CrossMult(double *u, double *v, double *w);

public:
	//////////////////////////////////////////////////////////////////////////
	//�����й�
	//////////////////////////////////////////////////////////////////////////
	bool isExitlter(double *pData, double val, int num);

public:
	//////////////////////////////////////////////////////////////////////////
	// L0��L1�������������
	//////////////////////////////////////////////////////////////////////////
	// ��ȡһ������ķ���
	double Mat_Norm(double *A, int m, int index=1);
	// ��ȡһ�������������
	double Mat_Cov(double *A, int m, int index=1);
	// ��ȡʵ�Գƾ��������ֵ����������
	bool JacbiCor(double *A, int n, double *pVec, double *pEigVal, double eps, int itenum);
	
	// ����Ʒ�(L�������)
	void GaussL(double *ATA,double *ATL, int n, double LTL);
	// L���ߺ���
	double L_curve(double *ATA, double *ATL, int n, double LTL);
	// L������ʹ�õĺ���
	void lcfun(double *lambda,int nump,double *ata,double *atl,int n ,double ltl,double *g);
	void lamda2etarho(double lambda,double *ata,double *atl,int n,double ltl,double &eta,double &rho);
	void lamda2detadrho(double lambda,double *ata,double *atl,int n,double ltl,double &deta,double &drho);
	void lamda2ddetaddrho(double lambda,double *ata,double *atl,int n,double ltl,double &ddeta,double &ddrho);
	int invers_matrix(double *m1, int n);
	void trmul(double *a, double *b, int m, int n,int k,double *c);
	double vectornorm(double *a,int m);
	void minvector(double *vec,int m,int &mini);
	double fminbnd(double reg_min,double reg_max,double *ata,double *atl,int n ,double ltl,double eps);

	// L0������̰���㷨������ƥ�������ⷽ��(OrthMatchPursuit)/�޵���
	int OMP_Solve(double *A, double *L, int m, int n, double *x, double d_res=10e-6, double min_res=10e-4);
	// L0������̰���㷨������ƥ�������ⷽ��(OrthMatchPursuit)/�е���
	int OMP_Solve2(double *A, double *L, int m, int n, double *x, double d_res=10e-6, double min_res=10e-4);

	// ������ģ�͵Ĵ�����֤
	void RFMdegreeTest(int degree, string outfile);
	// ������ģ�͵Ĵ�����֤
	void RFMdegreeTest1(int degree, string outfile);

public:
	//////////////////////////////////////////////////////////////////////////
	// ��Ϻ���
	//////////////////////////////////////////////////////////////////////////
	// ����ʽ���
	void PolynominalFitting(double *x,double *y,int n,int order,double *p);
	// ����ʽ��ϼ��侫��
	void PolynominalFittingError(double *x,double *y,int n,int order,double *error,double *p);
	// ����ʽ�����ֵ
	void PolyValue(double *p,double *pDelta, double UT, double *value);
	// ��ȡ��һ��ϵ��
	void Compute_avAnddx(double *a,int num, double &av, double &dx);
	// ���й�һ��
	void Normaliza_avAnddx(double *a, int num, double av, double dx);
	// ���з���һ��
	void DNormaliza_avAnddx(double *a, int num, double av, double dx);
	// �������ֵƽ��ֵ
	double FabsAndAve(double *a, int num);

public:
	// ����һ��������̬�ֲ��������
	double GaussRand(double mean, double sigma);
	// ����һ��������̬�ֲ��������
	double GaussRand2(double mean, double sigma);
	// ����������
	void RandomDistribution(double mean, double sigma, int n, long randCount, double *a);

public:
	// �ӹ��������ת�������λ�ú��ٶ�
	void OrbitEle2PosAndVel(double a, double e, double i, double omega, double AscNode, double f, double *X, bool istrueAnomaly = true);
	// �ӹ��λ�ú��ٶ�ת�������������,��ȷ�Դ���֤
	void PosAndVel2OrbitEle(double *X, double &a, double &e, double &i, double &omega, double &AscNode, double &f, bool istrueAnomaly = true);
	// ƫ����Ǻ�ƽ�����ת��
	void TransMandE(double &M, double &E, double e, bool isM2E);
};



//////////////////////////////////////////////////////////////////////////
// �ⲿ��ĺ�����,��Ҫ����J2000��WGS84���໥ת��
//////////////////////////////////////////////////////////////////////////
// ����EOP�ļ�·��
extern "C" int  _stdcall SetEopPath(char* EopPath);

// �Ӹ����������Լ�������յ�ת��
extern "C" int  _stdcall Cal2JD(int year, int month, int day,double fracday, double *jd0, double *mjd);  
// ��Լ�������յ������������ת��
extern "C" int  _stdcall JD2Cal(double jd0, double mjd, int *year, int *month, int *day,double *fracday); 
// ���TDB��UTC�Ĳ�ֵ
extern "C" double _stdcall  GetTDBminusUTC(int year, int month, int day, int hour, int minute, double second, char* eoppath);
// �������
extern "C" double _stdcall  GetLeapSecond(int year, int month, int day, int hour, int minute, double second, char* eoppath);

//�������
// IAU 2000A, ����CIO��ʽ,�ӹ���ϵת�����ǹ���ϵ
extern "C" void _stdcall IAU2000ABaseCIOCelToTer(int year,int month,int day, int hour,int minute,double second,
	char* EOP_path, int order, double *R=NULL,double *Pos=NULL,double *Vel=NULL);
// IAU 2000A, ����CIO��ʽ,�ӷǹ���ϵת��������ϵ
extern "C" void _stdcall IAU2000ABaseCIOTerToCel(int year,int month,int day, int hour,int minute,double second,
	char* EOP_path, int order, double *R=NULL,double *Pos=NULL,double *Vel=NULL);

// �������
// ����JPL_DE_LE,�ӹ���ϵת����������������ϵ
extern "C" void   __stdcall IAUMoonBaseDELE_CelToSel(int year, int month, int day, int hour, int minute,
	double second, char* JPL_path, char* EOPPath, double *R=NULL, double *Pos=NULL, double *Vel=NULL);
// ����JPL_DE_LE,��������������ϵת��������ϵ
extern "C" void   __stdcall IAUMoonBaseDELE_SelToCel(int year, int month, int day, int hour, int minute,
	double second, char* JPL_path, char* EOPPath, double *R=NULL, double *Pos=NULL, double *Vel=NULL);
// ����JPL_DE_LE,��������������ϵת�����¹�����
extern "C" void   __stdcall IAUMoonBaseDELE_SelToMer(int year, int month, int day, int hour, int minute,
	double second, char* JPL_path, char* EOPPath, double *R=NULL, double *Pos=NULL, double *Vel=NULL);
// ����JPL_DE_LE,���¹�����ϵת����������������
extern "C" void   __stdcall IAUMoonBaseDELE_MerToSel(int year, int month, int day, int hour, int minute,
	double second, char* JPL_path, char* EOPPath, double *R=NULL, double *Pos=NULL, double *Vel=NULL);


///////////////////////////////////////////////////////////////
// ���ϲ��л�����
///////////////////////////////////////////////////////////////
// ���ϲ��л������ಽ���ַ��ӿں���(������ʽ)
extern "C" void _stdcall CombineIntegratorInterfaceBase(double starttime, double step, long num, int order,int n,
												        double *posoutx, double *posouty,double *posoutz,
											  	        double *veloutx, double *velouty,double *veloutz);
// ���ϲ��л������ಽ���ַ��ӿں���(���м俪ʼ,������ʽ)
extern "C" void _stdcall CombineIntegratorInterfaceBaseMiddle(double starttime, double step, long num, int order,int n,
												        double *posoutx, double *posouty,double *posoutz,
											  	        double *veloutx, double *velouty,double *veloutz);
// ���ϲ��л������ಽ���ַ���֤����
extern "C" void _stdcall CombineIntegratorCheck(double a, double e, double i, double capitalomiga, double omiga,
	                            double t1, double t2, double step, int n, char* EopPath, char* outpath);



// ����λ�����
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
//               0 = MERCURY	ˮ��    7 = NEPTUNE		������
//               1 = VENUS		����    8 = PLUTO		ڤ����
//               2 = EARTH		����    9 = MOON		����
//               3 = MARS		����    10 = SUN		̫��
//               4 = JUPITER	ľ��    11 = SOLAR-SYSTEM BARYCENTER	̫��ϵ����
//               5 = SATURN		����    12 = EARTH-MOON BARYCENTER		����ϵִ��
//               6 = URANUS		������  13 = NUTATIONS (intITUDE AND OBLIQ)	�¶�
//              14 = LIBRATIONS, IF ON EPH FILE		��ƽ��
// �����
//		Pos���ڲ�״̬(λ�á��ٶ�)ֵ
// ����ֵ��
//		void:
/////////////////////////////////////////////////////
extern "C" void __stdcall PlanetEph(int year, int month, int day, int hour, int minute, 
		double second, char* JPL_path, char* EOPPath, int targ, int cent, double *Pos);

// ���÷���������������ڹ���ϵ��λ��
extern "C" void __stdcall GetMoonBaseAnalyse(int year, int month, int day, 
						int hour, int minute, double second, double *moonpos);

// ���÷����������̫���ڹ���ϵ��λ��
extern "C" void __stdcall GetSunBaseAnalyse(int year, int month, int day, 
						int hour, int minute, double second, double *sunpos);

#endif

