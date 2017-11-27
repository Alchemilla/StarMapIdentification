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
// 通用函数类
///////////////////////////////////////////////////////////////
class GeoBase
{
public:
	GeoBase(void);
	virtual ~GeoBase(void);

public:
	//////////////////////////////////////////////////////////////////////////
	// 通用函数
	//////////////////////////////////////////////////////////////////////////
	// 从一个文件夹中按照通配符去搜索所有满足条件的文件(单层)
	void DiretorySearch(string dirPath, vector<string> &outPath, string dirCode);
	// 从一个文件夹中按照通配符去搜索所有满足条件的文件(多层)
	void DiretorySearchAll(string dirPath, vector<string> &outPath, string dirCode);
	// 从一个文件夹中搜索所有子文件夹(单层)
	void DiretorySearchFolder(string dirPath, vector<string> &folder);
	// 更改前缀后缀名(全部)
	void ChangeAndSaveExt(string outFolder, string ext, vector<string> &list1, vector<string> &list2);
	// 截取得到路径名称
	void GetDiretory(vector<string> &list1, vector<string> &list2);
	// 更改前缀后缀名(单个)
	void ChangeAndSaveExt(string outFolder, string ext, string &list1, string &list2);

public:
	//////////////////////////////////////////////////////////////////////////
	// 时间转化函数
	//////////////////////////////////////////////////////////////////////////
	// 将格里高利历与UT时转化为用户规定的累计秒
	void FromYMDtoSecond(double refMJD, int year, int month, int day, int hour, 
						 int minute, double second, double& refsecond);
	// 将用户规定的累计秒转化为格里高利历和UT时
	void FromSecondtoYMD(double refMJD, double refsecond, int& year, int& month, 
						 int& day, int& hour, int& minute, double& second);

public:
	//////////////////////////////////////////////////////////////////////////
	// 坐标转化函数
	//////////////////////////////////////////////////////////////////////////
	// 获得参考椭球系数
	int GetRefEllipsoid(string datumname, StrDATUM &datum);
	// 大地坐标转直角坐标
	void Geograph2Rect(StrDATUM datum, double B, double L, double H, double &X, double &Y, double &Z);
	// 大地坐标转经纬度坐标(此方法对于地球质心附近情况不适用)
	void Rect2Geograph(StrDATUM datum, double X, double Y, double Z,double &B, double &L, double &H);

public:
	//////////////////////////////////////////////////////////////////////////
	// 内插函数
	//////////////////////////////////////////////////////////////////////////
	// 轨道拉格朗日内插
	void LagrangianInterpolation(StrOrbitPoint *Eph, long EphNum, double UT, StrOrbitPoint &m_point, int order=7);
	// 四元数内插
	void QuatInterpolation(StrAttPoint *Att, long AttNum, double UT, StrAttPoint &m_att);
	void QuatInterpolation(vector<Attitude>Att, double UT, Attitude &m_att);
	//四元数乘法
	void quatMult(double *q1, double *q2, double *q3);

public:
	//////////////////////////////////////////////////////////////////////////
	// 旋转矩阵与四元数的相互变换
	//////////////////////////////////////////////////////////////////////////
	// 从旋转矩阵获得四元数
	void Matrix2Quat(double *R, double &q1, double &q2, double &q3, double &q4);
	// 从四元数获得旋转矩阵
	void Quat2Matrix(double q1, double q2, double q3, double q4, double *R);

public:
	//////////////////////////////////////////////////////////////////////////
	// 旋转矩阵与欧拉角的相互变换
	//////////////////////////////////////////////////////////////////////////
	// 从旋转矩阵获得欧拉角
	void Matrix2Eulor(double *R, int rotateOrder, double &eulor1, double &eulor2, double &eulor3);
	// 从欧拉角获得旋转矩阵
	void Eulor2Matrix(double eulor1, double eulor2, double eulor3, int rotateOrder, double *R);

public:
	//////////////////////////////////////////////////////////////////////////
	// 绕轴的旋转
	//////////////////////////////////////////////////////////////////////////
	// 绕X轴转角angle的旋转矩阵
	void RotationX(double angle, double *R);
	// 绕Y轴转角angle的旋转矩阵
	void RotationY(double angle, double *R);
	// 绕Z轴转角angle的旋转矩阵
	void RotationZ(double angle, double *R);
	//旋转
	void rot(double phi,double omg,double kap,double *R);
	
public:
	//////////////////////////////////////////////////////////////////////////
	// 矩阵的若干运算
	//////////////////////////////////////////////////////////////////////////
	// 法化
	void pNormal(double *a,int n,double b,double *aa, double *ab,double p);
	// 高斯求解
	int Gauss(double *ATA,double *ATL,int n);
	// 3*3的高斯求解
	bool solve33(double *A, double *al);
	//王新洲等，谱修正迭代法及其在测量数据处理中的应用
	void GaussExt(double *ATA,double *ATL,double *x,int n);
	// 求矩阵转置，形参m为行，n为列,A转置后存为B 
	void Transpose(double *A,double *B, int m,int n);
	// 求矩阵转置，形参m为行，n为列,A转置后存为A
	void Transpose(double *A,int m,int n);
	// 求矩阵相乘,A矩阵为[m,p],B矩阵为[p,n],C为[m,n] 
	void Multi(double *A,double *B,double *C ,int m,int p,int n);
	// 求矩阵和常数相乘,A矩阵为[m,n] 
	void Multi(double *A, int m, int n, double p);
	// 求矩阵相加,A矩阵为[m,n],B矩阵为[m,n],将B矩阵值加到A矩阵上
	void Add(double *A,double *B, int m,int n);
	// 求矩阵相加,A矩阵为[m,n],B矩阵为[m,n],C矩阵为[m,n],将AB矩阵值加到C矩阵上
	void Add(double *A,double *B, double *C, int m,int n);
	// 求矩阵相加,A矩阵为[m,n],B矩阵为[l, k],将B矩阵值加到A矩阵上,其加的左上角为(p, q)
	void Add(double *A,double *B, int m,int n, int l, int k, int p, int q);
	// 求矩阵相减,A矩阵为[m,n],B矩阵为[m,n],将B矩阵值加到A矩阵上
	void Minus(double *A,double *B, int m,int n);
	// 求矩阵相减,A矩阵为[m,n],B矩阵为[m,n],C矩阵为[m,n],将AB矩阵值加到C矩阵上
	void Minus(double *A,double *B, double *C, int m,int n);
	// 求矩阵相减,A矩阵为[m,n],B矩阵为[l, k],将B矩阵值加到A矩阵上,其加的左上角为(p, q)
	void Minus(double *A,double *B, int m,int n, int l, int k, int p, int q);
	// 矩阵反号,A矩阵为[m,n],B矩阵为[m,n]
	void RevSig(double *A,double *B, int m,int n);
	// 求矩阵行列式
	double Det(double *A,int m);
	// 求A的逆矩阵C 
	void Inv(double *A,double *C,int m);
	// 求A的逆矩阵,直接改变A值
	void Inv33(double *A,double *C,int m);
	// 对向量进行归一化
	void NormVector(double *R, int num);
	// 求取向量的模
	double Norm(double *R, int num);
	// 两个向量点乘
	double Dot(double *A, double *B, int num);
	// 三阶向量叉乘
	void CrossMult(double *u, double *v, double *w);

public:
	//////////////////////////////////////////////////////////////////////////
	//迭代有关
	//////////////////////////////////////////////////////////////////////////
	bool isExitlter(double *pData, double val, int num);

public:
	//////////////////////////////////////////////////////////////////////////
	// L0和L1范数的若干求解
	//////////////////////////////////////////////////////////////////////////
	// 求取一个矩阵的范数
	double Mat_Norm(double *A, int m, int index=1);
	// 求取一个矩阵的条件数
	double Mat_Cov(double *A, int m, int index=1);
	// 求取实对称矩阵的特征值和特征向量
	bool JacbiCor(double *A, int n, double *pVec, double *pEigVal, double eps, int itenum);
	
	// 岭估计法(L曲线求解)
	void GaussL(double *ATA,double *ATL, int n, double LTL);
	// L曲线函数
	double L_curve(double *ATA, double *ATL, int n, double LTL);
	// L曲线所使用的函数
	void lcfun(double *lambda,int nump,double *ata,double *atl,int n ,double ltl,double *g);
	void lamda2etarho(double lambda,double *ata,double *atl,int n,double ltl,double &eta,double &rho);
	void lamda2detadrho(double lambda,double *ata,double *atl,int n,double ltl,double &deta,double &drho);
	void lamda2ddetaddrho(double lambda,double *ata,double *atl,int n,double ltl,double &ddeta,double &ddrho);
	int invers_matrix(double *m1, int n);
	void trmul(double *a, double *b, int m, int n,int k,double *c);
	double vectornorm(double *a,int m);
	void minvector(double *vec,int m,int &mini);
	double fminbnd(double reg_min,double reg_max,double *ata,double *atl,int n ,double ltl,double eps);

	// L0范数下贪婪算法：正交匹配跟踪求解方法(OrthMatchPursuit)/无迭代
	int OMP_Solve(double *A, double *L, int m, int n, double *x, double d_res=10e-6, double min_res=10e-4);
	// L0范数下贪婪算法：正交匹配跟踪求解方法(OrthMatchPursuit)/有迭代
	int OMP_Solve2(double *A, double *L, int m, int n, double *x, double d_res=10e-6, double min_res=10e-4);

	// 有理函数模型的次数验证
	void RFMdegreeTest(int degree, string outfile);
	// 有理函数模型的次数验证
	void RFMdegreeTest1(int degree, string outfile);

public:
	//////////////////////////////////////////////////////////////////////////
	// 拟合函数
	//////////////////////////////////////////////////////////////////////////
	// 多项式拟合
	void PolynominalFitting(double *x,double *y,int n,int order,double *p);
	// 多项式拟合及其精度
	void PolynominalFittingError(double *x,double *y,int n,int order,double *error,double *p);
	// 多项式拟合求值
	void PolyValue(double *p,double *pDelta, double UT, double *value);
	// 求取归一化系数
	void Compute_avAnddx(double *a,int num, double &av, double &dx);
	// 进行归一化
	void Normaliza_avAnddx(double *a, int num, double av, double dx);
	// 进行反归一化
	void DNormaliza_avAnddx(double *a, int num, double av, double dx);
	// 计算绝对值平均值
	double FabsAndAve(double *a, int num);

public:
	// 产生一个符合正态分布的随机数
	double GaussRand(double mean, double sigma);
	// 产生一个符合正态分布的随机数
	double GaussRand2(double mean, double sigma);
	// 生成随机误差
	void RandomDistribution(double mean, double sigma, int n, long randCount, double *a);

public:
	// 从轨道六根数转化到轨道位置和速度
	void OrbitEle2PosAndVel(double a, double e, double i, double omega, double AscNode, double f, double *X, bool istrueAnomaly = true);
	// 从轨道位置和速度转化到轨道六根数,正确性待验证
	void PosAndVel2OrbitEle(double *X, double &a, double &e, double &i, double &omega, double &AscNode, double &f, bool istrueAnomaly = true);
	// 偏近点角和平近点角转化
	void TransMandE(double &M, double &E, double e, bool isM2E);
};



//////////////////////////////////////////////////////////////////////////
// 外部封的函数库,主要用于J2000到WGS84的相互转化
//////////////////////////////////////////////////////////////////////////
// 设置EOP文件路径
extern "C" int  _stdcall SetEopPath(char* EopPath);

// 从格里高利历到约化儒略日的转化
extern "C" int  _stdcall Cal2JD(int year, int month, int day,double fracday, double *jd0, double *mjd);  
// 从约化儒略日到格里高利历的转化
extern "C" int  _stdcall JD2Cal(double jd0, double mjd, int *year, int *month, int *day,double *fracday); 
// 获得TDB和UTC的差值
extern "C" double _stdcall  GetTDBminusUTC(int year, int month, int day, int hour, int minute, double second, char* eoppath);
// 获得闰秒
extern "C" double _stdcall  GetLeapSecond(int year, int month, int day, int hour, int minute, double second, char* eoppath);

//地球相关
// IAU 2000A, 基于CIO方式,从惯性系转化到非惯性系
extern "C" void _stdcall IAU2000ABaseCIOCelToTer(int year,int month,int day, int hour,int minute,double second,
	char* EOP_path, int order, double *R=NULL,double *Pos=NULL,double *Vel=NULL);
// IAU 2000A, 基于CIO方式,从非惯性系转化到惯性系
extern "C" void _stdcall IAU2000ABaseCIOTerToCel(int year,int month,int day, int hour,int minute,double second,
	char* EOP_path, int order, double *R=NULL,double *Pos=NULL,double *Vel=NULL);

// 月球相关
// 基于JPL_DE_LE,从惯性系转化到月心天球坐标系
extern "C" void   __stdcall IAUMoonBaseDELE_CelToSel(int year, int month, int day, int hour, int minute,
	double second, char* JPL_path, char* EOPPath, double *R=NULL, double *Pos=NULL, double *Vel=NULL);
// 基于JPL_DE_LE,从月心天球坐标系转化到惯性系
extern "C" void   __stdcall IAUMoonBaseDELE_SelToCel(int year, int month, int day, int hour, int minute,
	double second, char* JPL_path, char* EOPPath, double *R=NULL, double *Pos=NULL, double *Vel=NULL);
// 基于JPL_DE_LE,从月心天球坐标系转化到月固坐标
extern "C" void   __stdcall IAUMoonBaseDELE_SelToMer(int year, int month, int day, int hour, int minute,
	double second, char* JPL_path, char* EOPPath, double *R=NULL, double *Pos=NULL, double *Vel=NULL);
// 基于JPL_DE_LE,从月固坐标系转化到月心天球坐标
extern "C" void   __stdcall IAUMoonBaseDELE_MerToSel(int year, int month, int day, int hour, int minute,
	double second, char* JPL_path, char* EOPPath, double *R=NULL, double *Pos=NULL, double *Vel=NULL);


///////////////////////////////////////////////////////////////
// 联合并行积分器
///////////////////////////////////////////////////////////////
// 联合并行积分器多步积分法接口函数(基本形式)
extern "C" void _stdcall CombineIntegratorInterfaceBase(double starttime, double step, long num, int order,int n,
												        double *posoutx, double *posouty,double *posoutz,
											  	        double *veloutx, double *velouty,double *veloutz);
// 联合并行积分器多步积分法接口函数(从中间开始,基本形式)
extern "C" void _stdcall CombineIntegratorInterfaceBaseMiddle(double starttime, double step, long num, int order,int n,
												        double *posoutx, double *posouty,double *posoutz,
											  	        double *veloutx, double *velouty,double *veloutz);
// 联合并行积分器多步积分法验证函数
extern "C" void _stdcall CombineIntegratorCheck(double a, double e, double i, double capitalomiga, double omiga,
	                            double t1, double t2, double step, int n, char* EopPath, char* outpath);



// 行星位置相关
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
//               0 = MERCURY	水星    7 = NEPTUNE		海王星
//               1 = VENUS		金星    8 = PLUTO		冥王星
//               2 = EARTH		地球    9 = MOON		月球
//               3 = MARS		火星    10 = SUN		太阳
//               4 = JUPITER	木星    11 = SOLAR-SYSTEM BARYCENTER	太阳系质心
//               5 = SATURN		土星    12 = EARTH-MOON BARYCENTER		地月系执行
//               6 = URANUS		天王星  13 = NUTATIONS (intITUDE AND OBLIQ)	章动
//              14 = LIBRATIONS, IF ON EPH FILE		天平动
// 输出：
//		Pos：内插状态(位置、速度)值
// 返回值：
//		void:
/////////////////////////////////////////////////////
extern "C" void __stdcall PlanetEph(int year, int month, int day, int hour, int minute, 
		double second, char* JPL_path, char* EOPPath, int targ, int cent, double *Pos);

// 采用分析方法获得月球在惯性系下位置
extern "C" void __stdcall GetMoonBaseAnalyse(int year, int month, int day, 
						int hour, int minute, double second, double *moonpos);

// 采用分析方法获得太阳在惯性系下位置
extern "C" void __stdcall GetSunBaseAnalyse(int year, int month, int day, 
						int hour, int minute, double second, double *sunpos);

#endif

