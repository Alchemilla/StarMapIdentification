#ifndef PHASE_CORRELATION_H
#define PHASE_CORRELATION_H
#include "GeoReadImage.h"

#if defined(_WIN32)
	#ifdef MATCHMETHOD_EXPORTS
		#define DLL_EXPORT __declspec(dllexport)
	#else
		#define DLL_EXPORT
	#endif
#else
	#define DLL_EXPORT
#endif

// 复数类型
struct ComplexDouble
{
	double real;
	double imag;
};


////////////////////////////////////////////////////////////
// 相位相关匹配类
////////////////////////////////////////////////////////////
class DLL_EXPORT PhaseCorrelation
{
public:
	PhaseCorrelation(void);
	~PhaseCorrelation(void);
	
	// 获取整数像素匹配结果(测试)
	bool GetPixelTest(GeoReadImage *img1, GeoReadImage *img2, long lx, long ly, float &rx, float &ry, int win=32);
	// 获取整数像素匹配结果
	bool GetPixel(GeoReadImage *img1, GeoReadImage *img2, long lx, long ly, float &rx, float &ry, int win=32);
	// 获取整数像素匹配结果
	bool GetPixel(GeoReadImage *img1, GeoReadImage *img2, double lx, double ly, double &rx, double &ry, int win=32);
	// 获取整数像素匹配结果(并行用)
	bool GetPixelOnly(GeoReadImage *img1, GeoReadImage *img2, double lx, double ly, double &rx, double &ry, int win=32);
	// 获取亚像素匹配结果(测试)
	bool GetSubPixelTest(GeoReadImage *img1, GeoReadImage *img2, long lx, long ly, float &rx, float &ry, 
					double &SNR, double mration = 0.9, double beta = 0.5, int win = 32);
	// 获取亚像素匹配结果
	bool GetSubPixel(GeoReadImage *img1, GeoReadImage *img2, long lx, long ly, float &rx, float &ry, 
					double &SNR, double mration = 0.9, double beta = 0.5, int win = 32);
	// 一维频率一维空间匹配
	bool GetOnePixel(string lpath, string rpath, double &line, double &sample, long delnum = 40, long startdelnum = 1);
	// ceshi
	void Test(string path);


public:
	// 复数乘法(共轭相乘)
	ComplexDouble ComplexMulti(ComplexDouble A, ComplexDouble B);
	// 复数除法
	ComplexDouble ComplexDiv(ComplexDouble A, double scale);
	// 复数加法
	ComplexDouble ComplexAdd(ComplexDouble A, ComplexDouble B);
	// 复数减法
	ComplexDouble ComplexSub(ComplexDouble A, ComplexDouble B);
	// 矩阵乘法
	void Multi(double *A, double *B, double *C, int m, int p, int n);
	// 二维傅里叶正变换
	void FFT3_2(ComplexDouble *A, ComplexDouble *B, int nx, int ny);
	// 二维傅里叶逆变换
	void iFFT3_2(ComplexDouble *A, ComplexDouble *B, int nx, int ny);
	// 二维傅里叶移频
	void FFT3shift_2(ComplexDouble *A, ComplexDouble *Res, int nx, int ny);
	// 一维傅里叶正变换
	void FFT3_1(ComplexDouble *A, ComplexDouble *B, int nx);
	// 一维傅里叶逆变换
	void iFFT3_1(ComplexDouble *A, ComplexDouble *B, int nx);
	// 一维傅里叶移频
	void FFT3shift_1(ComplexDouble *A, ComplexDouble *Res, int nx);
	// 求取复数的模
	double Cabs(ComplexDouble A);
	// 内插二维极值点,用于精化结果
	bool InterExtreValue(double *C, int size, int posx, int posy, double &x, double &y);
	// 创建内存
	void Create(int size2, bool isDel = false, double beta = 0.5);
	// 创建升余弦模板
	void CreateWRC(int size, double beta);
	// 释放内存
	void Destroy();
	// Two-Point Step Size Gradient Methods,最速下降法
	bool TwoPointStepSizeGradient(double &x, double &y, double *W, ComplexDouble *Q, int size);
	// 获取梯度(通过偏导)
	void GetGradient(double x, double y, double *W, ComplexDouble *Q, int size, double *g);

	// 测试输出
	void WriteComplex(int size, ComplexDouble *A, string pathComplex, string pathDouble="");
	void WriteDouble(int size, double *A, string path);

private:
	double m_beta;							// 升余弦窗口所用的beta值
	double *m_Wrc;							// 升余弦模板
	int m_win, m_win2;						// 窗口大小
	// 两幅影像之间的功率谱用到的变量
	ComplexDouble* img12_complr_fft;		// 功率谱在频率域的数
	ComplexDouble* img12_complr;			// 功率谱在空间域的数
	ComplexDouble* img12_complr_shift;		// 功率谱在空间域移位后的数
	double* img12_C;						// 功率谱的模
	// 左影像的内容
	double *blockimg1;						// 左窗口数据内容
	ComplexDouble* img1_complr;				// FFT变换前
	ComplexDouble* img1_complr_fft;			// FFT变换后
	ComplexDouble* img1_complr_shift;		// FFT移频后
	// 右影像的内容
	double *blockimg2;						// 右窗口数据内容
	ComplexDouble* img2_complr;				// FFT变换前
	ComplexDouble* img2_complr_fft;			// FFT变换后
	ComplexDouble* img2_complr_shift;		// FFT移频后
	// 频率掩膜的内容
	double *img12_LS;						// 功率谱取对数
	double *img12_W;						// 初始掩膜
};

#endif

