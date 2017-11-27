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

// ��������
struct ComplexDouble
{
	double real;
	double imag;
};


////////////////////////////////////////////////////////////
// ��λ���ƥ����
////////////////////////////////////////////////////////////
class DLL_EXPORT PhaseCorrelation
{
public:
	PhaseCorrelation(void);
	~PhaseCorrelation(void);
	
	// ��ȡ��������ƥ����(����)
	bool GetPixelTest(GeoReadImage *img1, GeoReadImage *img2, long lx, long ly, float &rx, float &ry, int win=32);
	// ��ȡ��������ƥ����
	bool GetPixel(GeoReadImage *img1, GeoReadImage *img2, long lx, long ly, float &rx, float &ry, int win=32);
	// ��ȡ��������ƥ����
	bool GetPixel(GeoReadImage *img1, GeoReadImage *img2, double lx, double ly, double &rx, double &ry, int win=32);
	// ��ȡ��������ƥ����(������)
	bool GetPixelOnly(GeoReadImage *img1, GeoReadImage *img2, double lx, double ly, double &rx, double &ry, int win=32);
	// ��ȡ������ƥ����(����)
	bool GetSubPixelTest(GeoReadImage *img1, GeoReadImage *img2, long lx, long ly, float &rx, float &ry, 
					double &SNR, double mration = 0.9, double beta = 0.5, int win = 32);
	// ��ȡ������ƥ����
	bool GetSubPixel(GeoReadImage *img1, GeoReadImage *img2, long lx, long ly, float &rx, float &ry, 
					double &SNR, double mration = 0.9, double beta = 0.5, int win = 32);
	// һάƵ��һά�ռ�ƥ��
	bool GetOnePixel(string lpath, string rpath, double &line, double &sample, long delnum = 40, long startdelnum = 1);
	// ceshi
	void Test(string path);


public:
	// �����˷�(�������)
	ComplexDouble ComplexMulti(ComplexDouble A, ComplexDouble B);
	// ��������
	ComplexDouble ComplexDiv(ComplexDouble A, double scale);
	// �����ӷ�
	ComplexDouble ComplexAdd(ComplexDouble A, ComplexDouble B);
	// ��������
	ComplexDouble ComplexSub(ComplexDouble A, ComplexDouble B);
	// ����˷�
	void Multi(double *A, double *B, double *C, int m, int p, int n);
	// ��ά����Ҷ���任
	void FFT3_2(ComplexDouble *A, ComplexDouble *B, int nx, int ny);
	// ��ά����Ҷ��任
	void iFFT3_2(ComplexDouble *A, ComplexDouble *B, int nx, int ny);
	// ��ά����Ҷ��Ƶ
	void FFT3shift_2(ComplexDouble *A, ComplexDouble *Res, int nx, int ny);
	// һά����Ҷ���任
	void FFT3_1(ComplexDouble *A, ComplexDouble *B, int nx);
	// һά����Ҷ��任
	void iFFT3_1(ComplexDouble *A, ComplexDouble *B, int nx);
	// һά����Ҷ��Ƶ
	void FFT3shift_1(ComplexDouble *A, ComplexDouble *Res, int nx);
	// ��ȡ������ģ
	double Cabs(ComplexDouble A);
	// �ڲ��ά��ֵ��,���ھ������
	bool InterExtreValue(double *C, int size, int posx, int posy, double &x, double &y);
	// �����ڴ�
	void Create(int size2, bool isDel = false, double beta = 0.5);
	// ����������ģ��
	void CreateWRC(int size, double beta);
	// �ͷ��ڴ�
	void Destroy();
	// Two-Point Step Size Gradient Methods,�����½���
	bool TwoPointStepSizeGradient(double &x, double &y, double *W, ComplexDouble *Q, int size);
	// ��ȡ�ݶ�(ͨ��ƫ��)
	void GetGradient(double x, double y, double *W, ComplexDouble *Q, int size, double *g);

	// �������
	void WriteComplex(int size, ComplexDouble *A, string pathComplex, string pathDouble="");
	void WriteDouble(int size, double *A, string path);

private:
	double m_beta;							// �����Ҵ������õ�betaֵ
	double *m_Wrc;							// ������ģ��
	int m_win, m_win2;						// ���ڴ�С
	// ����Ӱ��֮��Ĺ������õ��ı���
	ComplexDouble* img12_complr_fft;		// ��������Ƶ�������
	ComplexDouble* img12_complr;			// �������ڿռ������
	ComplexDouble* img12_complr_shift;		// �������ڿռ�����λ�����
	double* img12_C;						// �����׵�ģ
	// ��Ӱ�������
	double *blockimg1;						// �󴰿���������
	ComplexDouble* img1_complr;				// FFT�任ǰ
	ComplexDouble* img1_complr_fft;			// FFT�任��
	ComplexDouble* img1_complr_shift;		// FFT��Ƶ��
	// ��Ӱ�������
	double *blockimg2;						// �Ҵ�����������
	ComplexDouble* img2_complr;				// FFT�任ǰ
	ComplexDouble* img2_complr_fft;			// FFT�任��
	ComplexDouble* img2_complr_shift;		// FFT��Ƶ��
	// Ƶ����Ĥ������
	double *img12_LS;						// ������ȡ����
	double *img12_W;						// ��ʼ��Ĥ
};

#endif

