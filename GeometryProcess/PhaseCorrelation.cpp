#include <math.h>
#include <stdio.h>
#include "PhaseCorrelation.h"
#include "fftw3.h"
#include <float.h>
#include <math.h>
#include "GeoReadImage.h"

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

#ifndef PI2
#define PI2 6.2831853071795864769252867665590
#endif

////////////////////////////////////////////////////////////
// 构造函数
////////////////////////////////////////////////////////////
PhaseCorrelation::PhaseCorrelation(void)
{
	m_beta = -1;
	m_win = m_win2 = -1;
	// 升余弦窗口
	m_Wrc = NULL;
	// 两幅影像之间的功率谱用到的变量
	img12_complr_fft = img12_complr = img12_complr_shift = NULL;
	img12_C = NULL;
	// 左影像的内容
	blockimg1 = NULL;
	img1_complr = img1_complr_fft = img1_complr_shift = NULL;
	// 右影像的内容
	blockimg2 = NULL;
	img2_complr = img2_complr_fft = img2_complr_shift = NULL;
	// 频率掩膜的内容
	img12_LS = img12_W = NULL;
}


////////////////////////////////////////////////////////////
// 析构函数
////////////////////////////////////////////////////////////
PhaseCorrelation::~PhaseCorrelation(void)
{
	Destroy();
}


////////////////////////////////////////////////////////////
// 创建内存
// 输入：
//		int size2:			开辟空间大小
//		bool isDel：		是否全部重新开辟,默认为false
//							当为true时,则全部Destroy,重新开辟
//							当为false时，仅仅对当中的NULL进行开辟
//		double beta:		升余弦的beta值
// 返回值:
//		void
////////////////////////////////////////////////////////////
void PhaseCorrelation::Create(int size2, bool isDel, double beta)
{
	if(isDel==true)
		Destroy();
	if(m_Wrc==NULL)					m_Wrc = new double[size2];
	// 两幅影像之间的功率谱用到的变量
	if(img12_complr_fft==NULL)		img12_complr_fft = new ComplexDouble[size2];
	if(img12_complr==NULL)			img12_complr = new ComplexDouble[size2];
	if(img12_complr_shift==NULL)	img12_complr_shift = new ComplexDouble[size2];
	if(img12_C==NULL)				img12_C = new double[size2];
	// 左影像的内容
	if(blockimg1==NULL)				blockimg1 = new double[size2];
	if(img1_complr==NULL)			img1_complr = new ComplexDouble[size2];
	if(img1_complr_fft==NULL)		img1_complr_fft = new ComplexDouble[size2];
	if(img1_complr_shift==NULL)		img1_complr_shift = new ComplexDouble[size2];
	// 右影像的内容
	if(blockimg2==NULL)				blockimg2 = new double[size2];
	if(img2_complr==NULL)			img2_complr = new ComplexDouble[size2];
	if(img2_complr_fft==NULL)		img2_complr_fft = new ComplexDouble[size2];
	if(img2_complr_shift==NULL)		img2_complr_shift = new ComplexDouble[size2];
	// 频率掩膜的内容
	if(img12_LS==NULL)				img12_LS = new double[size2];
	if(img12_W==NULL)				img12_W = new double[size2];
	// 创建升余弦采样核
	if(isDel==true)
		CreateWRC((int)sqrt((double)size2), beta);
}

////////////////////////////////////////////////////////////
// 释放内存
////////////////////////////////////////////////////////////
void PhaseCorrelation::Destroy()
{
	if(m_Wrc!=NULL)					delete []m_Wrc;					m_Wrc = NULL;
	// 两幅影像之间的功率谱用到的变量
	if(img12_complr_fft!=NULL)		delete []img12_complr_fft;		img12_complr_fft = NULL;
	if(img12_complr!=NULL)			delete []img12_complr;			img12_complr = NULL;
	if(img12_complr_shift!=NULL)	delete []img12_complr_shift;	img12_complr_shift = NULL;
	if(img12_C!=NULL)				delete []img12_C;				img12_C = NULL;
	// 左影像的内容
	if(blockimg1!=NULL)				delete []blockimg1;				blockimg1 = NULL;
	if(img1_complr!=NULL)			delete []img1_complr;			img1_complr = NULL;
	if(img1_complr_fft!=NULL)		delete []img1_complr_fft;		img1_complr_fft = NULL;
	if(img1_complr_shift!=NULL)		delete []img1_complr_shift;		img1_complr_shift = NULL;
	// 右影像的内容
	if(blockimg2!=NULL)				delete []blockimg2;				blockimg2 = NULL;
	if(img2_complr!=NULL)			delete []img2_complr;			img2_complr = NULL;
	if(img2_complr_fft!=NULL)		delete []img2_complr_fft;		img2_complr_fft = NULL;
	if(img2_complr_shift!=NULL)		delete []img2_complr_shift;		img2_complr_shift = NULL;
	// 频率掩膜的内容
	if(img12_LS!=NULL)				delete []img12_LS;				img12_LS = NULL;
	if(img12_W!=NULL)				delete []img12_W;				img12_W = NULL;
}


////////////////////////////////////////////////////////////
// 创建升余弦模板
// 输入：
//		int size:			模板大小
//		double beta：		beta参数大小
// 返回值:
//		void
////////////////////////////////////////////////////////////
void PhaseCorrelation::CreateWRC(int size, double beta)
{
	m_beta = beta;
	double *wrc1=new double[size];
    int pos=int(size*(0.5-beta));
	int size2 = size/2;
	double param = PI/(2*beta*size);
	int absi,i;
	double cossita;
	int index=0;
	for(i=-size2; i<=size2; i++)
	{
		if(i==0)	continue;
		absi=abs(i);
		if(absi<=size2 && absi>=pos)
		{
			cossita=cos(param*(absi-pos));
			wrc1[index++]=cossita*cossita;
		}
		else
			wrc1[index++] = 1;
	}
	if(m_Wrc!=NULL)		delete []m_Wrc;		m_Wrc = NULL;
	m_Wrc = new double[size*size];
	Multi(wrc1, wrc1, m_Wrc, size, 1, size);
	delete []wrc1,wrc1 = NULL;	
}


////////////////////////////////////////////////////////////
// 求矩阵相乘,A矩阵为[m,p],B矩阵为[p,n],C为[m,n] 
////////////////////////////////////////////////////////////
void PhaseCorrelation::Multi(double *A, double *B, double *C, int m, int p, int n)
{
     for (int i=0;i<m;i++)
	 {
        for (int j=0;j<n;j++)
        {
            double sum=0;
            for (int k=0;k<p;k++)
                sum=sum+A[i*p+k]*B[k*n+j];
            C[i*n+j]=sum;
        }
	 }
}


////////////////////////////////////////////////////////////
// 二维傅里叶正变换
// 输入：
//		ComplexDouble *A：傅里叶变换前矩阵
//		int nx, ny:			傅里叶变换窗口
// 输入输出：
//		ComplexDouble *B:	傅里叶变换后矩阵
// 返回值:
//		void
////////////////////////////////////////////////////////////
void PhaseCorrelation::FFT3_2(ComplexDouble *A, ComplexDouble *B, int nx, int ny)
{
	double nxy = nx*ny;
	memcpy(B, A, sizeof(ComplexDouble)*nxy);
	fftw_plan fftw_tmp_plan;
	fftw_tmp_plan  = fftw_plan_dft_2d(nx, ny, reinterpret_cast<fftw_complex*>(B), 
		reinterpret_cast<fftw_complex*>(B), FFTW_FORWARD, FFTW_ESTIMATE | FFTW_DESTROY_INPUT);
	fftw_execute(fftw_tmp_plan);
	fftw_destroy_plan(fftw_tmp_plan);
	for(int i=0; i<nxy;i++)
	{
		B[i].real = B[i].real/nxy;
		B[i].imag = B[i].imag/nxy;
	}
}


////////////////////////////////////////////////////////////
// 二维傅里叶逆变换
// 输入：
//		ComplexDouble *A：傅里叶逆变换前矩阵
//		int nx, ny:			傅里叶逆变换窗口
// 输入输出：
//		ComplexDouble *B:	傅里叶逆变换后矩阵
// 返回值:
//		void
////////////////////////////////////////////////////////////
void PhaseCorrelation::iFFT3_2(ComplexDouble *A, ComplexDouble *B, int nx, int ny)
{
	double nxy = nx*ny;
	memcpy(B, A, sizeof(ComplexDouble)*nxy);
	fftw_plan fftw_tmp_plan;
	fftw_tmp_plan  = fftw_plan_dft_2d(nx, ny, reinterpret_cast<fftw_complex*>(B), 
		reinterpret_cast<fftw_complex*>(B), FFTW_BACKWARD, FFTW_ESTIMATE | FFTW_DESTROY_INPUT);
	fftw_execute(fftw_tmp_plan);
	fftw_destroy_plan(fftw_tmp_plan);
}


////////////////////////////////////////////////////////////
// 二维傅里叶移频
// 输入：
//		ComplexDouble *A：傅里叶移频前矩阵
//		int nx, ny:			傅里叶移频窗口
// 输入输出：
//		ComplexDouble *B:	傅里叶移频后矩阵
// 返回值:
//		void
////////////////////////////////////////////////////////////
void PhaseCorrelation::FFT3shift_2(ComplexDouble *A, ComplexDouble *B, int nx, int ny)
{
	int posi,i,j,posj;	
	for (i=0; i<ny; i++)
	{
		posi=(i+ny/2)%ny;
		for (j=0; j<nx; j++)
		{
			posj = (j+nx/2)%nx;
			B[posi*nx+posj] = A[i*nx+j];					
		}
	}
}

////////////////////////////////////////////////////////////
// 一维傅里叶正变换
// 输入：
//		ComplexDouble *A：傅里叶变换前矩阵
//		int nx, ny:			傅里叶变换窗口
// 输入输出：
//		ComplexDouble *B:	傅里叶变换后矩阵
// 返回值:
//		void
////////////////////////////////////////////////////////////
void PhaseCorrelation::FFT3_1(ComplexDouble *A, ComplexDouble *B, int nx)
{
	memcpy(B, A, sizeof(ComplexDouble)*nx);
	fftw_plan fftw_tmp_plan;
	fftw_tmp_plan  = fftw_plan_dft_1d(nx, reinterpret_cast<fftw_complex*>(B), 
		reinterpret_cast<fftw_complex*>(B), FFTW_FORWARD, FFTW_ESTIMATE | FFTW_DESTROY_INPUT);
	fftw_execute(fftw_tmp_plan);
	fftw_destroy_plan(fftw_tmp_plan);
	for(int i=0; i<nx;i++)
	{
		B[i].real = B[i].real/nx;
		B[i].imag = B[i].imag/nx;
	}
}


////////////////////////////////////////////////////////////
// 一维傅里叶逆变换
// 输入：
//		ComplexDouble *A：傅里叶逆变换前矩阵
//		int nx, ny:			傅里叶逆变换窗口
// 输入输出：
//		ComplexDouble *B:	傅里叶逆变换后矩阵
// 返回值:
//		void
////////////////////////////////////////////////////////////
void PhaseCorrelation::iFFT3_1(ComplexDouble *A, ComplexDouble *B, int nx)
{
	memcpy(B, A, sizeof(ComplexDouble)*nx);
	fftw_plan fftw_tmp_plan;
	fftw_tmp_plan  = fftw_plan_dft_1d(nx, reinterpret_cast<fftw_complex*>(B), 
		reinterpret_cast<fftw_complex*>(B), FFTW_BACKWARD, FFTW_ESTIMATE | FFTW_DESTROY_INPUT);
	fftw_execute(fftw_tmp_plan);
	fftw_destroy_plan(fftw_tmp_plan);
}


////////////////////////////////////////////////////////////
// 一维傅里叶移频
// 输入：
//		ComplexDouble *A：傅里叶移频前矩阵
//		int nx, ny:			傅里叶移频窗口
// 输入输出：
//		ComplexDouble *B:	傅里叶移频后矩阵
// 返回值:
//		void
////////////////////////////////////////////////////////////
void PhaseCorrelation::FFT3shift_1(ComplexDouble *A, ComplexDouble *B, int nx)
{
	for(int i=0; i<nx; i++)
	{
		B[(i+nx/2)%nx] = A[i];	
	}
}


////////////////////////////////////////////////////////////
// 求取复数的模
// 输入：
//		ComplexDouble A： 待求模的复数
// 返回值:
//		double：			模值
////////////////////////////////////////////////////////////
double PhaseCorrelation::Cabs(ComplexDouble A)
{
	return sqrt(A.real*A.real+A.imag*A.imag);
}



////////////////////////////////////////////////////////////
// 内插二维极值点,用于精化结果
// 输入：
//		double *C:			输入的二维方阵
//		int size:			二维方阵的维度
//		int posx, posy:		初始的x、y坐标
// 输出：
//		double &x,&y：		精化后的x、y坐标		
// 返回值:
//		bool
////////////////////////////////////////////////////////////
bool PhaseCorrelation::InterExtreValue(double *C, int size, int posx, int posy, double &x, double &y)
{
	// 内插位置
	if(posx>0&&posx<size-1&&posy>0&&posy<size-1)
	{
		long index = posy*size+posx;
		double temp0, temp1, temp2;
		// x方向的精化
		temp0 = C[index-1] + C[index-size-1] + C[index+size-1];
		temp1 = C[index] + C[index-size] + C[index+size];
		temp2 = C[index+1] + C[index-size+1] + C[index+size+1];
		x = ((posx-1)*temp0 + posx*temp1 + (posx+1)*temp2)/(temp0 + temp1 + temp2);
		// y方向的精化
		temp0 = C[index-1-size] + C[index-size] + C[index+1-size];
		temp1 = C[index-1] + C[index] + C[index+1];
		temp2 = C[index-1+size] + C[index+size] + C[index+1+size];
		y = ((posy-1)*temp0 + posy*temp1 + (posy+1)*temp2)/(temp0 + temp1 + temp2);
		return true;
	}
	else
		return false;
}


////////////////////////////////////////////////////////////
// 获取整数像素匹配结果(测试)
// 输入：
//		GeoReadImage *img1:	左影像指针
//		GeoReadImage *img2:	右影像指针
//		long lx,ly：		左影像像素坐标
//		int win:			匹配窗口大小(要求偶数)
// 输入输出：
//		float &rx,&ry：		粗匹配的右影像点及返回的准确点
// 返回值:
//		bool
////////////////////////////////////////////////////////////
bool PhaseCorrelation::GetPixelTest(GeoReadImage *img1, GeoReadImage *img2, long lx, long ly, float &rx, float &ry, int win)
{
	int win2 = win*win, i, j;
	int win0_5 = win/2;
	// 如果窗口大小不一样,则重新分配
	if(win!=m_win)
	{
		m_win = win;		m_win2 = win2;
		Create(win2, true);
	}
	// 如果一样,则不需要重新分配
	else
		Create(win2);

	// Test
	WriteDouble(win, m_Wrc, "D:\\Temp\\WRC.raw");

	/////////////////////////////////////////////
	// 对左影像进行傅里叶变换
	/////////////////////////////////////////////
	if(!img1->GetDataValueFromBuffer(img1->pBuffer[0], lx-win0_5, ly-win0_5, win, win, blockimg1))
		return false;
	for(i=0; i<win2; i++)
	{
		img1_complr[i].real = blockimg1[i];	//*m_Wrc[i];
		img1_complr[i].imag = 0;
	}
	// FFT
	FFT3_2(img1_complr, img1_complr_fft, win, win);
	FFT3shift_2(img1_complr_fft, img1_complr_shift, win, win);

	// test
	WriteDouble(win, blockimg1, "D:\\Temp\\1.raw");
	WriteComplex(win, img1_complr_fft, "D:\\Temp\\1_fft.raw", "D:\\Temp\\1_fft_double.raw");
	WriteComplex(win, img1_complr_shift, "D:\\Temp\\1_shift.raw", "D:\\Temp\\1_shift_double.raw");

	/////////////////////////////////////////////
	// 对右影像反复进行傅里叶变换求取互功率谱,以求取最佳位置
	/////////////////////////////////////////////
	ComplexDouble temp;
	double temp2;
	double cmax, cmax2, dx, dy;
	int maxx, maxy;
	int count = 0;
	double offx, offy;
	// 循环
	do 
	{	
		// 读取右影像	
		if(!img2->GetDataValueFromBuffer(img2->pBuffer[0], (long)(rx+0.5)-win0_5, (long)(ry+0.5)-win0_5, win, win, blockimg2))
			return false;
		for(i=0; i<win2; i++)
		{
			img2_complr[i].real = blockimg2[i];	//*m_Wrc[i];
			img2_complr[i].imag = 0;
		}
		// FFT
		FFT3_2(img2_complr, img2_complr_fft, win, win);
		FFT3shift_2(img2_complr_fft, img2_complr_shift, win, win);

		// test
		WriteDouble(win, blockimg2, "D:\\Temp\\2.raw");
		WriteComplex(win, img2_complr_fft, "D:\\Temp\\2_fft.raw", "D:\\Temp\\2_fft_double.raw");
		WriteComplex(win, img2_complr_shift, "D:\\Temp\\2_shift.raw", "D:\\Temp\\2_shift_double.raw");

		/////////////////////////////////////////////
        // 进行复相关,求取功率谱
		/////////////////////////////////////////////
		for(i=0; i<win2; i++)
		{
			temp = ComplexMulti(img1_complr_shift[i], img2_complr_shift[i]);
			img12_complr_fft[i] = ComplexDiv(temp, Cabs(temp));
		}
		// ifft
 		iFFT3_2(img12_complr_fft, img12_complr, win, win);
 		FFT3shift_2(img12_complr, img12_complr_shift, win, win);
		bool bTrue = false;		cmax = -999999;		maxx = maxy = -1;

		// test
		WriteComplex(win, img12_complr, "D:\\Temp\\12.raw", "D:\\Temp\\12_double.raw");
		WriteComplex(win, img12_complr_fft, "D:\\Temp\\12_fft.raw", "D:\\Temp\\12_fft_double.raw");
		WriteComplex(win, img12_complr_shift, "D:\\Temp\\12_shift.raw", "D:\\Temp\\12_shift_double.raw");

		// 求取响应最大的值
		long index;
		for(i=0; i<win; i++)
		{
			for(j=0; j<win; j++)
			{
				index = i*win+j;
				img12_C[index] = Cabs(img12_complr_shift[index]);
				if(img12_C[index]>cmax)
				{
//					cmax2 = cmax;
					cmax = img12_C[index];
					maxx = j;			maxy = i;
					bTrue = true;
				}
			}
		}
//		cmax2 = cmax/cmax2;
		// 注意:其实当两张影像重合到一个像素内的时候
		// 傅里叶反变换得到的结果是个无效值
		// 所以也就退出了循环,但是此时值是准确的
		if(!bTrue)		
			return true;
		// 二维内插极值点
		InterExtreValue(img12_C, win, maxx, maxy, dx, dy);
		offx = -(dx-win0_5);		offy = -(dy-win0_5);
 		rx = (long)(rx+0.5) + offx;		// 注意,因为影像是从(long)(rx+0.5)为起点
		ry = (long)(ry+0.5) + offy;		// 注意,因为影像是从(long)(ry+0.5)为起点
		count ++;		
	} while((fabs(offx)>1||fabs(offy)>1)&&count<20);
	// 迭代多次还没收敛,说明匹配失败
	if(count>19)
	{	
		return false;
 	}
	return true;
}



////////////////////////////////////////////////////////////
// 获取整数像素匹配结果
// 输入：
//		GeoReadImage *img1:	左影像指针
//		GeoReadImage *img2:	右影像指针
//		long lx,ly：		左影像像素坐标
//		int win:			匹配窗口大小(要求偶数)
//		double beta:		采样核beta系数
// 输入输出：
//		float &rx,&ry：		粗匹配的右影像点及返回的准确点
// 返回值:
//		bool
////////////////////////////////////////////////////////////
bool PhaseCorrelation::GetPixel(GeoReadImage *img1, GeoReadImage *img2, long lx, long ly, float &rx, float &ry, int win)
{
	int win2 = win*win, i, j;
	int win0_5 = win/2;
	// 如果窗口大小不一样,则重新分配
	if(win!=m_win)
	{
		m_win = win;		m_win2 = win2;
		Create(win2, true);
	}
	// 如果一样,则不需要重新分配
	else
		Create(win2);

	/////////////////////////////////////////////
	// 对左影像进行傅里叶变换
	/////////////////////////////////////////////
	if(!img1->GetDataValueFromBuffer(img1->pBuffer[0], lx-win0_5, ly-win0_5, win, win, blockimg1))
		return false;
	for(i=0; i<win2; i++)
	{
		img1_complr[i].real = blockimg1[i];
		img1_complr[i].imag = 0.0;
	}
	// FFT
	FFT3_2(img1_complr, img1_complr_fft, win, win);
	FFT3shift_2(img1_complr_fft, img1_complr_shift, win, win);

	/////////////////////////////////////////////
	// 对右影像反复进行傅里叶变换求取互功率谱,以求取最佳位置
	/////////////////////////////////////////////
	ComplexDouble temp;
	double cmax, cmax2, dx, dy;
	int maxx, maxy;
	int count = 0;
	double offx, offy;
	// 循环
	do 
	{	
		// 读取右影像	
		if(!img2->GetDataValueFromBuffer(img2->pBuffer[0], (long)(rx+0.5)-win0_5, (long)(ry+0.5)-win0_5, win, win, blockimg2))
			return false;
		for(i=0; i<win2; i++)
		{
			img2_complr[i].real = blockimg2[i];
			img2_complr[i].imag = 0.0;
		}
		// FFT
		FFT3_2(img2_complr, img2_complr_fft, win, win);
		FFT3shift_2(img2_complr_fft, img2_complr_shift, win, win);

		/////////////////////////////////////////////
        // 进行复相关,求取功率谱
		/////////////////////////////////////////////
		for(i=0; i<win2; i++)
		{
			temp = ComplexMulti(img1_complr_shift[i], img2_complr_shift[i]);
			img12_complr_fft[i] = ComplexDiv(temp, Cabs(temp));
		}
		// ifft
 		iFFT3_2(img12_complr_fft, img12_complr, win, win);
 		FFT3shift_2(img12_complr, img12_complr_shift, win, win);
		bool bTrue = false;		cmax = -999999;		maxx = maxy = -1;

		// 求取响应最大的值
		long index;
		for(i=0; i<win; i++)
		{
			for(j=0; j<win; j++)
			{
				index = i*win+j;
				img12_C[index] = Cabs(img12_complr_shift[index]);
				if(img12_C[index]>cmax)
				{
					cmax = img12_C[index];
					maxx = j;			maxy = i;
					bTrue = true;
				}
			}
		}
		// 注意:其实当两张影像重合到一个像素内的时候
		// 傅里叶反变换得到的结果是个无效值
		// 所以也就退出了循环,但是此时值是准确的
		if(!bTrue)	
			return false;
		// 二维内插极值点
		if(!InterExtreValue(img12_C, win, maxx, maxy, dx, dy))
			return false;
		offx = -(dx-win0_5);		offy = -(dy-win0_5);
 		rx = (long)(rx+0.5) + offx;		// 注意,因为影像是从(long)(rx+0.5)为起点
		ry = (long)(ry+0.5) + offy;		// 注意,因为影像是从(long)(ry+0.5)为起点
		count ++;		
	} while((fabs(offx)>1||fabs(offy)>1)&&count<20);
	// 迭代多次还没收敛,说明匹配失败
	if(count>18)
	{	
		return false;
 	}
	return true;
}



////////////////////////////////////////////////////////////
// 获取整数像素匹配结果
// 输入：
//		GeoReadImage *img1:	左影像指针
//		GeoReadImage *img2:	右影像指针
//		long lx,ly：		左影像像素坐标
//		int win:			匹配窗口大小(要求偶数)
//		double beta:		采样核beta系数
// 输入输出：
//		float &rx,&ry：		粗匹配的右影像点及返回的准确点
// 返回值:
//		bool
////////////////////////////////////////////////////////////
bool PhaseCorrelation::GetPixel(GeoReadImage *img1, GeoReadImage *img2, double lx, double ly, double &rx, double &ry, int win)
{
	int win2 = win*win, i, j;
	int win0_5 = win/2;
	// 如果窗口大小不一样,则重新分配
	if(win!=m_win)
	{
		m_win = win;		m_win2 = win2;
		Create(win2, true);
	}
	// 如果一样,则不需要重新分配
	else
		Create(win2);

	/////////////////////////////////////////////
	// 对左影像进行傅里叶变换
	/////////////////////////////////////////////
	if(!img1->GetDataValueFromBuffer(img1->pBuffer[0], lx-win0_5, ly-win0_5, win, win, blockimg1))
		return false;
	for(i=0; i<win2; i++)
	{
		img1_complr[i].real = blockimg1[i];
		img1_complr[i].imag = 0.0;
	}
	// FFT
	FFT3_2(img1_complr, img1_complr_fft, win, win);
	FFT3shift_2(img1_complr_fft, img1_complr_shift, win, win);

	/////////////////////////////////////////////
	// 对右影像反复进行傅里叶变换求取互功率谱,以求取最佳位置
	/////////////////////////////////////////////
	ComplexDouble temp;
	double cmax, cmax2, dx, dy;
	int maxx, maxy;
	int count = 0;
	double offx, offy;
	// 循环
	do 
	{	
		// 读取右影像	
		if(!img2->GetDataValueFromBuffer(img2->pBuffer[0], (long)(rx+0.5)-win0_5, (long)(ry+0.5)-win0_5, win, win, blockimg2))
			return false;
		for(i=0; i<win2; i++)
		{
			img2_complr[i].real = blockimg2[i];
			img2_complr[i].imag = 0.0;
		}
		// FFT
		FFT3_2(img2_complr, img2_complr_fft, win, win);
		FFT3shift_2(img2_complr_fft, img2_complr_shift, win, win);

		/////////////////////////////////////////////
        // 进行复相关,求取功率谱
		/////////////////////////////////////////////
		for(i=0; i<win2; i++)
		{
			temp = ComplexMulti(img1_complr_shift[i], img2_complr_shift[i]);
			img12_complr_fft[i] = ComplexDiv(temp, Cabs(temp));
		}
		// ifft
 		iFFT3_2(img12_complr_fft, img12_complr, win, win);
 		FFT3shift_2(img12_complr, img12_complr_shift, win, win);
		bool bTrue = false;		cmax = -999999;		maxx = maxy = -1;

		// 求取响应最大的值
		long index;
		for(i=0; i<win; i++)
		{
			for(j=0; j<win; j++)
			{
				index = i*win+j;
				img12_C[index] = Cabs(img12_complr_shift[index]);
				if(img12_C[index]>cmax)
				{
					cmax = img12_C[index];
					maxx = j;			maxy = i;
					bTrue = true;
				}
			}
		}
		// 注意:其实当两张影像重合到一个像素内的时候
		// 傅里叶反变换得到的结果是个无效值
		// 所以也就退出了循环,但是此时值是准确的
		if(!bTrue)	
			return false;
		// 二维内插极值点
		if(!InterExtreValue(img12_C, win, maxx, maxy, dx, dy))
			return false;
		offx = -(dx-win0_5);		offy = -(dy-win0_5);
 		rx = (long)(rx+0.5) + offx;		// 注意,因为影像是从(long)(rx+0.5)为起点
		ry = (long)(ry+0.5) + offy;		// 注意,因为影像是从(long)(ry+0.5)为起点
		count ++;		
	} while((fabs(offx)>1||fabs(offy)>1)&&count<20);
	// 迭代多次还没收敛,说明匹配失败
	if(count>18)
	{	
		return false;
 	}
	return true;
}



////////////////////////////////////////////////////////////
// 获取整数像素匹配结果
// 输入：
//		GeoReadImage *img1:	左影像指针
//		GeoReadImage *img2:	右影像指针
//		long lx,ly：		左影像像素坐标
//		int win:			匹配窗口大小(要求偶数)
//		double beta:		采样核beta系数
// 输入输出：
//		float &rx,&ry：		粗匹配的右影像点及返回的准确点
// 返回值:
//		bool
////////////////////////////////////////////////////////////
bool PhaseCorrelation::GetPixelOnly(GeoReadImage *img1, GeoReadImage *img2, double lx, double ly, double &rx, double &ry, int win)
{
	int win2 = win*win, i, j;
	int win0_5 = win/2;
	// 如果窗口大小不一样,则重新分配
	if(win!=m_win)
	{
		m_win = win;		m_win2 = win2;
		Create(win2, true);
	}
	// 如果一样,则不需要重新分配
	else
		Create(win2);

	/////////////////////////////////////////////
	// 对左影像进行傅里叶变换
	/////////////////////////////////////////////
	if(!img1->GetDataValueFromBuffer(img1->pBuffer[0], lx-win0_5, ly-win0_5, win, win, blockimg1))
		return false;
	for(i=0; i<win2; i++)
	{
		img1_complr[i].real = blockimg1[i];
		img1_complr[i].imag = 0.0;
	}
	// FFT
	FFT3_2(img1_complr, img1_complr_fft, win, win);
	FFT3shift_2(img1_complr_fft, img1_complr_shift, win, win);

	/////////////////////////////////////////////
	// 对右影像反复进行傅里叶变换求取互功率谱,以求取最佳位置
	/////////////////////////////////////////////
	ComplexDouble temp;
	double cmax, cmax2, dx, dy;
	int maxx, maxy;
	int count = 0;
	double offx, offy;
	// 循环
	do 
	{	
		// 读取右影像	
		if(!img2->GetDataValueFromBuffer(img2->pBuffer[0], (long)(rx+0.5)-win0_5, (long)(ry+0.5)-win0_5, win, win, blockimg2))
			return false;
		for(i=0; i<win2; i++)
		{
			img2_complr[i].real = blockimg2[i];
			img2_complr[i].imag = 0.0;
		}
		// FFT
		FFT3_2(img2_complr, img2_complr_fft, win, win);
		FFT3shift_2(img2_complr_fft, img2_complr_shift, win, win);

		/////////////////////////////////////////////
        // 进行复相关,求取功率谱
		/////////////////////////////////////////////
		for(i=0; i<win2; i++)
		{
			temp = ComplexMulti(img1_complr_shift[i], img2_complr_shift[i]);
			img12_complr_fft[i] = ComplexDiv(temp, Cabs(temp));
		}
		// ifft
 		iFFT3_2(img12_complr_fft, img12_complr, win, win);
 		FFT3shift_2(img12_complr, img12_complr_shift, win, win);
		bool bTrue = false;		cmax = -999999;		maxx = maxy = -1;

		// 求取响应最大的值
		long index;
		for(i=0; i<win; i++)
		{
			for(j=0; j<win; j++)
			{
				index = i*win+j;
				img12_C[index] = Cabs(img12_complr_shift[index]);
				if(img12_C[index]>cmax)
				{
					cmax = img12_C[index];
					maxx = j;			maxy = i;
					bTrue = true;
				}
			}
		}
		// 注意:其实当两张影像重合到一个像素内的时候
		// 傅里叶反变换得到的结果是个无效值
		// 所以也就退出了循环,但是此时值是准确的
		if(!bTrue)	
			return false;
		// 二维内插极值点
		if(!InterExtreValue(img12_C, win, maxx, maxy, dx, dy))
			return false;
		offx = -(dx-win0_5);		offy = -(dy-win0_5);
 		rx = (long)(rx+0.5) + offx;		// 注意,因为影像是从(long)(rx+0.5)为起点
		ry = (long)(ry+0.5) + offy;		// 注意,因为影像是从(long)(ry+0.5)为起点
		count ++;		
	} while((fabs(offx)>1||fabs(offy)>1)&&count<20);
	// 迭代多次还没收敛,说明匹配失败
	if(count>18)
	{	
		return false;
 	}
	return true;
}




////////////////////////////////////////////////////////////
// 获取亚像素匹配结果
// 输入：
//		GeoReadImage *img1:	左影像指针
//		GeoReadImage *img2:	右影像指针
//		long lx,ly：		左影像像素坐标
//		double mration:		比例系数
//		double beta:		采样核beta系数
//		int win:			匹配窗口大小(要求偶数)
// 输入输出：
//		float &rx,&ry：		粗匹配的右影像点及返回的准确点
//		double &SNR:		信噪比
// 返回值:
//		bool
////////////////////////////////////////////////////////////
bool PhaseCorrelation::GetSubPixel(GeoReadImage *img1, GeoReadImage *img2, long lx, long ly, float &rx, float &ry, 
									double &SNR, double mration, double beta, int win)
{
/*	int win2 = win*win, i, j;
	int win0_5 = win/2;
	// 如果窗口大小不一样,则重新分配
	if(win!=m_win)
	{
		m_win = win;		m_win2 = win2;
		Create(win2, true, beta);
	}
	// 如果一样,则不需要重新分配
	else
		Create(win2, false, beta);
	// 如果beta值不一样,则重新分配
	if(m_beta!=beta)
	{
		m_beta = beta;
		CreateWRC(win, m_beta);
	}

	// Test
	WriteDouble(win, m_Wrc, "D:\\Temp\\WRC.raw");
	/////////////////////////////////////////////
	// 对左影像进行傅里叶变换
	/////////////////////////////////////////////
	if(!img1->GetDataValueFromBuffer(img1->pBuffer[0], lx-win0_5, ly-win0_5, win, win, blockimg1))
		return false;
	for(i=0; i<win2; i++)
	{
		img1_complr[i].real = blockimg1[i]*m_Wrc[i];
		img1_complr[i].imag = 0.0;
	}
	// FFT
	FFT3_2(img1_complr, img1_complr_fft, win, win);
	FFT3shift_2(img1_complr_fft, img1_complr_shift, win, win);
	/////////////////////////////////////////////
	// 对右影像进行傅里叶变换
	/////////////////////////////////////////////	
	if(!img2->GetDataValueFromBuffer(img2->pBuffer[0], (long)(rx+0.5)-win0_5, (long)(ry+0.5)-win0_5, win, win, blockimg2))
		return false;
	for(i=0; i<win2; i++)
	{
		img2_complr[i].real = blockimg2[i]*m_Wrc[i];
		img2_complr[i].imag = 0.0;
	}
	// FFT
	FFT3_2(img2_complr, img2_complr_fft, win, win);
	FFT3shift_2(img2_complr_fft, img2_complr_shift, win, win);
	/////////////////////////////////////////////
	// 进行复相关,求取功率谱,同时求取频率掩膜的对数
	/////////////////////////////////////////////
	double uimg12_LS = 0.0;				// 归一化功率谱取对数
	double maximg12_LS = -9999;			// 功率谱取对数最大值
	ComplexDouble temp;
	double ctemp;
	for(i=0; i<win2; i++)
	{
		temp = ComplexMulti(img1_complr_shift[i], img2_complr_shift[i]);
		ctemp = Cabs(temp);
		img12_complr_fft[i] = ComplexDiv(temp, ctemp);	// 归一化互功率谱
		img12_LS[i] = log10(ctemp);						// 功率谱取对数
		uimg12_LS += img12_LS[i]/win2;					// 归一化功率谱取对数
		if(img12_LS[i]>maximg12_LS)	
			maximg12_LS = img12_LS[i];					// 功率谱取对数最大值
	}
	/////////////////////////////////////////////
	// 求取频率掩膜
	/////////////////////////////////////////////
	for(i=0; i<win2; i++)
	{
		if((img12_LS[i]-maximg12_LS)<=mration*(uimg12_LS-maximg12_LS))
			img12_W0[i]=0.0;
   		else
			img12_W0[i]=1.0;			
	}

	// Test
	WriteDouble(win, img12_W0, "D:\\Temp\\W0.raw");
	WriteDouble(win, img12_LS, "D:\\Temp\\LS.raw");
	WriteComplex(win, img12_complr_fft, "D:\\Temp\\12_fft.raw", "D:\\Temp\\12_fft_double.raw");


	/////////////////////////////////////////////
	// 迭代求取最佳偏移值
	/////////////////////////////////////////////
	double dx, dy, phisum, Wsum, wx, wy, wxy;
	ComplexDouble Qtemp;
	long rx0 = (long)(rx+0.5);
	long ry0 = (long)(ry+0.5);
	dx = rx - rx0;	
	dy = ry - ry0;
	rx = rx0;
	ry = ry0;
	int count = 0;
	int pos;
	// 迭代
	do 
	{
		// 首先用最速降法算出比较准确的偏移值
		if(!TwoPointStepSizeGradient(dx, dy, img12_W0, img12_complr_fft, win))
		{
			return false;
		}
		// 求取信噪比,并更新Q值和W值
		phisum = Wsum = 0.0;
        wx = wy = PI2/win;
		for(i=-win0_5;i<win0_5;i++)
		{
			for(j=-win0_5;j<win0_5;j++)
			{
				pos = (i+win0_5)*win+(j+win0_5);
				wxy = wx*(i)*dx+wy*(j)*dy;
				// 归一化互功率谱理论值
				img12_C0[pos].real = cos(wxy);		img12_C0[pos].imag = sin(wxy);	
				// 归一化功率谱带权残差矩阵
				img12_phi[pos]= img12_W0[pos]*pow(Cabs(ComplexSub(img12_complr_fft[pos],img12_C0[pos])), 2);
				// 重新计算新的W阵
				img12_W1[pos] = img12_W0[pos]*pow(1-img12_phi[pos]/4, 6);
				// 重新计算新的Q阵
				Qtemp.real = cos(-wxy);				Qtemp.imag = sin(-wxy);
				img12_Q1[pos] = ComplexMulti(img12_complr_fft[pos], Qtemp);
				// 累加用于计算SNR
				phisum += img12_phi[pos];  
				Wsum += img12_W0[pos];
			}
		}
		// 计算信噪比
		SNR = 1 - phisum/Wsum/4.0;
		// 修正偏移值
		rx += dx;		ry += dy;
		// 如果太大就认为无效
		if(fabs(dx)>1.5||fabs(dy)>1.5)
		{
			SNR = 0.0;				
			return false;
		}
		// 更新W阵和Q阵
		memcpy(img12_W0, img12_W1, sizeof(double)*win2);
		memcpy(img12_complr_fft, img12_Q1, sizeof(ComplexDouble)*win2);
		count++;
	} while(count<4);*/
	return true;
}



////////////////////////////////////////////////////////////
// 获取亚像素匹配结果(测试)
// 输入：
//		GeoReadImage *img1:	左影像指针
//		GeoReadImage *img2:	右影像指针
//		long lx,ly：		左影像像素坐标
//		double mration:		比例系数
//		double beta:		采样核beta系数
//		int win:			匹配窗口大小(要求偶数)
// 输入输出：
//		float &rx,&ry：		粗匹配的右影像点及返回的准确点
//		double &SNR:		信噪比
// 返回值:
//		bool
////////////////////////////////////////////////////////////
bool PhaseCorrelation::GetSubPixelTest(GeoReadImage *img1, GeoReadImage *img2, long lx, long ly, float &rx, float &ry, 
									double &SNR, double mration, double beta, int win)
{
	int win2 = win*win, i, j;
	int win0_5 = win/2;
	// 如果窗口大小不一样,则重新分配
	if(win!=m_win)
	{
		m_win = win;		m_win2 = win2;
		Create(win2, true, beta);
	}
	// 如果一样,则不需要重新分配
	else
		Create(win2, false, beta);
	// 如果beta值不一样,则重新分配
	if(m_beta!=beta)
	{
		m_beta = beta;
		CreateWRC(win, m_beta);
	}

	// Test
	WriteDouble(win, m_Wrc, "D:\\Temp\\WRC.raw");
	/////////////////////////////////////////////
	// 对左影像进行傅里叶变换
	/////////////////////////////////////////////
	if(!img1->GetDataValueFromBuffer(img1->pBuffer[0], lx-win0_5, ly-win0_5, win, win, blockimg1))
		return false;
	for(i=0; i<win2; i++)
	{
		img1_complr[i].real = blockimg1[i]*m_Wrc[i];
		img1_complr[i].imag = 0.0;
	}
	// FFT
	FFT3_2(img1_complr, img1_complr_fft, win, win);
	FFT3shift_2(img1_complr_fft, img1_complr_shift, win, win);
	/////////////////////////////////////////////
	// 对右影像进行傅里叶变换
	/////////////////////////////////////////////	
	if(!img2->GetDataValueFromBuffer(img2->pBuffer[0], (long)(rx+0.5)-win0_5, (long)(ry+0.5)-win0_5, win, win, blockimg2))
		return false;
	for(i=0; i<win2; i++)
	{
		img2_complr[i].real = blockimg2[i]*m_Wrc[i];
		img2_complr[i].imag = 0.0;
	}
	// FFT
	FFT3_2(img2_complr, img2_complr_fft, win, win);
	FFT3shift_2(img2_complr_fft, img2_complr_shift, win, win);
	/////////////////////////////////////////////
	// 进行复相关,求取功率谱,同时求取频率掩膜的对数
	/////////////////////////////////////////////
	double uimg12_LS = 0.0;				// 归一化功率谱取对数
	double maximg12_LS = -9999;			// 功率谱取对数最大值
	ComplexDouble temp;
	double ctemp;
	for(i=0; i<win2; i++)
	{
		temp = ComplexMulti(img1_complr_shift[i], img2_complr_shift[i]);
		ctemp = Cabs(temp);
		img12_complr_fft[i] = ComplexDiv(temp, ctemp);	// 归一化互功率谱
		img12_LS[i] = log10(ctemp);						// 功率谱取对数
		uimg12_LS += img12_LS[i]/win2;					// 归一化功率谱取对数
		if(img12_LS[i]>maximg12_LS)	
			maximg12_LS = img12_LS[i];					// 功率谱取对数最大值
	}
	/////////////////////////////////////////////
	// 求取频率掩膜
	/////////////////////////////////////////////
	for(i=0; i<win2; i++)
	{
		if((img12_LS[i]-maximg12_LS)<=mration*(uimg12_LS-maximg12_LS))
			img12_W[i]=0.0;
   		else
			img12_W[i]=1.0;			
	}

	// Test
	WriteDouble(win, img12_W, "D:\\Temp\\W0.raw");
	WriteDouble(win, img12_LS, "D:\\Temp\\LS.raw");
	WriteComplex(win, img12_complr_fft, "D:\\Temp\\12_fft.raw", "D:\\Temp\\12_fft_double.raw");

	/////////////////////////////////////////////
	// 迭代求取最佳偏移值
	/////////////////////////////////////////////
	double dx, dy, phisum, Wsum, wx, wy, wxy, sumdx, sumdy, img12_phi, img12_Wtemp;
	ComplexDouble Qtemp, img12_C, img12_Qtemp;
	long rx0 = (long)(rx+0.5);
	long ry0 = (long)(ry+0.5);
	sumdx = sumdy = 0;
	dx = rx - rx0;	
	dy = ry - ry0;
	rx = rx0;
	ry = ry0;
	int count = 0;
	int pos;
	// 迭代
	do 
	{
		dx = dy = 0;
		// 首先用最速降法算出比较准确的偏移值
		if(!TwoPointStepSizeGradient(dx, dy, img12_W, img12_complr_fft, win))
		{
			return false;
		}
		// 根据TPSS求取的偏移值,更新Q值和W值,并求取信噪比
		phisum = Wsum = 0.0;
        wx = wy = PI2/win;
		for(i=-win0_5;i<win0_5;i++)
		{
			for(j=-win0_5;j<win0_5;j++)
			{
				////////////////////////////////////
				// 信噪比
				///////////////////////////////////
				pos = (i+win0_5)*win+(j+win0_5);
				wxy = wx*(i)*dx + wy*(j)*dy;
				// 归一化互功率谱理论值
				img12_C.real = cos(wxy);		img12_C.imag = sin(wxy);	
				// 归一化功率谱带权残差矩阵
				img12_phi = img12_W[pos]*pow(Cabs(ComplexSub(img12_complr_fft[pos], img12_C)), 2);
				// 累加用于计算SNR
				phisum += img12_phi;  
				Wsum += img12_W[pos];
				////////////////////////////////////
				// 更新W和Q阵
				///////////////////////////////////
				// 重新计算新的W阵
				img12_Wtemp = img12_W[pos]*pow(1-img12_phi/4, 6);
				img12_W[pos] = img12_Wtemp;
				// 重新计算新的Q阵
				Qtemp.real = img12_C.real;		Qtemp.imag = -img12_C.imag;
				img12_Qtemp = ComplexMulti(img12_complr_fft[pos], Qtemp);
				img12_complr_fft[pos] = img12_Qtemp;
			}
		}
		// 计算信噪比
		SNR = 1 - phisum/Wsum/4.0;
		// 修正偏移值
		rx += dx;		ry += dy;
		sumdx += dx;	sumdy += dy;
		// 如果太大就认为无效
		if(fabs(dx)>1.5||fabs(dy)>1.5)
		{
			SNR = 0.0;				
			return false;
		}
		count++;
	} while(count<4);
	return true;
}


////////////////////////////////////////////////////////////
// Two-Point Step Size Gradient Methods最速下降法
// 输入：
//		double *W:			频率掩膜
//		ComplexDouble *Q：	归一化互功率谱
//		int size:			窗口大小
// 输入输出：
//		double &x,&y:		偏移值
// 返回值:
//		bool：
////////////////////////////////////////////////////////////
bool PhaseCorrelation::TwoPointStepSizeGradient(double &x, double &y, double *W, ComplexDouble *Q, int size)
{
	long size2 = size*size;
	// 获取初始偏移值
	double x0, y0, g0[2], g[2];
	x0 = x - 0.1;
	y0 = y - 0.1;
	// 获取初始梯度值
	double sumw = 0.0;
	for(int i=0; i<size2; i++)
		sumw += W[i];
	g0[0] = g0[1] = sumw;
//	GetGradient(x0, y0, W, Q, size, g0);
	// 开始进行迭代
	double dx, dy, dg[2];
	double a, dgtemp;
	int count = 0;
	do
	{
		// 获得当前的梯度
		GetGradient(x, y, W, Q, size, g);
		// 偏移值之差,初始值之差为0.1
		dx = x - x0;
		dy = y - y0;
		if(fabs(dx)>1.5||fabs(dy)>1.5)
		{
			return false;
		}
		// 梯度值之差
		dg[0] = g[0] - g0[0];
		dg[1] = g[1] - g0[1];
		dgtemp = dx*dg[0] + dy*dg[1];
		// 计算改正系数
		a = (dx*dx + dy*dy)/dgtemp;
		x0 = x;			y0 = y;
		memcpy(g0, g, sizeof(double)*2);
		x = x0 - a*g[0];
		y = y0 - a*g[1];
		count++;
	} while((fabs(dx)>0.001||fabs(dy)>0.001)&&count<=100);
	// 如果迭代次数太多,则认为寻找失败,否则则是成功
	if(count>=100)
		return false;
	return true;
}


////////////////////////////////////////////////////////////
// 获取梯度(通过偏导),分别有水平和垂直的梯度,求其累加值
// 输入：
//		double x,y:			偏移值
//		double *W:			频率掩膜
//		complex<double> *Q：归一化互功率谱
//		int size:			窗口大小
// 输出：
//		double *g:			计算出来的新的梯度值
// 返回值:
//		void
////////////////////////////////////////////////////////////
void PhaseCorrelation::GetGradient(double x, double y, double *W, ComplexDouble *Q, int size, double *g)
{
	int i, j, pos, size0_5;
	double wx, wy, cwx, cwy, ctemp, t;
	size0_5 = size/2;
	wx = wy = PI2/size;
	g[0] = g[1] = 0.0;	
	for(i=-size0_5; i<size0_5; i++)
	{
		cwy = i*wy;
		for(j=-size0_5; j<size0_5;j++)
		{
			cwx = j*wx;
			pos = (i + size0_5)*size + j + size0_5;
			ctemp = cwx*x + cwy*y;
			// 求取偏导数
			t = 2*W[pos]*(Q[pos].real*sin(ctemp) - Q[pos].imag*cos(ctemp));
			g[0] += t*cwx;
			g[1] += t*cwy;
		}
	}
}


////////////////////////////////////////////////////////////
// 测试输出
////////////////////////////////////////////////////////////
void PhaseCorrelation::WriteComplex(int size, ComplexDouble *A, string pathComplex, string pathDouble)
{
	long size2 = size*size;
	double real[2];
	FILE *fp = fopen(pathComplex.c_str(), "wb");
	if(fp!=NULL)
	{
		for(int i=0; i<size2; i++)
		{
			real[0] = A[i].real;
			real[1] = A[i].imag;
			fwrite(real, sizeof(double), 2, fp);
		}
		fclose(fp);		fp = NULL;
	}
	fp = fopen(pathDouble.c_str(), "wb");
	double temp;
	if(fp!=NULL)
	{
		for(int i=0; i<size2; i++)
		{
			temp = log(Cabs(A[i]))+1;
			//temp = Cabs(A[i]);
			fwrite(&temp, sizeof(double), 1, fp);
		}
		fclose(fp);		fp = NULL;
	}
}


////////////////////////////////////////////////////////////
// 测试输出
////////////////////////////////////////////////////////////
void PhaseCorrelation::WriteDouble(int size, double *A, string path)
{
	long size2 = size*size;
	FILE *fp = fopen(path.c_str(), "wb");
	if(fp!=NULL)
	{
		fwrite(A, sizeof(double), size2, fp);
		fclose(fp);		fp = NULL;
	}
}


////////////////////////////////////////////////////////////
// 复数乘法(共轭相乘)
////////////////////////////////////////////////////////////
ComplexDouble PhaseCorrelation::ComplexMulti(ComplexDouble A, ComplexDouble B)
{
	double a, b, c, d;
	ComplexDouble temp;
	a = A.real;		b = A.imag;		c = B.real;		d = -B.imag;	// 记得加负号
	temp.real = a*c-b*d;
	temp.imag = b*c+a*d;
	return temp;
}


////////////////////////////////////////////////////////////
// 复数除法
////////////////////////////////////////////////////////////
ComplexDouble PhaseCorrelation::ComplexDiv(ComplexDouble A, double scale)
{
	ComplexDouble temp;
	if(scale!=0)
	{
		temp.real = A.real/scale;
		temp.imag = A.imag/scale;
	}
	else
	{
		temp.real = 0;
		temp.imag = 0;
	}
	return temp;
}


////////////////////////////////////////////////////////////
// 复数加法
////////////////////////////////////////////////////////////
ComplexDouble PhaseCorrelation::ComplexAdd(ComplexDouble A, ComplexDouble B)
{
	ComplexDouble temp;
	temp.real = A.real + B.real;
	temp.imag = A.imag + B.imag;
	return temp;
}


////////////////////////////////////////////////////////////
// 复数减法
////////////////////////////////////////////////////////////
ComplexDouble PhaseCorrelation::ComplexSub(ComplexDouble A, ComplexDouble B)
{
	ComplexDouble temp;
	temp.real = A.real - B.real;
	temp.imag = A.imag - B.imag;
	return temp;
}


////////////////////////////////////////////////////////////
// 一维频率一维空间匹配
// 输入：
//		string lpath:		左影像路径
//		string rpath:		右影像路径
//		long delnum:		垂轨向最大重叠像元(这个从设计值得到)
//		long startdelnum:	垂轨向最小重叠像元(默认为1)
// 输出：
//		double &line:		沿轨向偏移(正数为右影像相对左影像向上平移,负数为右影像相对左影像向下平移)
//		double &sample:		垂轨向偏移（即重叠像素数）
// 返回值:
//		void
////////////////////////////////////////////////////////////
bool PhaseCorrelation::GetOnePixel(string lpath, string rpath, double &line, double &sample, long delnum, long startdelnum)
{
	//////////////////////////////////////////////
	// 打开影像并读取对应数据
	//////////////////////////////////////////////
	GeoReadImage img1, img2;
	img1.Open(lpath, GA_ReadOnly);
	img2.Open(rpath, GA_ReadOnly);
	long linenum = img1.m_yRasterSize<img2.m_yRasterSize?img1.m_yRasterSize:img2.m_yRasterSize;
	long left0 = img1.m_xRasterSize-1-delnum;
	// 读取左影像数据,左上角为(left0, 0),长宽分别为(delnum, linenum)
	img1.ReadBlock(left0, 0, delnum, linenum, 0, img1.pBuffer[0]);
	double *data1 = new double[delnum*linenum];
	if(!img1.GetDataValueFromBuffer(img1.pBuffer[0], 0, 0, delnum, linenum, data1))
		return false;
	// 读取右影像数据,左上角为(0,0),长宽分别为(delnum, linenum)
	img2.ReadBlock(0, 0, delnum, linenum, 0, img2.pBuffer[0]);
	double *data2 = new double[delnum*linenum];
	if(!img2.GetDataValueFromBuffer(img2.pBuffer[0], 0, 0, delnum, linenum, data2))
		return false;
	
	//////////////////////////////////////////////
	// 一列列匹配
	//////////////////////////////////////////////
	// 分配空间
	Create(linenum, true);
	// 构建匹配数据
	ComplexDouble temp, temp1;
	double temp2;
	double *maxvalue = new double[delnum];
	long *maxindex = new long[delnum];
	memset(maxvalue, 0, sizeof(double)*delnum);
	memset(maxindex, 0, sizeof(long)*delnum);

	long ktemp;
	for(int k=startdelnum; k<delnum; k++)
	{	
		// 奇数
		if(k%2==1)
		{
			ktemp = k/2;
			for(int i=0; i<linenum; i++)
			{
				img1_complr[i].real = data1[delnum*i+delnum-1-ktemp];	
				img1_complr[i].imag = 0.0;
				img2_complr[i].real = data2[delnum*i+ktemp];		
				img2_complr[i].imag = 0.0;	
			}
		}
		// 偶数
		else
		{
			ktemp = k/2-1;
			for(int i=0; i<linenum; i++)
			{
				img1_complr[i].real = (data1[delnum*i+delnum-1-ktemp]+data1[delnum*i+delnum-2-ktemp])/2;	
				img1_complr[i].imag = 0.0;
				img2_complr[i].real = (data2[delnum*i+ktemp]+data2[delnum*i+ktemp+1])/2;		
				img2_complr[i].imag = 0.0;	
			}
		}
		// 正变换
		FFT3_1(img1_complr, img1_complr_fft, linenum);
		FFT3shift_1(img1_complr_fft, img1_complr_shift, linenum);
		FFT3_1(img2_complr, img2_complr_fft, linenum);
		FFT3shift_1(img2_complr_fft, img2_complr_shift, linenum);
		// 进行复相关,求取功率谱
		for(int i=0; i<linenum; i++)
		{
			temp = ComplexMulti(img1_complr_shift[i], img2_complr_shift[i]);
			temp1 = ComplexDiv(temp, Cabs(temp));
			img12_complr_fft[i].real = temp1.real;
			img12_complr_fft[i].imag = temp1.imag;
		}
		// ifft
 		iFFT3_1(img12_complr_fft, img12_complr, linenum);
 		FFT3shift_1(img12_complr, img12_complr_shift, linenum);
		// 存储最大值及其对应索引号
		for(int i=0; i<linenum; i++)
		{
			temp2 = Cabs(img12_complr_shift[i]);
			if(temp2>maxvalue[k])
			{
				maxvalue[k] = temp2;
				maxindex[k] = i;
			}
		}
	}	
	/*
	FILE *fp = fopen("C:\\GetOnePixel.txt", "w");
	for(int k=0; k<delnum; k++)
		fprintf(fp, "%16.6lf\t%6d\n", maxvalue[k], maxindex[k]);
	fclose(fp);		fp = NULL;
	*/

	temp2 = 0;
	for(int k=0; k<delnum; k++)
	{
		if(maxvalue[k]>temp2)
		{
			temp2 = maxvalue[k];
			sample = k+1;
			line = linenum/2 - maxindex[k];
		}
	}
	printf("%lf\t%lf\n", sample, line);
	

	// 释放内存和影像
	if(maxvalue!=NULL)	delete []maxvalue;	maxvalue = NULL;
	if(maxindex!=NULL)	delete []maxindex;	maxindex = NULL;
	img1.Destroy();
	img2.Destroy();
	return 1;
}

// ceshi
void PhaseCorrelation::Test(string path)
{
	long num;
	double hwc1, hwc2;
	FILE *fp = fopen(path.c_str(), "r");
	fscanf(fp, "%ld", &num);
	// 分配空间
	Create(num, true);
	for(int i=0; i<num; i++)
	{	
		fscanf(fp, "%lf%lf", &hwc1, &hwc2);		
		img1_complr[i].real = hwc1;	
		img1_complr[i].imag = 0.0;
		img2_complr[i].real = hwc2;		
		img2_complr[i].imag = 0.0;	
	}
	// 正变换
	FFT3_1(img1_complr, img1_complr_fft, num);
	FFT3shift_1(img1_complr_fft, img1_complr_shift, num);
	FFT3_1(img2_complr, img2_complr_fft, num);
	FFT3shift_1(img2_complr_fft, img2_complr_shift, num);
	ComplexDouble temp, temp1;
	double temp2;
	// 进行复相关,求取功率谱
	for(int i=0; i<num; i++)
	{
		temp = ComplexMulti(img1_complr_shift[i], img2_complr_shift[i]);
		temp1 = ComplexDiv(temp, Cabs(temp));
		img12_complr_fft[i].real = temp1.real;
		img12_complr_fft[i].imag = temp1.imag;
	}
	// ifft
	iFFT3_1(img12_complr_fft, img12_complr, num);
	FFT3shift_1(img12_complr, img12_complr_shift, num);
	// 存储最大值及其对应索引号
	double maxvalue = -9999;
	long maxindex = 0;
	FILE *fp2 = fopen("D:\\test.txt", "w");
	for(int i=0; i<num; i++)
	{
		temp2 = Cabs(img12_complr_shift[i]);
		fprintf(fp2, "%lf\n", temp2);
		if(temp2>maxvalue)
		{
			maxvalue = temp2;
			maxindex = i;
		}
	}
	fclose(fp2);
	fp2 = NULL;
	printf("%ld", maxindex-num/2);

	fclose(fp);
	fp = NULL;
}



