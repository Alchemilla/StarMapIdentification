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
// ���캯��
////////////////////////////////////////////////////////////
PhaseCorrelation::PhaseCorrelation(void)
{
	m_beta = -1;
	m_win = m_win2 = -1;
	// �����Ҵ���
	m_Wrc = NULL;
	// ����Ӱ��֮��Ĺ������õ��ı���
	img12_complr_fft = img12_complr = img12_complr_shift = NULL;
	img12_C = NULL;
	// ��Ӱ�������
	blockimg1 = NULL;
	img1_complr = img1_complr_fft = img1_complr_shift = NULL;
	// ��Ӱ�������
	blockimg2 = NULL;
	img2_complr = img2_complr_fft = img2_complr_shift = NULL;
	// Ƶ����Ĥ������
	img12_LS = img12_W = NULL;
}


////////////////////////////////////////////////////////////
// ��������
////////////////////////////////////////////////////////////
PhaseCorrelation::~PhaseCorrelation(void)
{
	Destroy();
}


////////////////////////////////////////////////////////////
// �����ڴ�
// ���룺
//		int size2:			���ٿռ��С
//		bool isDel��		�Ƿ�ȫ�����¿���,Ĭ��Ϊfalse
//							��Ϊtrueʱ,��ȫ��Destroy,���¿���
//							��Ϊfalseʱ�������Ե��е�NULL���п���
//		double beta:		�����ҵ�betaֵ
// ����ֵ:
//		void
////////////////////////////////////////////////////////////
void PhaseCorrelation::Create(int size2, bool isDel, double beta)
{
	if(isDel==true)
		Destroy();
	if(m_Wrc==NULL)					m_Wrc = new double[size2];
	// ����Ӱ��֮��Ĺ������õ��ı���
	if(img12_complr_fft==NULL)		img12_complr_fft = new ComplexDouble[size2];
	if(img12_complr==NULL)			img12_complr = new ComplexDouble[size2];
	if(img12_complr_shift==NULL)	img12_complr_shift = new ComplexDouble[size2];
	if(img12_C==NULL)				img12_C = new double[size2];
	// ��Ӱ�������
	if(blockimg1==NULL)				blockimg1 = new double[size2];
	if(img1_complr==NULL)			img1_complr = new ComplexDouble[size2];
	if(img1_complr_fft==NULL)		img1_complr_fft = new ComplexDouble[size2];
	if(img1_complr_shift==NULL)		img1_complr_shift = new ComplexDouble[size2];
	// ��Ӱ�������
	if(blockimg2==NULL)				blockimg2 = new double[size2];
	if(img2_complr==NULL)			img2_complr = new ComplexDouble[size2];
	if(img2_complr_fft==NULL)		img2_complr_fft = new ComplexDouble[size2];
	if(img2_complr_shift==NULL)		img2_complr_shift = new ComplexDouble[size2];
	// Ƶ����Ĥ������
	if(img12_LS==NULL)				img12_LS = new double[size2];
	if(img12_W==NULL)				img12_W = new double[size2];
	// ���������Ҳ�����
	if(isDel==true)
		CreateWRC((int)sqrt((double)size2), beta);
}

////////////////////////////////////////////////////////////
// �ͷ��ڴ�
////////////////////////////////////////////////////////////
void PhaseCorrelation::Destroy()
{
	if(m_Wrc!=NULL)					delete []m_Wrc;					m_Wrc = NULL;
	// ����Ӱ��֮��Ĺ������õ��ı���
	if(img12_complr_fft!=NULL)		delete []img12_complr_fft;		img12_complr_fft = NULL;
	if(img12_complr!=NULL)			delete []img12_complr;			img12_complr = NULL;
	if(img12_complr_shift!=NULL)	delete []img12_complr_shift;	img12_complr_shift = NULL;
	if(img12_C!=NULL)				delete []img12_C;				img12_C = NULL;
	// ��Ӱ�������
	if(blockimg1!=NULL)				delete []blockimg1;				blockimg1 = NULL;
	if(img1_complr!=NULL)			delete []img1_complr;			img1_complr = NULL;
	if(img1_complr_fft!=NULL)		delete []img1_complr_fft;		img1_complr_fft = NULL;
	if(img1_complr_shift!=NULL)		delete []img1_complr_shift;		img1_complr_shift = NULL;
	// ��Ӱ�������
	if(blockimg2!=NULL)				delete []blockimg2;				blockimg2 = NULL;
	if(img2_complr!=NULL)			delete []img2_complr;			img2_complr = NULL;
	if(img2_complr_fft!=NULL)		delete []img2_complr_fft;		img2_complr_fft = NULL;
	if(img2_complr_shift!=NULL)		delete []img2_complr_shift;		img2_complr_shift = NULL;
	// Ƶ����Ĥ������
	if(img12_LS!=NULL)				delete []img12_LS;				img12_LS = NULL;
	if(img12_W!=NULL)				delete []img12_W;				img12_W = NULL;
}


////////////////////////////////////////////////////////////
// ����������ģ��
// ���룺
//		int size:			ģ���С
//		double beta��		beta������С
// ����ֵ:
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
// ��������,A����Ϊ[m,p],B����Ϊ[p,n],CΪ[m,n] 
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
// ��ά����Ҷ���任
// ���룺
//		ComplexDouble *A������Ҷ�任ǰ����
//		int nx, ny:			����Ҷ�任����
// ���������
//		ComplexDouble *B:	����Ҷ�任�����
// ����ֵ:
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
// ��ά����Ҷ��任
// ���룺
//		ComplexDouble *A������Ҷ��任ǰ����
//		int nx, ny:			����Ҷ��任����
// ���������
//		ComplexDouble *B:	����Ҷ��任�����
// ����ֵ:
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
// ��ά����Ҷ��Ƶ
// ���룺
//		ComplexDouble *A������Ҷ��Ƶǰ����
//		int nx, ny:			����Ҷ��Ƶ����
// ���������
//		ComplexDouble *B:	����Ҷ��Ƶ�����
// ����ֵ:
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
// һά����Ҷ���任
// ���룺
//		ComplexDouble *A������Ҷ�任ǰ����
//		int nx, ny:			����Ҷ�任����
// ���������
//		ComplexDouble *B:	����Ҷ�任�����
// ����ֵ:
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
// һά����Ҷ��任
// ���룺
//		ComplexDouble *A������Ҷ��任ǰ����
//		int nx, ny:			����Ҷ��任����
// ���������
//		ComplexDouble *B:	����Ҷ��任�����
// ����ֵ:
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
// һά����Ҷ��Ƶ
// ���룺
//		ComplexDouble *A������Ҷ��Ƶǰ����
//		int nx, ny:			����Ҷ��Ƶ����
// ���������
//		ComplexDouble *B:	����Ҷ��Ƶ�����
// ����ֵ:
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
// ��ȡ������ģ
// ���룺
//		ComplexDouble A�� ����ģ�ĸ���
// ����ֵ:
//		double��			ģֵ
////////////////////////////////////////////////////////////
double PhaseCorrelation::Cabs(ComplexDouble A)
{
	return sqrt(A.real*A.real+A.imag*A.imag);
}



////////////////////////////////////////////////////////////
// �ڲ��ά��ֵ��,���ھ������
// ���룺
//		double *C:			����Ķ�ά����
//		int size:			��ά�����ά��
//		int posx, posy:		��ʼ��x��y����
// �����
//		double &x,&y��		�������x��y����		
// ����ֵ:
//		bool
////////////////////////////////////////////////////////////
bool PhaseCorrelation::InterExtreValue(double *C, int size, int posx, int posy, double &x, double &y)
{
	// �ڲ�λ��
	if(posx>0&&posx<size-1&&posy>0&&posy<size-1)
	{
		long index = posy*size+posx;
		double temp0, temp1, temp2;
		// x����ľ���
		temp0 = C[index-1] + C[index-size-1] + C[index+size-1];
		temp1 = C[index] + C[index-size] + C[index+size];
		temp2 = C[index+1] + C[index-size+1] + C[index+size+1];
		x = ((posx-1)*temp0 + posx*temp1 + (posx+1)*temp2)/(temp0 + temp1 + temp2);
		// y����ľ���
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
// ��ȡ��������ƥ����(����)
// ���룺
//		GeoReadImage *img1:	��Ӱ��ָ��
//		GeoReadImage *img2:	��Ӱ��ָ��
//		long lx,ly��		��Ӱ����������
//		int win:			ƥ�䴰�ڴ�С(Ҫ��ż��)
// ���������
//		float &rx,&ry��		��ƥ�����Ӱ��㼰���ص�׼ȷ��
// ����ֵ:
//		bool
////////////////////////////////////////////////////////////
bool PhaseCorrelation::GetPixelTest(GeoReadImage *img1, GeoReadImage *img2, long lx, long ly, float &rx, float &ry, int win)
{
	int win2 = win*win, i, j;
	int win0_5 = win/2;
	// ������ڴ�С��һ��,�����·���
	if(win!=m_win)
	{
		m_win = win;		m_win2 = win2;
		Create(win2, true);
	}
	// ���һ��,����Ҫ���·���
	else
		Create(win2);

	// Test
	WriteDouble(win, m_Wrc, "D:\\Temp\\WRC.raw");

	/////////////////////////////////////////////
	// ����Ӱ����и���Ҷ�任
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
	// ����Ӱ�񷴸����и���Ҷ�任��ȡ��������,����ȡ���λ��
	/////////////////////////////////////////////
	ComplexDouble temp;
	double temp2;
	double cmax, cmax2, dx, dy;
	int maxx, maxy;
	int count = 0;
	double offx, offy;
	// ѭ��
	do 
	{	
		// ��ȡ��Ӱ��	
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
        // ���и����,��ȡ������
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

		// ��ȡ��Ӧ����ֵ
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
		// ע��:��ʵ������Ӱ���غϵ�һ�������ڵ�ʱ��
		// ����Ҷ���任�õ��Ľ���Ǹ���Чֵ
		// ����Ҳ���˳���ѭ��,���Ǵ�ʱֵ��׼ȷ��
		if(!bTrue)		
			return true;
		// ��ά�ڲ弫ֵ��
		InterExtreValue(img12_C, win, maxx, maxy, dx, dy);
		offx = -(dx-win0_5);		offy = -(dy-win0_5);
 		rx = (long)(rx+0.5) + offx;		// ע��,��ΪӰ���Ǵ�(long)(rx+0.5)Ϊ���
		ry = (long)(ry+0.5) + offy;		// ע��,��ΪӰ���Ǵ�(long)(ry+0.5)Ϊ���
		count ++;		
	} while((fabs(offx)>1||fabs(offy)>1)&&count<20);
	// ������λ�û����,˵��ƥ��ʧ��
	if(count>19)
	{	
		return false;
 	}
	return true;
}



////////////////////////////////////////////////////////////
// ��ȡ��������ƥ����
// ���룺
//		GeoReadImage *img1:	��Ӱ��ָ��
//		GeoReadImage *img2:	��Ӱ��ָ��
//		long lx,ly��		��Ӱ����������
//		int win:			ƥ�䴰�ڴ�С(Ҫ��ż��)
//		double beta:		������betaϵ��
// ���������
//		float &rx,&ry��		��ƥ�����Ӱ��㼰���ص�׼ȷ��
// ����ֵ:
//		bool
////////////////////////////////////////////////////////////
bool PhaseCorrelation::GetPixel(GeoReadImage *img1, GeoReadImage *img2, long lx, long ly, float &rx, float &ry, int win)
{
	int win2 = win*win, i, j;
	int win0_5 = win/2;
	// ������ڴ�С��һ��,�����·���
	if(win!=m_win)
	{
		m_win = win;		m_win2 = win2;
		Create(win2, true);
	}
	// ���һ��,����Ҫ���·���
	else
		Create(win2);

	/////////////////////////////////////////////
	// ����Ӱ����и���Ҷ�任
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
	// ����Ӱ�񷴸����и���Ҷ�任��ȡ��������,����ȡ���λ��
	/////////////////////////////////////////////
	ComplexDouble temp;
	double cmax, cmax2, dx, dy;
	int maxx, maxy;
	int count = 0;
	double offx, offy;
	// ѭ��
	do 
	{	
		// ��ȡ��Ӱ��	
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
        // ���и����,��ȡ������
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

		// ��ȡ��Ӧ����ֵ
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
		// ע��:��ʵ������Ӱ���غϵ�һ�������ڵ�ʱ��
		// ����Ҷ���任�õ��Ľ���Ǹ���Чֵ
		// ����Ҳ���˳���ѭ��,���Ǵ�ʱֵ��׼ȷ��
		if(!bTrue)	
			return false;
		// ��ά�ڲ弫ֵ��
		if(!InterExtreValue(img12_C, win, maxx, maxy, dx, dy))
			return false;
		offx = -(dx-win0_5);		offy = -(dy-win0_5);
 		rx = (long)(rx+0.5) + offx;		// ע��,��ΪӰ���Ǵ�(long)(rx+0.5)Ϊ���
		ry = (long)(ry+0.5) + offy;		// ע��,��ΪӰ���Ǵ�(long)(ry+0.5)Ϊ���
		count ++;		
	} while((fabs(offx)>1||fabs(offy)>1)&&count<20);
	// ������λ�û����,˵��ƥ��ʧ��
	if(count>18)
	{	
		return false;
 	}
	return true;
}



////////////////////////////////////////////////////////////
// ��ȡ��������ƥ����
// ���룺
//		GeoReadImage *img1:	��Ӱ��ָ��
//		GeoReadImage *img2:	��Ӱ��ָ��
//		long lx,ly��		��Ӱ����������
//		int win:			ƥ�䴰�ڴ�С(Ҫ��ż��)
//		double beta:		������betaϵ��
// ���������
//		float &rx,&ry��		��ƥ�����Ӱ��㼰���ص�׼ȷ��
// ����ֵ:
//		bool
////////////////////////////////////////////////////////////
bool PhaseCorrelation::GetPixel(GeoReadImage *img1, GeoReadImage *img2, double lx, double ly, double &rx, double &ry, int win)
{
	int win2 = win*win, i, j;
	int win0_5 = win/2;
	// ������ڴ�С��һ��,�����·���
	if(win!=m_win)
	{
		m_win = win;		m_win2 = win2;
		Create(win2, true);
	}
	// ���һ��,����Ҫ���·���
	else
		Create(win2);

	/////////////////////////////////////////////
	// ����Ӱ����и���Ҷ�任
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
	// ����Ӱ�񷴸����и���Ҷ�任��ȡ��������,����ȡ���λ��
	/////////////////////////////////////////////
	ComplexDouble temp;
	double cmax, cmax2, dx, dy;
	int maxx, maxy;
	int count = 0;
	double offx, offy;
	// ѭ��
	do 
	{	
		// ��ȡ��Ӱ��	
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
        // ���и����,��ȡ������
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

		// ��ȡ��Ӧ����ֵ
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
		// ע��:��ʵ������Ӱ���غϵ�һ�������ڵ�ʱ��
		// ����Ҷ���任�õ��Ľ���Ǹ���Чֵ
		// ����Ҳ���˳���ѭ��,���Ǵ�ʱֵ��׼ȷ��
		if(!bTrue)	
			return false;
		// ��ά�ڲ弫ֵ��
		if(!InterExtreValue(img12_C, win, maxx, maxy, dx, dy))
			return false;
		offx = -(dx-win0_5);		offy = -(dy-win0_5);
 		rx = (long)(rx+0.5) + offx;		// ע��,��ΪӰ���Ǵ�(long)(rx+0.5)Ϊ���
		ry = (long)(ry+0.5) + offy;		// ע��,��ΪӰ���Ǵ�(long)(ry+0.5)Ϊ���
		count ++;		
	} while((fabs(offx)>1||fabs(offy)>1)&&count<20);
	// ������λ�û����,˵��ƥ��ʧ��
	if(count>18)
	{	
		return false;
 	}
	return true;
}



////////////////////////////////////////////////////////////
// ��ȡ��������ƥ����
// ���룺
//		GeoReadImage *img1:	��Ӱ��ָ��
//		GeoReadImage *img2:	��Ӱ��ָ��
//		long lx,ly��		��Ӱ����������
//		int win:			ƥ�䴰�ڴ�С(Ҫ��ż��)
//		double beta:		������betaϵ��
// ���������
//		float &rx,&ry��		��ƥ�����Ӱ��㼰���ص�׼ȷ��
// ����ֵ:
//		bool
////////////////////////////////////////////////////////////
bool PhaseCorrelation::GetPixelOnly(GeoReadImage *img1, GeoReadImage *img2, double lx, double ly, double &rx, double &ry, int win)
{
	int win2 = win*win, i, j;
	int win0_5 = win/2;
	// ������ڴ�С��һ��,�����·���
	if(win!=m_win)
	{
		m_win = win;		m_win2 = win2;
		Create(win2, true);
	}
	// ���һ��,����Ҫ���·���
	else
		Create(win2);

	/////////////////////////////////////////////
	// ����Ӱ����и���Ҷ�任
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
	// ����Ӱ�񷴸����и���Ҷ�任��ȡ��������,����ȡ���λ��
	/////////////////////////////////////////////
	ComplexDouble temp;
	double cmax, cmax2, dx, dy;
	int maxx, maxy;
	int count = 0;
	double offx, offy;
	// ѭ��
	do 
	{	
		// ��ȡ��Ӱ��	
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
        // ���и����,��ȡ������
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

		// ��ȡ��Ӧ����ֵ
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
		// ע��:��ʵ������Ӱ���غϵ�һ�������ڵ�ʱ��
		// ����Ҷ���任�õ��Ľ���Ǹ���Чֵ
		// ����Ҳ���˳���ѭ��,���Ǵ�ʱֵ��׼ȷ��
		if(!bTrue)	
			return false;
		// ��ά�ڲ弫ֵ��
		if(!InterExtreValue(img12_C, win, maxx, maxy, dx, dy))
			return false;
		offx = -(dx-win0_5);		offy = -(dy-win0_5);
 		rx = (long)(rx+0.5) + offx;		// ע��,��ΪӰ���Ǵ�(long)(rx+0.5)Ϊ���
		ry = (long)(ry+0.5) + offy;		// ע��,��ΪӰ���Ǵ�(long)(ry+0.5)Ϊ���
		count ++;		
	} while((fabs(offx)>1||fabs(offy)>1)&&count<20);
	// ������λ�û����,˵��ƥ��ʧ��
	if(count>18)
	{	
		return false;
 	}
	return true;
}




////////////////////////////////////////////////////////////
// ��ȡ������ƥ����
// ���룺
//		GeoReadImage *img1:	��Ӱ��ָ��
//		GeoReadImage *img2:	��Ӱ��ָ��
//		long lx,ly��		��Ӱ����������
//		double mration:		����ϵ��
//		double beta:		������betaϵ��
//		int win:			ƥ�䴰�ڴ�С(Ҫ��ż��)
// ���������
//		float &rx,&ry��		��ƥ�����Ӱ��㼰���ص�׼ȷ��
//		double &SNR:		�����
// ����ֵ:
//		bool
////////////////////////////////////////////////////////////
bool PhaseCorrelation::GetSubPixel(GeoReadImage *img1, GeoReadImage *img2, long lx, long ly, float &rx, float &ry, 
									double &SNR, double mration, double beta, int win)
{
/*	int win2 = win*win, i, j;
	int win0_5 = win/2;
	// ������ڴ�С��һ��,�����·���
	if(win!=m_win)
	{
		m_win = win;		m_win2 = win2;
		Create(win2, true, beta);
	}
	// ���һ��,����Ҫ���·���
	else
		Create(win2, false, beta);
	// ���betaֵ��һ��,�����·���
	if(m_beta!=beta)
	{
		m_beta = beta;
		CreateWRC(win, m_beta);
	}

	// Test
	WriteDouble(win, m_Wrc, "D:\\Temp\\WRC.raw");
	/////////////////////////////////////////////
	// ����Ӱ����и���Ҷ�任
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
	// ����Ӱ����и���Ҷ�任
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
	// ���и����,��ȡ������,ͬʱ��ȡƵ����Ĥ�Ķ���
	/////////////////////////////////////////////
	double uimg12_LS = 0.0;				// ��һ��������ȡ����
	double maximg12_LS = -9999;			// ������ȡ�������ֵ
	ComplexDouble temp;
	double ctemp;
	for(i=0; i<win2; i++)
	{
		temp = ComplexMulti(img1_complr_shift[i], img2_complr_shift[i]);
		ctemp = Cabs(temp);
		img12_complr_fft[i] = ComplexDiv(temp, ctemp);	// ��һ����������
		img12_LS[i] = log10(ctemp);						// ������ȡ����
		uimg12_LS += img12_LS[i]/win2;					// ��һ��������ȡ����
		if(img12_LS[i]>maximg12_LS)	
			maximg12_LS = img12_LS[i];					// ������ȡ�������ֵ
	}
	/////////////////////////////////////////////
	// ��ȡƵ����Ĥ
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
	// ������ȡ���ƫ��ֵ
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
	// ����
	do 
	{
		// ���������ٽ�������Ƚ�׼ȷ��ƫ��ֵ
		if(!TwoPointStepSizeGradient(dx, dy, img12_W0, img12_complr_fft, win))
		{
			return false;
		}
		// ��ȡ�����,������Qֵ��Wֵ
		phisum = Wsum = 0.0;
        wx = wy = PI2/win;
		for(i=-win0_5;i<win0_5;i++)
		{
			for(j=-win0_5;j<win0_5;j++)
			{
				pos = (i+win0_5)*win+(j+win0_5);
				wxy = wx*(i)*dx+wy*(j)*dy;
				// ��һ��������������ֵ
				img12_C0[pos].real = cos(wxy);		img12_C0[pos].imag = sin(wxy);	
				// ��һ�������״�Ȩ�в����
				img12_phi[pos]= img12_W0[pos]*pow(Cabs(ComplexSub(img12_complr_fft[pos],img12_C0[pos])), 2);
				// ���¼����µ�W��
				img12_W1[pos] = img12_W0[pos]*pow(1-img12_phi[pos]/4, 6);
				// ���¼����µ�Q��
				Qtemp.real = cos(-wxy);				Qtemp.imag = sin(-wxy);
				img12_Q1[pos] = ComplexMulti(img12_complr_fft[pos], Qtemp);
				// �ۼ����ڼ���SNR
				phisum += img12_phi[pos];  
				Wsum += img12_W0[pos];
			}
		}
		// ���������
		SNR = 1 - phisum/Wsum/4.0;
		// ����ƫ��ֵ
		rx += dx;		ry += dy;
		// ���̫�����Ϊ��Ч
		if(fabs(dx)>1.5||fabs(dy)>1.5)
		{
			SNR = 0.0;				
			return false;
		}
		// ����W���Q��
		memcpy(img12_W0, img12_W1, sizeof(double)*win2);
		memcpy(img12_complr_fft, img12_Q1, sizeof(ComplexDouble)*win2);
		count++;
	} while(count<4);*/
	return true;
}



////////////////////////////////////////////////////////////
// ��ȡ������ƥ����(����)
// ���룺
//		GeoReadImage *img1:	��Ӱ��ָ��
//		GeoReadImage *img2:	��Ӱ��ָ��
//		long lx,ly��		��Ӱ����������
//		double mration:		����ϵ��
//		double beta:		������betaϵ��
//		int win:			ƥ�䴰�ڴ�С(Ҫ��ż��)
// ���������
//		float &rx,&ry��		��ƥ�����Ӱ��㼰���ص�׼ȷ��
//		double &SNR:		�����
// ����ֵ:
//		bool
////////////////////////////////////////////////////////////
bool PhaseCorrelation::GetSubPixelTest(GeoReadImage *img1, GeoReadImage *img2, long lx, long ly, float &rx, float &ry, 
									double &SNR, double mration, double beta, int win)
{
	int win2 = win*win, i, j;
	int win0_5 = win/2;
	// ������ڴ�С��һ��,�����·���
	if(win!=m_win)
	{
		m_win = win;		m_win2 = win2;
		Create(win2, true, beta);
	}
	// ���һ��,����Ҫ���·���
	else
		Create(win2, false, beta);
	// ���betaֵ��һ��,�����·���
	if(m_beta!=beta)
	{
		m_beta = beta;
		CreateWRC(win, m_beta);
	}

	// Test
	WriteDouble(win, m_Wrc, "D:\\Temp\\WRC.raw");
	/////////////////////////////////////////////
	// ����Ӱ����и���Ҷ�任
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
	// ����Ӱ����и���Ҷ�任
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
	// ���и����,��ȡ������,ͬʱ��ȡƵ����Ĥ�Ķ���
	/////////////////////////////////////////////
	double uimg12_LS = 0.0;				// ��һ��������ȡ����
	double maximg12_LS = -9999;			// ������ȡ�������ֵ
	ComplexDouble temp;
	double ctemp;
	for(i=0; i<win2; i++)
	{
		temp = ComplexMulti(img1_complr_shift[i], img2_complr_shift[i]);
		ctemp = Cabs(temp);
		img12_complr_fft[i] = ComplexDiv(temp, ctemp);	// ��һ����������
		img12_LS[i] = log10(ctemp);						// ������ȡ����
		uimg12_LS += img12_LS[i]/win2;					// ��һ��������ȡ����
		if(img12_LS[i]>maximg12_LS)	
			maximg12_LS = img12_LS[i];					// ������ȡ�������ֵ
	}
	/////////////////////////////////////////////
	// ��ȡƵ����Ĥ
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
	// ������ȡ���ƫ��ֵ
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
	// ����
	do 
	{
		dx = dy = 0;
		// ���������ٽ�������Ƚ�׼ȷ��ƫ��ֵ
		if(!TwoPointStepSizeGradient(dx, dy, img12_W, img12_complr_fft, win))
		{
			return false;
		}
		// ����TPSS��ȡ��ƫ��ֵ,����Qֵ��Wֵ,����ȡ�����
		phisum = Wsum = 0.0;
        wx = wy = PI2/win;
		for(i=-win0_5;i<win0_5;i++)
		{
			for(j=-win0_5;j<win0_5;j++)
			{
				////////////////////////////////////
				// �����
				///////////////////////////////////
				pos = (i+win0_5)*win+(j+win0_5);
				wxy = wx*(i)*dx + wy*(j)*dy;
				// ��һ��������������ֵ
				img12_C.real = cos(wxy);		img12_C.imag = sin(wxy);	
				// ��һ�������״�Ȩ�в����
				img12_phi = img12_W[pos]*pow(Cabs(ComplexSub(img12_complr_fft[pos], img12_C)), 2);
				// �ۼ����ڼ���SNR
				phisum += img12_phi;  
				Wsum += img12_W[pos];
				////////////////////////////////////
				// ����W��Q��
				///////////////////////////////////
				// ���¼����µ�W��
				img12_Wtemp = img12_W[pos]*pow(1-img12_phi/4, 6);
				img12_W[pos] = img12_Wtemp;
				// ���¼����µ�Q��
				Qtemp.real = img12_C.real;		Qtemp.imag = -img12_C.imag;
				img12_Qtemp = ComplexMulti(img12_complr_fft[pos], Qtemp);
				img12_complr_fft[pos] = img12_Qtemp;
			}
		}
		// ���������
		SNR = 1 - phisum/Wsum/4.0;
		// ����ƫ��ֵ
		rx += dx;		ry += dy;
		sumdx += dx;	sumdy += dy;
		// ���̫�����Ϊ��Ч
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
// Two-Point Step Size Gradient Methods�����½���
// ���룺
//		double *W:			Ƶ����Ĥ
//		ComplexDouble *Q��	��һ����������
//		int size:			���ڴ�С
// ���������
//		double &x,&y:		ƫ��ֵ
// ����ֵ:
//		bool��
////////////////////////////////////////////////////////////
bool PhaseCorrelation::TwoPointStepSizeGradient(double &x, double &y, double *W, ComplexDouble *Q, int size)
{
	long size2 = size*size;
	// ��ȡ��ʼƫ��ֵ
	double x0, y0, g0[2], g[2];
	x0 = x - 0.1;
	y0 = y - 0.1;
	// ��ȡ��ʼ�ݶ�ֵ
	double sumw = 0.0;
	for(int i=0; i<size2; i++)
		sumw += W[i];
	g0[0] = g0[1] = sumw;
//	GetGradient(x0, y0, W, Q, size, g0);
	// ��ʼ���е���
	double dx, dy, dg[2];
	double a, dgtemp;
	int count = 0;
	do
	{
		// ��õ�ǰ���ݶ�
		GetGradient(x, y, W, Q, size, g);
		// ƫ��ֵ֮��,��ʼֵ֮��Ϊ0.1
		dx = x - x0;
		dy = y - y0;
		if(fabs(dx)>1.5||fabs(dy)>1.5)
		{
			return false;
		}
		// �ݶ�ֵ֮��
		dg[0] = g[0] - g0[0];
		dg[1] = g[1] - g0[1];
		dgtemp = dx*dg[0] + dy*dg[1];
		// �������ϵ��
		a = (dx*dx + dy*dy)/dgtemp;
		x0 = x;			y0 = y;
		memcpy(g0, g, sizeof(double)*2);
		x = x0 - a*g[0];
		y = y0 - a*g[1];
		count++;
	} while((fabs(dx)>0.001||fabs(dy)>0.001)&&count<=100);
	// �����������̫��,����ΪѰ��ʧ��,�������ǳɹ�
	if(count>=100)
		return false;
	return true;
}


////////////////////////////////////////////////////////////
// ��ȡ�ݶ�(ͨ��ƫ��),�ֱ���ˮƽ�ʹ�ֱ���ݶ�,�����ۼ�ֵ
// ���룺
//		double x,y:			ƫ��ֵ
//		double *W:			Ƶ����Ĥ
//		complex<double> *Q����һ����������
//		int size:			���ڴ�С
// �����
//		double *g:			����������µ��ݶ�ֵ
// ����ֵ:
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
			// ��ȡƫ����
			t = 2*W[pos]*(Q[pos].real*sin(ctemp) - Q[pos].imag*cos(ctemp));
			g[0] += t*cwx;
			g[1] += t*cwy;
		}
	}
}


////////////////////////////////////////////////////////////
// �������
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
// �������
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
// �����˷�(�������)
////////////////////////////////////////////////////////////
ComplexDouble PhaseCorrelation::ComplexMulti(ComplexDouble A, ComplexDouble B)
{
	double a, b, c, d;
	ComplexDouble temp;
	a = A.real;		b = A.imag;		c = B.real;		d = -B.imag;	// �ǵüӸ���
	temp.real = a*c-b*d;
	temp.imag = b*c+a*d;
	return temp;
}


////////////////////////////////////////////////////////////
// ��������
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
// �����ӷ�
////////////////////////////////////////////////////////////
ComplexDouble PhaseCorrelation::ComplexAdd(ComplexDouble A, ComplexDouble B)
{
	ComplexDouble temp;
	temp.real = A.real + B.real;
	temp.imag = A.imag + B.imag;
	return temp;
}


////////////////////////////////////////////////////////////
// ��������
////////////////////////////////////////////////////////////
ComplexDouble PhaseCorrelation::ComplexSub(ComplexDouble A, ComplexDouble B)
{
	ComplexDouble temp;
	temp.real = A.real - B.real;
	temp.imag = A.imag - B.imag;
	return temp;
}


////////////////////////////////////////////////////////////
// һάƵ��һά�ռ�ƥ��
// ���룺
//		string lpath:		��Ӱ��·��
//		string rpath:		��Ӱ��·��
//		long delnum:		����������ص���Ԫ(��������ֵ�õ�)
//		long startdelnum:	��������С�ص���Ԫ(Ĭ��Ϊ1)
// �����
//		double &line:		�ع���ƫ��(����Ϊ��Ӱ�������Ӱ������ƽ��,����Ϊ��Ӱ�������Ӱ������ƽ��)
//		double &sample:		������ƫ�ƣ����ص���������
// ����ֵ:
//		void
////////////////////////////////////////////////////////////
bool PhaseCorrelation::GetOnePixel(string lpath, string rpath, double &line, double &sample, long delnum, long startdelnum)
{
	//////////////////////////////////////////////
	// ��Ӱ�񲢶�ȡ��Ӧ����
	//////////////////////////////////////////////
	GeoReadImage img1, img2;
	img1.Open(lpath, GA_ReadOnly);
	img2.Open(rpath, GA_ReadOnly);
	long linenum = img1.m_yRasterSize<img2.m_yRasterSize?img1.m_yRasterSize:img2.m_yRasterSize;
	long left0 = img1.m_xRasterSize-1-delnum;
	// ��ȡ��Ӱ������,���Ͻ�Ϊ(left0, 0),����ֱ�Ϊ(delnum, linenum)
	img1.ReadBlock(left0, 0, delnum, linenum, 0, img1.pBuffer[0]);
	double *data1 = new double[delnum*linenum];
	if(!img1.GetDataValueFromBuffer(img1.pBuffer[0], 0, 0, delnum, linenum, data1))
		return false;
	// ��ȡ��Ӱ������,���Ͻ�Ϊ(0,0),����ֱ�Ϊ(delnum, linenum)
	img2.ReadBlock(0, 0, delnum, linenum, 0, img2.pBuffer[0]);
	double *data2 = new double[delnum*linenum];
	if(!img2.GetDataValueFromBuffer(img2.pBuffer[0], 0, 0, delnum, linenum, data2))
		return false;
	
	//////////////////////////////////////////////
	// һ����ƥ��
	//////////////////////////////////////////////
	// ����ռ�
	Create(linenum, true);
	// ����ƥ������
	ComplexDouble temp, temp1;
	double temp2;
	double *maxvalue = new double[delnum];
	long *maxindex = new long[delnum];
	memset(maxvalue, 0, sizeof(double)*delnum);
	memset(maxindex, 0, sizeof(long)*delnum);

	long ktemp;
	for(int k=startdelnum; k<delnum; k++)
	{	
		// ����
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
		// ż��
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
		// ���任
		FFT3_1(img1_complr, img1_complr_fft, linenum);
		FFT3shift_1(img1_complr_fft, img1_complr_shift, linenum);
		FFT3_1(img2_complr, img2_complr_fft, linenum);
		FFT3shift_1(img2_complr_fft, img2_complr_shift, linenum);
		// ���и����,��ȡ������
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
		// �洢���ֵ�����Ӧ������
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
	

	// �ͷ��ڴ��Ӱ��
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
	// ����ռ�
	Create(num, true);
	for(int i=0; i<num; i++)
	{	
		fscanf(fp, "%lf%lf", &hwc1, &hwc2);		
		img1_complr[i].real = hwc1;	
		img1_complr[i].imag = 0.0;
		img2_complr[i].real = hwc2;		
		img2_complr[i].imag = 0.0;	
	}
	// ���任
	FFT3_1(img1_complr, img1_complr_fft, num);
	FFT3shift_1(img1_complr_fft, img1_complr_shift, num);
	FFT3_1(img2_complr, img2_complr_fft, num);
	FFT3shift_1(img2_complr_fft, img2_complr_shift, num);
	ComplexDouble temp, temp1;
	double temp2;
	// ���и����,��ȡ������
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
	// �洢���ֵ�����Ӧ������
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



