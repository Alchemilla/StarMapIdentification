
#include "LSMatching.h"
#include <math.h>

//////////////////////////////////////////////////////////
// 构造/析构函数
//////////////////////////////////////////////////////////
CLSMatching::CLSMatching()
{
	m_imgL = m_imgR = NULL;
	m_LH = m_LW = m_RH = m_RW = 0;
	m_winH = m_winW = 0;
	m_winH2 = m_winW2 = 0;
}

CLSMatching::~CLSMatching()
{
	m_imgL = m_imgR = NULL;
}


//////////////////////////////////////////////////////////
// 初始化单点最小二乘影像匹配的初值
// gL(xL,yL) = h0 + h1gR(a0+a1*xL+a2*yL, b0+b1*xL+b2*yL)
//////////////////////////////////////////////////////////
void CLSMatching::Init(GeoReadImage *imgL, GeoReadImage *imgR, int winH, int winW)
{
	//左影像信息
	m_imgL  = imgL;
	m_LH = m_imgL->m_yRasterSize;
	m_LW = m_imgL->m_xRasterSize;
	//右影像信息
	m_imgR  = imgR;
	m_RH = m_imgR->m_yRasterSize;
	m_RW = m_imgR->m_xRasterSize;
    //匹配窗口大小
	m_winH = winH;		m_winH2 = m_winH/2;
	m_winW = winW;		m_winW2 = m_winW/2;
}

bool CLSMatching::CalcCorrcoef(double *pData1,double *pData2,int numPixels,double& Corrcoef)
{
	double ave1 = 0.,ave2 = 0.;
	int i;
	for (i = 0;i<numPixels;i++)
	{
		ave1 += pData1[i]/numPixels;
		ave2 += pData2[i]/numPixels;
	}
	double sumNUM =0., sumDEN1 =0.,sumDEN2 = 0.;
	for (i = 0;i<numPixels;i++)
	{
		double sub_Data1 = pData1[i] - ave1;
		double sub_Data2 = pData2[i] - ave2;

		sumNUM += sub_Data1*sub_Data2;//(pData1[i]-ave1)*(pData2[i]-ave2);
		sumDEN1 += sub_Data1*sub_Data1;//(pData1[i]-ave1)*(pData1[i]-ave1);
		sumDEN2 += sub_Data2*sub_Data2;//(pData2[i]-ave2)*(pData2[i]-ave2);
	}
	if(fabs(sumDEN1)<1.e-10 || fabs(sumDEN2)<1.e-10)
		return false;
	Corrcoef = sumNUM/(sqrt(sumDEN1*sumDEN2));
	return true;
}


bool CLSMatching::MatchCorrcoef(double xl,double yl,double &xr,double& yr,int CcWindow_h, int CcWindow_v,int MaxWindow_h,int MaxWindow_v,double fnr)
{
	int CcCalcPixels = CcWindow_h*CcWindow_v;
	double *pData1 = new double[CcCalcPixels];
	if(m_imgL->ReadBlock(int(xl)-CcWindow_h/2,int(yl)-CcWindow_v/2,CcWindow_h,CcWindow_v,0, m_imgL->pBuffer[0]) == false)
	{
		delete []pData1,pData1 = NULL;
		return false;
	}
	m_imgL->GetDataValueFromBuffer(m_imgL->pBuffer[0], 0, 0, CcWindow_h, CcWindow_v, pData1);
	double *pData2 = new double[CcCalcPixels];
	double MaxCorrCeof = -99999.,CorrCeof;
	double maxI=-1,maxJ=-1;
	for (int i = 0;i<MaxWindow_v;i++)   //行
	{
		double y0 = yr - MaxWindow_v/2+i;
		for (int j = 0;j<MaxWindow_h;j++)   //列
		{
			CorrCeof = -9999.;
			double x0 = xr - MaxWindow_h/2+j;
			if(m_imgR->ReadBlock(int(x0)-CcWindow_h/2,int(y0)-CcWindow_v/2,CcWindow_h,CcWindow_v,0, m_imgL->pBuffer[0]) == false)
				continue;
			m_imgR->GetDataValueFromBuffer(m_imgR->pBuffer[0], 0, 0, CcWindow_h, CcWindow_v, pData2);
			if(CalcCorrcoef(pData1,pData2,CcCalcPixels,CorrCeof) == false)
				continue;
			if(CorrCeof>MaxCorrCeof)
			{
				maxI = i;
				maxJ = j;
				MaxCorrCeof =CorrCeof;
			}
		}
	}
	if(fabs(maxI+1)<1.e-10 || fabs(maxJ+1)<1.e-10 || MaxCorrCeof<fnr)
	{
		delete []pData1,pData1 = NULL;
		delete []pData2,pData2 = NULL;
		return false;
	}
	xr = long(xr) - MaxWindow_h/2+maxJ;
	yr = long(yr) - MaxWindow_v/2+maxI;
	delete []pData1,pData1 = NULL;
	delete []pData2,pData2 = NULL;
	return true;
}


//////////////////////////////////////////////////////////
// 进行单点最小二乘影像匹配
//////////////////////////////////////////////////////////
bool CLSMatching::LSMatch(double &xL, double &yL, double &xR, double &yR, double threhold, bool isModify)
{
	// 越界判断
	if(xL<m_winW2||xL>=m_LW-m_winW2||yL<m_winH2||yL>=m_LH-m_winH2)
		return false;
	// 区域灰度
	double *pLImg = new double[m_winH*m_winW];
	double *pRImg = new double[m_winH*m_winW];
	//单点最小二乘参数初始化
	double m_t0[8], m_t[8], t[8];
	// h0, h1, a0, a1, a2, b0, b1, b2
	m_t[0] = 0;	m_t[1] = 1.0;
	m_t[2] = 0;	m_t[3] = 1.0;	m_t[4] = 0;
	m_t[5] = 0;	m_t[6] = 0;		m_t[7] = 1.0;
	// 中间用到的参数
	double coef0 = 0.0, coef = 0.0;
	double lt_x, lt_y, x, y;
    //读取左影像块
	lt_x = (int)xL - m_winW2;
	lt_y = (int)yL - m_winH2;
	if(!m_imgL->GetDataValueFromBuffer(m_imgL->pBuffer[0], lt_x, lt_y, m_winW, m_winH, pLImg))
	{
		if(pLImg)	delete []pLImg;		pLImg = NULL;
		if(pRImg)	delete []pRImg;		pRImg = NULL;
		return false;
	}
	int time=0;
	double gray;
	do
	{
		//保存上次结果
		coef0 = coef;
		memcpy(m_t0, m_t, sizeof(double)*8);	
		//读取右影像块
	    double x,y;    
    	for(int j=-m_winH2; j<=m_winH2; j++)
		{
			for(int i=-m_winW2; i<=m_winW2; i++)
			{
				// 几何纠正
				x = xR + m_t[2] + m_t[3]*i + m_t[4]*j;
				y = yR + m_t[5] + m_t[6]*i + m_t[7]*j;
				gray = m_imgR->GetDataValue(x, y, -9999, 0, true);
				if(gray == -9999)
				{
					if(pLImg)	delete []pLImg;		pLImg = NULL;
					if(pRImg)	delete []pRImg;		pRImg = NULL;
					return false;
				}
				// 辐射纠正
				pRImg[(j+m_winH2)*m_winW+i+m_winW2] = m_t[0]+m_t[1]*gray;
			}
		}
		// 计算相关系数
		coef = CorrCoef(pLImg, m_winW, pRImg, m_winW, m_winW, m_winH);
		// 如果一开始相关系数就太离谱,也退出
		if(coef<0.3)
		{
			if(pLImg)	delete []pLImg;		pLImg = NULL;
			if(pRImg)	delete []pRImg;		pRImg = NULL;
			return false;
		}
		// 根据相关系数确定是否继续迭代
		// 如果当前相关系数下降,则采用之前的
		if(((coef<coef0)&&(coef0>threhold)))
		{
			break;
		}
		// 去除辐射校正
		for(int j=-m_winH2; j<=m_winH2; j++)
		{
			for(int i=-m_winW2; i<=m_winW2; i++)
			{
				pRImg[(j+m_winH2)*m_winW+i+m_winW2] = 
					(pRImg[(j+m_winH2)*m_winW+i+m_winW2]-m_t[0])/m_t[1];
			}
		}
		//组建误差方程
		if(!ErrorEquation(xL, yL, xR, yR, pLImg, pRImg, m_t, t))
		{
			if(pLImg)	delete []pLImg;		pLImg = NULL;
			if(pRImg)	delete []pRImg;		pRImg = NULL;
			return false;
		}
		//参数更新
		m_t[0] = m_t0[0]+t[0]+m_t0[0]*t[1];					// h0
		m_t[1] = m_t0[1]+     m_t0[1]*t[1];					// h1
		m_t[2] = m_t0[2]+t[2]+m_t0[2]*t[3]+m_t0[5]*t[4];	// a0
		m_t[3] = m_t0[3]+     m_t0[3]*t[3]+m_t0[6]*t[4];	// a1
		m_t[4] = m_t0[4]+     m_t0[4]*t[3]+m_t0[7]*t[4];	// a2
		m_t[5] = m_t0[5]+t[5]+m_t0[2]*t[6]+m_t0[5]*t[7];	// b0
		m_t[6] = m_t0[6]+     m_t0[3]*t[6]+m_t0[6]*t[7];	// b1
		m_t[7] = m_t0[7]+     m_t0[4]*t[6]+m_t0[7]*t[7];	// b2
		// 根据相关系数确定是否继续迭代
		if((time++)>20) 
		{
			if(pLImg)	delete []pLImg;		pLImg = NULL;
			if(pRImg)	delete []pRImg;		pRImg = NULL;
			return false;
		}
	}while(1);
	// 计算最佳匹配的点位
	if(isModify==true)
	{
		double bestx, besty;
		GetBestPos(pLImg, bestx, besty);
		xL += bestx;		yL += besty;
		// 计算对应的右影像坐标 
		xR = xR + m_t[2] + m_t[3]*bestx + m_t[4]*besty;
		yR = yR + m_t[5] + m_t[6]*bestx + m_t[7]*besty;
	}
	else
	{
		xR = xR + m_t[2];
		yR = yR + m_t[5];
	}
	// 释放内存
	if(pLImg)	delete []pLImg;		pLImg = NULL;
	if(pRImg)	delete []pRImg;		pRImg = NULL;
	return true;
}


//////////////////////////////////////////////////////////
// 组建误差方程以及法方程
//////////////////////////////////////////////////////////
bool CLSMatching::ErrorEquation(double xL, double yL, double xR, double yR, 
						double *pLImg, double *pRImg, double *m_t, double *t)
{
	int i,j;
	double A[8], L, dgx, dgy;
	double m_AA[64], m_AL[8];
	memset(m_AA, 0, sizeof(double)*64);
	memset(m_AL, 0, sizeof(double)*8);	
    long x,y;
	for(i=-m_winW2+1; i<=m_winW2-1; i++)
	{
		for(j=-m_winH2+1; j<=m_winH2-1; j++)
		{   
			x = i + m_winW2;
			y = j + m_winH2;
			dgx = (pRImg[y*m_winW+x+1]-pRImg[y*m_winW+x-1])/2;
			dgy = (pRImg[(y+1)*m_winW+x]-pRImg[(y-1)*m_winW+x])/2;
			A[0] = 1.0;				A[1] = pRImg[y*m_winW+x];
			A[2] = m_t[1]*dgx;		A[3] = (i)*A[2];		A[4] = (j)*A[2];
			A[5] = m_t[1]*dgy;		A[6] = (i)*A[5];		A[7] = (j)*A[5];
			L = m_t[0] + m_t[1]*A[1] - pLImg[y*m_winW+x];
			pNormal(A, 8, -L, m_AA, m_AL, 1.0);
		}
	}
	Gauss(m_AA, m_AL, 8);
	memcpy(t, m_AL, sizeof(double)*8);
	return true;
}


//////////////////////////////////////////////////////////
// 获取最佳位置
//////////////////////////////////////////////////////////
void CLSMatching::GetBestPos(double *pLImg, double &bestx, double &besty)
{
	long x, y;
	double dgx2, dgy2, gxsum, gysum;
	gxsum = gysum = bestx = besty = 0;
	for(int i=-m_winW2+1; i<=m_winW2-1; i++)
	{
		for(int j=-m_winH2+1; j<=m_winH2-1; j++)
		{   
			x = i + m_winW2;
			y = j + m_winH2;
			dgx2 = pow((pLImg[y*m_winW+x+1]-pLImg[y*m_winW+x-1])/2, 2);
			dgy2 = pow((pLImg[(y+1)*m_winW+x]-pLImg[(y-1)*m_winW+x])/2, 2);
			gxsum += dgx2;
			gysum += dgy2;
			bestx += i*dgx2;
			besty += j*dgy2;
		}
	}
	bestx /= gxsum;
	besty /= gysum;
}


////////////////////////////////////////////////////////// 
// 计算相关系数
// 输入：
//		double *imgL:	左影像灰度值
//		int imgLWid：	左影像宽度
//		double *imgR：	右影像灰度值
//		int imgRWid：	右影像宽度
//		int wid, hei:	相关窗口长度和宽度
//////////////////////////////////////////////////////////
double CLSMatching::CorrCoef(double *imgL, int imgLWid, double *imgR, int imgRWid, int wid, int hei)
{
	int i, j, dwL, dwR;
	double Sxx, Sxy, Syy, Sx, Sy, mn1 = 1.0/(hei*wid);
	double vx, vy;
	Sx = Sy = Sxx = Sxy = Syy = 0;
	dwL = imgLWid - wid;
	dwR = imgRWid - wid;
	for(i=0; i<hei; i++) 
	{
		for(j=0; j<wid; j++) 
		{
			Sx += (*imgL);	
			Sy += (*imgR);
			Sxx += (*imgL)*(*imgL);
			Sxy += (*imgL)*(*imgR);
			Syy += (*imgR)*(*imgR);
			imgL++;		imgR++;
		}
		imgL += dwL;	imgR += dwR;		
	}
	vx = Sxx-mn1*Sx*Sx;
	vy = Syy-mn1*Sy*Sy;	
	double coef;
	if(vx<1.0 || vy<1.0)
		coef = 0.01;
	else 
		coef = (Sxy-mn1*Sx*Sy) /sqrt(vx*vy);
	return (coef);
}


//////////////////////////////////////////////////////////////////////
// 各类矩阵运算
//////////////////////////////////////////////////////////////////////
int CLSMatching::Gauss(double *A,double *b,int n)
{
	int i,j,lm;
	double maxx;
	for (i=0; i<n-1; i++)
	{
		lm = findlm(A,i,n,&maxx);
		if ( maxx <0.001 )	return(0);
		if( lm )
			exlij(A,b,i,lm,n);	
		eliminate(A,b,i,n);
		A += n+1;	b++;
	}
	*b /= *A;
	for(i=1; i<n; i++)
	{
		A -= n+1;	b--;
		for (j=1; j<i+1; j++)
			*b -= A[j] * b[j];
		*b /= *A;
	}
	return(1);
}

void CLSMatching::pNormal(double *a,int n,double b,double *aa, double *ab,double p)
{
	int  i,j;
	for (i=0; i<n; i++) 
	{
		for (j=0; j<n; j++) 
			*aa++ += p* a[i] * a[j];
		*ab++ += p* a[i] * b;
	}
}

int CLSMatching::findlm(double *A,int ic,int n,double *maxx)
{
	int i,im=ic;	
	*maxx = fabs(*A);
	if( *maxx > 1 ) return 0;
	for ( i=ic+1; i<n; i++)
	{
		A += n;
		if( fabs(*A) > *maxx ) 
		{
			*maxx = fabs(*A);
			im=i;
		}
	}
	if (im == ic )	return(0);
	else		return(im);
}

void CLSMatching::exlij(double *A,double *b,int li,int lj,int n)
{
	int i,j;
	double t,*l;
	i = lj-li;
	l = A + i * n;
	for ( j = li; j<n; j++)
		t = *A,	*A++ = *l, *l++ = t;
	t = *b,	*b = b[i];	b[i] = t;
}

void CLSMatching::eliminate(double *A,double *b,int ii,int n)
{
	int i,j;
	double fac,b0,*line,*row;	
	line = A + n;
	b0 = *b++;
	for( i=ii+1; i<n; i++)
	{
		row = A+1;
		fac = *line++ / *A;
		for( j = ii + 1; j<n; j++)
			*line++ -= *row++ * fac;
		line += ii;
		*b++ -= b0 * fac;
	}
}


