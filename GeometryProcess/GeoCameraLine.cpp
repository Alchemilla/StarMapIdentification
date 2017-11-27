#include <io.h>
#include "GeoCameraLine.h"

//////////////////////////////////////////////////////////////////////////
// 构造/析构函数
//////////////////////////////////////////////////////////////////////////
GeoCameraLine::GeoCameraLine(void)
{
	m_X = m_Y = NULL;
	mark = 1;
}

GeoCameraLine::~GeoCameraLine(void)
{
	ClearData();
}


// 清空数据
void GeoCameraLine::ClearData()
{
	if(m_X!=NULL)	delete []m_X;		m_X = NULL;
	if(m_Y!=NULL)	delete []m_Y;		m_Y = NULL;
}


//////////////////////////////////////
// 功能：读取内方位元素文件
// 输入:
//		string filepath:			内方位元素文件路径
//									此项仅仅存储CCD在焦面上的排列
//									如果为"",则函数从input产生标准排列
//									input里面含有相机的偏心角和偏心距
//		StrCamParamInput input：	内方位元素配置输入参数
// 返回值：
//		void
//////////////////////////////////////
void GeoCameraLine::ReadCamFile(string filepath, StrCamParamInput input)
{
	m_Input = input;
	// 如果未提供相机文件,则直接产生标准CCD排列
	if(_access(filepath.c_str(), 0)==-1)
	{
		InitInnerFile(filepath, input);
	}
	// 如果提供了相机文件,则直接读取相机文件
	else
	{
		ClearData();
		try
		{
			FILE *fp = fopen(filepath.c_str(), "r");
			if(fp==NULL)
			{
				printf("GeoCameraLine::ReadCamFile Error 1!\n");	
				return;
			}
			fscanf(fp,"%d", &m_Input.Ynum);
			m_X = new double[m_Input.Ynum];
			m_Y = new double[m_Input.Ynum];
			double temp;
			for(long i = 0; i<m_Input.Ynum; i++)
			{
				fscanf(fp, "%lf%lf%lf", &temp, &(m_Y[i]), &(m_X[i]));
				//m_X[i] = -m_X[i];//注意X和实际相反的情况
			}
			fclose(fp);
			fp=NULL;
		}
		catch(...)
		{
			printf("GeoCameraLine::ReadCamFile 2!\n");	
			return;
		}
	}
	if(m_Input.Ynum>0)
	{
		if(m_Y[0]<m_Y[m_Input.Ynum-1])	mark = 1;
		if(m_Y[0]>m_Y[m_Input.Ynum-1])	mark = -1;
	}
}


//////////////////////////////////////
// 功能：读取内方位元素
// 输入:
// 返回值：
//		void
//////////////////////////////////////
void GeoCameraLine::ReadCamNoFile(int num, double *phi0, double *phi1, StrCamParamInput input)
{
	m_Input = input;
	ClearData();
	m_Input.Ynum = num;
	m_X = new double[m_Input.Ynum];
	m_Y = new double[m_Input.Ynum];
	memcpy(m_X, phi0, sizeof(double)*num);
	memcpy(m_Y, phi1, sizeof(double)*num);
	if(m_Input.Ynum>0)
	{
		if(m_Y[0]<m_Y[m_Input.Ynum-1])	mark = 1;
		if(m_Y[0]>m_Y[m_Input.Ynum-1])	mark = -1;
	}
}


//////////////////////////////////////
// 功能：输出内方位元素文件
// 输入:
//		string filepath:			内方位元素文件路径
// 返回值：
//		void
//////////////////////////////////////
void GeoCameraLine::WriteCamFile(string filepath)
{
	try
	{
		FILE *fp = fopen(filepath.c_str(), "w");
		if(fp==NULL)
		{
			printf("GeoCameraLine::WriteCamFile Error 1!\n");	
			return;
		}
		fprintf(fp,"%d\n", m_Input.Ynum);
		for(long i = 0; i<m_Input.Ynum; i++)
		{
			fprintf(fp, "%.8ld\t%20.16lf\t%20.16lf\n", i, m_Y[i], m_X[i]);
		}
		fclose(fp);
		fp=NULL;
	}
	catch(...)
	{
		printf("GeoCameraLine::WriteCamFile Error 2!\n");	
		return;
	}
}


//////////////////////////////////////
// 功能：根据CCD排列获取初始化的内方位元素文件
// 输入:
//		string outpath:				内方位元素输出文件,如果为"",则不输出
//		StrCamParamInput input：	内方位元素配置输入参数
// 返回值：
//		void
//////////////////////////////////////
void GeoCameraLine::InitInnerFile(string outpath, StrCamParamInput input)
{
	m_Input = input;
	ClearData();
	m_X = new double[input.Ynum];
	m_Y = new double[input.Ynum];
	
	double Xtemp1, Ytemp1, Xtemp, Ytemp;
	double sitasin = sin(input.sita);
	double sitacos = cos(input.sita);
	for(long i=0; i<input.Ynum; i++)
	{
		Xtemp1 = input.Xstart*input.Xsize;
		Ytemp1 = -(input.Ystart+i)*input.Ysize;
		Xtemp = Xtemp1*sitacos+Ytemp1*sitasin;
		Ytemp = Xtemp1*sitasin+Ytemp1*sitacos;
		m_X[i] = Xtemp/input.f;
		m_Y[i] = Ytemp/input.f;
	}
	if(strcmp(outpath.c_str(),""))
	{
		WriteCamFile(outpath);
	}
	if(m_Input.Ynum>0)
	{
		if(m_Y[0]<m_Y[m_Input.Ynum-1])	mark = 1;
		if(m_Y[0]>m_Y[m_Input.Ynum-1])	mark = -1;
	}
}


//////////////////////////////////////
// 功能：获取内方位元素(探元指向角形式)
// 输入:
//		string outpath:				内方位元素输出文件
//		StrCamParamInput input：	内方位元素配置输入参数
// 返回值：
//		void
//////////////////////////////////////
void GeoCameraLine::GetInner(double x, double y, double &phiX, double &phiY)
{
	// 边界判断
	int indexY = (int)(y);
	double delY = y - (int)y;
	if(indexY>m_Input.Ynum-2) 
	{
		indexY = m_Input.Ynum-2;
		delY = y - indexY;
	}
	if(indexY<0)
	{
		indexY = 0;
		delY = y - indexY;
	}
	// 进行内插
	phiX = (1-delY)*m_X[indexY] + delY*m_X[indexY+1];
	phiY = (1-delY)*m_Y[indexY] + delY*m_Y[indexY+1];
}


//////////////////////////////////////
// 功能：根据探元指向角的值取反求索引号
// 输入:
//		double y:					探元指向角的值
// 返回值：
//		double:						返回的列坐标索引号
//////////////////////////////////////
void GeoCameraLine::GetIndexBaseInner(double phiY, double phiX, double &y, double &x)
{
	// 搜索内插用的起始和结束点(对分查找)
	long posstart, posend, pos;
	posstart=0, posend=m_Input.Ynum-1, pos=0;
	if(mark==1)
	{
		while(posstart<posend)  
		{  
			pos = (posstart+posend)/2;
			if(pos==posstart) break;	// 记得加上这句判断,否则会陷入死循环
			if((m_Y[pos]<=phiY)&&(phiY<m_Y[pos+1]))
				break;  
			if (m_Y[pos] <= phiY)  
				posstart = pos;  
			else
				posend = pos;       
		}  
	}
	else
	{
		while(posstart<posend)  
		{  
			pos = (posstart+posend)/2;
			if(pos==posstart) break;	// 记得加上这句判断,否则会陷入死循环
			if((m_Y[pos]>=phiY)&&(phiY>m_Y[pos+1]))
				break;  
			if (m_Y[pos] >= phiY)  
				posstart = pos;  
			else
				posend = pos;       
		}  
	}
	if(pos<0)				pos = 0;
	if(pos>m_Input.Ynum-2)	pos = m_Input.Ynum-2;
	// 内插出位置
	y = pos + (phiY-m_Y[pos])/(m_Y[pos+1]-m_Y[pos]);
}

