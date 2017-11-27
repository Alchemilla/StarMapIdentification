#include <io.h>
#include "GeoCameraLine.h"

//////////////////////////////////////////////////////////////////////////
// ����/��������
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


// �������
void GeoCameraLine::ClearData()
{
	if(m_X!=NULL)	delete []m_X;		m_X = NULL;
	if(m_Y!=NULL)	delete []m_Y;		m_Y = NULL;
}


//////////////////////////////////////
// ���ܣ���ȡ�ڷ�λԪ���ļ�
// ����:
//		string filepath:			�ڷ�λԪ���ļ�·��
//									��������洢CCD�ڽ����ϵ�����
//									���Ϊ"",������input������׼����
//									input���溬�������ƫ�ĽǺ�ƫ�ľ�
//		StrCamParamInput input��	�ڷ�λԪ�������������
// ����ֵ��
//		void
//////////////////////////////////////
void GeoCameraLine::ReadCamFile(string filepath, StrCamParamInput input)
{
	m_Input = input;
	// ���δ�ṩ����ļ�,��ֱ�Ӳ�����׼CCD����
	if(_access(filepath.c_str(), 0)==-1)
	{
		InitInnerFile(filepath, input);
	}
	// ����ṩ������ļ�,��ֱ�Ӷ�ȡ����ļ�
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
				//m_X[i] = -m_X[i];//ע��X��ʵ���෴�����
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
// ���ܣ���ȡ�ڷ�λԪ��
// ����:
// ����ֵ��
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
// ���ܣ�����ڷ�λԪ���ļ�
// ����:
//		string filepath:			�ڷ�λԪ���ļ�·��
// ����ֵ��
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
// ���ܣ�����CCD���л�ȡ��ʼ�����ڷ�λԪ���ļ�
// ����:
//		string outpath:				�ڷ�λԪ������ļ�,���Ϊ"",�����
//		StrCamParamInput input��	�ڷ�λԪ�������������
// ����ֵ��
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
// ���ܣ���ȡ�ڷ�λԪ��(̽Ԫָ�����ʽ)
// ����:
//		string outpath:				�ڷ�λԪ������ļ�
//		StrCamParamInput input��	�ڷ�λԪ�������������
// ����ֵ��
//		void
//////////////////////////////////////
void GeoCameraLine::GetInner(double x, double y, double &phiX, double &phiY)
{
	// �߽��ж�
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
	// �����ڲ�
	phiX = (1-delY)*m_X[indexY] + delY*m_X[indexY+1];
	phiY = (1-delY)*m_Y[indexY] + delY*m_Y[indexY+1];
}


//////////////////////////////////////
// ���ܣ�����̽Ԫָ��ǵ�ֵȡ����������
// ����:
//		double y:					̽Ԫָ��ǵ�ֵ
// ����ֵ��
//		double:						���ص�������������
//////////////////////////////////////
void GeoCameraLine::GetIndexBaseInner(double phiY, double phiX, double &y, double &x)
{
	// �����ڲ��õ���ʼ�ͽ�����(�Էֲ���)
	long posstart, posend, pos;
	posstart=0, posend=m_Input.Ynum-1, pos=0;
	if(mark==1)
	{
		while(posstart<posend)  
		{  
			pos = (posstart+posend)/2;
			if(pos==posstart) break;	// �ǵü�������ж�,�����������ѭ��
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
			if(pos==posstart) break;	// �ǵü�������ж�,�����������ѭ��
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
	// �ڲ��λ��
	y = pos + (phiY-m_Y[pos])/(m_Y[pos+1]-m_Y[pos]);
}

