#include "GeoTranslation.h"


GeoTranslation::GeoTranslation(void)
{
	ReleaseTriangleModel(m_in, false);
	ReleaseTriangleModel(m_out, false);
	m_nCurTriangle = 0;
	m_num = 0;
	// ��ʼ������
	m_aff[0] = 0;	m_aff[1] = 1;	m_aff[2] = 0;
	m_aff[3] = 0;	m_aff[4] = 0;	m_aff[5] = 1;
	xscale = yscale = uscale = vscale = 1.0;		// ���Ų���
    xoff = yoff = uoff = voff = 0.0;				// ƽ�Ʋ���
}


GeoTranslation::~GeoTranslation(void)
{
	ReleaseTriangleModel(m_in);
	ReleaseTriangleModel(m_out);
}


////////////////////////////////////////////////////////
// ���ݹ�һ��
////////////////////////////////////////////////////////
void GeoTranslation::DataNormalization(long num, double *x, double *y, double *u, double *v)
{
	// ��ù�һ��ϵ��
	xoff = Average(x, num);     yoff = Average(y, num);
	uoff = Average(u, num);     voff = Average(v, num);
	MinMax(x, num, maxx, minx);
	MinMax(y, num, maxy, miny);
	MinMax(u, num, maxu, minu);
	MinMax(v, num, maxv, minv);
	xscale =  max(fabs(maxx-xoff), fabs(minx-xoff));
	yscale =  max(fabs(maxy-yoff), fabs(miny-yoff));
	uscale =  max(fabs(maxu-uoff), fabs(minu-uoff));
	vscale =  max(fabs(maxv-voff), fabs(minv-voff));
	// ������������й�һ��
	for(long i=0; i<num; i++)
	{
		x[i] = (x[i]-xoff)/xscale;
		y[i] = (y[i]-yoff)/yscale;		
		u[i] = (u[i]-uoff)/uscale;  
		v[i] = (v[i]-voff)/vscale;	
	}
	// ���÷������
	m_aff[0] = 0;	m_aff[1] = 1;	m_aff[2] = 0;
	m_aff[3] = 0;	m_aff[4] = 0;	m_aff[5] = 1;
}



////////////////////////////////////////////////////////
// ����ƽ��ֵ
////////////////////////////////////////////////////////
double GeoTranslation::Average(double *value, long num)
{
	double temp = 0;
	for(long i=0; i<num; i++)
	{
		temp += (value[i]/num);
	}
	return temp;
}


////////////////////////////////////////////////////////
// ���������Сֵ
////////////////////////////////////////////////////////
void GeoTranslation::MinMax(double *value, long num, double &max, double &min)
{
	min = DBL_MAX;
	max = DBL_MIN;
	for(long i=0; i<num; i++)
	{
		if(value[i]>max) max = value[i];
		if(value[i]<min) min = value[i];
	}
}



////////////////////////////////////////////////////////
// �������任ϵ��
////////////////////////////////////////////////////////
bool GeoTranslation::CalAffineParam(double *x, double *y, double *u, double *v, long num)
{
	if(num<3)
	{
		return false;
	}
	// ��ù�һ��ϵ��
	xoff = Average(x, num);     yoff = Average(y, num);
	uoff = Average(u, num);     voff = Average(v, num);
	MinMax(x, num, maxx, minx);
	MinMax(y, num, maxy, miny);
	MinMax(u, num, maxu, minu);
	MinMax(v, num, maxv, minv);
	xscale =  max(fabs(maxx-xoff), fabs(minx-xoff));
	yscale =  max(fabs(maxy-yoff), fabs(miny-yoff));
	uscale =  max(fabs(maxu-uoff), fabs(minu-uoff));
	vscale =  max(fabs(maxv-voff), fabs(minv-voff));
	// ������������й�һ��
	for(long i=0; i<num; i++)
	{
		x[i] = (x[i]-xoff)/xscale;
		y[i] = (y[i]-yoff)/yscale;		
		u[i] = (u[i]-uoff)/uscale;  
		v[i] = (v[i]-voff)/vscale;	
	}
	// ת�������󲢽���ƽ�����
	double ATA1[9], ATL1[3], A1[3], L1;
	double ATA2[9], ATL2[3], A2[3], L2;
	memset(ATA1, 0, sizeof(double)*9);	memset(ATA2, 0, sizeof(double)*9);
	memset(ATL1, 0, sizeof(double)*3);	memset(ATL2, 0, sizeof(double)*3);
	for(long i=0; i<num; i++)
	{
		A1[0] = 1;	A1[1] = x[i];	A1[2] = y[i];	L1 = u[i];
		m_Base.pNormal(A1, 3, L1, ATA1, ATL1, 1.0);
		A2[0] = 1;	A2[1] = x[i];	A2[2] = y[i];	L2 = v[i];
		m_Base.pNormal(A2, 3, L2, ATA2, ATL2, 1.0);
	}
	m_Base.solve33(ATA1, ATL1);		m_Base.solve33(ATA2, ATL2);
	memcpy(&m_aff[0], ATL1, sizeof(double)*3);
	memcpy(&m_aff[3], ATL2, sizeof(double)*3);
	return true;
}



////////////////////////////////////////////////////////
// ���ݷ���任ϵ�����ֵ
////////////////////////////////////////////////////////
void GeoTranslation::GetValueBaseAffine(double x, double y, double &u, double &v)
{
	x = (x-xoff)/xscale;
	y = (y-yoff)/yscale;
	u = m_aff[0] + m_aff[1]*x + m_aff[2]*y;
	v = m_aff[3] + m_aff[4]*x + m_aff[5]*y;
	u = u*uscale + uoff;
	v = v*vscale + voff;
}


////////////////////////////////////////////////////////
// ���ݶ���ʽϵ�����ֵ
////////////////////////////////////////////////////////
void GeoTranslation::GetValueBasePoly(double x, double y, double &u, double &v)
{
	u = v = 0;
	for(int m=0; m<m_xnum; m++)
	{
		for(int n=0; n<m_ynum; n++)
		{
			u += m_polyx[m*m_ynum+n]*pow(x, 1.0*m)*pow(y, 1.0*n);
			v += m_polyy[m*m_ynum+n]*pow(x, 1.0*m)*pow(y, 1.0*n);
		}
	}
}


////////////////////////////////////////////////////////
// ���ݷ������ʽϵ�����ֵ
////////////////////////////////////////////////////////
void GeoTranslation::GetValueBaseInvPoly(double u, double v, double &x, double &y)
{
	x = y = 0;
	for(int m=0; m<m_xnum; m++)
	{
		for(int n=0; n<m_ynum; n++)
		{
			x += m_polyxinv[m*m_ynum+n]*pow(u, 1.0*m)*pow(v, 1.0*n);
			y += m_polyyinv[m*m_ynum+n]*pow(u, 1.0*m)*pow(v, 1.0*n);
		}
	}
}


////////////////////////////////////////////////////////
// ���㷴��ϵ��
////////////////////////////////////////////////////////
void GeoTranslation::CalPolyParam()
{
	double x, y, u, v;
	// �����ڴ�
	double *A, *AxTAx, *AyTAy, *AxTL, *AyTL, *Xtemp, *Ytemp;
	A = AxTAx = AyTAy = AxTL = AyTL = Xtemp = Ytemp = NULL;
	A = new double[m_xynum];
	Xtemp = new double[m_xynum];
	Ytemp = new double[m_xynum];
	AxTAx = new double[m_xynum*m_xynum];
	AxTL = new double[m_xynum];
	AyTAy = new double[m_xynum*m_xynum];
	AyTL = new double[m_xynum];
	memset(AxTAx, 0, sizeof(double)*m_xynum*m_xynum);
	memset(AxTL, 0, sizeof(double)*m_xynum);
	memset(AyTAy, 0, sizeof(double)*m_xynum*m_xynum);
	memset(AyTL, 0, sizeof(double)*m_xynum);
	int xsize, ysize;
	xsize = 100;
	ysize = 10;
	for(int i=0; i<xsize; i++)
	{
		for(int j=0; j<ysize; j++)
		{
			x = i*1.0/xsize;
			y = j*1.0/ysize;
			GetValueBasePoly(x, y, u, v);
			for(int m=0; m<m_xnum; m++)
			{
				for(int n=0; n<m_ynum; n++)
				{
					A[m*m_ynum+n] = pow(u, 1.0*m)*pow(v, 1.0*n);
				}
			}
			m_Base.pNormal(A, m_xynum, x, AxTAx, AxTL, 1.0);
			m_Base.pNormal(A, m_xynum, y, AyTAy, AyTL, 1.0);
		}
	}
	// ƽ�����
	memset(Xtemp, 0, sizeof(double)*m_xynum);
	Xtemp[m_ynum] = 1.0;
	m_Base.GaussExt(AxTAx, AxTL, Xtemp, m_xynum);
	memset(Ytemp, 0, sizeof(double)*m_xynum);
	Ytemp[1] = 1.0;
	m_Base.GaussExt(AyTAy, AyTL, Ytemp, m_xynum);
	// ��ʼ��ֵ
	m_polyxinv.resize(m_xynum);
	m_polyyinv.resize(m_xynum);
	memcpy(&(m_polyxinv[0]), &(Xtemp[0]), sizeof(double)*m_xynum);
	memcpy(&(m_polyyinv[0]), &(Ytemp[0]), sizeof(double)*m_xynum);
	// �ͷ��ڴ�
	if(A!=NULL)			delete []A;			A = NULL;
	if(Xtemp!=NULL)		delete []Xtemp;		Xtemp = NULL;
	if(Ytemp!=NULL)		delete []Ytemp;		Ytemp = NULL;
	if(AxTAx!=NULL)		delete []AxTAx;		AxTAx = NULL;
	if(AyTAy!=NULL)		delete []AyTAy;		AyTAy = NULL;
	if(AxTL!=NULL)		delete []AxTL;		AxTL = NULL;
	if(AyTL!=NULL)		delete []AyTL;		AyTL = NULL;
}


////////////////////////////////////////////////////////
// �������任ϵ��_CE5ר��
////////////////////////////////////////////////////////
void GeoTranslation::CalAffineParamCE5(double *x, double *y, double *u, double *v, long num)
{
	if(num<3)
		return;
	// ��ù�һ��ϵ��
	xoff = Average(x, num);     yoff = Average(y, num);
	uoff = Average(u, num);     voff = Average(v, num);
	MinMax(x, num, maxx, minx);
	MinMax(y, num, maxy, miny);
	MinMax(u, num, maxu, minu);
	MinMax(v, num, maxv, minv);
	xscale =  max(fabs(maxx-xoff), fabs(minx-xoff));
	yscale =  max(fabs(maxy-yoff), fabs(miny-yoff));
	uscale =  max(fabs(maxu-uoff), fabs(minu-uoff));
	vscale =  max(fabs(maxv-voff), fabs(minv-voff));
	// ������������й�һ��
	for(long i=0; i<num; i++)
	{
		x[i] = (x[i]-xoff)/xscale;
		y[i] = (y[i]-yoff)/yscale;		
		u[i] = (u[i]-uoff)/uscale;  
		v[i] = (v[i]-voff)/vscale;	
	}
	// ת�������󲢽���ƽ�����
	double ATA1[9], ATL1[3], A1[3], L1;
	double ATA2[9], ATL2[3], A2[3], L2;
	memset(ATA1, 0, sizeof(double)*9);	memset(ATA2, 0, sizeof(double)*9);
	memset(ATL1, 0, sizeof(double)*3);	memset(ATL2, 0, sizeof(double)*3);
	for(long i=0; i<num; i++)
	{
		A1[0] = 1;	A1[1] = x[i];	A1[2] = y[i];	L1 = u[i];
		m_Base.pNormal(A1, 3, L1, ATA1, ATL1, 1.0);
		A2[0] = 1;	A2[1] = x[i];	A2[2] = y[i];	L2 = v[i];
		m_Base.pNormal(A2, 3, L2, ATA2, ATL2, 1.0);
	}
/*	double ATL1temp[3], ATL2temp[3];
	ATL1temp[0] = 0.0;	ATL1temp[1] = 1.0;	ATL1temp[2] = 0.0;
	ATL2temp[0] = 0.0;	ATL2temp[1] = 0.0;	ATL2temp[2] = 1.0;
	m_Base.GaussExt(ATA1, ATL1, ATL1temp, 3);
	m_Base.GaussExt(ATA2, ATL2, ATL2temp, 3);
	memcpy(&m_aff[0], ATL1temp, sizeof(double)*3);
	memcpy(&m_aff[3], ATL2temp, sizeof(double)*3);*/
	m_Base.solve33(ATA1, ATL1);		m_Base.solve33(ATA2, ATL2);
	memcpy(&m_aff[0], ATL1, sizeof(double)*3);
	memcpy(&m_aff[3], ATL2, sizeof(double)*3);
	// �������
	double afftemp[6];
	afftemp[0] = (m_aff[0]-m_aff[1]*xoff/xscale-m_aff[2]*yoff/yscale)*uscale+uoff;
	afftemp[1] = m_aff[1]*uscale/xscale;
	afftemp[2] = m_aff[2]*uscale/yscale;
	afftemp[3] = (m_aff[3]-m_aff[4]*xoff/xscale-m_aff[5]*yoff/yscale)*vscale+voff;
	afftemp[4] = m_aff[4]*vscale/xscale;
	afftemp[5] = m_aff[5]*vscale/yscale;
	memcpy(m_aff, afftemp, sizeof(double)*6);
}


////////////////////////////////////////////////////////
// ����ǰһ������ϵ��,��������ϵ��
////////////////////////////////////////////////////////
void GeoTranslation::ModifyAffineCE5(double aff[6])
{
	double afftemp[6];
	afftemp[0] = aff[0] + m_aff[0]*aff[1] + m_aff[3]*aff[2];
	afftemp[1] = m_aff[1]*aff[1] + m_aff[4]*aff[2];
	afftemp[2] = m_aff[2]*aff[1] + m_aff[5]*aff[2];
	afftemp[3] = aff[3] + m_aff[0]*aff[4] + m_aff[3]*aff[5];
	afftemp[4] = m_aff[1]*aff[4] + m_aff[4]*aff[5];
	afftemp[5] = m_aff[2]*aff[4] + m_aff[5]*aff[5];
	memcpy(m_aff, afftemp, sizeof(double)*6);
}


////////////////////////////////////////////////////////
// ��������,�Ӿֲ���ȫ��
////////////////////////////////////////////////////////
void GeoTranslation::GetValueCE5(double x, double y, double &u, double &v)
{
	u = m_aff[0] + m_aff[1]*x + m_aff[2]*y;
	v = m_aff[3] + m_aff[4]*x + m_aff[5]*y;
}


////////////////////////////////////////////////////////
// ���䷴��,��ȫ�ֵ��ֲ�
////////////////////////////////////////////////////////
void GeoTranslation::GetValueInvCE5(double u, double v, double &x, double &y)
{
	x = (m_aff[5]*(u-m_aff[0])-m_aff[2]*(v-m_aff[3]))/
		(m_aff[1]*m_aff[5]-m_aff[2]*m_aff[4]);
	y = (m_aff[4]*(u-m_aff[0])-m_aff[1]*(v-m_aff[3]))/
		(m_aff[2]*m_aff[4]-m_aff[1]*m_aff[5]);
}



/////////////////////////////////////////////////////
// �ͷ�������ģ��
/////////////////////////////////////////////////////
void GeoTranslation::ReleaseTriangleModel(triangulateio &m_tri, bool isdel)
{
	if(isdel==true)
	{
		if(m_tri.pointlist!=NULL)				free(m_tri.pointlist);
		if(m_tri.pointattributelist!=NULL)		free(m_tri.pointattributelist);
		if(m_tri.pointmarkerlist!=NULL)			free(m_tri.pointmarkerlist);
		if(m_tri.trianglelist!=NULL)			free(m_tri.trianglelist);
		if(m_tri.triangleattributelist!=NULL)	free(m_tri.triangleattributelist);
		if(m_tri.trianglearealist!=NULL)		free(m_tri.trianglearealist);
		if(m_tri.neighborlist!=NULL)			free(m_tri.neighborlist);
		if(m_tri.segmentlist!=NULL)				free(m_tri.segmentlist);
		if(m_tri.segmentmarkerlist!=NULL)		free(m_tri.segmentmarkerlist);
		if(m_tri.holelist!=NULL)				free(m_tri.holelist);
		if(m_tri.regionlist!=NULL)				free(m_tri.regionlist);
		if(m_tri.edgelist!=NULL)				free(m_tri.edgelist);
		if(m_tri.edgemarkerlist!=NULL)			free(m_tri.edgemarkerlist);
		if(m_tri.normlist!=NULL)				free(m_tri.normlist);
	}
	m_tri.pointlist = (REAL*)NULL;
	m_tri.pointattributelist = (REAL*)NULL;
	m_tri.pointmarkerlist = (int*)NULL;
	m_tri.trianglelist = (int*)NULL;
	m_tri.triangleattributelist = (REAL*)NULL;
	m_tri.trianglearealist = (REAL*)NULL;
	m_tri.neighborlist = (int*)NULL;
	m_tri.segmentlist = (int*)NULL;
	m_tri.segmentmarkerlist = (int*)NULL;
	m_tri.holelist = (REAL*)NULL;
	m_tri.regionlist = (REAL*)NULL;
	m_tri.edgelist = (int*)NULL;
	m_tri.edgemarkerlist = (int*)NULL;
	m_tri.normlist = (REAL*)NULL;
}


/////////////////////////////////////////////////////
// ����������ģ��
/////////////////////////////////////////////////////
void GeoTranslation::CreateTriangleModel(long num, double *x, double *y, double *u, double *v)
{
	m_num = num;
	// �����ͷ�ǰһ�ε�������ģ��
	ReleaseTriangleModel(m_in);
	ReleaseTriangleModel(m_out);
	// ��ʼ����ռ�
	m_in.numberofpoints = num;
	m_in.pointlist = (REAL*)malloc(m_in.numberofpoints*2*sizeof(REAL));
	m_in.numberofpointattributes = 2;
	m_in.pointattributelist = (REAL*)malloc(m_in.numberofpoints*m_in.numberofpointattributes*sizeof(REAL));
	// ��ʼ��ֵ
	for (int i=0; i<num; i++)
	{
		m_in.pointlist[2*i] = x[i];                
		m_in.pointlist[2*i+1] = y[i];              
		m_in.pointattributelist[2*i] = u[i];      
		m_in.pointattributelist[2*i+1] = v[i];   
	}
	triangulate("czAen", &m_in, &m_out, (struct triangulateio *) NULL);
	// �ͷſռ�
	ReleaseTriangleModel(m_in);
	m_nCurTriangle = 0;
	// ��ȡ����任ϵ��
	CalAffineParam(x, y, u, v, num);
}

int GeoTranslation::area(int a,int b, double x3, double y3)
{
	double x[3],y[3];
	double det;
	x[0] = m_out.pointlist[a*2+0];
	x[1] = m_out.pointlist[b*2+0];
	x[2] = x3;
	y[0] = m_out.pointlist[a*2+1];
	y[1] = m_out.pointlist[b*2+1];
	y[2] = y3;

	det=(x[1]-x[0])*(y[2]-y[0])-(y[1]-y[0])*(x[2]-x[0]);
	if(det<0)
		return -1;
	else if(det>0)
		return 1;
	else
		return 0;
}

int GeoTranslation::GetLocalTriangle(double x, double y)
{
	int *cneighborlist,k;	
	int t1,t2,t3;		
	int *cpointlist;
	k=0;
	int lasttri;
	if(m_nCurTriangle<0 || m_nCurTriangle>=m_out.numberoftriangles)
		m_nCurTriangle = 0;
	do
	{
		lasttri = m_nCurTriangle;
		cpointlist = &(m_out.trianglelist[m_nCurTriangle*3]);
		cneighborlist = &(m_out.neighborlist[m_nCurTriangle*3]);

		t3=area(cpointlist[0], cpointlist[1],x, y);
		t1=area(cpointlist[1], cpointlist[2],x, y);
		t2=area(cpointlist[2], cpointlist[0],x, y);		

		if(t1==-1)
			m_nCurTriangle=cneighborlist[0];
		else if(t2==-1)
			m_nCurTriangle=cneighborlist[1];
		else if(t3==-1)
			m_nCurTriangle=cneighborlist[2];
		else
			break;        
	}while(m_nCurTriangle!=lasttri&&m_nCurTriangle!=-1);	
	return m_nCurTriangle;
}

/////////////////////////////////////////////////////
// ���������������ڲ�
/////////////////////////////////////////////////////
void GeoTranslation::FromXY2UV(double x, double y, double& u, double& v)
{
	int triindex = GetLocalTriangle(x, y);
	// �����ڲ�����������������ĸ����㹻��,����������
	if((triindex != -1) && (m_num>30))
	{
		int i;
		double e0[3],n0[3],x0[3],y0[3];	

		int *cpointlist = &(m_out.trianglelist[triindex *3 ]);
		for(i=0;i<3;i++)
		{
			e0[i] = m_out.pointlist[cpointlist[i]*2];
			n0[i] = m_out.pointlist[cpointlist[i]*2+1];
			x0[i] = m_out.pointattributelist[cpointlist[i]*m_out.numberofpointattributes];
			y0[i] = m_out.pointattributelist[cpointlist[i]*m_out.numberofpointattributes+1];
		}
		e0[1] -= e0[0];		e0[2] -= e0[0];
		n0[1] -= n0[0];		n0[2] -= n0[0];
		x0[1] -= x0[0];		x0[2] -= x0[0];
		y0[1] -= y0[0];		y0[2] -= y0[0];

		u = x0[0]-((x-e0[0])*(n0[1]*x0[2]-n0[2]*x0[1])+
			(y-n0[0])*(x0[1]*e0[2]-x0[2]*e0[1]))/(e0[1]*n0[2]-e0[2]*n0[1]);
		v = y0[0]-((x-e0[0])*(n0[1]*y0[2]-n0[2]*y0[1])+
			(y-n0[0])*(y0[1]*e0[2]-y0[2]*e0[1]))/(e0[1]*n0[2]-e0[2]*n0[1]);
	}
	else
	{
		GetValueBaseAffine(x, y, u, v);
	}
}