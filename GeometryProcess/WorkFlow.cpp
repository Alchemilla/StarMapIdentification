#include "WorkFlow.h"
//#include "MatchMethod.h"
//#include "PhaseCorrelation.h"
//#include <io.h>


//////////////////////////////////////////////////////////////////////////
// ����/��������
//////////////////////////////////////////////////////////////////////////
WorkFlow::WorkFlow(void)	
{
//	pModel = NULL;
}

WorkFlow::~WorkFlow(void)	
{
//	if(pModel!=NULL)	delete []pModel;	pModel = NULL;
}

/*
//////////////////////////////////////////////////////////
// ��һ���ļ����а���ͨ�������ָ�����͵��ļ�
//////////////////////////////////////////////////////////
bool WorkFlow::AddFeaFileInDirectory(string dirpath, vector<string> &filelist)
{
	long handle;									//���ڲ��ҵľ��
	struct _finddata_t fileinfo;					//�ļ���Ϣ�Ľṹ��
	handle=_findfirst(dirpath.c_str(), &fileinfo);	//��һ�β���
	string path = dirpath.substr(0, dirpath.find_last_of("\\//")+1);
	if(-1==handle)
	{
		printf("ͨ�������ʧ��!\n");
		return false;
	}
	string temp = path;
	filelist.push_back(temp.append(fileinfo.name));
	//ѭ�������������ϵ��ļ���ֱ���Ҳ���������Ϊֹ
	while(!_findnext(handle, &fileinfo))               
	{
		temp = path;
		filelist.push_back(temp.append(fileinfo.name));
	}
	//�����˹رվ��
	_findclose(handle);
	return true;
}


//////////////////////////////////////
// ���ܣ�ǰ�����ṹ��DSM
// ����:
//		int num:			ģ�͸���
//		GeoModel **model��	����ģ��,����Ϊnum,�ҵ�һ��Ϊ�ο�ģ��
//		string *imgpath:	����ģ�Ͷ�Ӧ��Ӱ��,����Ϊnum,����ƥ��
//		string *ptspath:	����׼�ļ�,����Ϊnum-1,����Ԥ��
//		string outpath:		������ܼ���ά��
// ����ֵ��
//		void
////////////////////////////////////// 
void WorkFlow::GenerateDSM(int num, GeoModel **model, string *imgpath, string *ptspath, 
						string dempath, string outpath, int win, string datumname)
{
	// ��Ӱ��
	GeoReadImage *img = new GeoReadImage[num];
	for(int i=0; i<num; i++)
	{
		img[i].Open(imgpath[i], GA_ReadOnly);
		img[i].SetBuffer(0, 0, img[i].m_xRasterSize, img[i].m_yRasterSize, img[i].pBuffer[0]);
		img[i].ReadBlock(0, 0, img[i].m_xRasterSize, img[i].m_yRasterSize, 0, img[i].pBuffer[0], 1, 1);
	}
	GeoReadImage m_dem;
	m_dem.Open(dempath, GA_ReadOnly);
	m_dem.ReadBlock(0, 0, m_dem.m_xRasterSize, m_dem.m_yRasterSize, 0, m_dem.pBuffer[0]);
	// ��������Ԥ�����
	long nmatch;
	double *lx0, *ly0, *rx0, *ry0;
	lx0 = ly0 = rx0 = ry0 = NULL;
	GeoTranslation *trans = new GeoTranslation[num];
	for(int i=1; i<num; i++)
	{
		ReadMatchBaseEnvi(ptspath[i-1], nmatch, lx0, ly0, rx0, ry0);
		trans[i].CalAffineParam(lx0, ly0, rx0, ry0, nmatch);
	}
	///////////////////////////////////
	// ��������׼
	///////////////////////////////////
	long lt_x, lt_y, rb_x, rb_y;
	lt_x = lt_y = 32;
	rb_x = img[0].m_OffsetX-lt_x;
	rb_y = img[0].m_OffsetY-lt_y;
	PhaseCorrelation m_Corr;
	double SNR;
	StrDATUM datum;
	m_base.GetRefEllipsoid(datumname, datum);
	vector<double> m_x;		m_x.resize(num);
	vector<double> m_y;		m_y.resize(num);
	double lat, lon, h, X, Y, Z, x, y;
	FILE *fp = fopen(outpath.c_str(), "w");
	// ��ʼѭ��
	for(long j=lt_y; j<rb_y; j++)
	{
		float rx, ry, offx, offy;
		for(long i=lt_x; i<rb_x; i++)
		{
			bool isValid = true;
			m_x[0] = i;		m_y[0] = j;
			// �ܼ�ƥ��
			for(int k=1; k<num; k++)
			{
				trans[k].GetValueBaseAffine(m_x[0], m_y[0], m_x[k], m_y[k]);
				rx = m_x[k];		ry = m_y[k];
				isValid *= m_Corr.GetPixel(&(img[0]), &(img[k]), m_x[0], m_y[0], rx, ry, win);
				if(isValid==false)	break;
				if(fabs(rx-m_x[k])>10||fabs(ry-m_y[k])>10)	break;
				// ��ͶӰ
				h = 0;
				isValid = EleItera(model[0], ry, rx, &m_dem, lat, lon, h);
				if(isValid==false)	break;
				model[k]->FromLatLon2XY(lat, lon, h, y, x);
				m_x[k] = x;		m_y[k] = y;
				if(m_x[k]<0||m_x[k]>img[k].m_xRasterSize-1||m_y[k]<0||m_y[k]>img[k].m_yRasterSize-1)
				{
					isValid = false;
					break;
				}
			}
			// ǰ������
			if(isValid == true)
			{
				model[0]->RigorousForwardIntersection(num, &model[0], &(m_y[0]), &(m_x[0]), X, Y, Z);
				m_base.Rect2Geograph(datum, X, Y, Z, lat, lon, h);
				fprintf(fp, "%19.9lf\t%19.9lf\t%19.9lf\n", lon*180/PI, lat*180/PI, h);
			}
		}
		printf("%ld\n", j);
	}
	fclose(fp);		fp = NULL;
	// �ر�Ӱ��
	for(int i=0; i<num; i++)
		img[i].Destroy();
	delete []img;		img = NULL;
	delete []trans;		trans  = NULL;
}


////////////////////////////////////////////////////////////
// ��ENVI��ʽ����ƥ����
// ���룺
//		string ptspath:		����ƥ����ļ�·��
//		long &nmatch:		���ƥ���ĸ���
//		float* &lx:			��Ӱ��x����
//		float* &ly��		��Ӱ��y����
//		float* &rx��		��Ӱ��x����
//		float* &ry��		��Ӱ��y����
// ����ֵ:
//		void
////////////////////////////////////////////////////////////
void WorkFlow::ReadMatchBaseEnvi(string ptspath, long &nmatch, double* &lx, double* &ly, double* &rx, double* &ry)
{
	vector<double> x0, y0, u0, v0;
	double x, y, u, v;
	FILE *fp = fopen(ptspath.c_str(), "r");
	if(fp!=NULL)
	{
		char tmp[4096], w;
		for(long i=0; i<5; i++)
		{
			fscanf(fp, "%[^\n]s", tmp);		fscanf(fp, "%c", &w);
		}
		while(!feof(fp))
		{
			fscanf(fp, "%lf%lf%lf%lf", &x, &y, &u, &v);
			x0.push_back(x);	y0.push_back(y);
			u0.push_back(u);	v0.push_back(v);
		}
		x0.pop_back();	y0.pop_back();	u0.pop_back();	v0.pop_back();
		nmatch = x0.size();
		if(nmatch>0)
		{
			if(lx!=NULL)	delete []lx;	lx = NULL;
			if(ly!=NULL)	delete []ly;	ly = NULL;
			if(rx!=NULL)	delete []rx;	rx = NULL;
			if(ry!=NULL)	delete []ry;	ry = NULL;
			lx = new double[nmatch];	ly = new double[nmatch];
			rx = new double[nmatch];	ry = new double[nmatch];
			memcpy(lx, &(x0[0]), sizeof(double)*nmatch);
			memcpy(ly, &(y0[0]), sizeof(double)*nmatch);
			memcpy(rx, &(u0[0]), sizeof(double)*nmatch);
			memcpy(ry, &(v0[0]), sizeof(double)*nmatch);
		}
	}
	fclose(fp);		fp = NULL;
	x0.clear();		y0.clear();
	u0.clear();		v0.clear();
}

////////////////////////////////////////////////////////////
// ��ENVI��ʽ���ƥ����
// ���룺
//		string ptspath:		���ƥ����ļ�·��
//		long nmatch:		���ƥ���ĸ���
//		float* lx:			��Ӱ��x����
//		float* ly��			��Ӱ��y����
//		float* rx��			��Ӱ��x����
//		float* ry��			��Ӱ��y����
// ����ֵ:
//		void
////////////////////////////////////////////////////////////
void WorkFlow::SaveMatchBaseEnvi(string ptspath, long nmatch, double *lx, double *ly, double *rx, double *ry)
{
	FILE *fp = fopen(ptspath.c_str(), "w");
	int m, n;
	if(fp!=NULL)
	{
		fprintf(fp, "; ENVI Image to Image GCP File\n");
		fprintf(fp, "; base file: leftimg\n");
		fprintf(fp, "; warp file: rightimg\n");
		fprintf(fp, "; Base Image (x,y), Warp Image (x,y)\n;\n");
		for(long k=0; k<nmatch; k++)
		{
			fprintf(fp, "%19.6lf\t%19.6lf\t%19.6lf\t%19.6lf\n", lx[k], ly[k], rx[k], ry[k]);
		}
		fclose(fp);
		fp = NULL;
	}
}


//////////////////////////////////////
// ���ܣ�Ӱ���໥ͶӰ
// ����:
//		string imgsrc��		��׼Ӱ��·��
//		GeoModel *modelsrc: ��׼Ӱ�����ģ��
//      string imgpro:		ͶӰӰ��·��
//		GeoModel *modelpro:	ͶӰӰ�����ģ��
//		string imgdem:		�ο�DEM
//		string imgout:		���Ӱ��·��
// ����ֵ��
//		void
//////////////////////////////////////
void WorkFlow::ImageProject(string imgsrc, GeoModel *modelsrc, string imgpro, 
							GeoModel *modelpro, string imgdem, string imgout)
{
	long i, j;
	GeoReadImage m_dem, m_src, m_pro, m_out;
	// �򿪻�׼Ӱ��
	m_src.Open(imgsrc, GA_ReadOnly);
	for(i=0; i<m_src.m_nBands; i++)
	{
		m_src.ReadBlock(0, 0, m_src.m_xRasterSize, m_src.m_yRasterSize, i, m_src.pBuffer[i]);
	}
	// ��ͶӰӰ��
	m_pro.Open(imgpro, GA_ReadOnly);
	for(i=0; i<m_pro.m_nBands; i++)
	{
		m_pro.ReadBlock(0, 0, m_pro.m_xRasterSize, m_pro.m_yRasterSize, i, m_pro.pBuffer[i]);
	}
	// ��demӰ��
	m_dem.Open(imgdem, GA_ReadOnly);
	for(i=0; i<m_dem.m_nBands; i++)
	{
		m_dem.ReadBlock(0, 0, m_dem.m_xRasterSize, m_dem.m_yRasterSize, i, m_dem.pBuffer[i]);
	}
	// �����Ӱ��m_src.m_xRasterSize, m_src.m_yRasterSize
	m_out.New(imgout, "GTiff", m_pro.m_BandType, m_src.m_xRasterSize, m_src.m_yRasterSize, m_pro.m_nBands);
	m_out.Destroy();
	m_out.Open(imgout, GA_Update);
	for(i=0; i<m_out.m_nBands; i++)
	{
		m_out.SetBuffer(0, 0, m_out.m_xRasterSize, m_out.m_yRasterSize, m_out.pBuffer[i]);
	}
	///////////////////////////////////////
	// �����߳����
	///////////////////////////////////////
#pragma omp parallel 
	{  
#pragma omp for
		for(i=0; i<m_out.m_yRasterSize; i++)		// �ع�
		{
#pragma omp parallel 
			{  
#pragma omp for
				for(j=0; j<m_out.m_xRasterSize; j++)	// ����
				{
					double lat, lon, h, x, y, gray;
					h = 0;
					bool mark = EleItera(modelsrc, i, j, &m_dem, lat, lon, h);
					if(mark==true)
						modelpro->FromLatLon2XY(lat, lon, h, x, y);
					for(int k=0; k<m_out.m_nBands; k++)
					{
						if(mark==true)
							gray = m_pro.GetDataValue(y, x, 0, k, true);
						else
							gray = 0;
						m_out.SetDataValue(j, i, gray, k);
					}
				}
			}
		}
	}
	// ���Ӱ��
	for(i=0; i<m_out.m_nBands; i++)
	{
		m_out.WriteBlock(0, 0, m_out.m_xRasterSize, m_out.m_yRasterSize, i, m_out.pBuffer[i]);
	}
	// �ͷ��ڴ�
	m_src.Destroy();	m_pro.Destroy();
	m_dem.Destroy();	m_out.Destroy();
}


//////////////////////////////////////
// ���ܣ��̵߳����õ���Ӧ��������
// ����:
//		GeoModel *model:	����ģ��
//		double i,j:			Ӱ����������
//		GeoReadImage *pDEM��DEMӰ��
// �����
//		double &lat��		γ��
//		double &lon:		����
//		double &h��			�߳�
// ����ֵ��
//		bool
//////////////////////////////////////
bool WorkFlow::EleItera(GeoModel *model, double i, double j, GeoReadImage *pDEM, 
						double &lat, double &lon, double &h)
{
	double horigin = -9999;
	// ����DEM�ڲ�
	int num = 0;
	while(fabs(horigin-h)>1) 
	{
		horigin = h;
		model->FromXY2LatLon(i, j, h, lat, lon);
		// ת��Ϊ��γ��
		lat = lat*180/PI;
		lon = lon*180/PI;
		// ��ø߳�ֵ
		h = pDEM->GetDataValue(lon, lat, -9999, 0, false);
		num++;
		if(num>10||h<=-9999) return false;    // ע����Ҫ����-9999,����߽縳ֵ�����ѭ��������򲻶�Ӧ
	} 
	// ת��Ϊ����
	lat = lat*PI/180;
	lon = lon*PI/180;
	return true;
}


//////////////////////////////////////
// ���ܣ�����Ӱ����������ƥ��
// ����:
//		string leftpath:	��Ӱ��·��
//		string rightpath:	��Ӱ��·��
//		long lwidth,lheight:��Ӱ��ƥ������򳤶ȺͿ��
//		long rwidth,rheight:��Ӱ��ƥ������򳤶ȺͿ��
//		string ptspath:		�����pts�ļ�·��
//		long leftx, lefty:	��Ӱ��ƥ����������Ͻ�����
//		long rightx,righty:	��Ӱ��ƥ����������Ͻ�����
// ����ֵ��
//		void
//////////////////////////////////////
void WorkFlow::MatchBase_Data(string leftpath, string rightpath, long lwidth, long lheight, long rwidth, long rheight, 
							  string ptspath, long leftx, long lefty, long rightx, long righty)
{
	// ��ʼ��Ӱ����󲢴�Ӱ��
	GeoReadImage img1, img2;
	if(img1.Open(leftpath, GA_ReadOnly)==false)
	{
		printf("��Ӱ���ʧ��!\n");
		return;
	}
	if(img2.Open(rightpath, GA_ReadOnly)==false)
	{
		printf("��Ӱ���ʧ��!\n");
		return;
	}
	// ��ȡӰ������
	img1.SetBuffer(0, 0, lwidth, lheight, img1.pBuffer[0]);
	img1.ReadBlock(leftx, lefty, lwidth, lheight, 0, img1.pBuffer[0]);
	img2.SetBuffer(0, 0, rwidth, rheight, img2.pBuffer[0]);
	img2.ReadBlock(rightx, righty, rwidth, rheight, 0, img2.pBuffer[0]);
	// ����Ӱ����⻯
	unsigned char *pData1 = new unsigned char[lwidth*lheight];
	img1.GrayEqualize(pData1, img1.pBuffer[0], lwidth, lheight);
	unsigned char *pData2 = new unsigned char[lwidth*lheight];
	img2.GrayEqualize(pData2, img2.pBuffer[0], rwidth, rheight);
	// �ر�Ӱ��
	img1.Destroy();		img2.Destroy();
	// ��ʼ����ƥ��
	long nmatch;
	float *lx, *ly, *rx, *ry;
	lx = ly = rx = ry = NULL;
	MatchMethod m_match;
	m_match.MatchBaseSIFT_Data(pData1, pData2, lwidth, lheight, rwidth, 
		rheight, nmatch, lx, ly, rx, ry, leftx, lefty, rightx, righty);
	m_match.SaveMatchBaseEnvi(ptspath, nmatch, lx, ly, rx, ry);

	// ɾ���ڴ�
	if(pData1!=NULL)	delete []pData1;	pData1 = NULL;
	if(pData2!=NULL)	delete []pData2;	pData2 = NULL;
	if(lx!=NULL)		delete []lx;		lx = NULL;
	if(ly!=NULL)		delete []ly;		ly = NULL;
	if(rx!=NULL)		delete []rx;		rx = NULL;
	if(ry!=NULL)		delete []ry;		ry = NULL;
}*/
