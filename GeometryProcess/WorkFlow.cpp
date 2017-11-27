#include "WorkFlow.h"
//#include "MatchMethod.h"
//#include "PhaseCorrelation.h"
//#include <io.h>


//////////////////////////////////////////////////////////////////////////
// 构造/析构函数
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
// 从一个文件夹中按照通配符搜索指定类型的文件
//////////////////////////////////////////////////////////
bool WorkFlow::AddFeaFileInDirectory(string dirpath, vector<string> &filelist)
{
	long handle;									//用于查找的句柄
	struct _finddata_t fileinfo;					//文件信息的结构体
	handle=_findfirst(dirpath.c_str(), &fileinfo);	//第一次查找
	string path = dirpath.substr(0, dirpath.find_last_of("\\//")+1);
	if(-1==handle)
	{
		printf("通配符查找失败!\n");
		return false;
	}
	string temp = path;
	filelist.push_back(temp.append(fileinfo.name));
	//循环查找其他符合的文件，直到找不到其他的为止
	while(!_findnext(handle, &fileinfo))               
	{
		temp = path;
		filelist.push_back(temp.append(fileinfo.name));
	}
	//别忘了关闭句柄
	_findclose(handle);
	return true;
}


//////////////////////////////////////
// 功能：前方交会构建DSM
// 输入:
//		int num:			模型个数
//		GeoModel **model：	成像模型,个数为num,且第一个为参考模型
//		string *imgpath:	成像模型对应的影像,个数为num,用于匹配
//		string *ptspath:	粗配准文件,个数为num-1,用于预测
//		string outpath:		输出的密集三维点
// 返回值：
//		void
////////////////////////////////////// 
void WorkFlow::GenerateDSM(int num, GeoModel **model, string *imgpath, string *ptspath, 
						string dempath, string outpath, int win, string datumname)
{
	// 打开影像
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
	// 构建仿射预测格网
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
	// 逐像素配准
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
	// 开始循环
	for(long j=lt_y; j<rb_y; j++)
	{
		float rx, ry, offx, offy;
		for(long i=lt_x; i<rb_x; i++)
		{
			bool isValid = true;
			m_x[0] = i;		m_y[0] = j;
			// 密集匹配
			for(int k=1; k<num; k++)
			{
				trans[k].GetValueBaseAffine(m_x[0], m_y[0], m_x[k], m_y[k]);
				rx = m_x[k];		ry = m_y[k];
				isValid *= m_Corr.GetPixel(&(img[0]), &(img[k]), m_x[0], m_y[0], rx, ry, win);
				if(isValid==false)	break;
				if(fabs(rx-m_x[k])>10||fabs(ry-m_y[k])>10)	break;
				// 反投影
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
			// 前方交会
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
	// 关闭影像
	for(int i=0; i<num; i++)
		img[i].Destroy();
	delete []img;		img = NULL;
	delete []trans;		trans  = NULL;
}


////////////////////////////////////////////////////////////
// 以ENVI格式输入匹配结果
// 输入：
//		string ptspath:		输入匹配点文件路径
//		long &nmatch:		输出匹配点的个数
//		float* &lx:			左影像x坐标
//		float* &ly：		左影像y坐标
//		float* &rx：		右影像x坐标
//		float* &ry：		右影像y坐标
// 返回值:
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
// 以ENVI格式输出匹配结果
// 输入：
//		string ptspath:		输出匹配点文件路径
//		long nmatch:		输出匹配点的个数
//		float* lx:			左影像x坐标
//		float* ly：			左影像y坐标
//		float* rx：			右影像x坐标
//		float* ry：			右影像y坐标
// 返回值:
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
// 功能：影像相互投影
// 输入:
//		string imgsrc：		基准影像路径
//		GeoModel *modelsrc: 基准影像成像模型
//      string imgpro:		投影影像路径
//		GeoModel *modelpro:	投影影像成像模型
//		string imgdem:		参考DEM
//		string imgout:		输出影像路径
// 返回值：
//		void
//////////////////////////////////////
void WorkFlow::ImageProject(string imgsrc, GeoModel *modelsrc, string imgpro, 
							GeoModel *modelpro, string imgdem, string imgout)
{
	long i, j;
	GeoReadImage m_dem, m_src, m_pro, m_out;
	// 打开基准影像
	m_src.Open(imgsrc, GA_ReadOnly);
	for(i=0; i<m_src.m_nBands; i++)
	{
		m_src.ReadBlock(0, 0, m_src.m_xRasterSize, m_src.m_yRasterSize, i, m_src.pBuffer[i]);
	}
	// 打开投影影像
	m_pro.Open(imgpro, GA_ReadOnly);
	for(i=0; i<m_pro.m_nBands; i++)
	{
		m_pro.ReadBlock(0, 0, m_pro.m_xRasterSize, m_pro.m_yRasterSize, i, m_pro.pBuffer[i]);
	}
	// 打开dem影像
	m_dem.Open(imgdem, GA_ReadOnly);
	for(i=0; i<m_dem.m_nBands; i++)
	{
		m_dem.ReadBlock(0, 0, m_dem.m_xRasterSize, m_dem.m_yRasterSize, i, m_dem.pBuffer[i]);
	}
	// 打开输出影像m_src.m_xRasterSize, m_src.m_yRasterSize
	m_out.New(imgout, "GTiff", m_pro.m_BandType, m_src.m_xRasterSize, m_src.m_yRasterSize, m_pro.m_nBands);
	m_out.Destroy();
	m_out.Open(imgout, GA_Update);
	for(i=0; i<m_out.m_nBands; i++)
	{
		m_out.SetBuffer(0, 0, m_out.m_xRasterSize, m_out.m_yRasterSize, m_out.pBuffer[i]);
	}
	///////////////////////////////////////
	// 迭代高程求解
	///////////////////////////////////////
#pragma omp parallel 
	{  
#pragma omp for
		for(i=0; i<m_out.m_yRasterSize; i++)		// 沿轨
		{
#pragma omp parallel 
			{  
#pragma omp for
				for(j=0; j<m_out.m_xRasterSize; j++)	// 垂轨
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
	// 输出影像
	for(i=0; i<m_out.m_nBands; i++)
	{
		m_out.WriteBlock(0, 0, m_out.m_xRasterSize, m_out.m_yRasterSize, i, m_out.pBuffer[i]);
	}
	// 释放内存
	m_src.Destroy();	m_pro.Destroy();
	m_dem.Destroy();	m_out.Destroy();
}


//////////////////////////////////////
// 功能：高程迭代得到对应地面坐标
// 输入:
//		GeoModel *model:	成像模型
//		double i,j:			影像像素坐标
//		GeoReadImage *pDEM：DEM影像
// 输出：
//		double &lat：		纬度
//		double &lon:		经度
//		double &h：			高程
// 返回值：
//		bool
//////////////////////////////////////
bool WorkFlow::EleItera(GeoModel *model, double i, double j, GeoReadImage *pDEM, 
						double &lat, double &lon, double &h)
{
	double horigin = -9999;
	// 进行DEM内插
	int num = 0;
	while(fabs(horigin-h)>1) 
	{
		horigin = h;
		model->FromXY2LatLon(i, j, h, lat, lon);
		// 转化为经纬度
		lat = lat*180/PI;
		lon = lon*180/PI;
		// 获得高程值
		h = pDEM->GetDataValue(lon, lat, -9999, 0, false);
		num++;
		if(num>10||h<=-9999) return false;    // 注意需要加上-9999,否则边界赋值会继续循环造成区域不对应
	} 
	// 转化为弧度
	lat = lat*PI/180;
	lon = lon*PI/180;
	return true;
}


//////////////////////////////////////
// 功能：根据影像块进行特征匹配
// 输入:
//		string leftpath:	左影像路径
//		string rightpath:	右影像路径
//		long lwidth,lheight:左影像匹配的区域长度和宽度
//		long rwidth,rheight:右影像匹配的区域长度和宽度
//		string ptspath:		输出的pts文件路径
//		long leftx, lefty:	左影像匹配的区域左上角坐标
//		long rightx,righty:	右影像匹配的区域左上角坐标
// 返回值：
//		void
//////////////////////////////////////
void WorkFlow::MatchBase_Data(string leftpath, string rightpath, long lwidth, long lheight, long rwidth, long rheight, 
							  string ptspath, long leftx, long lefty, long rightx, long righty)
{
	// 初始化影像对象并打开影像
	GeoReadImage img1, img2;
	if(img1.Open(leftpath, GA_ReadOnly)==false)
	{
		printf("左影像打开失败!\n");
		return;
	}
	if(img2.Open(rightpath, GA_ReadOnly)==false)
	{
		printf("右影像打开失败!\n");
		return;
	}
	// 读取影像数据
	img1.SetBuffer(0, 0, lwidth, lheight, img1.pBuffer[0]);
	img1.ReadBlock(leftx, lefty, lwidth, lheight, 0, img1.pBuffer[0]);
	img2.SetBuffer(0, 0, rwidth, rheight, img2.pBuffer[0]);
	img2.ReadBlock(rightx, righty, rwidth, rheight, 0, img2.pBuffer[0]);
	// 进行影像均衡化
	unsigned char *pData1 = new unsigned char[lwidth*lheight];
	img1.GrayEqualize(pData1, img1.pBuffer[0], lwidth, lheight);
	unsigned char *pData2 = new unsigned char[lwidth*lheight];
	img2.GrayEqualize(pData2, img2.pBuffer[0], rwidth, rheight);
	// 关闭影像
	img1.Destroy();		img2.Destroy();
	// 开始进行匹配
	long nmatch;
	float *lx, *ly, *rx, *ry;
	lx = ly = rx = ry = NULL;
	MatchMethod m_match;
	m_match.MatchBaseSIFT_Data(pData1, pData2, lwidth, lheight, rwidth, 
		rheight, nmatch, lx, ly, rx, ry, leftx, lefty, rightx, righty);
	m_match.SaveMatchBaseEnvi(ptspath, nmatch, lx, ly, rx, ry);

	// 删除内存
	if(pData1!=NULL)	delete []pData1;	pData1 = NULL;
	if(pData2!=NULL)	delete []pData2;	pData2 = NULL;
	if(lx!=NULL)		delete []lx;		lx = NULL;
	if(ly!=NULL)		delete []ly;		ly = NULL;
	if(rx!=NULL)		delete []rx;		rx = NULL;
	if(ry!=NULL)		delete []ry;		ry = NULL;
}*/
