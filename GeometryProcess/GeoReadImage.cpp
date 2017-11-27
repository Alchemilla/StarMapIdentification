#include "GeoReadImage.h"

// 建立影像金字塔
bool GeoReadImage::CreatePyramids(string path, int ratio)
{
	// 如果没有金字塔,就建立金字塔
	GDALAllRegister();
//	CPLSetConfigOption("USE_RRD","YES");    //创建Erdas格式的字塔文件
	// 打开数据文件
	GDALDatasetH     hDataset;  
	hDataset = GDALOpen(path.c_str(), GA_ReadOnly);   
	GDALDriverH hDriver = GDALGetDatasetDriver(hDataset);  
	const char* pszDriver = GDALGetDriverShortName(hDriver);  
	//如果文件是Erdas的img或者PCI的pix格式，创建内金字塔，其他的创建外金字塔  
//	if (EQUAL(pszDriver, "HFA") || EQUAL(pszDriver, "PCIDSK"))  
//	{  
//		GDALClose(hDataset);    
//		hDataset = GDALOpen(path.c_str(), GA_Update);  
//	}  
	if( hDataset == NULL )  
	{  
		printf("打开图像失败，请检查图像是否存在或文件是否是图像文件！");
		return false;  
	}  
	// 获得影像基本信息
	long iWidth = GDALGetRasterXSize(hDataset);  
	long iHeigh = GDALGetRasterYSize(hDataset);  
	long iPixelNum = iWidth * iHeigh;    //图像中的总像元个数  
	long iTopNum = 4096;                 //顶层金字塔大小，64*64  
	long iCurNum = iPixelNum / pow(ratio, 2.0);  
	int anLevels[1024] = { 0 };  
	long nLevelCount = 0;                //金字塔级数  
	// 计算金字塔级数，从第二级到顶层  
	do    
	{  
		anLevels[nLevelCount] = static_cast<int>(pow(1.0*ratio, nLevelCount+1));  
		nLevelCount ++;  
		iCurNum /= pow(ratio, 2.0);  
	} while (iCurNum > iTopNum);  
	const char *pszResampling = "CUBIC"; //采样方式  "NEAREST", "GAUSS", "CUBIC", "AVERAGE", "MODE", "AVERAGE_MAGPHASE" or "NONE"
	// 开始建立金字塔
	if (nLevelCount > 0 && GDALBuildOverviews( hDataset,pszResampling, nLevelCount, anLevels, 0, NULL, NULL, NULL ) != CE_None )  
	{  
		printf("创建金字塔失败!");
		return false;  
	}  
	// 清理
	GDALClose(hDataset);  
	GDALDestroyDriverManager(); 
	return true;
}

//////////////////////////////////////////////////////////
// 计算缩放比率
//////////////////////////////////////////////////////////
void GeoReadImage::CalRatio(int &ratio)
{
	int width0 = GDALGetRasterBandXSize(GDALGetOverview(pBand[0], 0));
	int num = pBand[0]->GetOverviewCount();
	int temp0 = int(1.0*m_xRasterSize / width0 + 0.5);
	int temp1 = 1;
	int temp2 = temp0;
	for (int i = 1; i <= num; i++)
	{
		if ((ratio >= temp1) && (ratio < temp2))
		{
			if ((ratio - temp1) >= (temp2 - ratio))
			{
				ratio = temp2;
				return;
			}
			else
			{
				ratio = temp1;
				return;
			}
		}
		temp1 *= temp0;
		temp2 *= temp0;
	}
	ratio = pow(temp0, 1.0*num);
}



/////////////////////////////////////////////////////////
// 获取最大接边矩形坐标
/////////////////////////////////////////////////////////
bool GeoReadImage::GetRectangle(double *lat, double *lon, int extend, long &sample,
	long &line, long &width, long &height, double res, int &ratio)
{
	ratio = res;
	// °的单位
	if (m_xsize < 0.01)
	{
		ratio = ratio / 6378000 * 180 / PI;
	}
	if (ratio / m_xsize >= 2.0)
	{
		ratio = ratio / m_xsize;
		CalRatio(ratio);
	}
	else
	{
		ratio = 1;
	}
	// 没有地理参考系则无法运行
	if (m_xsize == 0 && m_ysize == 0)
		return false;
	double sample1, line1, width1, height1;
	GridInterBL2PixelRect(lon, lat, sample1, line1, width1, height1);
	int extend2 = extend * 2;
	sample1 -= extend;	line1 -= extend;
	width1 += extend2;	height1 += extend2;
	// 存储需要获取的影像像素范围
	OGRLinearRing ringtemp;
	long tempx = sample1 + width1;
	long tempy = line1 + height1;
	ringtemp.addPoint(sample1, line1);
	ringtemp.addPoint(tempx, line1);
	ringtemp.addPoint(tempx, tempy);
	ringtemp.addPoint(sample1, tempy);
	ringtemp.closeRings();
	OGRPolygon polygon;
	polygon.addRing(&ringtemp);
	//如果没有相交部分,则无法获取像素
	if (polygon.Intersect(&m_PolygonImage) == 0)
	{
		m_errorinfo = ("获取影像内容失败");
		return false;
	}
	OGRPolygon *polygontemp = (OGRPolygon *)polygon.Intersection(&m_PolygonImage); // 求交
	m_LinearRing = polygontemp->getExteriorRing();
	// 获取范围
	OGRPoint point[4];
	m_LinearRing->getPoint(0, &point[0]);
	m_LinearRing->getPoint(1, &point[1]);
	m_LinearRing->getPoint(2, &point[2]);
	m_LinearRing->getPoint(3, &point[3]);
	minx = min(min(point[0].getX(), point[1].getX()), min(point[2].getX(), point[3].getX()));
	miny = min(min(point[0].getY(), point[1].getY()), min(point[2].getY(), point[3].getY()));
	maxx = max(max(point[0].getX(), point[1].getX()), max(point[2].getX(), point[3].getX())) - 1;
	maxy = max(max(point[0].getY(), point[1].getY()), max(point[2].getY(), point[3].getY())) - 1;
	sample = minx;			line = miny;
	width = maxx - minx - 1;	height = maxy - miny - 1;

	return true;
}





GeoReadImage::GeoReadImage(void)
{
	poDataset = NULL;   // 数据集对象
	pXGrid = NULL;      // 格网数据X
	pYGrid = NULL;      // 格网数据Y
	pXGridinv = NULL;   // 反向格网数据X
	pYGridinv = NULL;   // 反向格网数据Y
	pBand = NULL;       // 波段对象指针
	pBuffer = NULL;		// 存储当前获取的数据区内容
	Destroy();
}

GeoReadImage::~GeoReadImage(void)
{
	Destroy();
}

// 清理各类内存函数
void GeoReadImage::Destroy()
{
	m_isopen = false;
	// 清空数据集对象
	if(poDataset!=NULL)
	{
		GDALClose(poDataset);
		poDataset = NULL;
	}
	// 清空波段对象
	if(pBand!=NULL)
	{
		for(int i=0; i<m_nBands; i++)
		{
			if(pBand[i] != NULL)
			{
				pBand[i] = NULL;   //不需要Del,只起指向作用
			}
		}
		delete []pBand;
		pBand = NULL;
	}
	// 清空格网数据
	if(pXGrid != NULL)
	{
		delete pXGrid;
		pXGrid = NULL;
	}
	if(pYGrid != NULL)
	{
		delete pYGrid;
		pYGrid = NULL;
	}
	if(pXGridinv != NULL)
	{
		delete pXGridinv;
		pXGridinv = NULL;
	}
	if(pYGridinv != NULL)
	{
		delete pYGridinv;
		pYGridinv = NULL;
	}
	// 清理存储当前获取的数据区内容
	if(pBuffer!=NULL)
	{
		for(int i=0;i<m_nBands;i++)
		{
			if(pBuffer[i] != NULL)
			{
				delete (pBuffer[i]);
				pBuffer[i] = NULL;
			}
		}
		delete []pBuffer;
		pBuffer = NULL;
	}
	//GDALDestroyDriverManager(); 
}


//////////////////////////////////////////////////////////
// 打开影像,专门为CE5设的
/////////////////////////////////////////////////////////
bool GeoReadImage::Open(string lpFilePath, unsigned char *&pData)
{
	Destroy();
	GDALAllRegister();		         // 注册驱动
	poDataset=(GDALDataset *)GDALOpen(lpFilePath.c_str(), GA_ReadOnly);  // GA_ReadOnly,GA_Update
	// 图像打开失败情况的处理
	if (poDataset==NULL)	                   
	{
		GDALClose(poDataset);         // 删除数据集对象
		poDataset=NULL;
		m_errorinfo = "Failed to Open Dataset!";
		return false;
	}
	// 如果打开成功，则获取图像的一些信息
	if(poDataset!=NULL)
	{
		// 获取图像基本信息
		m_nBands = poDataset->GetRasterCount();  // 获得波段数
		if(m_nBands<1)
		{
			GDALClose(poDataset);                // 删除数据集对象
		    poDataset=NULL;
			m_errorinfo = "The Number of Bands is Less Than One!";
			return false;
		}
		// 图像长宽
		m_xRasterSize = poDataset->GetRasterXSize();  // 获取数据的宽度
		m_yRasterSize = poDataset->GetRasterYSize();  // 获取数据的高度
		///////////////////////////////////////
		//以下开始读取波段信息
		unsigned char *pSrc = new unsigned char[m_xRasterSize*m_yRasterSize*3];
		if(pData!=NULL)		delete []pData;		pData = NULL;
		pData = new unsigned char[m_xRasterSize*m_yRasterSize];
		poDataset->RasterIO(GF_Read, 0, 0, m_xRasterSize, m_yRasterSize, pSrc, m_xRasterSize, m_yRasterSize, GDT_Byte, 3, NULL, 0, 0, 0);
		long index;
		long index1 = m_xRasterSize*m_yRasterSize;
		long index2 = index1*2;
		for(int i=0; i<m_yRasterSize; i++)
		{
			for(int j=0; j<m_xRasterSize; j++)
			{
				index = i*m_xRasterSize+j;
				pData[index] = (unsigned char)(0.10454f*pSrc[index]+0.60581f*pSrc[index1+index]+0.28965f*pSrc[index2+index]);
			}
		}
//		FILE *fp = fopen("D:\\1.raw", "wb");
//		fwrite(pData, sizeof(unsigned char), index1, fp);
//		fclose(fp);
		if(pSrc!=NULL)		delete []pSrc;		pSrc = NULL;
	}
	return true;
}


//////////////////////////////////////////////////////////
// 打开影像，并且获取影像的各种信息
// 打开图像函数，主要获得图像的WKT信息、长宽等各种信息
//		double  isCreateGird:	是否建立地理格网,默认值为true
/////////////////////////////////////////////////////////
bool GeoReadImage::Open(string lpFilePath, GDALAccess gdalaccess, bool isCreateGird)
{
	Destroy();
	GDALAllRegister();		         // 注册驱动
	poDataset=(GDALDataset *)GDALOpen(lpFilePath.c_str(), gdalaccess);  // GA_ReadOnly,GA_Update
	// 图像打开失败情况的处理
	if (poDataset==NULL)	                   
	{
		GDALClose(poDataset);         // 删除数据集对象
		poDataset=NULL;
		m_errorinfo = "Failed to Open Dataset!";
		return false;
	}
	// 如果打开成功，则获取图像的一些信息
	if(poDataset!=NULL)
	{
		// 获取图像基本信息
		m_nBands = poDataset->GetRasterCount();  // 获得波段数
		if(m_nBands<1)
		{
			GDALClose(poDataset);                // 删除数据集对象
		    poDataset=NULL;
			m_errorinfo = "The Number of Bands is Less Than One!";
			return false;
		}
		// 图像长宽
		m_xRasterSize = poDataset->GetRasterXSize();  // 获取数据的宽度
		m_yRasterSize = poDataset->GetRasterYSize();  // 获取数据的高度
		// 存储影像像素范围
		m_RingImage.empty();         m_PolygonImage.empty();
		m_RingImage.addPoint(0, 0);              
		m_RingImage.addPoint(m_xRasterSize, 0);
		m_RingImage.addPoint(m_xRasterSize, m_yRasterSize);  
		m_RingImage.addPoint(0, m_yRasterSize);  // 注意要按顺序加载
		m_RingImage.closeRings();
		m_PolygonImage.addRing(&m_RingImage);
		// 地理参考信息
		double adfGeoTransform[6];
		char *ProjectionRefTemp = (char*)poDataset->GetProjectionRef();
		if(strcmp(ProjectionRefTemp, "")==0)   
		{
			isCreateGird = false;
			m_errorinfo = "The Image have not GeoProjectionRef!";
		}
		if( poDataset->GetGeoTransform(adfGeoTransform)==CE_None) 
		{
			m_leftx = adfGeoTransform[0];
			m_lefty = adfGeoTransform[3];             // 左上角x，y坐标
			m_xsize = adfGeoTransform[1];
			m_ysize = adfGeoTransform[5];             // 横纵像素距离
		}
		else
		{
			isCreateGird = false;
			m_errorinfo = "The Image have not GeoTransform!";
		}	
		///////////////////////////////////////
		//以下开始读取波段信息
		pBand = new GDALRasterBand*[m_nBands];
		pBuffer = new void*[m_nBands];
		for(int i=0; i<m_nBands; i++)
		{
			// 将波段指针对象指向波段
			pBand[i] = poDataset->GetRasterBand(i+1); 
			pBuffer[i] = NULL;
		}
		m_BandType = pBand[0]->GetRasterDataType();     // 获得数据类型(GDAL格式)
		pBand[0]->GetBlockSize(&m_xBlock, &m_yBlock);	// 获得块大小
		if((m_xBlock<=1)||(m_yBlock<=1))
		{
			m_xBlock = 1;
			m_yBlock = 1;
		}
		if(m_BandType==GDT_Unknown || m_BandType==GDT_CInt16 || m_BandType==GDT_CInt32 ||
		   m_BandType==GDT_CFloat32 || m_BandType==GDT_CFloat64)
		{
			GDALClose(poDataset);                     // 删除数据集对象
		    poDataset=NULL;
			m_errorinfo = "Don't care about Unknown or Complex Data!";
			return false;
		}	
		// 对影像构建地理格网
		if(isCreateGird == true)
		{
			CreateGrid();
		}
		// 统计最大值最小值
		ComputeStatistics();
		/////////////////////////////////////////////////////////
		// 获得每个像素的字节大小
		/////////////////////////////////////////////////////////
		switch(m_BandType)
		{
		case GDT_Byte:
			{
				m_SizeInBytes = sizeof(unsigned char)*m_nBands;  
				break;
			}
		case GDT_UInt16:
			{
				m_SizeInBytes = sizeof(unsigned short)*m_nBands;   
				break;
			}
		case GDT_Int16:
			{
				m_SizeInBytes = sizeof(short)*m_nBands;   
				break;
			}
		case GDT_UInt32:
			{
				m_SizeInBytes = sizeof(unsigned long)*m_nBands;   
				break;
			}
		case GDT_Int32:
			{
				m_SizeInBytes = sizeof(long)*m_nBands;  
				break;
			}
		case GDT_Float32:
			{
				m_SizeInBytes = sizeof(float)*m_nBands;  
				break;
			}
		case GDT_Float64:
			{
				m_SizeInBytes = sizeof(double)*m_nBands;  
				break;
			}
		default:
			{
				m_errorinfo = "Unknown Format!";
				return false;
			}
		}
		m_isopen = true;	// 打开成功
	}
	return true;
}


//////////////////////////////////////////////////////////
//创建新影像
// lpFilePath: 影像保存路径
// pFormat：   影像驱动名称
//////////////////////////////////////////////////////////
bool GeoReadImage::New(string lpFilePath, string pFormat, GDALDataType type, long xsize, long ysize, int bandnum)
{
	m_xRasterSize = xsize;
	m_yRasterSize = ysize;
	m_BandType = type;
	m_nBands = bandnum;

	Destroy();
	GDALAllRegister();	// 注册驱动

	GDALDriver *pDriver;
	char** pMetadata;
	char **papszOptions = NULL;
	// 查看驱动是否支持DCAP_CREATE或DCAP_CREATECOPY
	pDriver = GetGDALDriverManager()->GetDriverByName(pFormat.c_str());
	if(pDriver==NULL)   
	{
		m_errorinfo = "Failed to Get GDAL Driver Manager!";
		return false;
	}
	pMetadata=pDriver->GetMetadata();
	if(!CSLFetchBoolean(pMetadata,GDAL_DCAP_CREATE,FALSE))
	{
		m_errorinfo = "GDAL don't Support this Format!";
		return false;
	}
	// 开始创建
	if(strcmp(pFormat.c_str(), "GTiff")==0)   
	{
//		papszOptions = CSLSetNameValue(papszOptions, "TILED", "YES");
//		papszOptions = CSLSetNameValue(papszOptions, "BLOCKXSIZE", "64");
//		if(xsize>5000||ysize>5000)
//			papszOptions = CSLSetNameValue(papszOptions, "COMPRESS", "JPEG");
		// papszOptions = CSLSetNameValue(papszOptions, "PREDICTOR", "3");
		// GDAL 2.1开始支持
		// papszOptions = CSLSetNameValue(papszOptions, "NUM_THREADS", "ALL_CPUS");
	}
	if(strcmp(pFormat.c_str(), "HFA")==0)   
	{
		papszOptions = CSLSetNameValue(papszOptions, "COMPRESSED", "YES");
	}
	poDataset = pDriver->Create(lpFilePath.c_str(), xsize, ysize, m_nBands, type, papszOptions);
	pBand = new GDALRasterBand*[m_nBands];
	pBuffer = new void*[m_nBands];
	for(int i=0; i<m_nBands; i++)
	{
		// 将波段指针对象指向波段
		pBand[i] = poDataset->GetRasterBand(i+1); 
		pBuffer[i] = NULL;
	}
	m_isopen = true;	// 创建成功

	return 0;
}


//////////////////////////////////////////////////////////
//创建新影像
// lpFilePath: 影像保存路径
// pFormat：   影像驱动名称
//////////////////////////////////////////////////////////
bool GeoReadImage::New(string lpFilePath, string pFormat, GDALDataType type, long xsize, long ysize,
	int bandnum, double *adfGeoTransform, string projectName)
{
	m_xRasterSize = xsize;
	m_yRasterSize = ysize;
	m_BandType = type;
	m_nBands = bandnum;

	Destroy();
	GDALAllRegister();	// 注册驱动
	GDALDriver *pDriver;
	char** pMetadata;
	char **papszOptions = NULL;
	pDriver = GetGDALDriverManager()->GetDriverByName(pFormat.c_str());
	if (pDriver == NULL)
	{
		m_errorinfo = ("驱动信息获取失败");
		return false;
	}
	pMetadata = pDriver->GetMetadata();
	if (!CSLFetchBoolean(pMetadata, GDAL_DCAP_CREATE, FALSE))
	{
		m_errorinfo =  ("此格式GDAL不支持写入");
		return false;
	}
	// 开始创建
	papszOptions = CSLSetNameValue(papszOptions, "BLOCKXSIZE", "256");
	poDataset = pDriver->Create(lpFilePath.c_str(), xsize, ysize, m_nBands, type, papszOptions);
	if (adfGeoTransform != NULL)
	{
		poDataset->SetGeoTransform(adfGeoTransform);
	}
	poDataset->SetProjection(projectName.c_str());
	pBand = new GDALRasterBand*[m_nBands];
	pBuffer = new void*[m_nBands];
	for (int i = 0; i < m_nBands; i++)
	{
		// 将波段指针对象指向波段
		pBand[i] = poDataset->GetRasterBand(i + 1);
		pBuffer[i] = NULL;
	}
	ratio = 1;

	return 0;
}

//////////////////////////////////////////////////////////
// 判断区域是否具有参考数据
// 参数的含义
// long nPosX:     图像数据块的起始横坐标
// long nPosY:     图像数据块的起始纵坐标
// long OffsetX:   图像数据块的宽度
// long OffsetY:   图像数据块的高度
/////////////////////////////////////////////////////////
bool GeoReadImage::IsInRefData(double nPosX, double nPosY, double OffsetX, double OffsetY)
{
	if(OffsetX<0||OffsetY<0)
		return false;
	// 存储需要获取的影像像素范围
	OGRLinearRing ringtemp; 
	double tempx = nPosX+OffsetX;
	double tempy = nPosY+OffsetY;
	ringtemp.addPoint(nPosX, nPosY);                  
	ringtemp.addPoint(tempx, nPosY);
	ringtemp.addPoint(tempx, tempy);   
	ringtemp.addPoint(nPosX, tempy);  // 注意要按顺序加载
	ringtemp.closeRings();
	OGRPolygon polygon;
	polygon.addRing(&ringtemp);
	//如果没有相交部分,则无法获取像素
	if(polygon.Intersect(&m_PolygonImage)==0)
	{
		m_errorinfo = "Failed to Get Image Context!";
		return false;
	}
	return true;
}


//////////////////////////////////////////////////////////
// 拷贝影像信息到此影像上
//////////////////////////////////////////////////////////
void GeoReadImage::CopyImageInfo(GeoReadImage m_image)
{
	// 整个数据集的最大、最小、平均数值(针对DEM)
	m_maxvalue = m_image.m_maxvalue;
	m_minvalue = m_image.m_minvalue;
	m_meanvalue = m_image.m_meanvalue;
	// 每个像素的字节大小
	m_SizeInBytes = m_image.m_SizeInBytes;
	// 输出状态信息
	m_errorinfo = m_image.m_errorinfo;
	// 数据集对象指针
	poDataset = m_image.poDataset;
	// 波段对象指针
	pBand = m_image.pBand;
	// 图像的长度(x)和高度(y)
	m_xRasterSize = m_image.m_xRasterSize;
	m_yRasterSize = m_image.m_yRasterSize;
	// 图像波段数,目前只处理单波段的数据
	m_nBands = m_image.m_nBands;
	// 左上角x、y坐标，以及每个像素的长度和宽度
	m_leftx = m_image.m_leftx;
	m_lefty = m_image.m_lefty;
	m_xsize = m_image.m_xsize;
	m_ysize = m_image.m_ysize;
	// 波段数据类型
	m_BandType = m_image.m_BandType;
	// 存储整个影像的像素范围(环)
	m_RingImage = m_image.m_RingImage;
	// 存储整个影像的像素范围(面)
	m_PolygonImage = m_image.m_PolygonImage;
	// 存储当前获取的数据区范围
	m_nPosX = m_image.m_nPosX;
	m_nPosY = m_image.m_nPosY;
	m_nPosX2 = m_image.m_nPosX2;
	m_nPosY2 = m_image.m_nPosY2;
	m_OffsetX = m_image.m_OffsetX;
	m_OffsetY = m_image.m_OffsetY;
	// 存储当前数据区域的像素范围
	m_LinearRing = m_image.m_LinearRing;
	// 划分的格网数,注意每一个格网点都有可能不一样
	m_nx = m_image.m_nx;
	m_ny = m_image.m_ny;
	// 格网划分后的分辨率	
	m_dXResolution = m_image.m_dXResolution;
	m_dYResolution = m_image.m_dYResolution;
	// 存储格网的具体值
	pXGrid = m_image.pXGrid;
	pYGrid = m_image.pYGrid;
	// 存储格网的最大和最小经纬度
	maxx = m_image.maxx;
	minx = m_image.minx;
	maxy = m_image.maxy;
	miny = m_image.miny;
	//非DEM区域数值,一般都是-9999
	m_invalidatedem = m_image.m_invalidatedem;
	if(m_nBands>0)
	{
		pBuffer = new void*[m_nBands];
		for(int i=0; i<m_nBands; i++)
		{
			pBuffer[i] = NULL;
		}
	}                  
}


//////////////////////////////////////////////////////////
// 获得影像的块信息
// 参数的含义
// long nPosX:     图像数据块的起始横坐标
// long nPosY:     图像数据块的起始纵坐标
// long OffsetX:   图像数据块的宽度
// long OffsetY:   图像数据块的高度
// int  indexBand: 索引的波段号
// void* &pBuffer:  数据块指针，接收图像数据(注意记得加&)
// float ratiox:	宽度缩放比例
// float ratioy:	高度缩放比例
/////////////////////////////////////////////////////////
bool GeoReadImage::ReadBlock(long nPosX, long nPosY,unsigned long OffsetX,unsigned long OffsetY, 
							int indexBand, void* &pData, float ratiox, float ratioy)
{	
	if(indexBand>m_nBands-1)
	{
		printf("The Band Index is Greater than the Band Num!");
		return false;
	}
	// 存储需要获取的影像像素范围
	OGRLinearRing ringtemp; 
	long tempx = nPosX+OffsetX;
	long tempy = nPosY+OffsetY;
	ringtemp.addPoint(nPosX, nPosY);                  
	ringtemp.addPoint(tempx, nPosY);
	ringtemp.addPoint(tempx, tempy);   
	ringtemp.addPoint(nPosX, tempy);  // 注意要按顺序加载
	ringtemp.closeRings();
	OGRPolygon polygon;
	polygon.addRing(&ringtemp);
	//如果没有相交部分,则无法获取像素
	if(polygon.Intersect(&m_PolygonImage)==0)
	{
		m_errorinfo = "Failed to Get Image Context!";
		return false;
	}
	OGRPolygon *polygontemp = (OGRPolygon *)polygon.Intersection(&m_PolygonImage); // 求交
	m_LinearRing = polygontemp->getExteriorRing();
	// 获取范围
	OGRPoint point[4];
	m_LinearRing->getPoint(0,&point[0]);
	m_LinearRing->getPoint(1,&point[1]); 
	m_LinearRing->getPoint(2,&point[2]);
	m_LinearRing->getPoint(3,&point[3]);
	m_nPosX = nPosX = min(min(point[0].getX(),point[1].getX()),min(point[2].getX(),point[3].getX()));
	m_nPosY = nPosY = min(min(point[0].getY(),point[1].getY()),min(point[2].getY(),point[3].getY()));
	m_nPosX2 = max(max(point[0].getX(),point[1].getX()),max(point[2].getX(),point[3].getX()))-1;
	m_nPosY2 = max(max(point[0].getY(),point[1].getY()),max(point[2].getY(),point[3].getY()))-1;
	m_OffsetX = OffsetX = m_nPosX2-m_nPosX+1;
	m_OffsetY = OffsetY = m_nPosY2-m_nPosY+1;
	// 清理存储当前获取的数据区内容
	if(pData != NULL)
	{
		delete []pData;
		pData = NULL;
	}
	/////////////////////////////////////////////////////////
	// 数据读取
	/////////////////////////////////////////////////////////
	switch(m_BandType)
	{
	case GDT_Byte:
		{
			pData = new unsigned char[((int)(OffsetX/ratiox))*((int)(OffsetY/ratioy))];
			if(pBand[indexBand]->RasterIO(GF_Read, nPosX, nPosY, OffsetX, OffsetY, pData, OffsetX/ratiox, OffsetY/ratioy, GDT_Byte, 0, 0) != CE_None)
			{
				m_errorinfo = "Failed to Read Image Block!";
				return false;
			}
			break;
		}
	case GDT_UInt16:
		{
			pData = new unsigned short[((int)(OffsetX/ratiox))*((int)(OffsetY/ratioy))];
			if(pBand[indexBand]->RasterIO(GF_Read, nPosX, nPosY, OffsetX, OffsetY, pData, OffsetX/ratiox, OffsetY/ratioy, GDT_UInt16, 0, 0) != CE_None)
			{
				m_errorinfo = "Failed to Read Image Block!";
				return false;
			}
			break;
		}
	case GDT_Int16:
		{
			pData = new short[((int)(OffsetX/ratiox))*((int)(OffsetY/ratioy))];
			if(pBand[indexBand]->RasterIO(GF_Read, nPosX, nPosY, OffsetX, OffsetY, pData, OffsetX/ratiox, OffsetY/ratioy, GDT_Int16, 0, 0) != CE_None)
			{
				m_errorinfo = "Failed to Read Image Block!";
				return false;
			}
			break;
		}
	case GDT_UInt32:
		{
			pData = new unsigned long[((int)(OffsetX/ratiox))*((int)(OffsetY/ratioy))];
			if(pBand[indexBand]->RasterIO(GF_Read, nPosX, nPosY, OffsetX, OffsetY, pData, OffsetX/ratiox, OffsetY/ratioy, GDT_UInt32, 0, 0) != CE_None)
			{
				m_errorinfo = "Failed to Read Image Block!";
				return false;
			}
			break;
		}
	case GDT_Int32:
		{
			pData = new long[((int)(OffsetX/ratiox))*((int)(OffsetY/ratioy))];
			if(pBand[indexBand]->RasterIO(GF_Read, nPosX, nPosY, OffsetX, OffsetY, pData, OffsetX/ratiox, OffsetY/ratioy, GDT_Int32, 0, 0) != CE_None)
			{
				m_errorinfo = "Failed to Read Image Block!";
				return false;
			}
			break;
		}
	case GDT_Float32:
		{
			pData = new float[((int)(OffsetX/ratiox))*((int)(OffsetY/ratioy))];
			if(pBand[indexBand]->RasterIO(GF_Read, nPosX, nPosY, OffsetX, OffsetY, pData, OffsetX/ratiox, OffsetY/ratioy, GDT_Float32, 0, 0) != CE_None)
			{
				m_errorinfo = "Failed to Read Image Block!";
				return false;
			}
			break;
		}
	case GDT_Float64:
		{
			pData = new double[((int)(OffsetX/ratiox))*((int)(OffsetY/ratioy))];
			if(pBand[indexBand]->RasterIO(GF_Read, nPosX, nPosY, OffsetX, OffsetY, pData, OffsetX/ratiox, OffsetY/ratioy, GDT_Float64, 0, 0) != CE_None)
			{
				m_errorinfo = "Failed to Read Image Block!";
				return false;
			}
			break;
		}
	default:
		{
			m_errorinfo = "Unknown Format!";
			return false;
		}
	}
	return true;
}

//////////////////////////////////////////////////////////
// 分配空间
// 参数的含义
// long nPosX:     图像数据块的起始横坐标
// long nPosY:     图像数据块的起始纵坐标
// long OffsetX:   图像数据块的宽度
// long OffsetY:   图像数据块的高度
// int  indexBand: 索引的波段号
// void* &pBuffer:  数据块指针，接收图像数据(注意记得加&)
/////////////////////////////////////////////////////////
bool GeoReadImage::SetBuffer(long nPosX, long nPosY, unsigned long OffsetX, unsigned long OffsetY, void* &pData)
{
	// 存储需要获取的影像像素范围
	OGRLinearRing ringtemp; 
	long tempx = nPosX+OffsetX;
	long tempy = nPosY+OffsetY;
	ringtemp.addPoint(nPosX, nPosY);                  
	ringtemp.addPoint(tempx, nPosY);
	ringtemp.addPoint(tempx, tempy);   
	ringtemp.addPoint(nPosX, tempy);  // 注意要按顺序加载
	ringtemp.closeRings();
	OGRPolygon polygon;
	polygon.addRing(&ringtemp);
	//如果没有相交部分,则无法获取像素
	if(polygon.Intersect(&m_PolygonImage)==0)
	{
		m_errorinfo = "Failed to Get Image Context!";
		return false;
	}
	OGRPolygon *polygontemp = (OGRPolygon *)polygon.Intersection(&m_PolygonImage); // 求交
	m_LinearRing = polygontemp->getExteriorRing();
	// 获取范围
	OGRPoint point[4];
	m_LinearRing->getPoint(0,&point[0]);
	m_LinearRing->getPoint(1,&point[1]); 
	m_LinearRing->getPoint(2,&point[2]);
	m_LinearRing->getPoint(3,&point[3]);
	m_nPosX = nPosX = min(min(point[0].getX(),point[1].getX()),min(point[2].getX(),point[3].getX()));
	m_nPosY = nPosY = min(min(point[0].getY(),point[1].getY()),min(point[2].getY(),point[3].getY()));
	m_nPosX2 = max(max(point[0].getX(),point[1].getX()),max(point[2].getX(),point[3].getX()))-1;
	m_nPosY2 = max(max(point[0].getY(),point[1].getY()),max(point[2].getY(),point[3].getY()))-1;
	m_OffsetX = OffsetX = m_nPosX2-m_nPosX+1;
	m_OffsetY = OffsetY = m_nPosY2-m_nPosY+1;
	// 清理存储当前获取的数据区内容
	if(pData != NULL)
	{
		delete pData;
		pData = NULL;
	}
	/////////////////////////////////////////////////////////
	// 数据读取
	/////////////////////////////////////////////////////////
	switch(m_BandType)
	{
	case GDT_Byte:
		{
			pData = new unsigned char[OffsetX*OffsetY];
			memset(pData, 0, sizeof(unsigned char)*OffsetX*OffsetY);
			break;
		}
	case GDT_UInt16:
		{
			pData = new unsigned short[OffsetX*OffsetY];
			memset(pData, 0, sizeof(unsigned short)*OffsetX*OffsetY);
			break;
		}
	case GDT_Int16:
		{
			pData = new short[OffsetX*OffsetY];
			memset(pData, 0, sizeof(short)*OffsetX*OffsetY);
			break;
		}
	case GDT_UInt32:
		{
			pData = new unsigned long[OffsetX*OffsetY];
			memset(pData, 0, sizeof(unsigned long)*OffsetX*OffsetY);
			break;
		}
	case GDT_Int32:
		{
			pData = new long[OffsetX*OffsetY];
			memset(pData, 0, sizeof(long)*OffsetX*OffsetY);
			break;
		}
	case GDT_Float32:
		{
			pData = new float[OffsetX*OffsetY];
			memset(pData, 0, sizeof(float)*OffsetX*OffsetY);
			break;
		}
	case GDT_Float64:
		{
			pData = new double[OffsetX*OffsetY];
			memset(pData, 0, sizeof(double)*OffsetX*OffsetY);
			break;
		}
	default:
		{
			m_errorinfo = "Unknown Format!";
			return false;
		}
	}
	return true;
}


//////////////////////////////////////////////////////////
// 设置影像的块信息
// 参数的含义
// long nPosX:     图像数据块的起始横坐标
// long nPosY:     图像数据块的起始纵坐标
// long OffsetX:   图像数据块的宽度
// long OffsetY:   图像数据块的高度
// void* pBuffer:  数据块指针，接收图像数据
/////////////////////////////////////////////////////////
bool GeoReadImage::WriteBlock(long nPosX, long nPosY,unsigned long OffsetX,unsigned long OffsetY, int indexBand, void* pData)
{	
	// 起始点是否越界判断
	if(nPosX<0 || nPosX>m_xRasterSize || nPosY<0 || nPosY>m_yRasterSize)
	{
		m_errorinfo = "The Start Overlap the Boundary!";
		return false;
	}
	// 偏移点是否越界判断
	if(OffsetX<0 || nPosX+OffsetX>m_xRasterSize || OffsetY<0 || nPosY+OffsetY>m_yRasterSize)
	{
		m_errorinfo = "The End Overlap the Boundary!";
		return false;
	}
	// 偏移量是否为0的处理及判断
	if(0 == OffsetX)
		OffsetX = m_xRasterSize - nPosX;
	if(0 == OffsetY)
		OffsetY = m_yRasterSize - nPosY;
	// 传入数据指针是否为空的判断
	if(NULL==pData)
	{
		m_errorinfo = "The Pointer is NULL!";
		return false;
	}
	/////////////////////////////////////////////////////////
	// 数据写入
	/////////////////////////////////////////////////////////
	switch(m_BandType)
	{
	case GDT_Byte:
		{
			if(pBand[indexBand]->RasterIO(GF_Write, nPosX, nPosY, OffsetX, OffsetY, pData, OffsetX, OffsetY, GDT_Byte, 0, 0) != CE_None)
			{
				m_errorinfo = "Failed to Write Data!";
				return false;
			}
			break;
		}
	case GDT_UInt16:
		{
			unsigned short *temp11 = (unsigned short*)pData;
			if(pBand[indexBand]->RasterIO(GF_Write, nPosX, nPosY, OffsetX, OffsetY, pData, OffsetX, OffsetY, GDT_UInt16, 0, 0) != CE_None)
			{
				m_errorinfo = "Failed to Write Data!";
				return false;
			}
			break;
		}
	case GDT_Int16:
		{
			if(pBand[indexBand]->RasterIO(GF_Write, nPosX, nPosY, OffsetX, OffsetY, pData, OffsetX, OffsetY, GDT_Int16, 0, 0) != CE_None)
			{
				m_errorinfo = "Failed to Write Data!";
				return false;
			}
			break;
		}
	case GDT_UInt32:
		{
			if(pBand[indexBand]->RasterIO(GF_Write, nPosX, nPosY, OffsetX, OffsetY, pData, OffsetX, OffsetY, GDT_UInt32, 0, 0) != CE_None)
			{
				m_errorinfo = "Failed to Write Data!";
				return false;
			}
			break;
		}
	case GDT_Int32:
		{
			if(pBand[indexBand]->RasterIO(GF_Write, nPosX, nPosY, OffsetX, OffsetY, pData, OffsetX, OffsetY, GDT_Int32, 0, 0) != CE_None)
			{
				m_errorinfo = "Failed to Write Data!";
				return false;
			}
			break;
		}
	case GDT_Float32:
		{
			if(pBand[indexBand]->RasterIO(GF_Write, nPosX, nPosY, OffsetX, OffsetY, pData, OffsetX, OffsetY, GDT_Float32, 0, 0) != CE_None)
			{
				m_errorinfo = "Failed to Write Data!";
				return false;
			}
			break;
		}
	case GDT_Float64:
		{
			if(pBand[indexBand]->RasterIO(GF_Write, nPosX, nPosY, OffsetX, OffsetY, pData, OffsetX, OffsetY, GDT_Float64, 0, 0) != CE_None)
			{
				m_errorinfo = "Failed to Write Data!";
				return false;
			}
			break;
		}
	default:
		{
			m_errorinfo = "Failed to Write Data!";
			return false;
		}
	}
	poDataset->FlushCache();
	return 0;
}


/////////////////////////////////////////////////////////
// 获得指定位置的像素值
// bool isPixel:	 是否输入为像素值
// double x:		 如果isPixel为ture,则为像素x坐标
//					 如果isPixel为false,则为经度,单位为度
// double y:         如果isPixel为ture,则为像素y坐标
//					 如果isPixel为false,则为纬度,单位为度
// double invValue:  无效值的赋值
/////////////////////////////////////////////////////////
double GeoReadImage::GetDataValue(double x, double y, double invValue, int indexBand, bool isPixel)
{
	// 通过格网将经纬度坐标转化成像素坐标
	if(isPixel==false)
	{
		double xtemp, ytemp;
		GridInterBL2Pixel(x, y, xtemp, ytemp);
		x = xtemp;
		y = ytemp;
	}

	// 越界判断
	if((x>=m_nPosX)&&(x<m_nPosX2)&&(y>=m_nPosY)&&(y<m_nPosY2))
	{
		unsigned char *temp1 = NULL;
		unsigned short *temp2 = NULL;
		short *temp3 = NULL;
		unsigned long *temp4 = NULL;
		long *temp5 = NULL;
		float *temp6 = NULL;
		double *temp7 = NULL;
		// 左上角点
		double index;
		index = 1.0*long(y-m_nPosY)*m_OffsetX + long(x-m_nPosX);
		double delx = x - long(x);      // sample
		double dely = y - long(y);      // line
		double coef[4];
		coef[0] = (1-dely)*(1-delx);
		coef[1] = (1-dely)*delx;
		coef[2] = dely*(1-delx);
		coef[3] = dely*delx;
		double value = 0;
		switch(m_BandType)
		{
		case GDT_Byte:
			{
				temp1 = (unsigned char*)pBuffer[indexBand];
				value = coef[0]*temp1[((long)index)] + coef[1]*temp1[((long)index)+1]
				      + coef[2]*temp1[((long)index)+m_OffsetX] + coef[3]*temp1[((long)index)+m_OffsetX+1];
				break;
			}
		case GDT_UInt16:
			{
				temp2 = (unsigned short*)pBuffer[indexBand];
				value = coef[0]*temp2[((long)index)] + coef[1]*temp2[((long)index)+1]
				      + coef[2]*temp2[((long)index)+m_OffsetX] + coef[3]*temp2[((long)index)+m_OffsetX+1];
				break;
			}
		case GDT_Int16:
			{
				temp3 = (short*)pBuffer[indexBand];
				value = coef[0]*temp3[((long)index)] + coef[1]*temp3[((long)index)+1]
				      + coef[2]*temp3[((long)index)+m_OffsetX] + coef[3]*temp3[((long)index)+m_OffsetX+1];
				break;
			}
		case GDT_UInt32:
			{
				temp4 = (unsigned long*)pBuffer[indexBand];
				value = coef[0]*temp4[((long)index)] + coef[1]*temp4[((long)index)+1]
				      + coef[2]*temp4[((long)index)+m_OffsetX] + coef[3]*temp4[((long)index)+m_OffsetX+1];
				break;
			}
		case GDT_Int32:
			{
				temp5 = (long*)pBuffer[indexBand];
				value = coef[0]*temp5[((long)index)] + coef[1]*temp5[((long)index)+1]
				      + coef[2]*temp5[((long)index)+m_OffsetX] + coef[3]*temp5[((long)index)+m_OffsetX+1];
				break;
			}
		case GDT_Float32:
			{
				temp6 = (float*)pBuffer[indexBand];
				value = coef[0]*temp6[((long)index)] + coef[1]*temp6[((long)index)+1]
				      + coef[2]*temp6[((long)index)+m_OffsetX] + coef[3]*temp6[((long)index)+m_OffsetX+1];
				break;
			}
		case GDT_Float64:
			{
				temp7 = (double*)pBuffer[indexBand];
				value = coef[0]*temp7[((long)index)] + coef[1]*temp7[((long)index)+1]
				      + coef[2]*temp7[((long)index)+m_OffsetX] + coef[3]*temp7[((long)index)+m_OffsetX+1];
				break;
			}
		default:
			{
				value = invValue;
				break;
			}
		}
		// 月球不适用
//		if(value<-500) value = invValue;
		if(value<-20000) value = invValue;
		return value;
	}
	else
	{
		return invValue;
	}
}


/////////////////////////////////////////////////////////
// 获得整数像素的值
// double x:		 像素x坐标
// double y:         像素y坐标
// double invValue:  无效值的赋值
/////////////////////////////////////////////////////////
double GeoReadImage::GetDataValue(long x, long y, double invValue, int indexBand)
{
	
	// 越界判断
	if((x>=m_nPosX)&&(x<m_nPosX2)&&(y>=m_nPosY)&&(y<m_nPosY2))
	{
		// 左上角点
		double index = 1.0*long(y-m_nPosY)*m_OffsetX + long(x-m_nPosX);
		double value = 0;
		switch(m_BandType)
		{
		case GDT_Byte:
			{
				value = ((unsigned char*)pBuffer[indexBand])[(long)index];
				break;
			}
		case GDT_UInt16:
			{
				value = ((unsigned short*)pBuffer[indexBand])[(long)index];
				break;
			}
		case GDT_Int16:
			{
				value = ((short*)pBuffer[indexBand])[(long)index];
				break;
			}
		case GDT_UInt32:
			{
				value = ((unsigned long*)pBuffer[indexBand])[(long)index];
				break;
			}
		case GDT_Int32:
			{
				value = ((long*)pBuffer[indexBand])[(long)index];
				break;
			}
		case GDT_Float32:
			{
				value = ((float*)pBuffer[indexBand])[(long)index];
				break;
			}
		case GDT_Float64:
			{
				value = ((double*)pBuffer[indexBand])[(long)index];
				break;
			}
		default:
			{
				value = invValue;
				break;
			}
		}
		// 月球不适用
		if(value<-500) value = invValue;
//		if(value<-20000) value = invValue;
		return value;
	}
	else
	{
		return invValue;
	}
}


/////////////////////////////////////////////////////////
// 从Buffer获得指定范围的的像素值
//	void *pData:		输入的Buffer数据
//	int lt_x:			范围的左上角x坐标
//	int lt_y:			范围的左上角y坐标
//	int bufferW：		范围的宽度
//	int bufferH：		范围的高度
//	double *pOut：		得到的范围数据
// 返回值：
//	bool：				true说明读取成功
//						false说明读取失败
/////////////////////////////////////////////////////////
bool GeoReadImage::GetDataValueFromBuffer(void* pData, int lt_x, int lt_y, int bufferW, int bufferH, double* &pOut)
{
	// 越界判断
	if((lt_x>=0)&&(lt_x+bufferW<=m_OffsetX)&&(lt_y>=0)&&(lt_y+bufferH<=m_OffsetY))
	{
		switch(m_BandType)
		{
		case GDT_Byte:
			{
				for(int i=0; i<bufferW; i++)
				{
					for(int j=0; j<bufferH; j++)
					{
						pOut[j*bufferW+i] = ((unsigned char*)pData)[(lt_y+j)*m_OffsetX+lt_x+i];
					}
				}
				break;
			}
		case GDT_UInt16:
			{
				for(int i=0; i<bufferW; i++)
				{
					for(int j=0; j<bufferH; j++)
					{
						pOut[j*bufferW+i] = ((unsigned short*)pData)[(lt_y+j)*m_OffsetX+lt_x+i];
					}
				}
				break;
			}
		case GDT_Int16:
			{
				for(int i=0; i<bufferW; i++)
				{
					for(int j=0; j<bufferH; j++)
					{
						pOut[j*bufferW+i] = ((short*)pData)[(lt_y+j)*m_OffsetX+lt_x+i];
					}
				}
				break;
			}
		case GDT_UInt32:
			{
				for(int i=0; i<bufferW; i++)
				{
					for(int j=0; j<bufferH; j++)
					{
						pOut[j*bufferW+i] = ((unsigned long*)pData)[(lt_y+j)*m_OffsetX+lt_x+i];
					}
				}
				break;
			}
		case GDT_Int32:
			{
				for(int i=0; i<bufferW; i++)
				{
					for(int j=0; j<bufferH; j++)
					{
						pOut[j*bufferW+i] = ((long*)pData)[(lt_y+j)*m_OffsetX+lt_x+i];
					}
				}
				break;
			}
		case GDT_Float32:
			{
				for(int i=0; i<bufferW; i++)
				{
					for(int j=0; j<bufferH; j++)
					{
						pOut[j*bufferW+i] = ((float*)pData)[(lt_y+j)*m_OffsetX+lt_x+i];
					}
				}
				break;
			}
		case GDT_Float64:
			{
				for(int i=0; i<bufferW; i++)
				{
					for(int j=0; j<bufferH; j++)
					{
						pOut[j*bufferW+i] = ((double*)pData)[(lt_y+j)*m_OffsetX+lt_x+i];
					}
				}
				break;
			}
		default:
			{
				break;
			}
		}
		return true;
	}
	else
	{
		return false;
	}
}


/////////////////////////////////////////////////////////
// 获得指定位置的像素值,未先读取数据
// bool isPixel:	 是否输入为像素值
// double x:		 如果isPixel为ture,则为像素x坐标
//					 如果isPixel为false,则为经度,单位为度
// double y:         如果isPixel为ture,则为像素y坐标
//					 如果isPixel为false,则为纬度,单位为度
// double invValue:  无效值的赋值
/////////////////////////////////////////////////////////
double GeoReadImage::GetDataValueWithoutPrepare(double x, double y, double invValue, int indexBand, bool isPixel)
{
	// 通过格网将经纬度坐标转化成像素坐标
	if(isPixel==false)
	{
		double xtemp, ytemp;
		GridInterBL2Pixel(x, y, xtemp, ytemp);
		x = xtemp;
		y = ytemp;
	}

	// 越界判断
	if((x>=0)&&(x<m_xRasterSize-1)&&(y>=0)&&(y<m_yRasterSize-1))
	{
		unsigned char temp1[4];
		unsigned short temp2[4];
		short temp3[4];
		unsigned long temp4[4];
		long temp5[4];
		float temp6[4];
		double temp7[4];
		double delx = x - long(x);      // sample
		double dely = y - long(y);      // line
		double coef[4];
		coef[0] = (1-dely)*(1-delx);
		coef[1] = (1-dely)*delx;
		coef[2] = dely*(1-delx);
		coef[3] = dely*delx;
		double value = 0;
		switch(m_BandType)
		{
		case GDT_Byte:
			{
				if(pBand[indexBand]->RasterIO(GF_Read, long(x), long(y), 2, 2, temp1, 2, 2, GDT_Byte, 0, 0) != CE_None)
				{
					m_errorinfo = "Failed to Read Image Block!";
					return false;
				}
				value = coef[0]*temp1[0] + coef[1]*temp1[1] + coef[2]*temp1[2] + coef[3]*temp1[3];
				break;
			}
		case GDT_UInt16:
			{
				if(pBand[indexBand]->RasterIO(GF_Read, long(x), long(y), 2, 2, temp2, 2, 2, GDT_UInt16, 0, 0) != CE_None)
				{
					m_errorinfo = "Failed to Read Image Block!";
					return false;
				}
				value = coef[0]*temp2[0] + coef[1]*temp2[1] + coef[2]*temp2[2] + coef[3]*temp2[3];
				break;
			}
		case GDT_Int16:
			{
				if(pBand[indexBand]->RasterIO(GF_Read, long(x), long(y), 2, 2, temp3, 2, 2, GDT_Int16, 0, 0) != CE_None)
				{
					m_errorinfo = "Failed to Read Image Block!";
					return false;
				}
				value = coef[0]*temp3[0] + coef[1]*temp3[1] + coef[2]*temp3[2] + coef[3]*temp3[3];
				break;
			}
		case GDT_UInt32:
			{
				if(pBand[indexBand]->RasterIO(GF_Read, long(x), long(y), 2, 2, temp4, 2, 2, GDT_UInt32, 0, 0) != CE_None)
				{
					m_errorinfo = "Failed to Read Image Block!";
					return false;
				}
				value = coef[0]*temp4[0] + coef[1]*temp4[1] + coef[2]*temp4[2] + coef[3]*temp4[3];
				break;
			}
		case GDT_Int32:
			{
				if(pBand[indexBand]->RasterIO(GF_Read, long(x), long(y), 2, 2, temp5, 2, 2, GDT_Int32, 0, 0) != CE_None)
				{
					m_errorinfo = "Failed to Read Image Block!";
					return false;
				}
				value = coef[0]*temp5[0] + coef[1]*temp5[1] + coef[2]*temp5[2] + coef[3]*temp5[3];
				break;
			}
		case GDT_Float32:
			{
				if(pBand[indexBand]->RasterIO(GF_Read, long(x), long(y), 2, 2, temp6, 2, 2, GDT_Float32, 0, 0) != CE_None)
				{
					m_errorinfo = "Failed to Read Image Block!";
					return false;
				}
				value = coef[0]*temp6[0] + coef[1]*temp6[1] + coef[2]*temp6[2] + coef[3]*temp6[3];
				break;
			}
		case GDT_Float64:
			{
				if(pBand[indexBand]->RasterIO(GF_Read, long(x), long(y), 2, 2, temp7, 2, 2, GDT_Float64, 0, 0) != CE_None)
				{
					m_errorinfo = "Failed to Read Image Block!";
					return false;
				}
				value = coef[0]*temp7[0] + coef[1]*temp7[1] + coef[2]*temp7[2] + coef[3]*temp7[3];
				break;
			}
		default:
			{
				value = invValue;
				break;
			}
		}
		// 月球不适用
		if(value<-200) value = invValue;
//		if(value<-20000) value = invValue;
		return value;
	}
	else
	{
		return invValue;
	}
}


/////////////////////////////////////////////////////////
// 为数值指针的某个像素赋值
// long x:        像素x坐标
// long y:        像素y坐标
// double value:  赋的值
// void *pData:   需要被赋的数值指针
/////////////////////////////////////////////////////////
bool GeoReadImage::SetDataValue(long x, long y, double value, int indexBand)
{
	// 越界判断
	if((x>=m_nPosX)&&(x<=m_nPosX2)&&(y>=m_nPosY)&&(y<=m_nPosY2))
	{
		x = x - m_nPosX;
		y = y - m_nPosY;
		unsigned char *temp1 = NULL;
		unsigned short *temp2 = NULL;
		short *temp3 = NULL;
		unsigned long *temp4 = NULL;
		long *temp5 = NULL;
		float *temp6 = NULL;
		double *temp7 = NULL;
		switch(m_BandType)
		{
		case GDT_Byte:
			{
				temp1 = (unsigned char*)pBuffer[indexBand];
				temp1[y*m_OffsetX+x] = (unsigned char)value;
				break;
			}
		case GDT_UInt16:
			{
				temp2 = (unsigned short*)pBuffer[indexBand];
				temp2[y*m_OffsetX+x] = (unsigned short)value;
				break;
			}
		case GDT_Int16:
			{
				temp3 = (short*)pBuffer[indexBand];
				temp3[y*m_OffsetX+x] = (short)value;
				break;
			}
		case GDT_UInt32:
			{
				temp4 = (unsigned long*)pBuffer[indexBand];
				temp4[y*m_OffsetX+x] = (unsigned long)value;
				break;
			}
		case GDT_Int32:
			{
				temp5 = (long*)pBuffer[indexBand];
				temp5[y*m_OffsetX+x] = (long)value;
				break;
			}
		case GDT_Float32:
			{
				temp6 = (float*)pBuffer[indexBand];
				temp6[y*m_OffsetX+x] = (float)value;
				break;
			}
		case GDT_Float64:
			{
				temp7 = (double*)pBuffer[indexBand];
				temp7[y*m_OffsetX+x] = (double)value;
				break;
			}
		default:
			{
				return false;
			}
		}
		return true;
	}
	else
	{
		return false;
	}
}

//////////////////////////////////////////////////////////
// 统计整个波段的值
//////////////////////////////////////////////////////////
void GeoReadImage::ComputeStatistics()
{
	double minvalue, maxvalue;
	m_minvalue = FLT_MAX;
	m_maxvalue = FLT_MIN;
	for(int i=0; i<m_nBands; i++)
	{
		pBand[i]->ComputeStatistics(TRUE, &minvalue, &maxvalue, &m_meanvalue, NULL, NULL, NULL);
		if(minvalue<m_minvalue)		m_minvalue = minvalue;
		if(maxvalue>m_maxvalue)		m_maxvalue = maxvalue;
	}
}


//////////////////////////////////////////////////////////
// 对影像构建格网
//////////////////////////////////////////////////////////
bool GeoReadImage::CreateGrid()
{
	bool istrue = true;
	istrue *= CreateGridBL2Pixel();
	istrue *= CreateGridPixel2BL();
	return istrue;
}

// 构建从经纬度到像素的格网
bool GeoReadImage::CreateGridBL2Pixel()
{
	char *wgs84 = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.01745329251994328]]";
	double xx[4],yy[4];
	xx[0] = xx[2] = m_leftx;
	xx[1] = xx[3] = m_leftx + m_xsize*m_xRasterSize;
	yy[0] = yy[1] = m_lefty;
	yy[2] = yy[3] = m_lefty + m_ysize*m_yRasterSize;

	// 从影像定义的投影坐标转化到地理坐标(对于DEM可能多余,但是为了统一性还是进行一次)
	OGRSpatialReference m_sourceSRS,m_targetSRS;
	char *temp;
	temp = (char *)poDataset->GetProjectionRef();
	m_sourceSRS.importFromWkt(&temp);
	m_targetSRS.importFromWkt(&wgs84);

	OGRCoordinateTransformation *pTrans = OGRCreateCoordinateTransformation( &m_sourceSRS,&m_targetSRS);
	if(!pTrans->Transform(4,xx,yy)) 
	{
		delete []pTrans; pTrans = NULL;
		m_errorinfo = "Failed to Transform From Project that the Image Defines to Geographic Coordinates!";
		return false;
	}
	delete []pTrans; pTrans = NULL;

	// 以下求取其在经纬度地理坐标下的最大外接矩形
	maxx = max(max(xx[0],xx[1]),max(xx[2],xx[3]));
	minx = min(min(xx[0],xx[1]),min(xx[2],xx[3]));
	maxy = max(max(yy[0],yy[1]),max(yy[2],yy[3]));
	miny = min(min(yy[0],yy[1]),min(yy[2],yy[3]));

	m_nx = m_ny = 200;
	// 求取通过格网划分后的分辨率
	m_dXResolution = (maxx - minx)/m_nx;
	m_dYResolution = (miny - maxy)/m_ny;
	// 清空格网数据
	if(pXGrid != NULL) { delete pXGrid; pXGrid = NULL;}
	if(pYGrid != NULL) { delete pYGrid; pYGrid = NULL;}
	// 分配新空间
	pXGrid = new double[m_nx*m_ny];
	pYGrid = new double[m_nx*m_ny];
	// 构建格网
	long i,j,index=0;
	double east,north;
	for ( i=0; i<m_ny; ++i)
	{
		north = maxy + i*m_dYResolution;
		for ( j=0; j<m_nx; ++j)
		{
			east = minx + j*m_dXResolution;
			pXGrid[index] = east;
			pYGrid[index] = north;
			index++;
		}
	}
	// 从地理坐标转化到影像定义的投影坐标(对于DEM可能多余,但是为了统一性还是进行一次)
	pTrans = OGRCreateCoordinateTransformation(&m_targetSRS, &m_sourceSRS);
	if(!pTrans->Transform(index,pXGrid,pYGrid))
	{
		delete []pTrans; pTrans = NULL;
		m_errorinfo = "Failed to Transform From Geographic Coordinates to Project that the Image Defines!";
		return false;
	}
	delete []pTrans; pTrans = NULL;
	return true;
}
	
// 构建从像素到经纬度的格网
bool GeoReadImage::CreateGridPixel2BL()
{
	char *wgs84 = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.01745329251994328]]";
	OGRSpatialReference m_sourceSRS,m_targetSRS;
	char *temp;
	temp = (char *)poDataset->GetProjectionRef();
	m_sourceSRS.importFromWkt(&temp);
	m_targetSRS.importFromWkt(&wgs84);
	OGRCoordinateTransformation *pTrans = OGRCreateCoordinateTransformation(&m_sourceSRS,&m_targetSRS);
	m_nxinv = m_nyinv = 200;
	// 求取通过格网划分后的分辨率
	m_dXResolutioninv = m_xRasterSize/m_nxinv;
	m_dYResolutioninv = m_yRasterSize/m_nyinv;
	// 清空格网数据
	if(pXGridinv != NULL) { delete pXGridinv; pXGridinv = NULL;}
	if(pYGridinv != NULL) { delete pYGridinv; pYGridinv = NULL;}
	// 分配新空间
	pXGridinv = new double[m_nxinv*m_nyinv];
	pYGridinv = new double[m_nxinv*m_nyinv];
	// 构建格网
	long i,j,index=0;
	double east,north;
	for(i=0; i<m_nyinv; ++i)
	{
		north = m_lefty + i*m_dYResolutioninv*m_ysize;
		for(j=0; j<m_nxinv; ++j)
		{
			east = m_leftx + j*m_dXResolutioninv*m_xsize;
			pXGridinv[index] = east;
			pYGridinv[index] = north;
			index++;
		}
	}
	if(!pTrans->Transform(index,pXGridinv,pYGridinv))
	{
		m_errorinfo = "Failed to Transform From Project that the Image Defines to Geographic Coordinates!";
		return false;
	}
	delete []pTrans; pTrans = NULL;
	return true;
}


/////////////////////////////////////////////////////////
// 通过格网内插出对应点:从经纬度到像素
// 输入:
//   double lon:   经度
//   double lat:   纬度
// 输出：
//   double xto：  像素x坐标
//   double yto：  像素y坐标
/////////////////////////////////////////////////////////
bool GeoReadImage::GridInterBL2Pixel(double lon, double lat, double &xto, double &yto)
{
	// 简单处理，假设为格网
	double x = (lon - minx)/m_dXResolution;      // lon
	double y = (lat - maxy)/m_dYResolution;      // lat
	// 左上角点
	long xcor[4], ycor[4];
	if(x<0)				{ xcor[0] = xcor[3] = 0;       xcor[1] = xcor[2] = 1;     }
	else if(x>=m_nx-1)  { xcor[0] = xcor[3] = m_nx-2;  xcor[1] = xcor[2] = m_nx-1;}  // 注意要-1,否则可能越界造成读取影像很大
	else                
	{ 
		xcor[0] = long(x);     xcor[1] = long(x+1);
		xcor[2] = long(x+1);   xcor[3] = long(x);
	}
	if(y<0)				{ ycor[0] = ycor[1] = 0;	   ycor[2] = ycor[3] = 1;     }
	else if(y>=m_ny-1)  { ycor[0] = ycor[1] = m_ny-2;  ycor[2] = ycor[3] = m_ny-1;}  // 注意要-1,否则可能越界造成读取影像很大
	else                
	{ 
		ycor[0] = long(y);     ycor[1] = long(y);
		ycor[2] = long(y+1);   ycor[3] = long(y+1);
	}
	// 左上角点
	double index[4];
	index[0] = ycor[0]*m_nx + xcor[0];
	index[1] = ycor[1]*m_nx + xcor[1];
	index[2] = ycor[2]*m_nx + xcor[2];
	index[3] = ycor[3]*m_nx + xcor[3];

	double delx = x - xcor[0];      // lon
	double dely = y - ycor[0];      // lat
	double coef[4];
	coef[0] = (1-dely)*(1-delx);
	coef[1] = (1-dely)*delx;
	coef[2] = dely*delx;
	coef[3] = dely*(1-delx);
	// 获得大地坐标
	xto = 0;   yto = 0;
	for (int i=0; i<4; ++i)
	{
		xto += coef[i]*pXGrid[(long)index[i]];
		yto += coef[i]*pYGrid[(long)index[i]];	
	}
	// 获得像素坐标
	xto = (xto - m_leftx)/m_xsize;
	yto = (yto - m_lefty)/m_ysize;
	return true;
}


/////////////////////////////////////////////////////////
// 通过格网内插出对应点:从像素到经纬度
// 输入:
//   double xto：  像素x坐标
//   double yto：  像素y坐标
// 输出：
//   double lon:   经度
//   double lat:   纬度
/////////////////////////////////////////////////////////
bool GeoReadImage::GridInterPixel2BL(double xto, double yto, double &lon, double &lat)
{
	// 简单处理，假设为格网
	double x = xto/m_dXResolutioninv;
	double y = yto/m_dYResolutioninv;
	// 左上角点
	long xcor[4], ycor[4];
	if(x<0)				   { xcor[0] = xcor[3] = 0;			 xcor[1] = xcor[2] = 1;     }
	else if(x>=m_nxinv-1)  { xcor[0] = xcor[3] = m_nxinv-2;  xcor[1] = xcor[2] = m_nxinv-1;}  // 注意要-1,否则可能越界造成读取影像很大
	else                
	{ 
		xcor[0] = long(x);     xcor[1] = long(x+1);
		xcor[2] = long(x+1);   xcor[3] = long(x);
	}
	if(y<0)					{ ycor[0] = ycor[1] = 0;			ycor[2] = ycor[3] = 1;     }
	else if(y>=m_nyinv-1)   { ycor[0] = ycor[1] = m_nyinv-2;	ycor[2] = ycor[3] = m_nyinv-1;}  // 注意要-1,否则可能越界造成读取影像很大
	else                
	{ 
		ycor[0] = long(y);     ycor[1] = long(y);
		ycor[2] = long(y+1);   ycor[3] = long(y+1);
	}
	// 左上角点
	double index[4];
	index[0] = ycor[0]*m_nxinv + xcor[0];
	index[1] = ycor[1]*m_nxinv + xcor[1];
	index[2] = ycor[2]*m_nxinv + xcor[2];
	index[3] = ycor[3]*m_nxinv + xcor[3];

	double delx = x - xcor[0];      // lon
	double dely = y - ycor[0];      // lat
	double coef[4];
	coef[0] = (1-dely)*(1-delx);
	coef[1] = (1-dely)*delx;
	coef[2] = dely*delx;
	coef[3] = dely*(1-delx);
	// 获得经纬度坐标
	xto = 0;   yto = 0;
	for (int i=0; i<4; ++i)
	{
		xto += coef[i]*pXGridinv[(long)index[i]];
		yto += coef[i]*pYGridinv[(long)index[i]];	
	}
	lon = xto;
	lat = yto;
	return true;
}


/////////////////////////////////////////////////////////
// 通过格网内插出对应点:从经纬度到像素(针对矩形区域,获得矩形的像素框)
// 输入:
//   double *lon:   经度
//   double *lat:   纬度
// 输出：
//   long sample：  像素左上角x坐标
//   long line：    像素左上角y坐标
//   long width:    获取数据区域的宽度
//   long height:   获取数据区域的高度
/////////////////////////////////////////////////////////
bool GeoReadImage::GridInterBL2PixelRect(double *lon, double *lat, double &sample, double &line, double &width, double &height)
{
	double xtemp[4],ytemp[4];
	// 转化到像素坐标
	for(int i=0;i<4;i++)
	{
		GridInterBL2Pixel(lon[i], lat[i], xtemp[i], ytemp[i]);
	}
	// 以下求取其在像素坐标下的最大外接矩形
	double maxx, minx, maxy, miny;
	maxx = max(max(xtemp[0],xtemp[1]),max(xtemp[2],xtemp[3]));
	minx = min(min(xtemp[0],xtemp[1]),min(xtemp[2],xtemp[3]));
	maxy = max(max(ytemp[0],ytemp[1]),max(ytemp[2],ytemp[3]));
	miny = min(min(ytemp[0],ytemp[1]),min(ytemp[2],ytemp[3]));
	sample = (double)minx;
	line   = (double)miny;
	width  = (double)(maxx-minx);
	height = (double)(maxy-miny);
	return 0;
}


////////////////////////////////////////////////////////////	
//	灰度均衡函数
//	输入：
//		void *pSrc:		输入的原始影像数据,类型由datatype决定
//						如果不是unsigned char型,转化为unsigned char
//		int nWidth：	输入原始影像的宽度
//		int nHeight：	输入原始影像的高度
//	输出：
//		unsigned char* pDst：	输出的均衡化影像数据
////////////////////////////////////////////////////////////
void GeoReadImage::GrayEqualize(unsigned char* pDst, void* pSrc, int nWidth, int nHeight)
{
	switch(m_BandType)
	{
	case GDT_Byte:
		{
			GrayEqualizeTemplate(pDst, (unsigned char*)pSrc, nWidth, nHeight);
			break;
		}
	case GDT_UInt16:
		{
			GrayEqualizeTemplate(pDst, (unsigned short*)pSrc, nWidth, nHeight);
			break;
		}
	case GDT_Int16:
		{
			GrayEqualizeTemplate(pDst, (short*)pSrc, nWidth, nHeight);
			break;
		}
	case GDT_UInt32:
		{
			GrayEqualizeTemplate(pDst, (unsigned long*)pSrc, nWidth, nHeight);
			break;
		}
	case GDT_Int32:
		{
			GrayEqualizeTemplate(pDst, (long*)pSrc, nWidth, nHeight);
			break;
		}
	case GDT_Float32:
		{
			GrayEqualizeTemplate(pDst, (float*)pSrc, nWidth, nHeight);
			break;
		}
	case GDT_Float64:
		{
			GrayEqualizeTemplate(pDst, (double*)pSrc, nWidth, nHeight);
			break;
		}
	default:
		{
			return;
		}
	}
}

////////////////////////////////////////////////////////////	
//	灰度均衡函数内部使用,模板
//	输入：
//		void *pSrc:		输入的原始影像数据,类型由datatype决定
//						如果不是unsigned char型,转化为unsigned char
//		int nWidth：	输入原始影像的宽度
//		int nHeight：	输入原始影像的高度
//	输出：
//		unsigned char* pDst：	输出的均衡化影像数据
////////////////////////////////////////////////////////////
template <class T>
void GeoReadImage::GrayEqualizeTemplate(unsigned char* pDst, T* pSrc, int nWidth, int nHeight)
{
	if(!pSrc||!pDst)
	{
		return;
	}
	long pixelnum = nWidth*nHeight;
	double valuetemp = m_maxvalue - m_minvalue;
	if(valuetemp==0)
		return;
	// 计算各灰度值个数
	for(long i=0; i<pixelnum; i++)
	{
		//pDst[i] = pSrc[i];
		pDst[i] = (pSrc[i]-m_minvalue)/valuetemp*255;
	}

/*	// 灰度映射表
	unsigned char map[256];
	long lCounts[256];
	memset(lCounts, 0, sizeof(long)*256);
	long pixelnum = nWidth*nHeight;
	double valuetemp = m_maxvalue - m_minvalue;
	// 计算各灰度值个数
	for(long i=0; i<pixelnum; i++)
	{
		pSrc[i] = (pSrc[i]-m_minvalue)/valuetemp*255;
		lCounts[(unsigned char)pSrc[i]]++;
	}
	// 保存运算中的临时值
	long lTemp;
	for(long i=0; i<256; i++)
	{
		lTemp = 0;
		for(int j=0; j<=i; j++)
			lTemp += lCounts[j];
		map[i] = (unsigned char)(lTemp*255.0f/valuetemp);
	}
	// 变换后的值直接在映射表中查找
	for(long i=0; i<pixelnum; i++)
	{
		pDst[i] = map[(unsigned char)pSrc[i]];
	}*/
}

