#include "GeoReadImage.h"

// ����Ӱ�������
bool GeoReadImage::CreatePyramids(string path, int ratio)
{
	// ���û�н�����,�ͽ���������
	GDALAllRegister();
//	CPLSetConfigOption("USE_RRD","YES");    //����Erdas��ʽ�������ļ�
	// �������ļ�
	GDALDatasetH     hDataset;  
	hDataset = GDALOpen(path.c_str(), GA_ReadOnly);   
	GDALDriverH hDriver = GDALGetDatasetDriver(hDataset);  
	const char* pszDriver = GDALGetDriverShortName(hDriver);  
	//����ļ���Erdas��img����PCI��pix��ʽ�������ڽ������������Ĵ����������  
//	if (EQUAL(pszDriver, "HFA") || EQUAL(pszDriver, "PCIDSK"))  
//	{  
//		GDALClose(hDataset);    
//		hDataset = GDALOpen(path.c_str(), GA_Update);  
//	}  
	if( hDataset == NULL )  
	{  
		printf("��ͼ��ʧ�ܣ�����ͼ���Ƿ���ڻ��ļ��Ƿ���ͼ���ļ���");
		return false;  
	}  
	// ���Ӱ�������Ϣ
	long iWidth = GDALGetRasterXSize(hDataset);  
	long iHeigh = GDALGetRasterYSize(hDataset);  
	long iPixelNum = iWidth * iHeigh;    //ͼ���е�����Ԫ����  
	long iTopNum = 4096;                 //�����������С��64*64  
	long iCurNum = iPixelNum / pow(ratio, 2.0);  
	int anLevels[1024] = { 0 };  
	long nLevelCount = 0;                //����������  
	// ����������������ӵڶ���������  
	do    
	{  
		anLevels[nLevelCount] = static_cast<int>(pow(1.0*ratio, nLevelCount+1));  
		nLevelCount ++;  
		iCurNum /= pow(ratio, 2.0);  
	} while (iCurNum > iTopNum);  
	const char *pszResampling = "CUBIC"; //������ʽ  "NEAREST", "GAUSS", "CUBIC", "AVERAGE", "MODE", "AVERAGE_MAGPHASE" or "NONE"
	// ��ʼ����������
	if (nLevelCount > 0 && GDALBuildOverviews( hDataset,pszResampling, nLevelCount, anLevels, 0, NULL, NULL, NULL ) != CE_None )  
	{  
		printf("����������ʧ��!");
		return false;  
	}  
	// ����
	GDALClose(hDataset);  
	GDALDestroyDriverManager(); 
	return true;
}

//////////////////////////////////////////////////////////
// �������ű���
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
// ��ȡ���ӱ߾�������
/////////////////////////////////////////////////////////
bool GeoReadImage::GetRectangle(double *lat, double *lon, int extend, long &sample,
	long &line, long &width, long &height, double res, int &ratio)
{
	ratio = res;
	// ��ĵ�λ
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
	// û�е���ο�ϵ���޷�����
	if (m_xsize == 0 && m_ysize == 0)
		return false;
	double sample1, line1, width1, height1;
	GridInterBL2PixelRect(lon, lat, sample1, line1, width1, height1);
	int extend2 = extend * 2;
	sample1 -= extend;	line1 -= extend;
	width1 += extend2;	height1 += extend2;
	// �洢��Ҫ��ȡ��Ӱ�����ط�Χ
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
	//���û���ཻ����,���޷���ȡ����
	if (polygon.Intersect(&m_PolygonImage) == 0)
	{
		m_errorinfo = ("��ȡӰ������ʧ��");
		return false;
	}
	OGRPolygon *polygontemp = (OGRPolygon *)polygon.Intersection(&m_PolygonImage); // ��
	m_LinearRing = polygontemp->getExteriorRing();
	// ��ȡ��Χ
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
	poDataset = NULL;   // ���ݼ�����
	pXGrid = NULL;      // ��������X
	pYGrid = NULL;      // ��������Y
	pXGridinv = NULL;   // �����������X
	pYGridinv = NULL;   // �����������Y
	pBand = NULL;       // ���ζ���ָ��
	pBuffer = NULL;		// �洢��ǰ��ȡ������������
	Destroy();
}

GeoReadImage::~GeoReadImage(void)
{
	Destroy();
}

// ��������ڴ溯��
void GeoReadImage::Destroy()
{
	m_isopen = false;
	// ������ݼ�����
	if(poDataset!=NULL)
	{
		GDALClose(poDataset);
		poDataset = NULL;
	}
	// ��ղ��ζ���
	if(pBand!=NULL)
	{
		for(int i=0; i<m_nBands; i++)
		{
			if(pBand[i] != NULL)
			{
				pBand[i] = NULL;   //����ҪDel,ֻ��ָ������
			}
		}
		delete []pBand;
		pBand = NULL;
	}
	// ��ո�������
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
	// ����洢��ǰ��ȡ������������
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
// ��Ӱ��,ר��ΪCE5���
/////////////////////////////////////////////////////////
bool GeoReadImage::Open(string lpFilePath, unsigned char *&pData)
{
	Destroy();
	GDALAllRegister();		         // ע������
	poDataset=(GDALDataset *)GDALOpen(lpFilePath.c_str(), GA_ReadOnly);  // GA_ReadOnly,GA_Update
	// ͼ���ʧ������Ĵ���
	if (poDataset==NULL)	                   
	{
		GDALClose(poDataset);         // ɾ�����ݼ�����
		poDataset=NULL;
		m_errorinfo = "Failed to Open Dataset!";
		return false;
	}
	// ����򿪳ɹ������ȡͼ���һЩ��Ϣ
	if(poDataset!=NULL)
	{
		// ��ȡͼ�������Ϣ
		m_nBands = poDataset->GetRasterCount();  // ��ò�����
		if(m_nBands<1)
		{
			GDALClose(poDataset);                // ɾ�����ݼ�����
		    poDataset=NULL;
			m_errorinfo = "The Number of Bands is Less Than One!";
			return false;
		}
		// ͼ�񳤿�
		m_xRasterSize = poDataset->GetRasterXSize();  // ��ȡ���ݵĿ��
		m_yRasterSize = poDataset->GetRasterYSize();  // ��ȡ���ݵĸ߶�
		///////////////////////////////////////
		//���¿�ʼ��ȡ������Ϣ
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
// ��Ӱ�񣬲��һ�ȡӰ��ĸ�����Ϣ
// ��ͼ��������Ҫ���ͼ���WKT��Ϣ������ȸ�����Ϣ
//		double  isCreateGird:	�Ƿ����������,Ĭ��ֵΪtrue
/////////////////////////////////////////////////////////
bool GeoReadImage::Open(string lpFilePath, GDALAccess gdalaccess, bool isCreateGird)
{
	Destroy();
	GDALAllRegister();		         // ע������
	poDataset=(GDALDataset *)GDALOpen(lpFilePath.c_str(), gdalaccess);  // GA_ReadOnly,GA_Update
	// ͼ���ʧ������Ĵ���
	if (poDataset==NULL)	                   
	{
		GDALClose(poDataset);         // ɾ�����ݼ�����
		poDataset=NULL;
		m_errorinfo = "Failed to Open Dataset!";
		return false;
	}
	// ����򿪳ɹ������ȡͼ���һЩ��Ϣ
	if(poDataset!=NULL)
	{
		// ��ȡͼ�������Ϣ
		m_nBands = poDataset->GetRasterCount();  // ��ò�����
		if(m_nBands<1)
		{
			GDALClose(poDataset);                // ɾ�����ݼ�����
		    poDataset=NULL;
			m_errorinfo = "The Number of Bands is Less Than One!";
			return false;
		}
		// ͼ�񳤿�
		m_xRasterSize = poDataset->GetRasterXSize();  // ��ȡ���ݵĿ��
		m_yRasterSize = poDataset->GetRasterYSize();  // ��ȡ���ݵĸ߶�
		// �洢Ӱ�����ط�Χ
		m_RingImage.empty();         m_PolygonImage.empty();
		m_RingImage.addPoint(0, 0);              
		m_RingImage.addPoint(m_xRasterSize, 0);
		m_RingImage.addPoint(m_xRasterSize, m_yRasterSize);  
		m_RingImage.addPoint(0, m_yRasterSize);  // ע��Ҫ��˳�����
		m_RingImage.closeRings();
		m_PolygonImage.addRing(&m_RingImage);
		// ����ο���Ϣ
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
			m_lefty = adfGeoTransform[3];             // ���Ͻ�x��y����
			m_xsize = adfGeoTransform[1];
			m_ysize = adfGeoTransform[5];             // �������ؾ���
		}
		else
		{
			isCreateGird = false;
			m_errorinfo = "The Image have not GeoTransform!";
		}	
		///////////////////////////////////////
		//���¿�ʼ��ȡ������Ϣ
		pBand = new GDALRasterBand*[m_nBands];
		pBuffer = new void*[m_nBands];
		for(int i=0; i<m_nBands; i++)
		{
			// ������ָ�����ָ�򲨶�
			pBand[i] = poDataset->GetRasterBand(i+1); 
			pBuffer[i] = NULL;
		}
		m_BandType = pBand[0]->GetRasterDataType();     // �����������(GDAL��ʽ)
		pBand[0]->GetBlockSize(&m_xBlock, &m_yBlock);	// ��ÿ��С
		if((m_xBlock<=1)||(m_yBlock<=1))
		{
			m_xBlock = 1;
			m_yBlock = 1;
		}
		if(m_BandType==GDT_Unknown || m_BandType==GDT_CInt16 || m_BandType==GDT_CInt32 ||
		   m_BandType==GDT_CFloat32 || m_BandType==GDT_CFloat64)
		{
			GDALClose(poDataset);                     // ɾ�����ݼ�����
		    poDataset=NULL;
			m_errorinfo = "Don't care about Unknown or Complex Data!";
			return false;
		}	
		// ��Ӱ�񹹽��������
		if(isCreateGird == true)
		{
			CreateGrid();
		}
		// ͳ�����ֵ��Сֵ
		ComputeStatistics();
		/////////////////////////////////////////////////////////
		// ���ÿ�����ص��ֽڴ�С
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
		m_isopen = true;	// �򿪳ɹ�
	}
	return true;
}


//////////////////////////////////////////////////////////
//������Ӱ��
// lpFilePath: Ӱ�񱣴�·��
// pFormat��   Ӱ����������
//////////////////////////////////////////////////////////
bool GeoReadImage::New(string lpFilePath, string pFormat, GDALDataType type, long xsize, long ysize, int bandnum)
{
	m_xRasterSize = xsize;
	m_yRasterSize = ysize;
	m_BandType = type;
	m_nBands = bandnum;

	Destroy();
	GDALAllRegister();	// ע������

	GDALDriver *pDriver;
	char** pMetadata;
	char **papszOptions = NULL;
	// �鿴�����Ƿ�֧��DCAP_CREATE��DCAP_CREATECOPY
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
	// ��ʼ����
	if(strcmp(pFormat.c_str(), "GTiff")==0)   
	{
//		papszOptions = CSLSetNameValue(papszOptions, "TILED", "YES");
//		papszOptions = CSLSetNameValue(papszOptions, "BLOCKXSIZE", "64");
//		if(xsize>5000||ysize>5000)
//			papszOptions = CSLSetNameValue(papszOptions, "COMPRESS", "JPEG");
		// papszOptions = CSLSetNameValue(papszOptions, "PREDICTOR", "3");
		// GDAL 2.1��ʼ֧��
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
		// ������ָ�����ָ�򲨶�
		pBand[i] = poDataset->GetRasterBand(i+1); 
		pBuffer[i] = NULL;
	}
	m_isopen = true;	// �����ɹ�

	return 0;
}


//////////////////////////////////////////////////////////
//������Ӱ��
// lpFilePath: Ӱ�񱣴�·��
// pFormat��   Ӱ����������
//////////////////////////////////////////////////////////
bool GeoReadImage::New(string lpFilePath, string pFormat, GDALDataType type, long xsize, long ysize,
	int bandnum, double *adfGeoTransform, string projectName)
{
	m_xRasterSize = xsize;
	m_yRasterSize = ysize;
	m_BandType = type;
	m_nBands = bandnum;

	Destroy();
	GDALAllRegister();	// ע������
	GDALDriver *pDriver;
	char** pMetadata;
	char **papszOptions = NULL;
	pDriver = GetGDALDriverManager()->GetDriverByName(pFormat.c_str());
	if (pDriver == NULL)
	{
		m_errorinfo = ("������Ϣ��ȡʧ��");
		return false;
	}
	pMetadata = pDriver->GetMetadata();
	if (!CSLFetchBoolean(pMetadata, GDAL_DCAP_CREATE, FALSE))
	{
		m_errorinfo =  ("�˸�ʽGDAL��֧��д��");
		return false;
	}
	// ��ʼ����
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
		// ������ָ�����ָ�򲨶�
		pBand[i] = poDataset->GetRasterBand(i + 1);
		pBuffer[i] = NULL;
	}
	ratio = 1;

	return 0;
}

//////////////////////////////////////////////////////////
// �ж������Ƿ���вο�����
// �����ĺ���
// long nPosX:     ͼ�����ݿ����ʼ������
// long nPosY:     ͼ�����ݿ����ʼ������
// long OffsetX:   ͼ�����ݿ�Ŀ��
// long OffsetY:   ͼ�����ݿ�ĸ߶�
/////////////////////////////////////////////////////////
bool GeoReadImage::IsInRefData(double nPosX, double nPosY, double OffsetX, double OffsetY)
{
	if(OffsetX<0||OffsetY<0)
		return false;
	// �洢��Ҫ��ȡ��Ӱ�����ط�Χ
	OGRLinearRing ringtemp; 
	double tempx = nPosX+OffsetX;
	double tempy = nPosY+OffsetY;
	ringtemp.addPoint(nPosX, nPosY);                  
	ringtemp.addPoint(tempx, nPosY);
	ringtemp.addPoint(tempx, tempy);   
	ringtemp.addPoint(nPosX, tempy);  // ע��Ҫ��˳�����
	ringtemp.closeRings();
	OGRPolygon polygon;
	polygon.addRing(&ringtemp);
	//���û���ཻ����,���޷���ȡ����
	if(polygon.Intersect(&m_PolygonImage)==0)
	{
		m_errorinfo = "Failed to Get Image Context!";
		return false;
	}
	return true;
}


//////////////////////////////////////////////////////////
// ����Ӱ����Ϣ����Ӱ����
//////////////////////////////////////////////////////////
void GeoReadImage::CopyImageInfo(GeoReadImage m_image)
{
	// �������ݼ��������С��ƽ����ֵ(���DEM)
	m_maxvalue = m_image.m_maxvalue;
	m_minvalue = m_image.m_minvalue;
	m_meanvalue = m_image.m_meanvalue;
	// ÿ�����ص��ֽڴ�С
	m_SizeInBytes = m_image.m_SizeInBytes;
	// ���״̬��Ϣ
	m_errorinfo = m_image.m_errorinfo;
	// ���ݼ�����ָ��
	poDataset = m_image.poDataset;
	// ���ζ���ָ��
	pBand = m_image.pBand;
	// ͼ��ĳ���(x)�͸߶�(y)
	m_xRasterSize = m_image.m_xRasterSize;
	m_yRasterSize = m_image.m_yRasterSize;
	// ͼ�񲨶���,Ŀǰֻ�������ε�����
	m_nBands = m_image.m_nBands;
	// ���Ͻ�x��y���꣬�Լ�ÿ�����صĳ��ȺͿ��
	m_leftx = m_image.m_leftx;
	m_lefty = m_image.m_lefty;
	m_xsize = m_image.m_xsize;
	m_ysize = m_image.m_ysize;
	// ������������
	m_BandType = m_image.m_BandType;
	// �洢����Ӱ������ط�Χ(��)
	m_RingImage = m_image.m_RingImage;
	// �洢����Ӱ������ط�Χ(��)
	m_PolygonImage = m_image.m_PolygonImage;
	// �洢��ǰ��ȡ����������Χ
	m_nPosX = m_image.m_nPosX;
	m_nPosY = m_image.m_nPosY;
	m_nPosX2 = m_image.m_nPosX2;
	m_nPosY2 = m_image.m_nPosY2;
	m_OffsetX = m_image.m_OffsetX;
	m_OffsetY = m_image.m_OffsetY;
	// �洢��ǰ������������ط�Χ
	m_LinearRing = m_image.m_LinearRing;
	// ���ֵĸ�����,ע��ÿһ�������㶼�п��ܲ�һ��
	m_nx = m_image.m_nx;
	m_ny = m_image.m_ny;
	// �������ֺ�ķֱ���	
	m_dXResolution = m_image.m_dXResolution;
	m_dYResolution = m_image.m_dYResolution;
	// �洢�����ľ���ֵ
	pXGrid = m_image.pXGrid;
	pYGrid = m_image.pYGrid;
	// �洢������������С��γ��
	maxx = m_image.maxx;
	minx = m_image.minx;
	maxy = m_image.maxy;
	miny = m_image.miny;
	//��DEM������ֵ,һ�㶼��-9999
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
// ���Ӱ��Ŀ���Ϣ
// �����ĺ���
// long nPosX:     ͼ�����ݿ����ʼ������
// long nPosY:     ͼ�����ݿ����ʼ������
// long OffsetX:   ͼ�����ݿ�Ŀ��
// long OffsetY:   ͼ�����ݿ�ĸ߶�
// int  indexBand: �����Ĳ��κ�
// void* &pBuffer:  ���ݿ�ָ�룬����ͼ������(ע��ǵü�&)
// float ratiox:	������ű���
// float ratioy:	�߶����ű���
/////////////////////////////////////////////////////////
bool GeoReadImage::ReadBlock(long nPosX, long nPosY,unsigned long OffsetX,unsigned long OffsetY, 
							int indexBand, void* &pData, float ratiox, float ratioy)
{	
	if(indexBand>m_nBands-1)
	{
		printf("The Band Index is Greater than the Band Num!");
		return false;
	}
	// �洢��Ҫ��ȡ��Ӱ�����ط�Χ
	OGRLinearRing ringtemp; 
	long tempx = nPosX+OffsetX;
	long tempy = nPosY+OffsetY;
	ringtemp.addPoint(nPosX, nPosY);                  
	ringtemp.addPoint(tempx, nPosY);
	ringtemp.addPoint(tempx, tempy);   
	ringtemp.addPoint(nPosX, tempy);  // ע��Ҫ��˳�����
	ringtemp.closeRings();
	OGRPolygon polygon;
	polygon.addRing(&ringtemp);
	//���û���ཻ����,���޷���ȡ����
	if(polygon.Intersect(&m_PolygonImage)==0)
	{
		m_errorinfo = "Failed to Get Image Context!";
		return false;
	}
	OGRPolygon *polygontemp = (OGRPolygon *)polygon.Intersection(&m_PolygonImage); // ��
	m_LinearRing = polygontemp->getExteriorRing();
	// ��ȡ��Χ
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
	// ����洢��ǰ��ȡ������������
	if(pData != NULL)
	{
		delete []pData;
		pData = NULL;
	}
	/////////////////////////////////////////////////////////
	// ���ݶ�ȡ
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
// ����ռ�
// �����ĺ���
// long nPosX:     ͼ�����ݿ����ʼ������
// long nPosY:     ͼ�����ݿ����ʼ������
// long OffsetX:   ͼ�����ݿ�Ŀ��
// long OffsetY:   ͼ�����ݿ�ĸ߶�
// int  indexBand: �����Ĳ��κ�
// void* &pBuffer:  ���ݿ�ָ�룬����ͼ������(ע��ǵü�&)
/////////////////////////////////////////////////////////
bool GeoReadImage::SetBuffer(long nPosX, long nPosY, unsigned long OffsetX, unsigned long OffsetY, void* &pData)
{
	// �洢��Ҫ��ȡ��Ӱ�����ط�Χ
	OGRLinearRing ringtemp; 
	long tempx = nPosX+OffsetX;
	long tempy = nPosY+OffsetY;
	ringtemp.addPoint(nPosX, nPosY);                  
	ringtemp.addPoint(tempx, nPosY);
	ringtemp.addPoint(tempx, tempy);   
	ringtemp.addPoint(nPosX, tempy);  // ע��Ҫ��˳�����
	ringtemp.closeRings();
	OGRPolygon polygon;
	polygon.addRing(&ringtemp);
	//���û���ཻ����,���޷���ȡ����
	if(polygon.Intersect(&m_PolygonImage)==0)
	{
		m_errorinfo = "Failed to Get Image Context!";
		return false;
	}
	OGRPolygon *polygontemp = (OGRPolygon *)polygon.Intersection(&m_PolygonImage); // ��
	m_LinearRing = polygontemp->getExteriorRing();
	// ��ȡ��Χ
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
	// ����洢��ǰ��ȡ������������
	if(pData != NULL)
	{
		delete pData;
		pData = NULL;
	}
	/////////////////////////////////////////////////////////
	// ���ݶ�ȡ
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
// ����Ӱ��Ŀ���Ϣ
// �����ĺ���
// long nPosX:     ͼ�����ݿ����ʼ������
// long nPosY:     ͼ�����ݿ����ʼ������
// long OffsetX:   ͼ�����ݿ�Ŀ��
// long OffsetY:   ͼ�����ݿ�ĸ߶�
// void* pBuffer:  ���ݿ�ָ�룬����ͼ������
/////////////////////////////////////////////////////////
bool GeoReadImage::WriteBlock(long nPosX, long nPosY,unsigned long OffsetX,unsigned long OffsetY, int indexBand, void* pData)
{	
	// ��ʼ���Ƿ�Խ���ж�
	if(nPosX<0 || nPosX>m_xRasterSize || nPosY<0 || nPosY>m_yRasterSize)
	{
		m_errorinfo = "The Start Overlap the Boundary!";
		return false;
	}
	// ƫ�Ƶ��Ƿ�Խ���ж�
	if(OffsetX<0 || nPosX+OffsetX>m_xRasterSize || OffsetY<0 || nPosY+OffsetY>m_yRasterSize)
	{
		m_errorinfo = "The End Overlap the Boundary!";
		return false;
	}
	// ƫ�����Ƿ�Ϊ0�Ĵ����ж�
	if(0 == OffsetX)
		OffsetX = m_xRasterSize - nPosX;
	if(0 == OffsetY)
		OffsetY = m_yRasterSize - nPosY;
	// ��������ָ���Ƿ�Ϊ�յ��ж�
	if(NULL==pData)
	{
		m_errorinfo = "The Pointer is NULL!";
		return false;
	}
	/////////////////////////////////////////////////////////
	// ����д��
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
// ���ָ��λ�õ�����ֵ
// bool isPixel:	 �Ƿ�����Ϊ����ֵ
// double x:		 ���isPixelΪture,��Ϊ����x����
//					 ���isPixelΪfalse,��Ϊ����,��λΪ��
// double y:         ���isPixelΪture,��Ϊ����y����
//					 ���isPixelΪfalse,��Ϊγ��,��λΪ��
// double invValue:  ��Чֵ�ĸ�ֵ
/////////////////////////////////////////////////////////
double GeoReadImage::GetDataValue(double x, double y, double invValue, int indexBand, bool isPixel)
{
	// ͨ����������γ������ת������������
	if(isPixel==false)
	{
		double xtemp, ytemp;
		GridInterBL2Pixel(x, y, xtemp, ytemp);
		x = xtemp;
		y = ytemp;
	}

	// Խ���ж�
	if((x>=m_nPosX)&&(x<m_nPosX2)&&(y>=m_nPosY)&&(y<m_nPosY2))
	{
		unsigned char *temp1 = NULL;
		unsigned short *temp2 = NULL;
		short *temp3 = NULL;
		unsigned long *temp4 = NULL;
		long *temp5 = NULL;
		float *temp6 = NULL;
		double *temp7 = NULL;
		// ���Ͻǵ�
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
		// ��������
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
// ����������ص�ֵ
// double x:		 ����x����
// double y:         ����y����
// double invValue:  ��Чֵ�ĸ�ֵ
/////////////////////////////////////////////////////////
double GeoReadImage::GetDataValue(long x, long y, double invValue, int indexBand)
{
	
	// Խ���ж�
	if((x>=m_nPosX)&&(x<m_nPosX2)&&(y>=m_nPosY)&&(y<m_nPosY2))
	{
		// ���Ͻǵ�
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
		// ��������
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
// ��Buffer���ָ����Χ�ĵ�����ֵ
//	void *pData:		�����Buffer����
//	int lt_x:			��Χ�����Ͻ�x����
//	int lt_y:			��Χ�����Ͻ�y����
//	int bufferW��		��Χ�Ŀ��
//	int bufferH��		��Χ�ĸ߶�
//	double *pOut��		�õ��ķ�Χ����
// ����ֵ��
//	bool��				true˵����ȡ�ɹ�
//						false˵����ȡʧ��
/////////////////////////////////////////////////////////
bool GeoReadImage::GetDataValueFromBuffer(void* pData, int lt_x, int lt_y, int bufferW, int bufferH, double* &pOut)
{
	// Խ���ж�
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
// ���ָ��λ�õ�����ֵ,δ�ȶ�ȡ����
// bool isPixel:	 �Ƿ�����Ϊ����ֵ
// double x:		 ���isPixelΪture,��Ϊ����x����
//					 ���isPixelΪfalse,��Ϊ����,��λΪ��
// double y:         ���isPixelΪture,��Ϊ����y����
//					 ���isPixelΪfalse,��Ϊγ��,��λΪ��
// double invValue:  ��Чֵ�ĸ�ֵ
/////////////////////////////////////////////////////////
double GeoReadImage::GetDataValueWithoutPrepare(double x, double y, double invValue, int indexBand, bool isPixel)
{
	// ͨ����������γ������ת������������
	if(isPixel==false)
	{
		double xtemp, ytemp;
		GridInterBL2Pixel(x, y, xtemp, ytemp);
		x = xtemp;
		y = ytemp;
	}

	// Խ���ж�
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
		// ��������
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
// Ϊ��ֵָ���ĳ�����ظ�ֵ
// long x:        ����x����
// long y:        ����y����
// double value:  ����ֵ
// void *pData:   ��Ҫ��������ֵָ��
/////////////////////////////////////////////////////////
bool GeoReadImage::SetDataValue(long x, long y, double value, int indexBand)
{
	// Խ���ж�
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
// ͳ���������ε�ֵ
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
// ��Ӱ�񹹽�����
//////////////////////////////////////////////////////////
bool GeoReadImage::CreateGrid()
{
	bool istrue = true;
	istrue *= CreateGridBL2Pixel();
	istrue *= CreateGridPixel2BL();
	return istrue;
}

// �����Ӿ�γ�ȵ����صĸ���
bool GeoReadImage::CreateGridBL2Pixel()
{
	char *wgs84 = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.01745329251994328]]";
	double xx[4],yy[4];
	xx[0] = xx[2] = m_leftx;
	xx[1] = xx[3] = m_leftx + m_xsize*m_xRasterSize;
	yy[0] = yy[1] = m_lefty;
	yy[2] = yy[3] = m_lefty + m_ysize*m_yRasterSize;

	// ��Ӱ�����ͶӰ����ת������������(����DEM���ܶ���,����Ϊ��ͳһ�Ի��ǽ���һ��)
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

	// ������ȡ���ھ�γ�ȵ��������µ������Ӿ���
	maxx = max(max(xx[0],xx[1]),max(xx[2],xx[3]));
	minx = min(min(xx[0],xx[1]),min(xx[2],xx[3]));
	maxy = max(max(yy[0],yy[1]),max(yy[2],yy[3]));
	miny = min(min(yy[0],yy[1]),min(yy[2],yy[3]));

	m_nx = m_ny = 200;
	// ��ȡͨ���������ֺ�ķֱ���
	m_dXResolution = (maxx - minx)/m_nx;
	m_dYResolution = (miny - maxy)/m_ny;
	// ��ո�������
	if(pXGrid != NULL) { delete pXGrid; pXGrid = NULL;}
	if(pYGrid != NULL) { delete pYGrid; pYGrid = NULL;}
	// �����¿ռ�
	pXGrid = new double[m_nx*m_ny];
	pYGrid = new double[m_nx*m_ny];
	// ��������
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
	// �ӵ�������ת����Ӱ�����ͶӰ����(����DEM���ܶ���,����Ϊ��ͳһ�Ի��ǽ���һ��)
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
	
// ���������ص���γ�ȵĸ���
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
	// ��ȡͨ���������ֺ�ķֱ���
	m_dXResolutioninv = m_xRasterSize/m_nxinv;
	m_dYResolutioninv = m_yRasterSize/m_nyinv;
	// ��ո�������
	if(pXGridinv != NULL) { delete pXGridinv; pXGridinv = NULL;}
	if(pYGridinv != NULL) { delete pYGridinv; pYGridinv = NULL;}
	// �����¿ռ�
	pXGridinv = new double[m_nxinv*m_nyinv];
	pYGridinv = new double[m_nxinv*m_nyinv];
	// ��������
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
// ͨ�������ڲ����Ӧ��:�Ӿ�γ�ȵ�����
// ����:
//   double lon:   ����
//   double lat:   γ��
// �����
//   double xto��  ����x����
//   double yto��  ����y����
/////////////////////////////////////////////////////////
bool GeoReadImage::GridInterBL2Pixel(double lon, double lat, double &xto, double &yto)
{
	// �򵥴�������Ϊ����
	double x = (lon - minx)/m_dXResolution;      // lon
	double y = (lat - maxy)/m_dYResolution;      // lat
	// ���Ͻǵ�
	long xcor[4], ycor[4];
	if(x<0)				{ xcor[0] = xcor[3] = 0;       xcor[1] = xcor[2] = 1;     }
	else if(x>=m_nx-1)  { xcor[0] = xcor[3] = m_nx-2;  xcor[1] = xcor[2] = m_nx-1;}  // ע��Ҫ-1,�������Խ����ɶ�ȡӰ��ܴ�
	else                
	{ 
		xcor[0] = long(x);     xcor[1] = long(x+1);
		xcor[2] = long(x+1);   xcor[3] = long(x);
	}
	if(y<0)				{ ycor[0] = ycor[1] = 0;	   ycor[2] = ycor[3] = 1;     }
	else if(y>=m_ny-1)  { ycor[0] = ycor[1] = m_ny-2;  ycor[2] = ycor[3] = m_ny-1;}  // ע��Ҫ-1,�������Խ����ɶ�ȡӰ��ܴ�
	else                
	{ 
		ycor[0] = long(y);     ycor[1] = long(y);
		ycor[2] = long(y+1);   ycor[3] = long(y+1);
	}
	// ���Ͻǵ�
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
	// ��ô������
	xto = 0;   yto = 0;
	for (int i=0; i<4; ++i)
	{
		xto += coef[i]*pXGrid[(long)index[i]];
		yto += coef[i]*pYGrid[(long)index[i]];	
	}
	// �����������
	xto = (xto - m_leftx)/m_xsize;
	yto = (yto - m_lefty)/m_ysize;
	return true;
}


/////////////////////////////////////////////////////////
// ͨ�������ڲ����Ӧ��:�����ص���γ��
// ����:
//   double xto��  ����x����
//   double yto��  ����y����
// �����
//   double lon:   ����
//   double lat:   γ��
/////////////////////////////////////////////////////////
bool GeoReadImage::GridInterPixel2BL(double xto, double yto, double &lon, double &lat)
{
	// �򵥴�������Ϊ����
	double x = xto/m_dXResolutioninv;
	double y = yto/m_dYResolutioninv;
	// ���Ͻǵ�
	long xcor[4], ycor[4];
	if(x<0)				   { xcor[0] = xcor[3] = 0;			 xcor[1] = xcor[2] = 1;     }
	else if(x>=m_nxinv-1)  { xcor[0] = xcor[3] = m_nxinv-2;  xcor[1] = xcor[2] = m_nxinv-1;}  // ע��Ҫ-1,�������Խ����ɶ�ȡӰ��ܴ�
	else                
	{ 
		xcor[0] = long(x);     xcor[1] = long(x+1);
		xcor[2] = long(x+1);   xcor[3] = long(x);
	}
	if(y<0)					{ ycor[0] = ycor[1] = 0;			ycor[2] = ycor[3] = 1;     }
	else if(y>=m_nyinv-1)   { ycor[0] = ycor[1] = m_nyinv-2;	ycor[2] = ycor[3] = m_nyinv-1;}  // ע��Ҫ-1,�������Խ����ɶ�ȡӰ��ܴ�
	else                
	{ 
		ycor[0] = long(y);     ycor[1] = long(y);
		ycor[2] = long(y+1);   ycor[3] = long(y+1);
	}
	// ���Ͻǵ�
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
	// ��þ�γ������
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
// ͨ�������ڲ����Ӧ��:�Ӿ�γ�ȵ�����(��Ծ�������,��þ��ε����ؿ�)
// ����:
//   double *lon:   ����
//   double *lat:   γ��
// �����
//   long sample��  �������Ͻ�x����
//   long line��    �������Ͻ�y����
//   long width:    ��ȡ��������Ŀ��
//   long height:   ��ȡ��������ĸ߶�
/////////////////////////////////////////////////////////
bool GeoReadImage::GridInterBL2PixelRect(double *lon, double *lat, double &sample, double &line, double &width, double &height)
{
	double xtemp[4],ytemp[4];
	// ת������������
	for(int i=0;i<4;i++)
	{
		GridInterBL2Pixel(lon[i], lat[i], xtemp[i], ytemp[i]);
	}
	// ������ȡ�������������µ������Ӿ���
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
//	�ҶȾ��⺯��
//	���룺
//		void *pSrc:		�����ԭʼӰ������,������datatype����
//						�������unsigned char��,ת��Ϊunsigned char
//		int nWidth��	����ԭʼӰ��Ŀ��
//		int nHeight��	����ԭʼӰ��ĸ߶�
//	�����
//		unsigned char* pDst��	����ľ��⻯Ӱ������
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
//	�ҶȾ��⺯���ڲ�ʹ��,ģ��
//	���룺
//		void *pSrc:		�����ԭʼӰ������,������datatype����
//						�������unsigned char��,ת��Ϊunsigned char
//		int nWidth��	����ԭʼӰ��Ŀ��
//		int nHeight��	����ԭʼӰ��ĸ߶�
//	�����
//		unsigned char* pDst��	����ľ��⻯Ӱ������
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
	// ������Ҷ�ֵ����
	for(long i=0; i<pixelnum; i++)
	{
		//pDst[i] = pSrc[i];
		pDst[i] = (pSrc[i]-m_minvalue)/valuetemp*255;
	}

/*	// �Ҷ�ӳ���
	unsigned char map[256];
	long lCounts[256];
	memset(lCounts, 0, sizeof(long)*256);
	long pixelnum = nWidth*nHeight;
	double valuetemp = m_maxvalue - m_minvalue;
	// ������Ҷ�ֵ����
	for(long i=0; i<pixelnum; i++)
	{
		pSrc[i] = (pSrc[i]-m_minvalue)/valuetemp*255;
		lCounts[(unsigned char)pSrc[i]]++;
	}
	// ���������е���ʱֵ
	long lTemp;
	for(long i=0; i<256; i++)
	{
		lTemp = 0;
		for(int j=0; j<=i; j++)
			lTemp += lCounts[j];
		map[i] = (unsigned char)(lTemp*255.0f/valuetemp);
	}
	// �任���ֱֵ����ӳ����в���
	for(long i=0; i<pixelnum; i++)
	{
		pDst[i] = map[(unsigned char)pSrc[i]];
	}*/
}

