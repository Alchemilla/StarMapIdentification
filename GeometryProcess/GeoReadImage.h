#ifndef _GEOREADIMAGE
#define	_GEOREADIMAGE

#include "GDAL/gdal_priv.h"
#include "GDAL/gdalwarper.h"
#include "GDAL/gdal_pam.h"
#include "GDAL/ogrsf_frmts.h"
#include "GDAL/ogr_geometry.h"
#include "GDAL/ogr_spatialref.h"
#include "GeoDefine.h"
#include <vector>
using namespace std;

#ifndef max
#define max(a,b)   (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b)   (((a) < (b)) ? (a) : (b))
#endif


///////////////////////////////////////////////////////////////
// Ӱ���ȡ��
///////////////////////////////////////////////////////////////
class  GeoReadImage
{
public:
	GeoReadImage(void);
	~GeoReadImage(void);
	void Destroy();
	// ��Ӱ�񣬲��һ�ȡӰ��ĸ�����Ϣ
	bool Open(string lpFilePath, GDALAccess gdalaccess, bool isCreateGird = true);
	// ��Ӱ��,ר��ΪCE5���
	bool Open(string lpFilePath, unsigned char *&pData);
	//������Ӱ��
	bool New(string lpFilePath, string pFormat, GDALDataType type, long xsize, long ysize, int bandnum);
	bool New(string lpFilePath, string pFormat, GDALDataType type, long xsize, long ysize,
		int bandnum, double *adfGeoTransform, string projectName);
	// ���Ӱ��Ŀ���Ϣ
	bool ReadBlock(long nPosX, long nPosY, unsigned long OffsetX, unsigned long OffsetY, int indexBand, void* &pData, float ratiox = 1, float ratioy = 1);
	// ����ռ�
	bool SetBuffer(long nPosX, long nPosY, unsigned long OffsetX, unsigned long OffsetY, void* &pData);
	// ����Ӱ��Ŀ���Ϣ
	bool WriteBlock(long nPosX, long nPosY, unsigned long OffsetX, unsigned long OffsetY,int indexBand, void* pData);
	// ��Ӱ�񴴽�����
	bool CreateGrid();
	// �����Ӿ�γ�ȵ����صĸ���
	bool CreateGridBL2Pixel();
	// ���������ص���γ�ȵĸ���
	bool CreateGridPixel2BL();
	// ͨ�������ڲ����Ӧ��:�Ӿ�γ�ȵ�����
	bool GridInterBL2Pixel(double lon, double lat, double &xto, double &yto);
	// ͨ�������ڲ����Ӧ��:�����ص���γ��
	bool GridInterPixel2BL(double xto, double yto, double &lon, double &lat);
	// ͨ�������ڲ����Ӧ��:�Ӿ�γ�ȵ�����(��Ծ�������,��þ��ε����ؿ�)
	bool GridInterBL2PixelRect(double *lon, double *lat, double &sample, double &line, double &width, double &height);
	// ���ָ��λ�õ�����ֵ
	double GetDataValue(double x, double y, double invValue, int indexBand, bool isPixel=true);
	// ����������ص�ֵ
	double GetDataValue(long x, long y, double invValue, int indexBand);
	// ��Buffer���ָ����Χ�ĵ�����ֵ
	bool GetDataValueFromBuffer(void* pData, int lt_x, int lt_y, int bufferW, int bufferH, double* &pOut);
	// ���ָ��λ�õ�����ֵ,δ�ȶ�ȡ����
	double GetDataValueWithoutPrepare(double x, double y, double invValue, int indexBand, bool isPixel=true);
	// Ϊ��ֵָ���ĳ�����ظ�ֵ
	bool SetDataValue(long x, long y, double value, int indexBand);
	// ͳ���������ε�ֵ
	void ComputeStatistics(); 
	// �ж������Ƿ���вο�����
	bool IsInRefData(double nPosX, double nPosY, double OffsetX, double OffsetY);
	// ����Ӱ����Ϣ����Ӱ����
	void CopyImageInfo(GeoReadImage m_image);
	// �ҶȾ��⻯����
	void GrayEqualize(unsigned char* pDst, void* pSrc, int nWidth, int nHeight);
	// �ҶȾ��⻯����
	template <class T>
	void GrayEqualizeTemplate(unsigned char* pDst, T* pSrc, int nWidth, int nHeight);
	// ����Ӱ�������
	bool CreatePyramids(string path, int ratio = 2);
	// �������ű���
	void CalRatio(int &ratio);
	// ��ȡ���ӱ߾�������
	bool GetRectangle(double *lat, double *lon, int extend, long &sample, long &line, long &width, long &height, double res, int &ratio);
	
public:
	double m_maxvalue, m_minvalue, m_meanvalue; // �������ݼ��������С��ƽ����ֵ(���DEM)
	int m_SizeInBytes;                          // ÿ�����ص��ֽڴ�С
	int ratio;									// ����������Ŀǰ�����Զ���Ч
	string m_ProjectionRef;						// ͶӰ��Ϣ

public:
	bool m_isopen;								// �Ƿ��Ӱ��ɹ��ı�ʶ
	string m_errorinfo;                         // ���״̬��Ϣ
	GDALDataset *poDataset;                     // ���ݼ�����ָ��
	GDALRasterBand ** pBand;                    // ���ζ���ָ��
	long m_xRasterSize, m_yRasterSize;          // ͼ��ĳ���(x)�͸߶�(y)
	int m_nBands;                               // ͼ�񲨶���,Ŀǰֻ�������ε�����
	double m_leftx, m_lefty, m_xsize, m_ysize;  // ���Ͻ�x��y���꣬�Լ�ÿ�����صĳ��ȺͿ��
	GDALDataType m_BandType;                    // ������������
	OGRLinearRing m_RingImage;                  // �洢����Ӱ������ط�Χ(��)
	OGRPolygon m_PolygonImage;                  // �洢����Ӱ������ط�Χ(��)

	int m_xBlock, m_yBlock;						// Ӱ��洢���ֵĿ��С

public:
	void **pBuffer;								// �洢��ǰ��ȡ������������
	long m_nPosX, m_nPosY;
	long m_nPosX2,m_nPosY2;
	unsigned long m_OffsetX, m_OffsetY;         // �洢��ǰ��ȡ����������Χ
	OGRLinearRing *m_LinearRing;                // �洢��ǰ������������ط�Χ

public:
	long m_nx, m_ny;                            // ���ֵĸ�����,ע��ÿһ�������㶼�п��ܲ�һ��
	double m_dXResolution, m_dYResolution;      // �������ֺ�ķֱ���	
	double *pXGrid,*pYGrid;                     // �洢�����ľ���ֵ
	double maxx, minx,maxy, miny;               // �洢������������С��γ��

	long m_nxinv, m_nyinv;						// ���ֵķ��������,ע��ÿһ�������㶼�п��ܲ�һ��
	double m_dXResolutioninv, m_dYResolutioninv;// ����������ֺ�ķֱ���
	double *pXGridinv, *pYGridinv;				// �洢��������ľ���ֵ

	int m_invalidatedem;                        //��DEM������ֵ,һ�㶼��-9999
};

#endif

