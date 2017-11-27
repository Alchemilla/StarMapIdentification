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
// 影像读取类
///////////////////////////////////////////////////////////////
class  GeoReadImage
{
public:
	GeoReadImage(void);
	~GeoReadImage(void);
	void Destroy();
	// 打开影像，并且获取影像的各种信息
	bool Open(string lpFilePath, GDALAccess gdalaccess, bool isCreateGird = true);
	// 打开影像,专门为CE5设的
	bool Open(string lpFilePath, unsigned char *&pData);
	//创建新影像
	bool New(string lpFilePath, string pFormat, GDALDataType type, long xsize, long ysize, int bandnum);
	bool New(string lpFilePath, string pFormat, GDALDataType type, long xsize, long ysize,
		int bandnum, double *adfGeoTransform, string projectName);
	// 获得影像的块信息
	bool ReadBlock(long nPosX, long nPosY, unsigned long OffsetX, unsigned long OffsetY, int indexBand, void* &pData, float ratiox = 1, float ratioy = 1);
	// 分配空间
	bool SetBuffer(long nPosX, long nPosY, unsigned long OffsetX, unsigned long OffsetY, void* &pData);
	// 设置影像的块信息
	bool WriteBlock(long nPosX, long nPosY, unsigned long OffsetX, unsigned long OffsetY,int indexBand, void* pData);
	// 对影像创建格网
	bool CreateGrid();
	// 构建从经纬度到像素的格网
	bool CreateGridBL2Pixel();
	// 构建从像素到经纬度的格网
	bool CreateGridPixel2BL();
	// 通过格网内插出对应点:从经纬度到像素
	bool GridInterBL2Pixel(double lon, double lat, double &xto, double &yto);
	// 通过格网内插出对应点:从像素到经纬度
	bool GridInterPixel2BL(double xto, double yto, double &lon, double &lat);
	// 通过格网内插出对应点:从经纬度到像素(针对矩形区域,获得矩形的像素框)
	bool GridInterBL2PixelRect(double *lon, double *lat, double &sample, double &line, double &width, double &height);
	// 获得指定位置的像素值
	double GetDataValue(double x, double y, double invValue, int indexBand, bool isPixel=true);
	// 获得整数像素的值
	double GetDataValue(long x, long y, double invValue, int indexBand);
	// 从Buffer获得指定范围的的像素值
	bool GetDataValueFromBuffer(void* pData, int lt_x, int lt_y, int bufferW, int bufferH, double* &pOut);
	// 获得指定位置的像素值,未先读取数据
	double GetDataValueWithoutPrepare(double x, double y, double invValue, int indexBand, bool isPixel=true);
	// 为数值指针的某个像素赋值
	bool SetDataValue(long x, long y, double value, int indexBand);
	// 统计整个波段的值
	void ComputeStatistics(); 
	// 判断区域是否具有参考数据
	bool IsInRefData(double nPosX, double nPosY, double OffsetX, double OffsetY);
	// 拷贝影像信息到此影像上
	void CopyImageInfo(GeoReadImage m_image);
	// 灰度均衡化函数
	void GrayEqualize(unsigned char* pDst, void* pSrc, int nWidth, int nHeight);
	// 灰度均衡化函数
	template <class T>
	void GrayEqualizeTemplate(unsigned char* pDst, T* pSrc, int nWidth, int nHeight);
	// 建立影像金字塔
	bool CreatePyramids(string path, int ratio = 2);
	// 计算缩放比率
	void CalRatio(int &ratio);
	// 获取最大接边矩形坐标
	bool GetRectangle(double *lat, double *lon, int extend, long &sample, long &line, long &width, long &height, double res, int &ratio);
	
public:
	double m_maxvalue, m_minvalue, m_meanvalue; // 整个数据集的最大、最小、平均数值(针对DEM)
	int m_SizeInBytes;                          // 每个像素的字节大小
	int ratio;									// 放缩比例，目前仅仅对读有效
	string m_ProjectionRef;						// 投影信息

public:
	bool m_isopen;								// 是否打开影像成功的标识
	string m_errorinfo;                         // 输出状态信息
	GDALDataset *poDataset;                     // 数据集对象指针
	GDALRasterBand ** pBand;                    // 波段对象指针
	long m_xRasterSize, m_yRasterSize;          // 图像的长度(x)和高度(y)
	int m_nBands;                               // 图像波段数,目前只处理单波段的数据
	double m_leftx, m_lefty, m_xsize, m_ysize;  // 左上角x、y坐标，以及每个像素的长度和宽度
	GDALDataType m_BandType;                    // 波段数据类型
	OGRLinearRing m_RingImage;                  // 存储整个影像的像素范围(环)
	OGRPolygon m_PolygonImage;                  // 存储整个影像的像素范围(面)

	int m_xBlock, m_yBlock;						// 影像存储划分的块大小

public:
	void **pBuffer;								// 存储当前获取的数据区内容
	long m_nPosX, m_nPosY;
	long m_nPosX2,m_nPosY2;
	unsigned long m_OffsetX, m_OffsetY;         // 存储当前获取的数据区范围
	OGRLinearRing *m_LinearRing;                // 存储当前数据区域的像素范围

public:
	long m_nx, m_ny;                            // 划分的格网数,注意每一个格网点都有可能不一样
	double m_dXResolution, m_dYResolution;      // 格网划分后的分辨率	
	double *pXGrid,*pYGrid;                     // 存储格网的具体值
	double maxx, minx,maxy, miny;               // 存储格网的最大和最小经纬度

	long m_nxinv, m_nyinv;						// 划分的反向格网数,注意每一个格网点都有可能不一样
	double m_dXResolutioninv, m_dYResolutioninv;// 反向格网划分后的分辨率
	double *pXGridinv, *pYGridinv;				// 存储反向格网的具体值

	int m_invalidatedem;                        //非DEM区域数值,一般都是-9999
};

#endif

