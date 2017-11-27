// GeoImage.cpp : Implementation of CGeoImage
#include "GeoImage.h"


//////////////////////////////////////////////////////////
// 对影像的裁剪
//////////////////////////////////////////////////////////
bool GeoImage::ImageWarp(string inpath, string outpath, GeoModelLine *model)
{
	// 求取影像长度和宽度
	long photowidth, photoheight;
	photoheight = model->get_m_height();
	photowidth = model->get_m_width();
	// 求取影像地面分辨率
	double latmin, lonmin, latmax, lonmax, lat[4], lon[4],hh=0;
	model->FromXY2LatLon(0, 0, hh, latmin, lonmin);
	model->FromXY2LatLon(0, 1, hh, latmax, lonmax);
	double res = sqrt(pow(6378000 * fabs(latmin - latmax), 2) + pow(6378000 * fabs(lonmin - lonmax), 2));
	// 影像对象
	GeoReadImage m_img, m_New;
	// DEM文件初始化
	m_img.Open(inpath, GA_ReadOnly, res);
	// 获取四个角点
	model->FromXY2LatLon(0, 0, hh, lat[0], lon[0]);
	lat[0] = lat[0] * 180 / PI;  lon[0] = lon[0] * 180 / PI;
	model->FromXY2LatLon(0, photowidth, hh, lat[1], lon[1]);
	lat[1] = lat[1] * 180 / PI;  lon[1] = lon[1] * 180 / PI;
	model->FromXY2LatLon(photoheight, photowidth, hh, lat[2], lon[2]);
	lat[2] = lat[2] * 180 / PI;  lon[2] = lon[2] * 180 / PI;
	model->FromXY2LatLon(photoheight, 0, hh, lat[3], lon[3]);
	lat[3] = lat[3] * 180 / PI;  lon[3] = lon[3] * 180 / PI;
	// 获取最大接边矩形坐标及缩放比率
	long sample, line, width, height;
	int ratio;
	bool isOK = !m_img.GetRectangle(lat, lon, 2000, sample, line, width, height, res, ratio);
	if (isOK != 0)
		return false;
	// 长度和宽度要按照比率的倍数来
	width = width / ratio*ratio;
	height = height / ratio*ratio;
	sample = sample / ratio*ratio;
	line = line / ratio*ratio;
	if (ratio >= 2)
	{
		sample += ratio / 2;
		line += ratio / 2;
	}
	//else
	//{
	//	*isOK = 1;
	//	return !S_OK;
	//}
	long width0 = width / ratio;
	long height0 = height / ratio;
	// New文件初始化
	double adfGeoTransform[6];
	adfGeoTransform[0] = m_img.m_leftx + m_img.m_xsize*sample;
	adfGeoTransform[1] = m_img.m_xsize*ratio;
	adfGeoTransform[2] = 0.0;
	adfGeoTransform[3] = m_img.m_lefty + m_img.m_ysize*line;
	adfGeoTransform[4] = 0.0;
	adfGeoTransform[5] = m_img.m_ysize*ratio;
	m_New.New(outpath, ("HFA"), m_img.m_BandType, width0, height0,
		m_img.m_nBands, adfGeoTransform, m_img.m_ProjectionRef);
	m_New.Destroy();
	// 获得分块大小
	m_step = 5000;
	// 分块并将各个块的四个角点存储下来
	long sampletemp, linetemp;
	m_ImageRect.clear();
	for (long line0 = 0; line0 < height0; line0 += m_step)
	{
		vector<ImageRect> m_ImageRectTemp;
		for (long sample0 = 0; sample0 < width0; sample0 += m_step)
		{
			ImageRect m_Rect;
			if ((sample0 + m_step) >= (width0))
				sampletemp = width0;
			else sampletemp = sample0 + m_step;
			if ((line0 + m_step) >= (height0))
				linetemp = height0;
			else linetemp = line0 + m_step;
			m_Rect.ts = sample0;
			m_Rect.tl = line0;
			m_Rect.tw = sampletemp - sample0;
			m_Rect.th = linetemp - line0;
			// 压入堆栈
			m_ImageRectTemp.push_back(m_Rect);
		}
		m_ImageRect.push_back(m_ImageRectTemp);
	}
	// 重新打开输出影像
	m_New.Open(outpath, GA_Update);
	// 分块循环赋值
	for (int i = 0; i < m_ImageRect.size(); i++)
	{
		for (int j = 0; j < m_ImageRect[i].size(); j++)
		{
			// 读取数据
			sampletemp = sample + m_ImageRect[i][j].ts*ratio;
			linetemp = line + m_ImageRect[i][j].tl*ratio;
			width0 = m_ImageRect[i][j].tw*ratio;
			height0 = m_ImageRect[i][j].th*ratio;
			for (int k = 0; k < m_New.m_nBands; k++)
			{
				m_img.ReadBlock(sampletemp, linetemp, width0, height0, k, m_img.pBuffer[k]);
			}
			// 写出数据
			for (int k = 0; k < m_New.m_nBands; k++)
			{
				m_New.WriteBlock(m_ImageRect[i][j].ts, m_ImageRect[i][j].tl, m_ImageRect[i][j].tw,
					m_ImageRect[i][j].th, k, m_img.pBuffer[k]);
			}
		}
	}
	// 关闭影像
	m_New.Destroy();
	m_img.Destroy();
	return true;
}

