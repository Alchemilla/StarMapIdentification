#include "StarExtract.h"

StarExtract::StarExtract()
{
}


StarExtract::~StarExtract()
{
}

//////////////////////////////////////////////////////////////////////////
//功能：提取星点坐标
//输入：在主函数中输入工作路径
//输出：在工作路径下输出星点提取文件
//注意：函数中GeoReadImage的对象需要用引用
//作者：GZC
//日期：2017.01.06
//////////////////////////////////////////////////////////////////////////
void StarExtract::StarPointExtraction(int index)
{
	GeoReadImage ImgStarMap;
	GeoReadImage ImgBackground, ImgBW, ImgStarPoint;
	char pathtmp[512];
	sprintf(pathtmp, "星图 (%d).tiff", index);
	string path = workpath + string(pathtmp);
	ImgStarMap.Open(path, GA_ReadOnly);
	width = ImgStarMap.m_xRasterSize;
	height = ImgStarMap.m_yRasterSize;
	ImgStarMap.ReadBlock(0, 0, width, height, 0, ImgStarMap.pBuffer[0]);
		
	path = workpath + "背底噪声新3.tiff";
	ImgBackground.Open(path, GA_ReadOnly);
	ImgBackground.ReadBlock(0, 0, 1024, 1024, 0, ImgBackground.pBuffer[0]);

	sprintf(pathtmp, "星点提取结果 (%d).tiff", index);
	path = workpath + (string)pathtmp;
	ImgBW.New(path, "GTiff", ImgStarMap.m_BandType, width, height, 1);
	ImgBW.Destroy();
	ImgBW.Open(path, GA_Update);
	ImgBW.SetBuffer(0, 0, width, height, ImgBW.pBuffer[0]);


	for (long i = 0; i < height; i++)
	{
		for (long j = 0; j < width; j++)
		{
			double StarMapDN, BackgrounDN;
			StarMapDN = ImgStarMap.GetDataValue(j, i, 0, 0);
			if (StarMapDN > 255) StarMapDN = 255;
			BackgrounDN = ImgBackground.GetDataValue(j, i, 0, 0);
			if ((StarMapDN - BackgrounDN) > 3)//阈值选择
			{
				ImgBW.SetDataValue(j, i, 255, 0);
			}
			else
			{
				ImgBW.SetDataValue(j, i, 0, 0);
			}
		}
	}
	//ImgBW.WriteBlock(0, 0, width, height, 0, ImgBW.pBuffer[0]);

	bwlabel(ImgBW, ImgStarMap);

	ImgStarMap.Destroy();
	ImgBackground.Destroy();
	ImgBW.Destroy();
}

//////////////////////////////////////////////////////////////////////////
//功能：对二值图像连通域进行标记
//输入：GeoReadImage &ImgBW：读取二值影像
//输出：类成员参数StarPointExtract：星点提取vector结构体
//注意：函数中GeoReadImage的对象需要用引用
//作者：GZC
//日期：2017.01.06
//////////////////////////////////////////////////////////////////////////
void StarExtract::bwlabel(GeoReadImage &ImgBW, GeoReadImage &ImgStarMap)
{
	vector<int> stRun, enRun, rowRun, runLabels;
	vector<pair<int, int>> equivalences;
	int NumberOfRuns = 0, offset = 0, equaListsize;
	fillRunVectors(ImgBW, NumberOfRuns, stRun, enRun, rowRun);
	firstPass(stRun,enRun, rowRun, NumberOfRuns,runLabels, equivalences, offset);
	replaceSameLabel(runLabels,equivalences, equaListsize);

	vector<double>plot_x(equaListsize);
	vector<double>plot_y(equaListsize);
	vector<double>StarMag(equaListsize);
	double *areaNum = new double[equaListsize];
	memset(areaNum, 0, sizeof(double)*equaListsize);
	for (int k = 1; k <= equaListsize; k++)
	{
		double sum_x = 0, sum_y = 0, area = 0;
		for (int i = 0; i < runLabels.size(); i++)
		{
			if (runLabels[i] == k)
			{
				for (long col = stRun[i]; col <= enRun[i]; col++)
				{
					double DN = ImgStarMap.GetDataValue(col, long(rowRun[i]),  0, 0);
					sum_x += (rowRun[i]+1) *DN*DN*DN;//这里x定义为沿轨向
					sum_y += (col + 1)*DN*DN*DN;//这里y定义为垂轨向
					area += DN*DN*DN;
					areaNum[k-1]++;
				}
			}
		}
		plot_x[k - 1] = sum_x / area;
		plot_y[k - 1] = sum_y / area;
		double DN;
		//避免取到边界以外的值
		if ((plot_x[k - 1])>1&& (plot_x[k - 1])<1023&&(plot_y[k - 1])>1&& (plot_y[k - 1])<1023)
		{
				double DNall[9] = { ImgStarMap.GetDataValue(plot_x[k - 1] - 1, plot_y[k - 1] - 1, 0, 0, 1),
				ImgStarMap.GetDataValue(plot_x[k - 1], plot_y[k - 1] - 1, 0, 0, 1),
				ImgStarMap.GetDataValue(plot_x[k - 1] + 1, plot_y[k - 1] - 1, 0, 0, 1),
				ImgStarMap.GetDataValue(plot_x[k - 1] - 1, plot_y[k - 1], 0, 0, 1),
				ImgStarMap.GetDataValue(plot_x[k - 1], plot_y[k - 1], 0, 0, 1),
				ImgStarMap.GetDataValue(plot_x[k - 1] + 1, plot_y[k - 1], 0, 0, 1),
				ImgStarMap.GetDataValue(plot_x[k - 1] - 1, plot_y[k - 1] + 1, 0, 0, 1),
				ImgStarMap.GetDataValue(plot_x[k - 1], plot_y[k - 1] + 1, 0, 0, 1),
				ImgStarMap.GetDataValue(plot_x[k - 1] + 1, plot_y[k - 1] + 1, 0, 0, 1) };
				DN = *(max_element(DNall, DNall + 9));//C++ STL中算法		
				StarMag[k - 1] = pow((DN - 65), -0.213)*9.234;
		}				
		else
		{
			DN = ImgStarMap.GetDataValue(plot_x[k - 1], plot_y[k - 1], 0, 0, 1);
			StarMag[k - 1] = pow((DN - 65), -0.213)*9.234;
		}
		/*double DN = ImgStarMap.GetDataValue(plot_x[k - 1], plot_y[k - 1], 0, 0, 1);
		StarMag[k - 1] = 2.2 - 5 * log((DN - 65) / 166) / log(100);*/
	}
	
	//筛选多于9个像素的星点
	for (int i = 0; i < equaListsize; i++)
	{
		if (areaNum[i]>9)
		{
			StarPoint StarPointExtractTmp;
			StarPointExtractTmp.x = plot_x[i];
			StarPointExtractTmp.y = plot_y[i];
			StarPointExtractTmp.Mag = StarMag[i];
			StarPointExtract.push_back(StarPointExtractTmp);			
		}
	}

	/*string path = workpath + "星点提取结果.txt";
	FILE *fp = fopen(path.c_str(), "w");
	for (int i = 0; i < equaListsize; i++)
	{
		if (areaNum[i]>9)
		{
			fprintf(fp, "%.5f\t%.5f\t%.2f\n", plot_x[i], plot_y[i], StarMag[i]);
		}
	}
	fclose(fp);*/
}

//////////////////////////////////////////////////////////////////////////
//功能：以下三个函数是对连通区域进行标记的小函数
//			 采用逐行数连续亮点，然后对不同行相关亮点进行连接的方式
//输入：ImgBW：二值影像，NumberOfRuns：连续亮点数，
//			 stRun：连续亮点起始列，enRun：连续亮点终止列，rowRun：所在行
//输出：runLabels：独立的标签个数，equivalence：等价的标签
//注意：函数中GeoReadImage的对象需要用引用
//作者：GZC
//日期：2017.01.06
//////////////////////////////////////////////////////////////////////////
void StarExtract::fillRunVectors(GeoReadImage &ImgBW, int& NumberOfRuns, vector<int>& stRun, vector<int>& enRun, vector<int>& rowRun)
{

	for (long i = 0; i < width; i++)
	{
		int *rowData = new int[1024];
		for (long ii = 0; ii < width; ii++)
		{
			rowData[ii] = ImgBW.GetDataValue(i, ii, 0, 0);
		}
		
		if (rowData[0] == 255)
		{
			NumberOfRuns++;
			stRun.push_back(0);
			rowRun.push_back(i);
		}
		for (int j = 1; j < height; j++)
		{
			if (rowData[j - 1] == 0 && rowData[j] == 255)
			{
				NumberOfRuns++;
				stRun.push_back(j);
				rowRun.push_back(i);
			}
			else if (rowData[j - 1] == 255 && rowData[j] == 0)
			{
				enRun.push_back(j - 1);
			}
		}
		if (rowData[height - 1])
		{
			enRun.push_back(height - 1);
		}
	}
}
// 把run理解一行图像上的一个连续的 白色像素条
void StarExtract::firstPass(vector<int>& stRun, vector<int>& enRun, vector<int>& rowRun, int NumberOfRuns,
	vector<int>& runLabels, vector<pair<int, int>>& equivalences, int offset)
{
	runLabels.assign(NumberOfRuns, 0);
	int idxLabel = 1;
	int curRowIdx = 0;
	int firstRunOnCur = 0;
	int firstRunOnPre = 0;
	int lastRunOnPre = -1;
	for (int i = 0; i < NumberOfRuns; i++)
	{
		// 如果是该行的第一个run，则更新上一行第一个run第最后一个run的序号
		if (rowRun[i] != curRowIdx)
		{
			curRowIdx = rowRun[i]; // 更新行的序号
			firstRunOnPre = firstRunOnCur;
			lastRunOnPre = i - 1;
			firstRunOnCur = i;
		}
		// 遍历上一行的所有run，判断是否于当前run有重合的区域
		for (int j = firstRunOnPre; j <= lastRunOnPre; j++)
		{
			// 区域重合 且 处于相邻的两行
			if (stRun[i] <= enRun[j] + offset && enRun[i] >= stRun[j] - offset && rowRun[i] == rowRun[j] + 1)
			{
				if (runLabels[i] == 0) // 没有被标号过
					runLabels[i] = runLabels[j];
				else if (runLabels[i] != runLabels[j])// 已经被标号            
					equivalences.push_back(make_pair(runLabels[i], runLabels[j])); // 保存等价对
			}
		}
		if (runLabels[i] == 0) // 没有与前一列的任何run重合
		{
			runLabels[i] = idxLabel++;
		}

	}
}
void StarExtract::replaceSameLabel(vector<int>& runLabels, vector<pair<int, int>>&equivalence, int &equaListsize)
{
	int maxLabel = *max_element(runLabels.begin(), runLabels.end());
	vector<vector<bool>> eqTab(maxLabel, vector<bool>(maxLabel, false));
	vector<pair<int, int>>::iterator vecPairIt = equivalence.begin();
	while (vecPairIt != equivalence.end())
	{
		eqTab[vecPairIt->first - 1][vecPairIt->second - 1] = true;
		eqTab[vecPairIt->second - 1][vecPairIt->first - 1] = true;
		vecPairIt++;
	}
	vector<int> labelFlag(maxLabel, 0);
	vector<vector<int>> equaList;
	vector<int> tempList;
	//cout << maxLabel << endl;
	for (int i = 1; i <= maxLabel; i++)
	{
		if (labelFlag[i - 1])
		{
			continue;
		}
		labelFlag[i - 1] = equaList.size() + 1;
		tempList.push_back(i);
		for (vector<int>::size_type j = 0; j < tempList.size(); j++)
		{
			for (vector<bool>::size_type k = 0; k != eqTab[tempList[j] - 1].size(); k++)
			{
				if (eqTab[tempList[j] - 1][k] && !labelFlag[k])
				{
					tempList.push_back(k + 1);
					labelFlag[k] = equaList.size() + 1;
				}
			}
		}
		equaList.push_back(tempList);
		tempList.clear();
	}
	//cout << equaList.size() << endl;
	equaListsize = equaList.size();
	for (vector<int>::size_type i = 0; i != runLabels.size(); i++)
	{
		runLabels[i] = labelFlag[runLabels[i] - 1];
	}
}

//////////////////////////////////////////////////////////////////////////
//功能：根据曲面拟合星点坐标
//输入：预测星点坐标：m_sample，m_line；
//			 影像数据：IMAGEFILE
//输出：精化后星点坐标：fm_sample，fm_line；
//			 窗口大小：num
//注意：来自SAR相关软件
//作者：原：ZRS；改写：GZC
//日期：2017.01.06
//////////////////////////////////////////////////////////////////////////
bool StarExtract::GetPreciseXYbyFitting(double m_sample, double m_line, double &fm_sample,
	double &fm_line, int num, GeoReadImage &IMAGEFILE)
{	
	double a[6];
	
	if (num == 0)
	{
		fm_line = m_line;
		fm_sample = m_sample;
	}
	else if (num == 3)
	{
		double nValue[9];
		memset(nValue, 0, sizeof(double) * 9);

		double xx[9], yy[9];
		memset(xx, 0, sizeof(double) * 9);
		memset(yy, 0, sizeof(double) * 9);

		for (int xi = 0; xi<num; xi++)
		{
			xx[xi*num + 0] = m_sample - 1;
			xx[xi*num + 1] = m_sample;
			xx[xi*num + 2] = m_sample + 1;
		}

		for (int yi = 0; yi<num; yi++)
		{
			yy[yi + num * 0] = m_line - 1;
			yy[yi + num * 1] = m_line;
			yy[yi + num * 2] = m_line + 1;
		}

		double aa[6], aaaa[6 * 6], aabb[6];
		memset(aa, 0, sizeof(double) * 6);
		memset(aaaa, 0, sizeof(double) * 6 * 6);
		memset(aabb, 0, sizeof(double) * 6);

		for (int ii = 0; ii<num*num; ii++)
		{
			memset(aa, 0, sizeof(double) * 6);

			aa[0] = 1;
			aa[1] = xx[ii];
			aa[2] = yy[ii];
			aa[3] = xx[ii] * xx[ii];
			aa[4] = xx[ii] * yy[ii];
			aa[5] = yy[ii] * yy[ii];

			nValue[ii] = IMAGEFILE.GetDataValue(xx[ii], yy[ii], 0, 0, 1);

			mbase.pNormal(aa, 6, nValue[ii], aaaa, aabb, 1.0);
		}

		memset(a, 0, sizeof(double) * 6);
		mbase.Gauss(aaaa, aabb, 6);
		memcpy(a, aabb, sizeof(double) * 6);

		if (a[3] != 0 && a[3]<1e6)
		{
			fm_line = (2 * a[2] * a[3] - a[1] * a[4]) / (a[4] * a[4] - 4 * a[3] * a[5]);
			fm_sample = -(a[4] * fm_line + a[1]) / (2 * a[3]);
		}

		else if (a[4] != 0)
		{
			fm_sample = (a[2] * a[4] - 2 * a[1] * a[5]) / (4 * a[3] * a[5] - a[4] * a[4]);
			fm_line = -(a[1] + 2 * a[3] * fm_sample) / a[4];
		}
	}
	else if (num == 5)
	{
		double nValue[5 * 5];
		memset(nValue, 0, sizeof(double) * 5 * 5);

		double xx[5 * 5], yy[5 * 5];
		memset(xx, 0, sizeof(double) * 5 * 5);
		memset(yy, 0, sizeof(double) * 5 * 5);

		for (int xi = 0; xi<num; xi++)
		{
			xx[xi*num + 0] = m_sample - 2;
			xx[xi*num + 1] = m_sample - 1;
			xx[xi*num + 2] = m_sample;
			xx[xi*num + 3] = m_sample + 1;
			xx[xi*num + 4] = m_sample + 2;
		}

		for (int yi = 0; yi<num; yi++)
		{
			yy[yi + num * 0] = m_line - 2;
			yy[yi + num * 1] = m_line - 1;
			yy[yi + num * 2] = m_line;
			yy[yi + num * 3] = m_line + 1;
			yy[yi + num * 4] = m_line + 2;
		}
		
		double aa[6], aaaa[6 * 6], aabb[6];
		memset(aa, 0, sizeof(double) * 6);
		memset(aaaa, 0, sizeof(double) * 6 * 6);
		memset(aabb, 0, sizeof(double) * 6);

		for (int ii = 0; ii<num*num; ii++)
		{
			memset(aa, 0, sizeof(double) * 6);

			aa[0] = 1;
			aa[1] = xx[ii];
			aa[2] = yy[ii];
			aa[3] = xx[ii] * xx[ii];
			aa[4] = xx[ii] * yy[ii];
			aa[5] = yy[ii] * yy[ii];

			nValue[ii] = IMAGEFILE.GetDataValue(xx[ii], yy[ii], 0, 0, 1);

			mbase.pNormal(aa, 6, nValue[ii], aaaa, aabb, 1.0);
		}

		memset(a, 0, sizeof(double) * 6);
		mbase.Gauss(aaaa, aabb, 6);
		memcpy(a, aabb, sizeof(double) * 6);

		if (a[3] != 0 && a[3]<1e6)
		{
			fm_line = (2 * a[2] * a[3] - a[1] * a[4]) / (a[4] * a[4] - 4 * a[3] * a[5]);
			fm_sample = -(a[4] * fm_line + a[1]) / (2 * a[3]);
		}

		else if (a[4] != 0)
		{
			fm_sample = (a[2] * a[4] - 2 * a[1] * a[5]) / (4 * a[3] * a[5] - a[4] * a[4]);
			fm_line = -(a[1] + 2 * a[3] * fm_sample) / a[4];
		}

	}
	else if (num == 7)
	{
		double nValue[7 * 7];
		memset(nValue, 0, sizeof(double) * 7 * 7);

		double xx[7 * 7], yy[7 * 7];
		memset(xx, 0, sizeof(double) * 7 * 7);
		memset(yy, 0, sizeof(double) * 7 * 7);

		for (int xi = 0; xi<num; xi++)
		{
			xx[xi*num + 0] = m_sample - 3;
			xx[xi*num + 1] = m_sample - 2;
			xx[xi*num + 2] = m_sample - 1;
			xx[xi*num + 3] = m_sample;
			xx[xi*num + 4] = m_sample + 1;
			xx[xi*num + 5] = m_sample + 2;
			xx[xi*num + 6] = m_sample + 3;
		}

		for (int yi = 0; yi<num; yi++)
		{
			yy[yi + num * 0] = m_line - 3;
			yy[yi + num * 1] = m_line - 2;
			yy[yi + num * 2] = m_line - 1;
			yy[yi + num * 3] = m_line;
			yy[yi + num * 4] = m_line + 1;
			yy[yi + num * 5] = m_line + 2;
			yy[yi + num * 6] = m_line + 3;
		}

		double aa[6], aaaa[6 * 6], aabb[6];
		memset(aa, 0, sizeof(double) * 6);
		memset(aaaa, 0, sizeof(double) * 6 * 6);
		memset(aabb, 0, sizeof(double) * 6);

		for (int ii = 0; ii<num*num; ii++)
		{
			memset(aa, 0, sizeof(double) * 6);

			aa[0] = 1;
			aa[1] = xx[ii];
			aa[2] = yy[ii];
			aa[3] = xx[ii] * xx[ii];
			aa[4] = xx[ii] * yy[ii];
			aa[5] = yy[ii] * yy[ii];

			nValue[ii] = IMAGEFILE.GetDataValue(xx[ii], yy[ii], 0, 0, 1);

			mbase.pNormal(aa, 6, nValue[ii], aaaa, aabb, 1.0);
		}

		memset(a, 0, sizeof(double) * 6);
		mbase.Gauss(aaaa, aabb, 6);
		memcpy(a, aabb, sizeof(double) * 6);

		if (a[3] != 0 && a[3]<1e6)
		{
			fm_line = (2 * a[2] * a[3] - a[1] * a[4]) / (a[4] * a[4] - 4 * a[3] * a[5]);
			fm_sample = -(a[4] * fm_line + a[1]) / (2 * a[3]);
		}

		else if (a[4] != 0)
		{
			fm_sample = (a[2] * a[4] - 2 * a[1] * a[5]) / (4 * a[3] * a[5] - a[4] * a[4]);
			fm_line = -(a[1] + 2 * a[3] * fm_sample) / a[4];
		}

	}
	else if (num == 9)
	{
		double nValue[9 * 9];
		memset(nValue, 0, sizeof(double) * 9 * 9);

		double xx[9 * 9], yy[9 * 9];
		memset(xx, 0, sizeof(double) * 9 * 9);
		memset(yy, 0, sizeof(double) * 9 * 9);

		for (int xi = 0; xi<num; xi++)
		{
			xx[xi*num + 0] = m_sample - 4;
			xx[xi*num + 1] = m_sample - 3;
			xx[xi*num + 2] = m_sample - 2;
			xx[xi*num + 3] = m_sample - 1;
			xx[xi*num + 4] = m_sample;
			xx[xi*num + 5] = m_sample + 1;
			xx[xi*num + 6] = m_sample + 2;
			xx[xi*num + 7] = m_sample + 3;
			xx[xi*num + 8] = m_sample + 4;
		}

		for (int yi = 0; yi<num; yi++)
		{
			yy[yi + num * 0] = m_line - 4;
			yy[yi + num * 1] = m_line - 3;
			yy[yi + num * 2] = m_line - 2;
			yy[yi + num * 3] = m_line - 1;
			yy[yi + num * 4] = m_line;
			yy[yi + num * 5] = m_line + 1;
			yy[yi + num * 6] = m_line + 2;
			yy[yi + num * 7] = m_line + 3;
			yy[yi + num * 8] = m_line + 4;
		}

		double aa[6], aaaa[6 * 6], aabb[6];
		memset(aa, 0, sizeof(double) * 6);
		memset(aaaa, 0, sizeof(double) * 6 * 6);
		memset(aabb, 0, sizeof(double) * 6);

		for (int ii = 0; ii<num*num; ii++)
		{
			memset(aa, 0, sizeof(double) * 6);

			aa[0] = 1;
			aa[1] = xx[ii];
			aa[2] = yy[ii];
			aa[3] = xx[ii] * xx[ii];
			aa[4] = xx[ii] * yy[ii];
			aa[5] = yy[ii] * yy[ii];

			nValue[ii] = IMAGEFILE.GetDataValue(xx[ii], yy[ii], 0, 0, 1);

			mbase.pNormal(aa, 6, nValue[ii], aaaa, aabb, 1.0);
		}

		memset(a, 0, sizeof(double) * 6);
		mbase.Gauss(aaaa, aabb, 6);
		memcpy(a, aabb, sizeof(double) * 6);

		if (a[3] != 0 && a[3]<1e6)
		{
			fm_line = (2 * a[2] * a[3] - a[1] * a[4]) / (a[4] * a[4] - 4 * a[3] * a[5]);
			fm_sample = -(a[4] * fm_line + a[1]) / (2 * a[3]);
		}

		else if (a[4] != 0)
		{
			fm_sample = (a[2] * a[4] - 2 * a[1] * a[5]) / (4 * a[3] * a[5] - a[4] * a[4]);
			fm_line = -(a[1] + 2 * a[3] * fm_sample) / a[4];
		}

	}
	else
	{
		printf("输入的拟合点数有误，请输入3或5或7或9！（本版本目前仅支持9点或25点或49点或81点曲面拟合！）");
		return FALSE;
	}
	return TRUE;
}