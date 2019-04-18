#include "StarExtract.h"

StarExtract::StarExtract()
{
}


StarExtract::~StarExtract()
{
}

//////////////////////////////////////////////////////////////////////////
//���ܣ���ȡ��ͼ�ı�������
//���룺
//�����
//ע�⣺
//���ߣ�GZC
//���ڣ�2019.04.17
//////////////////////////////////////////////////////////////////////////
void StarExtract::StarCameraBackground(int index1,int index2)
{
	GeoReadImage ImgStarMap, ImgBackground, ImgBW;
	char pathtmp[512];
	width = 4096;
	height = 2048;
	string path = "C:\\Users\\wcsgz\\Downloads\\����0����Ʒ\\";
	ImgBackground.New(path+"����һ�ű�������.tif", "GTiff", GDT_UInt32, width, height, 1);
	ImgBackground.Destroy();
	ImgBackground.Open(path + "����һ�ű�������.tif", GA_ReadOnly);
	ImgBackground.ReadBlock(0, 0, width, height, 0, ImgBackground.pBuffer[0]);

	int *ImgDN = new int[width * height];
	int *ImgDNcount = new int[width * height];
	memset(ImgDN, 0, sizeof(int) * width * height);
	memset(ImgDNcount, 0, sizeof(int) * width * height);
	for (int a = index1; a < index2; a++)
	{
		if (a % 10 == 0)
		{
			printf("\r���ڶ�ȡ��%d��", a);
		}
		if (a==77)
		{
			a++;
		}
		sprintf_s(pathtmp, "%04d_L0.tif", a);
		string Tifpath =path + "LuoJia1-01_LR201904012992_HDR_" + string(pathtmp);
		ImgStarMap.Open(Tifpath, GA_ReadOnly);
		ImgStarMap.ReadBlock(0, 0, width, height, 0, ImgStarMap.pBuffer[0]);

		for (long i = 0; i < height; i++)
		{
			for (long j = 0; j < width; j++)
			{
				ImgDN[width * i + j] += ImgStarMap.GetDataValue(j, i, 0, 0);
				ImgDNcount[width * i + j]++;
				//int StarMapDN, BackgrounDN;
				//StarMapDN = ImgStarMap.GetDataValue(j, i, 0, 0);
				////if (StarMapDN > 255) StarMapDN = 255;
				//BackgrounDN = ImgBackground.GetDataValue(j, i, 0, 0);
				//if ((StarMapDN - BackgrounDN) > 3)//��ֵѡ��
				//{
				//	ImgDN[width * i + j] = StarMapDN - BackgrounDN;
				//	ImgDNcount[width * i + j]++;
				//}
			}
		}
	}
	sprintf_s(pathtmp, "%d-%d���ǵ���ӽ��.tiff", index1,index2);
	string pathres = path + (string)pathtmp;
	ImgBW.New(pathres, "GTiff", GDT_UInt32, width, height, 1);
	ImgBW.Destroy();
	ImgBW.Open(pathres, GA_Update);
	ImgBW.SetBuffer(0, 0, width, height, ImgBW.pBuffer[0]);
	for (long i = 0; i < height; i++)
	{
		for (long j = 0; j < width; j++)
		{
			if (ImgDNcount[width * i + j] != 0)
			{
				double DN = ImgDN[width * i + j] / ImgDNcount[width * i + j];
				ImgBW.SetDataValue(j, i, DN, 0);
			}
			else
			{
				ImgBW.SetDataValue(j, i, 0, 0);
			}
		}
	}
	ImgBW.WriteBlock(0, 0, width, height, 0, ImgBW.pBuffer[0]);

	free(ImgDN);
	free(ImgDNcount);
	ImgStarMap.Destroy();
	ImgBackground.Destroy();
	ImgBW.Destroy();
}

//////////////////////////////////////////////////////////////////////////
//���ܣ���ȡ�ǵ�����
//���룺�������������빤��·��
//������ڹ���·��������ǵ���ȡ�ļ�
//ע�⣺������GeoReadImage�Ķ�����Ҫ������
//���ߣ�GZC
//���ڣ�2017.01.06
//////////////////////////////////////////////////////////////////////////
void StarExtract::StarPointExtraction(int index)
{
	GeoReadImage ImgStarMap;
	GeoReadImage ImgBackground, ImgBW, ImgStarPoint;
	char pathtmp[512];
	sprintf_s(pathtmp, "��ͼ (%d).tiff", index);
	string path = workpath + string(pathtmp);
	ImgStarMap.Open(path, GA_ReadOnly);
	width = ImgStarMap.m_xRasterSize;
	height = ImgStarMap.m_yRasterSize;
	ImgStarMap.ReadBlock(0, 0, width, height, 0, ImgStarMap.pBuffer[0]);
		
	path = "D:\\2_ImageData\\ZY3-02\\��ͼ����\\����������3.tiff";
	ImgBackground.Open(path, GA_ReadOnly);
	ImgBackground.ReadBlock(0, 0, 1024, 1024, 0, ImgBackground.pBuffer[0]);

	sprintf_s(pathtmp, "�ǵ���ȡ��� (%d).tiff", index);
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
			if ((StarMapDN - BackgrounDN) > 3)//��ֵѡ��
			{
				ImgBW.SetDataValue(j, i, 255, 0);
			}
			else
			{
				ImgBW.SetDataValue(j, i, 0, 0);
			}
		}
	}
	ImgBW.WriteBlock(0, 0, width, height, 0, ImgBW.pBuffer[0]);
	//��ȡ��ֵͼ����ȡ���Ƶ㡣
	bwlabel(ImgBW, ImgStarMap);
	//�����ǵ���ȡ����(�����½���)
	//GetPreciseXYbySubDevision(index);

	ImgStarMap.Destroy();
	ImgBackground.Destroy();
	ImgBW.Destroy();
}

//////////////////////////////////////////////////////////////////////////
//���ܣ����ǵ��������
//���룺�������������빤��·��
//������ڹ���·��������ǵ�����ļ�
//ע�⣺������GeoReadImage�Ķ�����Ҫ������
//���ߣ�GZC
//���ڣ�2017.03.08
//////////////////////////////////////////////////////////////////////////
void StarExtract::StarPointMulti(int index)
{
	GeoReadImage ImgStarMap, ImgBackground, ImgBW;
	char pathtmp[512]; 
	width = 1024;
	height = 1024;
	string path = "D:\\2_ImageData\\ZY3-02\\��ͼ����\\����������3.tiff";
	ImgBackground.Open(path, GA_ReadOnly);
	ImgBackground.ReadBlock(0, 0, width, height, 0, ImgBackground.pBuffer[0]);

	double *ImgDN = new double[1024 * 1024];
	double *ImgDNcount = new double[1024 * 1024];
	memset(ImgDN, 0, sizeof(double) * 1024 * 1024);
	memset(ImgDNcount, 0, sizeof(double)*1024*1024);
	for (int a = 0; a < index; a++)
	{
		if (a%100==0)
		{
			printf("\r���ڶ�ȡ��%d��", a);
		}
		sprintf_s(pathtmp, "��ͼ (%d).tiff", a+1);
		path = workpath + string(pathtmp);
		ImgStarMap.Open(path, GA_ReadOnly);		
		ImgStarMap.ReadBlock(0, 0, width, height, 0, ImgStarMap.pBuffer[0]);
	
		for (long i = 0; i < height; i++)
		{
			for (long j = 0; j < width; j++)
			{
				double StarMapDN, BackgrounDN;
				StarMapDN = ImgStarMap.GetDataValue(j, i, 0, 0);
				if (StarMapDN > 255) StarMapDN = 255;
				BackgrounDN = ImgBackground.GetDataValue(j, i, 0, 0);
				if ((StarMapDN - BackgrounDN) > 3)//��ֵѡ��
				{
					ImgDN[1024 * i + j] = StarMapDN - BackgrounDN;
					ImgDNcount[1024 * i + j]++;
				}
			}
		}
	}
	sprintf_s(pathtmp, "%d���ǵ���ӽ��.tiff", index);
	string pathres = workpath + (string)pathtmp;
	ImgBW.New(pathres, "GTiff", ImgStarMap.m_BandType, width, height, 1);
	ImgBW.Destroy();
	ImgBW.Open(pathres, GA_Update);
	ImgBW.SetBuffer(0, 0, width, height, ImgBW.pBuffer[0]);
	for (long i = 0; i < height; i++)
	{
		for (long j = 0; j < width; j++)
		{
			if (ImgDNcount[1024 * i + j]!=0)
			{
				double DN = ImgDN[1024 * i + j] / ImgDNcount[1024 * i + j];
				ImgBW.SetDataValue(j, i, DN, 0);
			}
			else
			{
				ImgBW.SetDataValue(j, i, 0, 0);
			}
		}
	}
	ImgBW.WriteBlock(0, 0, width, height, 0, ImgBW.pBuffer[0]);

	free(ImgDN);
	free(ImgDNcount);
	ImgStarMap.Destroy();
	ImgBackground.Destroy();
	ImgBW.Destroy();
}

//////////////////////////////////////////////////////////////////////////
//���ܣ��Զ�ֵͼ����ͨ����б��
//���룺GeoReadImage &ImgBW����ȡ��ֵӰ��
//��������Ա����StarPointExtract���ǵ���ȡvector�ṹ��
//ע�⣺������GeoReadImage�Ķ�����Ҫ������
//���ߣ�GZC
//���ڣ�2017.01.06
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
					//sum_x += (rowRun[i]+1) *DN*DN*DN;//����x����Ϊ�ع���
					//sum_y += (col + 1)*DN*DN*DN;//����y����Ϊ������
					sum_x += (rowRun[i] + 0.5) *pow(DN,3);//����x����Ϊ�ع���
					sum_y += (col + 0.5)*pow(DN, 3);//����y����Ϊ������
					area += pow(DN, 3);
					areaNum[k-1]++;
				}
			}
		}
		//double px = sum_x / area;
		//double py = sum_y / area;
		plot_x[k - 1] = sum_x / area;
		plot_y[k - 1] = sum_y / area;

		//GetPreciseXYbyFitting(py, px, plot_y[k - 1], plot_x[k - 1], 9, ImgStarMap);

		double DN;
		//����ȡ���߽������ֵ
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
				DN = *(max_element(DNall, DNall + 9));//C++ STL���㷨		
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
	
	//ɸѡ����9�����ص��ǵ�
	for (int i = 0; i < equaListsize; i++)
	{
		//if (areaNum[i]>=9)
			if (areaNum[i] >= 9|| StarMag[i]<6.5)
		{
			StarPoint StarPointExtractTmp;
			StarPointExtractTmp.x = plot_x[i];
			StarPointExtractTmp.y = plot_y[i];
			StarPointExtractTmp.Mag = StarMag[i];
			StarPointExtract.push_back(StarPointExtractTmp);			
		}
	}

	/*string path = workpath + "�ǵ���ȡ���.txt";
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
//���ܣ��������������Ƕ���ͨ������б�ǵ�С����
//			 �����������������㣬Ȼ��Բ�ͬ���������������ӵķ�ʽ
//���룺ImgBW����ֵӰ��NumberOfRuns��������������
//			 stRun������������ʼ�У�enRun������������ֹ�У�rowRun��������
//�����runLabels�������ı�ǩ������equivalence���ȼ۵ı�ǩ
//ע�⣺������GeoReadImage�Ķ�����Ҫ������
//���ߣ�GZC
//���ڣ�2017.01.06
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
// ��run���һ��ͼ���ϵ�һ�������� ��ɫ������
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
		// ����Ǹ��еĵ�һ��run���������һ�е�һ��run�����һ��run�����
		if (rowRun[i] != curRowIdx)
		{
			curRowIdx = rowRun[i]; // �����е����
			firstRunOnPre = firstRunOnCur;
			lastRunOnPre = i - 1;
			firstRunOnCur = i;
		}
		// ������һ�е�����run���ж��Ƿ��ڵ�ǰrun���غϵ�����
		for (int j = firstRunOnPre; j <= lastRunOnPre; j++)
		{
			// �����غ� �� �������ڵ�����
			if (stRun[i] <= enRun[j] + offset && enRun[i] >= stRun[j] - offset && rowRun[i] == rowRun[j] + 1)
			{
				if (runLabels[i] == 0) // û�б���Ź�
					runLabels[i] = runLabels[j];
				else if (runLabels[i] != runLabels[j])// �Ѿ������            
					equivalences.push_back(make_pair(runLabels[i], runLabels[j])); // ����ȼ۶�
			}
		}
		if (runLabels[i] == 0) // û����ǰһ�е��κ�run�غ�
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
//���ܣ�������������ǵ�����
//���룺Ԥ���ǵ����꣺m_sample��m_line��
//			 Ӱ�����ݣ�IMAGEFILE
//������������ǵ����꣺fm_sample��fm_line��
//			 ���ڴ�С��num
//ע�⣺����SAR������
//���ߣ�ԭ��ZRS����д��GZC
//���ڣ�2017.01.06
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

		//if (a[3] != 0 && a[3]<1e6)
		//{
			fm_line = (2 * a[2] * a[3] - a[1] * a[4]) / (a[4] * a[4] - 4 * a[3] * a[5]);
			fm_sample = -(a[4] * fm_line + a[1]) / (2 * a[3]);
		//}

		//else if (a[4] != 0)
		//{
			fm_sample = (a[2] * a[4] - 2 * a[1] * a[5]) / (4 * a[3] * a[5] - a[4] * a[4]);
			fm_line = -(a[1] + 2 * a[3] * fm_sample) / a[4];
		//}

	}
	else
	{
		printf("�������ϵ�������������3��5��7��9�������汾Ŀǰ��֧��9���25���49���81��������ϣ���");
		return FALSE;
	}
	return TRUE;
}

//////////////////////////////////////////////////////////////////////////
//���ܣ�����ϸ�ַ�����ǵ����꾫��
//���룺StarPointExtract�����п��Ƶ��Ա
//�����StarPointExtract�����п��Ƶ��Ա�����£�
//ע�⣺���ڴ�СΪ3
//���ߣ�ԭ��ZRS����д��GZC
//���ڣ�2017.03.27
//////////////////////////////////////////////////////////////////////////
bool StarExtract::GetPreciseXYbySubDevision(int index)
{
	vector<StarPoint>StarPointExtractDel;
	StarPoint StarPointExtractDelTmp;
	int pointnum = StarPointExtract.size();
	for (size_t i = 0; i < pointnum; i++)
	{
		if (StarPointExtract[i].x > 4 && StarPointExtract[i].x < 1020 && StarPointExtract[i].y > 4 && StarPointExtract[i].y < 1020)
		{
			StarPointExtractDelTmp = StarPointExtract[i];
			StarPointExtractDel.push_back(StarPointExtractDelTmp);
		}
	}
	StarPointExtract.clear();
	pointnum = StarPointExtractDel.size();
	double *sample0 = new double[pointnum];
	double *line0 = new double[pointnum];
	double *centerX = new double[pointnum];
	double *centerY = new double[pointnum];
	for (size_t i = 0; i < pointnum; i++)
	{
		sample0[i] = StarPointExtractDel[i].x;
		line0[i] = StarPointExtractDel[i].y;
	}
	StarPointExtractDel.clear();
	StarPoint StarPointExtractTmp;

	//��������
	int num_range = 3;	//��Ϸ�Χ
	int num_subdivision = 100;	//ϸ�ֳߴ�
	
	float intervalSubdivision = 1 / (float)num_subdivision;
	int width_subdivision = (num_range - 1)*num_subdivision;
	int height_subdivision = (num_range - 1)*num_subdivision;
	
	//��tiff�ļ�
	string path;
	char pathtmp[512];
	sprintf_s(pathtmp, "��ͼ (%d).tiff", index);
	path = workpath + (string)pathtmp;
	GDALDataset *pImgIn = (GDALDataset *)GDALOpen(path.c_str(), GA_ReadOnly);
	if (pImgIn == NULL)
	{
		printf("Read tiff failed!\n");
		return 0;
	}
	GDALRasterBand *poBand;
	poBand = pImgIn->GetRasterBand(1);
	GDALDataType aa = poBand->GetRasterDataType();

	//��ȡ��Ϸ�Χ������ֵ
	void *tempValue = (void*)malloc(num_range*num_range * sizeof(double));
	float *pValue0 = new float[num_range*num_range];
	float *x0 = new float[num_range], *y0 = new float[num_range];

	float *xc = new float[height_subdivision];	//ÿһ�е���������
	float *pValueC = new float[height_subdivision];	//ÿһ�е�ƽ���Ҷ�ֵ

	float *pValueD = new float[width_subdivision];	//ϸ�ֺ������ֵ
	float *xd = new float[width_subdivision], *yd = new float[height_subdivision];	//ϸ�ֺ������

	float numerator_inter, numerator_divi;	//ÿ�����ļ��㹫ʽ��ķ���
	float denominator_inter, denominator_divi;	//ÿ�����ļ��㹫ʽ��ķ�ĸ
	float numerator_cx, numerator_cy;	//���Ĺ�ʽ����x,y
	float denominator_c;

	for (int j = 0; j<pointnum; j++)
	{
		memset(pValue0, 0, sizeof(float)*num_range*num_range);
		memset(x0, 0, sizeof(float)*num_range);
		memset(y0, 0, sizeof(float)*num_range);
		memset(xc, 0, sizeof(float)*height_subdivision);
		memset(pValueC, 0, sizeof(float)*height_subdivision);
		memset(pValueD, 0, sizeof(float)*width_subdivision);
		memset(xd, 0, sizeof(float)*width_subdivision);
		memset(yd, 0, sizeof(float)*height_subdivision);
		numerator_inter = 0;
		numerator_divi = 0;
		denominator_inter = 0;
		denominator_divi = 0;
		numerator_cx = 0;
		numerator_cy = 0;
		denominator_c = 0;

		//��ȡ��Ϸ�Χ������ֵ������
		int tempSAMPLE, tempLINE;
		tempSAMPLE = (floor)(sample0[j]);
		tempLINE = (floor)(line0[j]);

		pImgIn->RasterIO(GF_Read, tempSAMPLE - num_range / 2, tempLINE - num_range / 2, num_range, num_range, tempValue,
			num_range, num_range, aa, 1, 0, 0, 0, 0);

		//printf("  Pixel value aquired from tiff:\n");
		for (int t = 0; t<num_range*num_range; t++)
		{
			if (aa == GDT_Byte)	pValue0[t] = ((unsigned char*)tempValue)[t];
			else if (aa == GDT_UInt16)	pValue0[t] = ((unsigned short*)tempValue)[t];
			else if (aa == GDT_Int16)	pValue0[t] = ((short*)tempValue)[t];
			else if (aa == GDT_UInt32)	pValue0[t] = ((unsigned long*)tempValue)[t];
			else if (aa == GDT_Int32)	pValue0[t] = ((int*)tempValue)[t];
			else if (aa == GDT_Float32)	pValue0[t] = ((float*)tempValue)[t];
			else if (aa == GDT_Float64)	pValue0[t] = ((double*)tempValue)[t];
			else { printf("Data type error!\n"); return 0; }

			/*printf("  %f ", pValue0[t]);
			if ((t + 1) % num_range == 0)
			{
				printf("\n");
			}*/
		}
		//printf("\n");

		for (int p = 0; p<num_range; p++)
		{
			x0[p] = (tempSAMPLE + 0.5) - num_range / 2 + p;
			y0[p] = (tempLINE + 0.5) - num_range / 2 + p;
		}

		//printf("  Pixel value and coordinates of insert point: [pixel value] [x] [y]\n");
		//for (int p = 0; p<num_range; p++)
		//{

		//	for (int n = 0; n<num_range; n++)
		//	{
		//		printf("  %f %f %f\n", pValue0[p*num_range + n], x0[n], y0[p]);
		//	}

		//	printf("\n");
		//}
		//		printf("\n");


		//ϸ�ֺ������
		for (int d = 0; d<width_subdivision; d++)
		{
			xd[d] = x0[0] /*+ intervalSubdivision/2*/ + d*intervalSubdivision;
			yd[d] = y0[0] /*+ intervalSubdivision/2*/ + d*intervalSubdivision;
		}


		//��ÿ�е����������ƽ���Ҷ�ֵ
		for (int l = 0; l<height_subdivision; l++)
		{
			int nx0, ny0;	//ϸ��ʱ��Ҫ���ڲ�����к�
			ny0 = (floor)(yd[l] - y0[0]);

			float totalValue = 0.0;

			//˫�����ڲ�ÿ��ϸ�����صĻҶ�ֵ
			for (int s = 0; s<width_subdivision; s++)
			{
				nx0 = (floor)(xd[s] - x0[0]);

				// 				pValueD[s] = pValue0[nx0+ny0*num_range]*(x0[nx0+1]-xd[s])*(y0[ny0+1]-yd[l])/((x0[nx0+1]-x0[nx0])*(y0[ny0+1]-y0[ny0]))
				// 					+pValue0[nx0+1+ny0*num_range]*(xd[s]-x0[nx0])*(y0[ny0+1]-yd[l])/((x0[nx0+1]-x0[nx0])*(y0[ny0+1]-y0[ny0]))
				// 					+pValue0[nx0+(ny0+1)*num_range]*(x0[nx0+1]-xd[s])*(yd[l]-y0[ny0])/((x0[nx0+1]-x0[nx0])*(y0[ny0+1]-y0[ny0]))
				// 					+pValue0[nx0+1+(ny0+1)*num_range]*(xd[s]-x0[nx0])*(yd[l]-y0[ny0])/((x0[nx0+1]-x0[nx0])*(y0[ny0+1]-y0[ny0]));

				float uu, vv;
				uu = xd[s] - x0[nx0];
				vv = yd[l] - y0[ny0];

				pValueD[s] = (1 - uu) * (1 - vv) * pValue0[nx0 + ny0*num_range]
					+ (1 - uu) * vv * pValue0[nx0 + (ny0 + 1)*num_range]
					+ uu * (1 - vv) * pValue0[nx0 + 1 + ny0*num_range]
					+ uu * vv * pValue0[nx0 + 1 + (ny0 + 1)*num_range];


				totalValue += pValueD[s];
			}

			//ÿ�е�ƽ������ֵ
			pValueC[l] = totalValue / width_subdivision;

			//����ÿ���������깫ʽ��ķ������ĸ
			for (int n = 0; n<num_range; n++)
			{
				numerator_inter += x0[n] * pValue0[n + ny0*num_range];
				denominator_inter += pValue0[n + ny0*num_range];
			}

			for (int m = 0; m<width_subdivision; m++)
			{
				numerator_divi += xd[m] * pValueD[m];
				denominator_divi += pValueD[m];
			}

			//ÿ�е���������
			xc[l] = (numerator_inter + numerator_divi) / (denominator_inter + denominator_divi);
			//			printf("[%d] %f ", l, xc[l]);
		}
		//		printf("\n");

		//��������Ŀ�������������깫ʽ�ķ������ĸ
		for (int m = 0; m<height_subdivision; m++)
		{
			numerator_cx += xc[m] * pValueC[m];
			numerator_cy += yd[m] * pValueC[m];
			denominator_c += pValueC[m];
		}

		//��������Ŀ��������������
		centerX[j] = numerator_cx / denominator_c;
		centerY[j] = numerator_cy / denominator_c;
		StarPointExtractTmp.x = centerX[j];
		StarPointExtractTmp.y = centerY[j];
		StarPointExtract.push_back(StarPointExtractTmp);
	}


	delete[] tempValue; tempValue = NULL;
	delete[] pValue0; pValue0 = NULL;
	delete[] x0; x0 = NULL;
	delete[] y0; y0 = NULL;
	delete[] pValueC; pValueC = NULL;
	delete[] xc; xc = NULL;
	delete[] pValueD; pValueD = NULL;
	delete[] xd; xd = NULL;


	if (pImgIn != NULL)
	{
		delete[]pImgIn;
		pImgIn = NULL;
	}

	delete[] sample0; sample0 = NULL;
	delete[] line0; line0 = NULL;
	delete[] centerX; centerX = NULL;
	delete[] centerY; centerY = NULL;

	//GDALDestroyDriverManager();

	//printf("Finished!\n");
	return 0;
}