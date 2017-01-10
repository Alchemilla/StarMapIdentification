#include "ParseSTG.h"

//////////////////////////////////////////////////////////////////////////
//���ܣ�����STG�ļ��е���Ԫ������Ϣ
//���룺STGpath��STG�ļ�·��
//�����ZY3_02STGdata������̬���������ɵĽṹ��
//ע�⣺������ΪAPS��������
//���ڣ�2016.07.01
//////////////////////////////////////////////////////////////////////////
void ParseSTG::ParseZY302_STG(string STGpath, vector<STGData> &ZY3_02STGdata)
{
	ifstream in;
	in.open(STGpath.c_str(), ios::binary);

	in.seekg(0, ios::end);
	ios::pos_type ss = in.tellg();
	int nSize = (int)ss;
	nSize = nSize / 120;
	in.seekg(0, ios::beg);

	int i;
	double ta, tb, tc;
	int ka = 0, kb = 0, kc = 0;
	STGData *ZY_3Data = new STGData[nSize];
	memset(ZY_3Data, 0, 4096);
	for (i = 0; i < nSize; i++)
	{
		in.seekg(4, ios::cur);
		unsigned char a[8];
		in.read((char*)&a, 8);
		ta = mBase.RevDouble(a);
		ZY_3Data[i].StarA.UTC = ta;
		//Ϊ�������STG��������ӵ��ж�
		if (ka == 0) { ZY_3Data[i].StarA.UTC = ta; ka++; }
		else if (ta - ZY_3Data[i - 1].StarA.UTC < -0.3) { ZY_3Data[i].StarA.UTC = ta + 1; }
		else if (ta - ZY_3Data[i - 1].StarA.UTC > 0.3) { ZY_3Data[i].StarA.UTC = ta - 1; }
		else { ZY_3Data[i].StarA.UTC = ta; }
		in.seekg(1, ios::cur);
		long Q1temp, Q2temp, Q3temp;
		in.read((char*)&Q1temp, 4);
		ZY_3Data[i].StarA.Q1 = mBase.ReverseQ(Q1temp);
		in.read((char*)&Q2temp, 4);
		ZY_3Data[i].StarA.Q2 = mBase.ReverseQ(Q2temp);
		in.read((char*)&Q3temp, 4);
		ZY_3Data[i].StarA.Q3 = mBase.ReverseQ(Q3temp);
		ZY_3Data[i].StarA.Q0 = sqrt(1 - ZY_3Data[i].StarA.Q1*ZY_3Data[i].StarA.Q1 - ZY_3Data[i].StarA.Q2*ZY_3Data[i].StarA.Q2 - ZY_3Data[i].StarA.Q3*ZY_3Data[i].StarA.Q3);

		in.read((char*)&a, 8);
		tb = mBase.RevDouble(a);
		ZY_3Data[i].StarB.UTC = tb;
		//Ϊ�������STG��������ӵ��ж�
		if (kb == 0) { ZY_3Data[i].StarB.UTC = tb; kb++; }
		else if (tb - ZY_3Data[i - 1].StarB.UTC < -0.3) { ZY_3Data[i].StarB.UTC = tb + 1; }
		else if (tb - ZY_3Data[i - 1].StarB.UTC > 0.3) { ZY_3Data[i].StarB.UTC = tb - 1; }
		else { ZY_3Data[i].StarB.UTC = tb; }
		in.seekg(1, ios::cur);
		in.read((char*)&Q1temp, 4);
		ZY_3Data[i].StarB.Q1 = mBase.ReverseQ(Q1temp);
		in.read((char*)&Q2temp, 4);
		ZY_3Data[i].StarB.Q2 = mBase.ReverseQ(Q2temp);
		in.read((char*)&Q3temp, 4);
		ZY_3Data[i].StarB.Q3 = mBase.ReverseQ(Q3temp);
		ZY_3Data[i].StarB.Q0 = sqrt(1 - ZY_3Data[i].StarB.Q1*ZY_3Data[i].StarB.Q1 - ZY_3Data[i].StarB.Q2*ZY_3Data[i].StarB.Q2 - ZY_3Data[i].StarB.Q3*ZY_3Data[i].StarB.Q3);

		in.read((char*)&a, 8);
		tc = mBase.RevDouble(a);
		ZY_3Data[i].StarC.UTC = tc;
		//Ϊ�������STG��������ӵ��ж�
		if (kc == 0) { ZY_3Data[i].StarC.UTC = tc; kc++; }
		else if (tc - ZY_3Data[i - 1].StarC.UTC < -0.3) { ZY_3Data[i].StarC.UTC = tc + 1; }
		else if (tc - ZY_3Data[i - 1].StarC.UTC > 0.3) { ZY_3Data[i].StarC.UTC = tc - 1; }
		else { ZY_3Data[i].StarC.UTC = tc; }
		in.seekg(1, ios::cur);
		in.read((char*)&Q1temp, 4);
		ZY_3Data[i].StarC.Q1 = mBase.ReverseQ(Q1temp);
		in.read((char*)&Q2temp, 4);
		ZY_3Data[i].StarC.Q2 = mBase.ReverseQ(Q2temp);
		in.read((char*)&Q3temp, 4);
		ZY_3Data[i].StarC.Q3 = mBase.ReverseQ(Q3temp);
		ZY_3Data[i].StarC.Q0 = sqrt(1 - ZY_3Data[i].StarC.Q1*ZY_3Data[i].StarC.Q1 - ZY_3Data[i].StarC.Q2*ZY_3Data[i].StarC.Q2 - ZY_3Data[i].StarC.Q3*ZY_3Data[i].StarC.Q3);

		in.read((char*)&a, 8);
		ZY_3Data[i].utgyro = mBase.RevDouble(a);
		short gy;
		in.read((char*)&gy, 2);
		ZY_3Data[i].g1 = gy*0.000022 / 0.25;
		//ZY_3Data[i].g1=gy*0.000022/0.25*1.115454545;//ZY3-01�����ݵı���ϵ��
		in.read((char*)&gy, 2);
		ZY_3Data[i].g2 = gy*0.000022 / 0.25;
		in.read((char*)&gy, 2);
		ZY_3Data[i].g3 = gy*0.000022 / 0.25;
		//ZY_3Data[i].g3=gy*0.000022/0.25*1.073181818;//ZY3-01�����ݵı���ϵ��
		in.read((char*)&gy, 2);
		ZY_3Data[i].g4 = gy*0.000022 / 0.25;
		in.read((char*)&gy, 2);
		ZY_3Data[i].g5 = gy*0.000022 / 0.25;
		//ZY_3Data[i].g5=gy*0.000022/0.25*1.103181818;//ZY3-01�����ݵı���ϵ��
		in.read((char*)&gy, 2);
		ZY_3Data[i].g6 = gy*0.000022 / 0.25;
		in.read((char*)&gy, 2);
		ZY_3Data[i].g7 = gy*0.00005 / 0.25;
		in.read((char*)&gy, 2);
		ZY_3Data[i].g8 = gy*0.00005 / 0.25;
		in.read((char*)&gy, 2);
		ZY_3Data[i].g9 = gy*0.00005 / 0.25;

		long bsx, bsy, bsz;
		in.read((char*)&bsx, 4);
		in.read((char*)&bsy, 4);
		in.read((char*)&bsz, 4);
		ZY_3Data[i].bx = mBase.Reverse2(bsx);
		ZY_3Data[i].by = mBase.Reverse2(bsy);
		ZY_3Data[i].bz = mBase.Reverse2(bsz);

		in.seekg(3, ios::cur);
		in.read((char*)&gy, 2);
		ZY_3Data[i].g10 = gy*0.000025 / 0.25;
		in.read((char*)&gy, 2);
		ZY_3Data[i].g11 = gy*0.000025 / 0.25;
		in.read((char*)&gy, 2);
		ZY_3Data[i].g12 = gy*0.000025 / 0.25;

		char sign;
		in.read((char*)&sign, 1);
		//�ж�����B��C�ķ���
		if (sign == 0)
		{
		}
		else if (sign == 1)
		{
			ZY_3Data[i].StarB.Q0 = -ZY_3Data[i].StarB.Q0;
		}
		else if (sign == 2)
		{
			ZY_3Data[i].StarC.Q0 = -ZY_3Data[i].StarC.Q0;
		}
		else if (sign == 3)
		{
			ZY_3Data[i].StarB.Q0 = -ZY_3Data[i].StarB.Q0;
			ZY_3Data[i].StarC.Q0 = -ZY_3Data[i].StarC.Q0;
		}
		ZY3_02STGdata.push_back(ZY_3Data[i]);
		in.seekg(5, ios::cur);
	}

	string path = string(STGpath);
	path = path.substr(0, path.rfind('.'));
	string strpath = path + "_STG_parse.txt";
	FILE *fpres = fopen(strpath.c_str(), "w");
	fprintf(fpres,"%d\t",nSize);
	for (i = 0; i < nSize; i++)
	{
		fprintf(fpres, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t", ZY_3Data[i].StarA.UTC, ZY_3Data[i].StarA.Q0, ZY_3Data[i].StarA.Q1, ZY_3Data[i].StarA.Q2, ZY_3Data[i].StarA.Q3);
		fprintf(fpres, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t", ZY_3Data[i].StarB.UTC, ZY_3Data[i].StarB.Q0, ZY_3Data[i].StarB.Q1, ZY_3Data[i].StarB.Q2, ZY_3Data[i].StarB.Q3);
		fprintf(fpres, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t", ZY_3Data[i].StarC.UTC, ZY_3Data[i].StarC.Q0, ZY_3Data[i].StarC.Q1, ZY_3Data[i].StarC.Q2, ZY_3Data[i].StarC.Q3);
		fprintf(fpres, "%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n", ZY_3Data[i].utgyro, ZY_3Data[i].g1,
			ZY_3Data[i].g2, ZY_3Data[i].g3, ZY_3Data[i].g4, ZY_3Data[i].g5, ZY_3Data[i].g6, ZY_3Data[i].g7, ZY_3Data[i].g8, ZY_3Data[i].g9, ZY_3Data[i].g10,
			ZY_3Data[i].g11, ZY_3Data[i].g12, ZY_3Data[i].bx, ZY_3Data[i].by, ZY_3Data[i].bz);
	}
	fcloseall;

}

//////////////////////////////////////////////////////////////////////////
//���ܣ�����STI�ļ��е���ͼ
//���룺STIpath��STI�ļ�·��
//��������������Ŀ¼
//ע�⣺������ΪAPS��������
//���ڣ�2017.01.08
//////////////////////////////////////////////////////////////////////////
bool ParseSTG::ParseZY302_STI(string STIpath)
{
	FILE *fp = fopen(STIpath.c_str(), "rb");
	if (!fp)
	{
		printf("Failed to open the RAW File(%s)\n", STIpath.c_str());
		return false;
	}

	//һ�ν��ļ�ȫ�������ڴ�pData
	_fseeki64(fp, 0, SEEK_END);
	__int64 nSize = _ftelli64(fp);
	_fseeki64(fp, 0, SEEK_SET);
	byte *pData = new byte[nSize];
	if (fread(pData, sizeof(byte), nSize, fp) != nSize)
	{
		fclose(fp);
		return false;
	}
	fclose(fp);	fp = NULL;


	long width = 1024, height = 1024;// Ӱ��Ŀ�Ⱥ͸߶�
	unsigned short *pFred = new unsigned short[width * height];	     //����ֵ
	int nSizeFred = nSize / 1310736;   //֡��
	nSizeFred = 1;//������10����˵
	vector<vector<double>>pUnitData(nSizeFred, vector<double>(1024 * 1024));//�ö�άvector������ྰӰ��

	// �ۼ����2009��1��1��0ʱ��ʼ����
	double jd0, refMJD;
	Cal2JD(2009, 1, 1, 0, &jd0, &refMJD);
	string outpath = workpath + "��ʱ.txt";
	FILE *ftime = fopen(outpath.c_str(), "w");
	if (!ftime)
	{
		printf("Failed to open the RAW File(%s)\n", outpath);
		return false;
	}
	fprintf(ftime, "֡��\t ��ʱ1(�ۼ��룬����̬����ļ���Ӧ)\t��ʱ2��YYYY:MM:DD��\n");	
	char chr[41], c[10];
	bitset<8> bit8;
	string str;
	unsigned int tmp, tmps = 0, tmpms = 0;
	double  second;
	double *UT = new double[nSizeFred];
	int year, month, day, hour, minute;
	//���Ӱ���ʽ
	string outdriver = "GTiff";	    //Ҳ���Դ�Ϊbmp
	GeoReadImage m_out;
	//ͶӰ
	string tarProject;
	tarProject = "PROJCS[\"UTM_Zone_50N\", GEOGCS[\"GCS_WGS_1984\", DATUM[\"WGS_1984\", SPHEROID[\"WGS_1984\", 6378137.0, 298.2572235630016],TOWGS84[0,0,0,0,0,0,0]], PRIMEM[\"Greenwich\", 0.0], UNIT[\"Degree\", 0.0174532925199433]], PROJECTION[\"Transverse_Mercator\"], PARAMETER[\"False_Easting\", 500000.0], PARAMETER[\"False_Northing\", 0.0], PARAMETER[\"Central_Meridian\", 117.0], PARAMETER[\"Scale_Factor\", 0.9996], PARAMETER[\"Latitude_Of_Origin\", 0.0], UNIT[\"Meter\", 1.0]]";
	//����任
	double minx = 0, maxy = 1024, resolution = 1;
	double adfGeoTransform[6] = { minx, resolution, 0, maxy, 0, -resolution };
	for (int i = 0; i < nSizeFred; i++)
	{
		//��ʱ
		//����
		memcpy(&bit8, pData + i * 1310736 + 4, sizeof(byte));
		tmps = pow(2, 24)* bit8.to_ulong();
		memcpy(&bit8, pData + i * 1310736 + 5, sizeof(byte));
		tmps += pow(2, 16)* bit8.to_ulong();
		memcpy(&bit8, pData + i * 1310736 + 6, sizeof(byte));
		tmps += pow(2, 8)* bit8.to_ulong();
		memcpy(&bit8, pData + i * 1310736 + 7, sizeof(byte));
		tmps += pow(2, 0)* bit8.to_ulong();
		//΢��
		memcpy(&bit8, pData + i * 1310736 + 8, sizeof(byte));
		tmpms = pow(2, 16)* bit8.to_ulong();
		memcpy(&bit8, pData + i * 1310736 + 9, sizeof(byte));
		tmpms += pow(2, 8)* bit8.to_ulong();
		memcpy(&bit8, pData + i * 1310736 + 10, sizeof(byte));
		tmpms += pow(2, 0)* bit8.to_ulong();
		//�ۼ���
		UT[i] = tmps + tmpms / 1000000.0;
		//����ʱ
		FromSecondtoYMD(refMJD, UT[i], year, month, day, hour, minute, second);
		fprintf(ftime, "%3d\t%16.6lf\n", i + 1, UT[i]);
		//fprintf(ftime, "%04d:%02d:%02d:%02d:%02d:%06.3lf\n", year, month, day, hour, minute, second);

		//������֯һ֡����
		for (int j = 0; j < 262144; j++)
		{
			//ÿ�δ��ڴ�ȡ5Bת��Ϊ4������ֵ������pFred
			for (int k = 0; k < 5; k++)
			{
				memcpy(&bit8, pData + i * 1310736 + 15 + j * 5 + k, sizeof(byte));
				str = bit8.to_string();
				strcpy(chr + k * 8, str.c_str());
			}
			for (int k = 0; k < 4; k++)
			{
				memcpy(c, &chr[10 * k], sizeof(char) * 10);
				pFred[j * 4 + k] = (c[0] - 48) * 512 + (c[1] - 48) * 256 + (c[2] - 48) * 128 + (c[3] - 48) * 64 + (c[4] - 48) * 32
					+ (c[5] - 48) * 16 + (c[6] - 48) * 8 + (c[7] - 48) * 4 + (c[8] - 48) * 2 + (c[9] - 48) * 1;
			}
		}
		//����Ӱ��
		char tempath[100];
		sprintf(tempath, "��ͼ (%d).tiff", i + 1);
		string imgpath = workpath + "��ͼ\\" + tempath;
		m_out.New(imgpath, outdriver, GDT_Byte, width, height, 1);
		m_out.poDataset->SetProjection(tarProject.c_str());
		m_out.poDataset->SetGeoTransform(adfGeoTransform);

		//���·�ʽ��Ӱ��
		m_out.Open(imgpath, GA_Update);
		if (m_out.m_isopen == true)
			printf("\rUpdate Img (%s)", imgpath.c_str());
		else
		{
			printf("\rUpdate Img (%s) Failed", imgpath.c_str());
			return 0;
		}
		//����out������
		m_out.SetBuffer(0, 0, width, height, m_out.pBuffer[0]);
		double gray;
		for (long y = 0; y < height; y++)       //y����
		{
			for (long x = 0; x < width; x++)   //x����
			{
				//��������
				gray = pFred[y*width + x];
				if (gray > 255)gray = 255;
				//���ͼ������ϵ˳ʱ��90��תΪ������ƽ������ϵ
				m_out.SetDataValue(x, y, gray, 0);    //��ֵ
			}
		}
		//д������
		bool iswrite = true;
		iswrite *= m_out.WriteBlock(0, 0, width, height, 0, m_out.pBuffer[0]);
		//�ر�Ӱ��
		m_out.Destroy();
			
		int rate = 100 * (i+1) / nSizeFred;
		printf("\r�Ѵ���������%d;  Process:%d%%", i+1, rate);               //�������
	}
	if (pFred != NULL)		delete[]pFred;       pFred = NULL;
	if (pData != NULL)		delete[]pData;		 pData = NULL;

	//printf("\n******************************************************************************\nstep2:Ӱ�����\n");
	////////////////////////////////////////////////
	////2���ڴ���pUnitData���ݣ������Ӱ��
	////////////////////////////////////////////////
	//// ���Ӱ�����
	//string outdriver = "GTiff";	    //Ҳ���Դ�Ϊbmp
	//GeoReadImage *m_out = new GeoReadImage[nSizeFred + 1];
	////ͶӰ
	//string tarProject;
	//tarProject = "PROJCS[\"UTM_Zone_50N\", GEOGCS[\"GCS_WGS_1984\", DATUM[\"WGS_1984\", SPHEROID[\"WGS_1984\", 6378137.0, 298.2572235630016],TOWGS84[0,0,0,0,0,0,0]], PRIMEM[\"Greenwich\", 0.0], UNIT[\"Degree\", 0.0174532925199433]], PROJECTION[\"Transverse_Mercator\"], PARAMETER[\"False_Easting\", 500000.0], PARAMETER[\"False_Northing\", 0.0], PARAMETER[\"Central_Meridian\", 117.0], PARAMETER[\"Scale_Factor\", 0.9996], PARAMETER[\"Latitude_Of_Origin\", 0.0], UNIT[\"Meter\", 1.0]]";
	////����任
	//double minx = 0, maxy = 1024, resolution = 1;
	//double adfGeoTransform[6] = { minx, resolution, 0, maxy, 0, -resolution };
	//for (int k = 0; k < nSizeFred; k++)
	//{
	//	//����Ӱ��
	//	char tempath[100];
	//	//sprintf(tempath, "ԭʼ��ͼ%02d_UT%.4lf.tiff", k + 1, UT[k]);
	//	sprintf(tempath, "��ͼ (%02d).tiff", k + 1);
	//	string imgpath = workpath + "��ͼ\\" + tempath;
	//	m_out[k].New(imgpath, outdriver, GDT_Byte, width, height, 1);
	//	m_out[k].poDataset->SetProjection(tarProject.c_str());
	//	m_out[k].poDataset->SetGeoTransform(adfGeoTransform);
	//	//���·�ʽ��Ӱ��
	//	m_out[k].Open(imgpath, GA_Update);
	//	if (m_out[k].m_isopen == true)
	//		printf("\rUpdate Img (%s)", imgpath.c_str());
	//	else
	//	{
	//		printf("\rUpdate Img (%s) Failed", imgpath.c_str());
	//		return 0;
	//	}
	//	//����out������
	//	m_out[k].SetBuffer(0, 0, width, height, m_out[k].pBuffer[0]);
	//	double gray;
	//	for (long y = 0; y < height; y++)       //y����
	//	{
	//		for (long x = 0; x < width; x++)   //x����
	//		{
	//			//��������
	//			gray = pUnitData[k][y*width + x];
	//			if (gray > 255)gray = 255;
	//			//���ͼ������ϵ˳ʱ��90��תΪ������ƽ������ϵ
	//			m_out[k].SetDataValue(x, y, gray, 0);    //��ֵ
	//		}
	//	}
	//	//д������
	//	bool iswrite = true;
	//	iswrite *= m_out[k].WriteBlock(0, 0, width, height, 0, m_out[k].pBuffer[0]);
	//	//�ر�Ӱ��
	//	m_out[k].Destroy();
	//}
	//fclose(ftime);  ftime = NULL;
	//pUnitData.empty();

	//printf("\n*******************************************************************************\nstep3:��ȡ���\n");
	//unsigned short *pNoise = new unsigned short[width * height];	 //��ձ��������)
	//GeoReadImage mNoise;	
	//string Noisepath = workpath + "��ͼ\\��������.tiff";
	//mNoise.New(Noisepath, outdriver, GDT_Byte, width, height, 1);
	//mNoise.Open(Noisepath, GA_Update);
	////����������
	//mNoise.SetBuffer(0, 0, width, height, mNoise.pBuffer[0]);
	//for (int y = 0; y < height; y++)       //y����
	//{
	//	for (int x = 0; x < width; x++)   //x����
	//	{
	//		int sum = 0;
	//		//����Ӱ��Ҷ�ֵ
	//		for (int k = 0; k < nSizeFred; k++)
	//		{
	//			int gray = pUnitData[k][y*width + x];
	//			sum += gray;
	//		}
	//		pNoise[y*width + x] = sum / nSizeFred;
	//		mNoise.SetDataValue(x, y, pNoise[y*width + x], 0);    //��ֵ
	//		 //д������
	//		bool iswrite = true;
	//		iswrite *= mNoise.WriteBlock(0, 0, width, height, 0, mNoise.pBuffer[0]);			
	//	}
	//	//�������
	//	int rate = 100 * y / height;
	//	printf("\rProcessed:%d%%", rate);
	//}
	////�ر�Ӱ��
	//mNoise.Destroy();

	//printf("\n*******************************************************************************\nstep3:��ȡ�ǵ�\n");
	////////////////////////////////////////////////
	////3��step3:��ȡ�ǵ�
	////////////////////////////////////////////////
	//unsigned short *pNoise = new unsigned short[width * height];	 //��ձ��������)
	//sprintf(outpath, "E:\\02������Ŀ\\05����ռ�Ŀ�����\\10�ӿ�����\\��Դ����02��ͼ\\��ȡ�ǵ�\\����Ӱ��Ҷ�ֵ.txt");
	//FILE *fgray = fopen(outpath, "w");
	//if (!fgray)
	//{
	//	printf("Failed to open the RAW File(%s)\n", outpath);
	//	return false;
	//}
	//sprintf(outpath, "E:\\02������Ŀ\\05����ռ�Ŀ�����\\10�ӿ�����\\��Դ����02��ͼ\\��ȡ�ǵ�\\�ǵ�����.txt");
	//FILE *fstar = fopen(outpath, "w");
	//if (!fgray)
	//{
	//	printf("Failed to open the RAW File(%s)\n", outpath);
	//	return false;
	//}
	//fprintf(fstar, "֡��\tx\ty\tgray\tnoise\n");
	//for (int y = 0; y < height; y++)       //y����
	//{
	//	for (int x = 0; x < width; x++)   //x����
	//	{
	//		int sum = 0;
	//		//����Ӱ��Ҷ�ֵ
	//		fprintf(fgray, "(%4d,%4d)\t", x, y);
	//		for (int k = 0; k < nSizeFred; k++)
	//		{
	//			int gray = pUnitData[k].imgdata[y*width + x];
	//			fprintf(fgray, "%3d\t", gray);
	//			sum += gray;
	//		}
	//		fprintf(fgray, "\n");
	//		//�ǵ�����
	//		pNoise[y*width + x] = sum / nSizeFred;
	//		for (int k = 0; k < nSizeFred; k++)
	//		{
	//			int gray = pUnitData[k].imgdata[y*width + x];
	//			if (gray - pNoise[y*width + x] < 5)
	//			{
	//				pUnitData[k].imgdata[y*width + x] = 0;
	//			}
	//			else
	//			{
	//				fprintf(fstar, "%4d\t%4d\t%4d\t%3d\t%3d\n", k, x, y, gray, pNoise[y*width + x]);
	//			}
	//		}
	//		//�������
	//		int rate = 100 * (y*width + x) / width / height;
	//		printf("\rProncessing(%4d,%4d);  Processed:%d%%", x, y, rate);
	//	}
	//}
	//fclose(ftime);  ftime = NULL;
	//fclose(fstar);  fstar = NULL;
	//fclose(fgray);  fgray = NULL;

	//printf("*******************************************************************************\nstep4:�ǵ�Ӱ�����\n");
	////////////////////////////////////////////////
	////4���ǵ�Ӱ�����
	////////////////////////////////////////////////
	//// ���Ӱ�����
	////string outdriver = "GTiff";
	////GeoReadImage *m_out = new GeoReadImage[nSizeFred];
	//////ͶӰ
	////string tarProject;
	////tarProject = "PROJCS[\"UTM_Zone_50N\", GEOGCS[\"GCS_WGS_1984\", DATUM[\"WGS_1984\", SPHEROID[\"WGS_1984\", 6378137.0, 298.2572235630016],TOWGS84[0,0,0,0,0,0,0]], PRIMEM[\"Greenwich\", 0.0], UNIT[\"Degree\", 0.0174532925199433]], PROJECTION[\"Transverse_Mercator\"], PARAMETER[\"False_Easting\", 500000.0], PARAMETER[\"False_Northing\", 0.0], PARAMETER[\"Central_Meridian\", 117.0], PARAMETER[\"Scale_Factor\", 0.9996], PARAMETER[\"Latitude_Of_Origin\", 0.0], UNIT[\"Meter\", 1.0]]";
	//////����任
	////double minx = 0, maxy = 1024, resolution = 1;
	////double adfGeoTransform[6] = { minx, resolution, 0, maxy, 0, -resolution };
	//for (int k = 0; k < nSizeFred + 1; k++)
	//{
	//	//����Ӱ��
	//	sprintf(outpath, "E:\\02������Ŀ\\05����ռ�Ŀ�����\\10�ӿ�����\\��Դ����02��ͼ\\��ȡ�ǵ�\\�ǵ�Ӱ��%02d.tif", k + 1);
	//	if (k == nSizeFred)sprintf(outpath, "E:\\02������Ŀ\\05����ռ�Ŀ�����\\10�ӿ�����\\��Դ����02��ͼ\\���.tif");
	//	m_out[k].New(outpath, outdriver, GDT_Byte, width, height, 1);
	//	//m_out[k].poDataset->SetProjection(tarProject.c_str());
	//	//m_out[k].poDataset->SetGeoTransform(adfGeoTransform);
	//	//���·�ʽ��Ӱ��
	//	m_out[k].Open(outpath, GA_Update, false);
	//	if (m_out[k].m_isopen == true)
	//		printf("\rUpdate Img (%s)", outpath);
	//	else
	//	{
	//		printf("\rUpdate Img (%s) Failed", outpath);
	//		return 0;
	//	}
	//	//����out������
	//	m_out[k].SetBuffer(0, 0, width, height, m_out[k].pBuffer[0]);
	//	double gray;
	//	int Xm, Ym;
	//	for (int y = 0; y < height; y++)       //y����
	//	{
	//		for (int x = 0; x < width; x++)   //x����
	//		{
	//			//��������
	//			if (k < nSizeFred)gray = pUnitData[k].imgdata[y*width + x];
	//			else gray = pNoise[y*width + x];
	//			if (gray > 255)gray = 255;
	//			//���ͼ������ϵ˳ʱ��90��תΪ������ƽ������ϵ
	//			Xm = y, Ym = 1024 - x;
	//			m_out[k].SetDataValue(Xm, Ym, gray, 0);    //��ֵ
	//		}
	//	}
	//	//д������
	//	bool iswrite = true;
	//	iswrite *= m_out[k].WriteBlock(0, 0, width, height, 0, m_out[k].pBuffer[0]);
	//	//�ر�Ӱ��
	//	m_out[k].Destroy();
	//}
	//if (pNoise != NULL)		delete[]pNoise, pNoise = NULL;
	//if (pUnitData != NULL)		delete[]pUnitData, pUnitData = NULL;
	return true;
}

//////////////////////////////////////////////////////////////////////////
//���ܣ����ݺ��ǳྭ��γ��������̬������������������
//���룺starCatlog�����ź��ǲ���������J2000ϵ�µĵ�λʸ��V[3]
//		R����������ϵ��J2000ϵ����ת��ϵ��Crj
//�����x,y����������
//ע�⣺������ΪAPS��������
//���ڣ�2016.11.01
//////////////////////////////////////////////////////////////////////////
void ParseSTG::FromLL2XY(Star starCatlog, double *R, double &x, double &y)
{
	double V[3], W[3];
	V[0] = starCatlog.V[0]; V[1] = starCatlog.V[1]; V[2] = starCatlog.V[2];
	mBase.Multi(R, V, W, 3, 3, 1);
	//x0=y0=512,f=43.3mm,��Ԫ��С0.015mm
	if (W[2] > 0)
	{
		x = (512 * 0.015 - W[0] / W[2] * 43.3) / 0.015;
		y = (512 * 0.015 - W[1] / W[2] * 43.3) / 0.015;
	}
	else
	{
		x = -1, y = -1;
	}//����������ж��Ƿ������ָ��İ�����һ��
}

//////////////////////////////////////////////////////////////////////////
//���ܣ����ݺ������ȵȼ������������ͼ������DNֵ
//���룺Mag:�����ǵȣ�
//�����DNֵ
//ע�⣺�����ǵ�3Ϊ255���ȣ�С���ǵ�3��ҲΪ255
//���ڣ�2016.12.08
//////////////////////////////////////////////////////////////////////////
int ParseSTG::Mag2DN(double Mag)
{
	int DN = 166 * pow(100, (3 - Mag) / 5)+65;
	return DN;
}

//////////////////////////////////////////////////////////////////////////
//���ܣ���������APS������Ԫ��������ͼ
//���룺ZY3_02STGdata��STG����������̬����
//�������Ԫ����Ӧ��ͼ
//ע�⣺����AΪAPS��������
//���ڣ�2016.10.31
//////////////////////////////////////////////////////////////////////////
void ParseSTG::StarMap(vector<STGData> ZY3_02STGdata)
{
	//���Ǳ��ļ�
	string starCatlogpath = "D:\\2_ImageData\\ZY3-02\\��ͼ����\\star1";
	FILE *fp;
	fp = fopen(starCatlogpath.c_str(), "r");
	if (fp == NULL)
	{
		printf("��%s�ļ�ʧ��,��ȷ���ļ�·���Ƿ���ȷ!\n", starCatlogpath.c_str());
		return;
	}
	int i, n;
	fscanf(fp, "%d", &n);//��ȡ�Ǳ�����
	Star *starCatlog = new Star[n];
	for (i = 0; i < n; i++)
	{
		fscanf(fp, "%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%d\n", &starCatlog[i].ID, &starCatlog[i].phiX, &starCatlog[i].phiY,
			&starCatlog[i].mag, &starCatlog[i].V[0], &starCatlog[i].V[1], &starCatlog[i].V[2], &starCatlog[i].DN);
	}

	int j, m = ZY3_02STGdata.size();
	double R[9], za[3] = { 0,0,1 }, zc[3];
	double px, py, x, y;
	// Ӱ��Ŀ�Ⱥ͸߶�
	long width = 1024, height = 1024;
	//������ƽ������ֵ����
	byte *UnitData = new byte[1024 * 1024];

	//������ͼ�ļ���
	string imgtmp = workpath + "��ͼ";
	char * imgpath = (char *)imgtmp.data();
	if (_mkdir(imgpath) == 0);
	string imgtxt = workpath + "��ͼ\\��������.txt";
	FILE *fptxt = fopen(imgtxt.c_str(), "w");

	//������һ֡����������������	
	mBase.quat2matrix(ZY3_02STGdata[0].StarA.Q1, ZY3_02STGdata[0].StarA.Q2,
		ZY3_02STGdata[0].StarA.Q3, ZY3_02STGdata[0].StarA.Q0, R);//Crj
	for (j = 0; j < n; j++)
	{
		FromLL2XY(starCatlog[j], R, x, y);//���Ǳ�ÿ���Ǳ�����������������
		if (x > 0 && x < 1024 && y>0 && y < 1024)
		{
			/*int xPixel = int(x + 0.5);
			int yPixel = int(y + 0.5);*/
			int xPixel = int(y + 0.5);
			int yPixel = 1024 - int(x + 0.5);
			UnitData[yPixel*width + xPixel] = starCatlog[j].DN;
			fprintf(fptxt, "%d\t%d\t%.1f\n", xPixel, yPixel, starCatlog[j].mag);
		}
	}
	fclose(fptxt);

	//������̬�³���ͼ
	m = 1;
	for (i=0;i<m;i++)
	//for (i = 900; i < 1100; i++)
	{
		memset(UnitData, 0, sizeof(byte) * 1024 * 1024);//Ӱ������ֵ��Ϊ0
		//����A��APS��������
		mBase.quat2matrix(ZY3_02STGdata[i].StarA.Q1,ZY3_02STGdata[i].StarA.Q2,
			ZY3_02STGdata[i].StarA.Q3,ZY3_02STGdata[i].StarA.Q0,R);//Crj
		//����B����
		//mBase.quat2matrix(ZY3_02STGdata[i].StarB.Q1, ZY3_02STGdata[i].StarB.Q2,
		//	ZY3_02STGdata[i].StarB.Q3, ZY3_02STGdata[i].StarB.Q0, R);//Crj
		for (j = 0; j < n; j++)
		{
			FromLL2XY(starCatlog[j], R, x, y);//���Ǳ�ÿ���Ǳ�����������������
			if (x > 2 && x < 1022 && y>2 && y < 1022)
			{
				int xPixel = int(x + 0.5) - 2;
				int yPixel = int(y + 0.5) - 2;
				for (int ii = 0; ii < 5; ii++)//����5�����ش�С����ͼ
				{
					for (int jj = 0; jj < 5; jj++)
					{
						//UnitData[yPixel*width + xPixel] = starCatlog[j].DN;
						//�� X��Y�� ת��Ϊ X��Y��
						int xTrans = yPixel;
						int yTrans = 1024 - xPixel;
						int DN = Mag2DN(starCatlog[j].mag);
						if (DN > 255) DN = 255;
						UnitData[yTrans*width + xTrans] = DN;
						//UnitData[yTrans*width + xTrans] = starCatlog[j].mag*10;
						xPixel++;
					}
					xPixel -= 5;
					yPixel++;
				}
			}
		}
		//����Ӱ��
		char tempath[100];
		// ���Ӱ�����
		string outdriver = "GTiff";
		//ͶӰ
		string tarProject = "PROJCS[\"UTM_Zone_50N\", GEOGCS[\"GCS_WGS_1984\", DATUM[\"WGS_1984\", SPHEROID[\"WGS_1984\", 6378137.0, 298.2572235630016],TOWGS84[0,0,0,0,0,0,0]], PRIMEM[\"Greenwich\", 0.0], UNIT[\"Degree\", 0.0174532925199433]], PROJECTION[\"Transverse_Mercator\"], PARAMETER[\"False_Easting\", 500000.0], PARAMETER[\"False_Northing\", 0.0], PARAMETER[\"Central_Meridian\", 117.0], PARAMETER[\"Scale_Factor\", 0.9996], PARAMETER[\"Latitude_Of_Origin\", 0.0], UNIT[\"Meter\", 1.0]]";
		//����任
		double minx = 0, maxy = 1024, resolution = 1;
		double adfGeoTransform[6] = { minx, resolution, 0, maxy, 0, -resolution };
		sprintf(tempath, "��ͼ (%d) APS��.tiff", i+1);
		//sprintf(tempath, "����B���� (%d).tiff", i + 1, ZY3_02STGdata[i].StarB.UTC);
		string imgpath = workpath + "��ͼ\\" + tempath;
		GeoReadImage m_out;
		m_out.New(imgpath, outdriver, GDT_Byte, width, height, 1);
		m_out.poDataset->SetProjection(tarProject.c_str());
		m_out.poDataset->SetGeoTransform(adfGeoTransform);
		//���·�ʽ��Ӱ��
		m_out.Open(imgpath, GA_Update);
		if (m_out.m_isopen == true)
			printf("\rUpdate Img (%s)", imgpath.c_str());
		else
		{
			printf("\rUpdate Img (%s) Failed", imgpath.c_str());
			return;
		}
		//����out������
		m_out.SetBuffer(0, 0, width, height, m_out.pBuffer[0]);
		double gray;
		for (int yPixel = 0; yPixel < height; yPixel++)       //y����
		{
			for (int xPixel = 0; xPixel < width; xPixel++)   //x����
			{
				//��������
				gray = UnitData[yPixel*width + xPixel];
				//gray = 0;
				m_out.SetDataValue(xPixel, yPixel, gray, 0);    //��ֵ
			}
		}
		//д������
		bool iswrite = true;
		iswrite *= m_out.WriteBlock(0, 0, width, height, 0, m_out.pBuffer[0]);
		//�ر�Ӱ��
		m_out.Destroy();
	}
}

//////////////////////////////////////////////////////////////////////////
//���ܣ�����APS��ͼʶ�����������´�STG���ݶԱ�
//���룺ZY3_02STGdata��STG����������̬����
//      IDpath����ͼʶ��õ���APS������Ԫ��
//�����ͬĿ¼�¶ԱȽ��
//ע�⣺����AΪAPS��������
//���ڣ�2016.11.02
//////////////////////////////////////////////////////////////////////////
//void ParseSTG::StarIDComp(vector<STGData> ZY3_02STGdata,string IDpath)
//{
//	FILE *fp;
//	fp = fopen(IDpath.c_str(),"r");
//	int i,m,n;
//	fscanf(fp,"%d",&m);//��ͼʶ����Ԫ������ 2Hz
//	n = ZY3_02STGdata.size();//STG����������Ԫ������ 4Hz
//	Quat *Qid = new Quat[m];
//	Quat *Qidint = new Quat[n];
//	double *UT = new double[n];
//	for (i=0;i<m;i++)
//	{
//		fscanf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%*lf\t%*lf\n",&Qid[i].UTC,&Qid[i].Q0,&Qid[i].Q1,&Qid[i].Q2,&Qid[i].Q3);
//	}
//	for (i=0;i<n;i++)
//	{
//		UT[i] = ZY3_02STGdata[i].StarA.UTC;
//	}
//	QuatInterpolation(Qid,m,UT,n,Qidint);//��̬�ڲ�
//	
//	string resPath = workpath + "StarId_Compare.txt";
//	FILE *fpres;
//	fpres = fopen(resPath.c_str(),"w");
//	//Ȼ����бȽ����
//	double qL[4],qR[4],dQ[4];
//	SateEuler *EulerArray = new SateEuler[n];
//	for (i=0;i<n;i++)
//	{
//		qL[0]=ZY3_02STGdata[i].StarA.Q0,qL[1]=ZY3_02STGdata[i].StarA.Q1,
//		qL[2]=ZY3_02STGdata[i].StarA.Q2,qL[3]=ZY3_02STGdata[i].StarA.Q3;
//		qR[0]=-Qidint[i].Q0,qR[1]=Qidint[i].Q1,qR[2]=Qidint[i].Q2,qR[3]=Qidint[i].Q3;
//		quatmult(qL,qR,dQ);
//		dQ[1]=2*dQ[1]/PI*180*3600;dQ[2]=2*dQ[2]/PI*180*3600;dQ[3]=2*dQ[3]/PI*180*3600;
//		fprintf(fpres,"%.9lf\t%.9lf\t%.9lf\n",dQ[1],dQ[2],dQ[3]);
//		EulerArray[i].R = dQ[1];	
//		EulerArray[i].P = dQ[2];	
//		EulerArray[i].Y = dQ[3];
//		EulerArray[i].UTC = UT[i]-97499271;
//	}
//	Sim.MatPlotDetQ(EulerArray,n);
//}

//////////////////////////////////////////////////////////////////////////
//���ܣ�APS��ͼʶ������Ԫ��������B������C����֮��ļнǹ�ϵ
//���룺ZY3_02STGdata��STG����������̬����
//      IDpath����ͼʶ��õ���APS������Ԫ��
//      StarTag��intֵ��12��ʾ����A��B
//�����ͬĿ¼�£�����нǶԱȽ��
//ע�⣺����AΪAPS��������
//���ڣ�2016.11.02
//////////////////////////////////////////////////////////////////////////
void ParseSTG::StarAngleAPS_B_C(vector<STGData> ZY3_02STGdata, int StarTag)
{
	int i,m,n;
	m=ZY3_02STGdata.size();	
	//string IDpath = workpath + "0830_STG_parse.txt";
	//FILE *fp = fopen(IDpath.c_str(),"r");
	/*fscanf(fp,"%d\n",&n);*/
	Quat *StarA = new Quat[m];
	Quat *StarB = new Quat[m];
	Quat *StarC = new Quat[m];
	Quat *StarAi = new Quat[m];
	Quat *StarCi = new Quat[m];
	double *UT = new double[m];
	/*for (i=0;i<n;i++)
	{
		fscanf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%*lf\t%*lf\n",&StarA[i].UTC,&StarA[i].Q0,&StarA[i].Q1,&StarA[i].Q2,&StarA[i].Q3);
	}*/
	double mz[3]={0,0,1},RA[9],RB[9],za[3],zb[3],Angle;
	string strpath;
	if (StarTag==23)
	{
		for (i=0;i<m;i++)//����B��C���ݣ���BΪʱ���׼
		{
			UT[i]=ZY3_02STGdata.at(i).StarB.UTC;
			StarB[i].Q1=ZY3_02STGdata.at(i).StarB.Q1;StarB[i].Q2=ZY3_02STGdata.at(i).StarB.Q2;
			StarB[i].Q3=ZY3_02STGdata.at(i).StarB.Q3,StarB[i].Q0=ZY3_02STGdata.at(i).StarB.Q0;
			StarC[i].UTC=ZY3_02STGdata.at(i).StarC.UTC;
			StarC[i].Q1=ZY3_02STGdata.at(i).StarC.Q1;StarC[i].Q2=ZY3_02STGdata.at(i).StarC.Q2;
			StarC[i].Q3=ZY3_02STGdata.at(i).StarC.Q3,StarC[i].Q0=ZY3_02STGdata.at(i).StarC.Q0;
		}
		mBase.QuatInterpolation(StarC, m, UT, m,StarCi);
		strpath = workpath+"Star23Angle.txt";	
		FILE *fpres=fopen(strpath.c_str(),"w");
		for(int i=0;i<m;i++)
		{
			mBase.quat2matrix(StarB[i].Q1,StarB[i].Q2,StarB[i].Q3,StarB[i].Q0,RA);
			mBase.quat2matrix(StarCi[i].Q1,StarCi[i].Q2,StarCi[i].Q3,StarCi[i].Q0,RB);//Crj
			mBase.invers_matrix(RA,3);
			mBase.invers_matrix(RB,3);//Cjr
			mBase.Multi(RA,mz,za,3,3,1);
			mBase.Multi(RB,mz,zb,3,3,1);//���������ڹ���ϵ�е�����ֵ
			Angle=acos((za[0]*zb[0]+za[1]*zb[1]+za[2]*zb[2])/sqrt(za[0]*za[0]+za[1]*za[1]+za[2]*za[2])/sqrt(zb[0]*zb[0]+zb[1]*zb[1]+zb[2]*zb[2]))/PI*180;
			fprintf(fpres,"%.9f\t%.9f\n",UT[i],Angle);
		}
	} 
	else if(StarTag==12)
	{
		for (i=0;i<m;i++)//����A��B���ݣ���BΪʱ���׼
		{
			UT[i]=ZY3_02STGdata.at(i).StarB.UTC;
			StarB[i].Q1=ZY3_02STGdata.at(i).StarB.Q1;StarB[i].Q2=ZY3_02STGdata.at(i).StarB.Q2;
			StarB[i].Q3=ZY3_02STGdata.at(i).StarB.Q3,StarB[i].Q0=ZY3_02STGdata.at(i).StarB.Q0;
		}
		mBase.QuatInterpolation(StarA, m, UT, m, StarAi);
		strpath =workpath+"Star12Angle.txt";
		FILE *fpres=fopen(strpath.c_str(),"w");
		for(int i=0;i<m;i++)
		{
			mBase.quat2matrix(StarAi[i].Q1,StarAi[i].Q2,StarAi[i].Q3,StarAi[i].Q0,RA);
			mBase.quat2matrix(StarB[i].Q1,StarB[i].Q2,StarB[i].Q3,StarB[i].Q0,RB);//Crj
			mBase.invers_matrix(RA,3);
			mBase.invers_matrix(RB,3);//Cjr
			mBase.Multi(RA,mz,za,3,3,1);
			mBase.Multi(RB,mz,zb,3,3,1);//���������ڹ���ϵ�е�����ֵ
			Angle=acos((za[0]*zb[0]+za[1]*zb[1]+za[2]*zb[2])/sqrt(za[0]*za[0]+za[1]*za[1]+za[2]*za[2])/sqrt(zb[0]*zb[0]+zb[1]*zb[1]+zb[2]*zb[2]))/PI*180;
			fprintf(fpres,"%.9f\t%.9f\n",UT[i],Angle);
		}
	} 
	else if(StarTag==13)
	{
		for (i=0;i<m;i++)//����A��C���ݣ���CΪʱ���׼
		{
			UT[i]=ZY3_02STGdata.at(i).StarC.UTC;
			StarC[i].Q1=ZY3_02STGdata.at(i).StarC.Q1;StarC[i].Q2=ZY3_02STGdata.at(i).StarC.Q2;
			StarC[i].Q3=ZY3_02STGdata.at(i).StarC.Q3,StarC[i].Q0=ZY3_02STGdata.at(i).StarC.Q0;
		}
		mBase.QuatInterpolation(StarA, m, UT, m, StarAi);
		strpath =workpath+"Star13Angle.txt";
		FILE *fpres=fopen(strpath.c_str(),"w");
		for(int i=0;i<m;i++)
		{
			mBase.quat2matrix(StarAi[i].Q1,StarAi[i].Q2,StarAi[i].Q3,StarAi[i].Q0,RA);
			mBase.quat2matrix(StarC[i].Q1,StarC[i].Q2,StarC[i].Q3,StarC[i].Q0,RB);//Crj
			mBase.invers_matrix(RA,3);
			mBase.invers_matrix(RB,3);//Cjr
			mBase.Multi(RA,mz,za,3,3,1);
			mBase.Multi(RB,mz,zb,3,3,1);//���������ڹ���ϵ�е�����ֵ
			Angle=acos((za[0]*zb[0]+za[1]*zb[1]+za[2]*zb[2])/sqrt(za[0]*za[0]+za[1]*za[1]+za[2]*za[2])/sqrt(zb[0]*zb[0]+zb[1]*zb[1]+zb[2]*zb[2]))/PI*180;
			fprintf(fpres,"%.9f\t%.9f\n",UT[i],Angle);
		}
	} 	
}

//////////////////////////////////////////////////////////////////////////
//���ܣ�STG�������ݣ�APS������B������C����֮��ļнǹ�ϵ
//���룺ZY3_02STGdata��STG����������̬����
//          StarTag��intֵ��12��ʾ����A��B
//�����ͬĿ¼�£�����нǶԱȽ��
//ע�⣺����AΪAPS��������
//���ڣ�2016.11.02--2017.01.09
//////////////////////////////////////////////////////////////////////////
void ParseSTG::StarAngle(vector<STGData> StarDat, int StarTag)
{
	int m, n;
	m = StarDat.size();
	Quat *StarX = new Quat[m];
	Quat *StarY = new Quat[m];
	Quat *StarYi = new Quat[m];
	double *UTC = new double[m];
	string strpath;

	if (StarTag == 23)
	{
		for (int i = 0; i<m; i++)//����2��3����
		{
			UTC[i] = StarDat.at(i).StarB.UTC;
			StarX[i].Q1 = StarDat.at(i).StarB.Q1; StarX[i].Q2 = StarDat.at(i).StarB.Q2; StarX[i].Q3 = StarDat.at(i).StarB.Q3, StarX[i].Q0 = StarDat.at(i).StarB.Q0;
			StarY[i].UTC = StarDat.at(i).StarC.UTC;
			StarY[i].Q1 = StarDat.at(i).StarC.Q1; StarY[i].Q2 = StarDat.at(i).StarC.Q2; StarY[i].Q3 = StarDat.at(i).StarC.Q3, StarY[i].Q0 = StarDat.at(i).StarC.Q0;
		}
		mBase.QuatInterpolation(StarY, m, UTC, m, StarYi);
		strpath = workpath + "Star23Angle.txt";
	}
	else if (StarTag == 12)
	{
		for (int i = 0; i<m; i++)//����1��2����
		{
			UTC[i] = StarDat.at(i).StarA.UTC;
			StarX[i].Q1 = StarDat.at(i).StarA.Q1; StarX[i].Q2 = StarDat.at(i).StarA.Q2; StarX[i].Q3 = StarDat.at(i).StarA.Q3, StarX[i].Q0 = StarDat.at(i).StarA.Q0;
			StarY[i].UTC = StarDat.at(i).StarB.UTC;
			StarY[i].Q1 = StarDat.at(i).StarB.Q1; StarY[i].Q2 = StarDat.at(i).StarB.Q2; StarY[i].Q3 = StarDat.at(i).StarB.Q3, StarY[i].Q0 = StarDat.at(i).StarB.Q0;
		}
		mBase.QuatInterpolation(StarY, m, UTC, m, StarYi);
		strpath = workpath + "Star12Angle.txt";
	}
	else if (StarTag == 13)
	{
		for (int i = 0; i<m; i++)//����1��3����
		{
			UTC[i] = StarDat.at(i).StarA.UTC;
			StarX[i].Q1 = StarDat.at(i).StarA.Q1; StarX[i].Q2 = StarDat.at(i).StarA.Q2; StarX[i].Q3 = StarDat.at(i).StarA.Q3, StarX[i].Q0 = StarDat.at(i).StarA.Q0;
			StarY[i].UTC = StarDat.at(i).StarC.UTC;
			StarY[i].Q1 = StarDat.at(i).StarC.Q1; StarY[i].Q2 = StarDat.at(i).StarC.Q2; StarY[i].Q3 = StarDat.at(i).StarC.Q3, StarY[i].Q0 = StarDat.at(i).StarC.Q0;
		}
		mBase.QuatInterpolation(StarY, m, UTC, m, StarYi);
		strpath = workpath + "Star13Angle.txt";
	}

	FILE *fpres = fopen(strpath.c_str(), "w");
	fprintf(fpres, "����%d����\n", StarTag);
	double mz[3] = { 0,0,1 }, RA[9], RB[9], za[3], zb[3], Angle;
	for (int i = 0; i<m; i++)
	{
		mBase.quat2matrix(StarX[i].Q1, StarX[i].Q2, StarX[i].Q3, StarX[i].Q0, RA);
		mBase.quat2matrix(StarYi[i].Q1, StarYi[i].Q2, StarYi[i].Q3, StarYi[i].Q0, RB);//Crj
		mBase.invers_matrix(RA, 3);
		mBase.invers_matrix(RB, 3);//Cjr
		mBase.Multi(RA, mz, za, 3, 3, 1);
		mBase.Multi(RB, mz, zb, 3, 3, 1);//���������ڹ���ϵ�е�����ֵ
		Angle = acos((za[0] * zb[0] + za[1] * zb[1] + za[2] * zb[2]) / sqrt(za[0] * za[0] + za[1] * za[1] + za[2] * za[2]) / sqrt(zb[0] * zb[0] + zb[1] * zb[1] + zb[2] * zb[2])) / PI * 180;
		fprintf(fpres, "%.9f\t%.9f\n", UTC[i], Angle);
	}
}

//////////////////////////////////////////////////////////////////////////
//���ܣ���ȡAPS������ͼʶ����
//���룺IDpath��ʶ�����ļ�·��
//�����APS��Ӧ��Ԫ��
//ע�⣺ֻ�õ�������A
//���ڣ�2016.12.05
//////////////////////////////////////////////////////////////////////////
void ParseSTG::ReadStarID(string IDpath, vector<STGData>&APSQ)
{
	FILE *fp;
	fp = fopen(IDpath.c_str(), "r");
	int i, m;
	fscanf(fp, "%d", &m);//��ͼʶ����Ԫ������ 2Hz
	STGData APStemp;
	for (i = 0; i < m; i++)
	{
		fscanf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\t%*lf\t%*lf\n", &APStemp.StarA.UTC, &APStemp.StarA.Q0,
			&APStemp.StarA.Q1, &APStemp.StarA.Q2, &APStemp.StarA.Q3);
		APSQ.push_back(APStemp);
	}
}

//////////////////////////////////////////////////////////////////////////
//���ܣ���ȡSTG��ȡ������̬����STA
//���룺STA����·��
//�������ȡ��STA��������
//ע�⣺
//���ڣ�2016.12.08
//////////////////////////////////////////////////////////////////////////
void ParseSTG::ReadSTAtxt(string STApath, vector<STGData>&STAdat)
{
	FILE *fp;
	fp = fopen(STApath.c_str(), "r");
	int i, m;
	STGData STAtemp, STAtemp2;
	//fscanf(fp,"%*s\t%*s\t\n%*s\t%*s\t\n%d\n",&m);//����ǰ���ж�ȡ������
	fscanf(fp, "%*s\t%*s\t\n%*s\t%*s\t\n%d\n%*s\n", &m);//����ǰ���ж�ȡ�����У�˳������������
	fscanf(fp, "%*s\t%lf\t%*d\t%lf\t%lf\t%lf\t%lf\t%*d\t%lf\t%lf\t%lf\t%*d\t%lf\t%*d\t%lf\t%lf\t%lf\t%*d\n",
		&STAtemp2.StarA.UTC, &STAtemp2.StarA.Q1, &STAtemp2.StarA.Q2, &STAtemp2.StarA.Q3,
		&STAtemp2.StarB.UTC, &STAtemp2.StarB.Q1, &STAtemp2.StarB.Q2, &STAtemp2.StarB.Q3,
		&STAtemp2.StarC.UTC, &STAtemp2.StarC.Q1, &STAtemp2.StarC.Q2, &STAtemp2.StarC.Q3);
	for (i = 0; i < m; i++)
	{
		fscanf(fp, "%*s\t%lf\t%*d\t%lf\t%lf\t%lf\t%lf\t%*d\t%lf\t%lf\t%lf\t%*d\t%lf\t%*d\t%lf\t%lf\t%lf\t%*d\n",
			&STAtemp.StarA.UTC, &STAtemp.StarA.Q1, &STAtemp.StarA.Q2, &STAtemp.StarA.Q3,
			&STAtemp.StarB.UTC, &STAtemp.StarB.Q1, &STAtemp.StarB.Q2, &STAtemp.StarB.Q3,
			&STAtemp.StarC.UTC, &STAtemp.StarC.Q1, &STAtemp.StarC.Q2, &STAtemp.StarC.Q3);
		if (STAtemp.StarA.UTC < STAtemp2.StarA.UTC)
		{
			STAtemp.StarA.UTC += 1;
		}
		STAtemp2.StarA.UTC = STAtemp.StarA.UTC;
		if (STAtemp.StarB.UTC < STAtemp2.StarB.UTC)
		{
			STAtemp.StarB.UTC -= 1;
		}
		STAtemp2.StarB.UTC = STAtemp.StarB.UTC;
		if (STAtemp.StarC.UTC < STAtemp2.StarC.UTC)
		{
			STAtemp.StarC.UTC -= 1;
		}
		STAtemp2.StarC.UTC = STAtemp.StarC.UTC;
		STAtemp.StarA.Q0 = sqrt(1 - pow(STAtemp.StarA.Q1, 2) - pow(STAtemp.StarA.Q2, 2) - pow(STAtemp.StarA.Q3, 2));
		STAtemp.StarB.Q0 = sqrt(1 - pow(STAtemp.StarB.Q1, 2) - pow(STAtemp.StarB.Q2, 2) - pow(STAtemp.StarB.Q3, 2));
		STAtemp.StarC.Q0 = sqrt(1 - pow(STAtemp.StarC.Q1, 2) - pow(STAtemp.StarC.Q2, 2) - pow(STAtemp.StarC.Q3, 2));
		STAdat.push_back(STAtemp);
	}
}

