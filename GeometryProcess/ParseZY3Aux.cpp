#include "ParseZY3Aux.h"


ParseZY3Aux::ParseZY3Aux(void)
{
}


ParseZY3Aux::~ParseZY3Aux(void)
{
}

bool ParseZY3Aux::parseZY302FrameData(unsigned char*pData,FrameData_ZY3& perBagData)
{
	int nFrames = 15; //15 frames for one bag;
	int perFrameLength = 16;

	for (int i = 0;i<nFrames;i++)
	{
		if(pData[i*perFrameLength+4]!=i)  //to decide missing frame;
		{
			printf("=>missing frame, invalid!\n");
			return false;
		}

		//to parse the index for line;
		perBagData.frame_lineIndex[i] = pData[i*perFrameLength+1]*65536+pData[i*perFrameLength+2]*256+pData[i*perFrameLength+3];

		//to parse the index for time;
		perBagData.frame_timeIndex[i] = pData[i*perFrameLength+5]*65536+pData[i*perFrameLength+6]*256+pData[i*perFrameLength+7];
	}

	for (int i = 0 ;i<nFrames-1;i++)
	{
		if(perBagData.frame_lineIndex[i+1]-perBagData.frame_lineIndex[i]!=1)
		{
			printf("=>the line index miscontinued!\n");
			return false;
		}
	}

	//parse the gps;
	unsigned char gpsData[34];
	for (int i = 0;i<4;i++)
	{
		memcpy(gpsData+i*8,pData+i*perFrameLength+8,8);
	}
	gpsData[32] = pData[72],gpsData[33] = pData[73];

	//parse the orbit data;
	perBagData.frame_Ep.UTC = gpsData[2]*65536+gpsData[3]*16777216+gpsData[4]+gpsData[5]*256+(gpsData[6]*256+gpsData[7]*65536+gpsData[9])*1.e-6;
	perBagData.frame_Ep.X = (gpsData[10]*65536+gpsData[11]*16777216+gpsData[12]+gpsData[13]*256)*0.1;
	perBagData.frame_Ep.Y = (gpsData[14]*65536+gpsData[15]*16777216+gpsData[16]+gpsData[17]*256)*0.1;
	perBagData.frame_Ep.Z = (gpsData[18]*65536+gpsData[19]*16777216+gpsData[20]+gpsData[21]*256)*0.1;
	perBagData.frame_Ep.Xv = (gpsData[22]*65536+gpsData[23]*16777216+gpsData[24]+gpsData[25]*256)*0.01;
	perBagData.frame_Ep.Yv = (gpsData[26]*65536+gpsData[27]*16777216+gpsData[28]+gpsData[29]*256)*0.01;
	perBagData.frame_Ep.Zv = (gpsData[30]*65536+gpsData[31]*16777216+gpsData[32]+gpsData[33]*256)*0.01;

	unsigned char attData[44];
	if(pData[0]==0xDA)  //多光谱相机;
	{
		memcpy(attData,pData+76,4);
		for (int i = 5;i<=9;i++)
		{
			memcpy(attData+4+(i-5)*8,pData+i*perFrameLength+8,8);
		}

		perBagData.frame_clock.intUTC = pData[217]*16777216+pData[218]*65536+pData[219]*256+pData[220];
		perBagData.frame_clock.timeIndex = perBagData.frame_timeIndex[13];
		perBagData.frame_clock.clockFreq = pData[221]*65536+pData[222]*256+pData[223];
	}

	else
	{
		//parse the attitude;
		memcpy(attData,pData+90,6);
		for (int i = 6;i<=9;i++)
		{
			memcpy(attData+6+(i-6)*8,pData+i*perFrameLength+8,8);
		}
		memcpy(attData+38,pData+168,6);

		perBagData.frame_clock.intUTC = pData[218]*16777216+pData[219]*65536+pData[220]*256+pData[221];
		perBagData.frame_clock.timeIndex = perBagData.frame_timeIndex[13];
		perBagData.frame_clock.clockFreq = pData[233]*65536+pData[234]*256+pData[235];
	}

	//parse the attitude;
	unsigned char transType_4byte[4];

	Attitude att0,att1;
	perBagData.frame_Att[0].UTC = (attData[2]*256+attData[3])*25*1.e-6+attData[4]*256+attData[5]+attData[6]*16777216+attData[7]*65536;

	transType_4byte[0] = attData[9], transType_4byte[1] = attData[8],transType_4byte[2] = attData[11],transType_4byte[3] = attData[10];
	perBagData.frame_Att[0].Q1 = double(*(float*)transType_4byte);

	transType_4byte[0] = attData[13], transType_4byte[1] = attData[12],transType_4byte[2] = attData[15],transType_4byte[3] = attData[14];
	perBagData.frame_Att[0].Q2 = double(*(float*)transType_4byte);

	transType_4byte[0] = attData[17], transType_4byte[1] = attData[16],transType_4byte[2] = attData[19],transType_4byte[3] = attData[18];
	perBagData.frame_Att[0].Q3 = double(*(float*)transType_4byte);

	perBagData.frame_Att[0].Q0 = sqrt(1 - perBagData.frame_Att[0].Q1*perBagData.frame_Att[0].Q1 - 
		perBagData.frame_Att[0].Q2*perBagData.frame_Att[0].Q2 - perBagData.frame_Att[0].Q3*perBagData.frame_Att[0].Q3);

	perBagData.frame_Att[1].UTC = (attData[24]*256+attData[25])*25*1.e-6+attData[26]*256+attData[27]+attData[28]*16777216+attData[29]*65536;

	transType_4byte[0] = attData[31], transType_4byte[1] = attData[30],transType_4byte[2] = attData[33],transType_4byte[3] = attData[32];
	perBagData.frame_Att[1].Q1 = double(*(float*)transType_4byte);

	transType_4byte[0] = attData[35], transType_4byte[1] = attData[34],transType_4byte[2] = attData[37],transType_4byte[3] = attData[36];
	perBagData.frame_Att[1].Q2 = double(*(float*)transType_4byte);

	transType_4byte[0] = attData[39], transType_4byte[1] = attData[38],transType_4byte[2] = attData[41],transType_4byte[3] = attData[40];
	perBagData.frame_Att[1].Q3 = double(*(float*)transType_4byte);

	perBagData.frame_Att[1].Q0 = sqrt(1 - perBagData.frame_Att[1].Q1*perBagData.frame_Att[1].Q1 - 
		perBagData.frame_Att[1].Q2*perBagData.frame_Att[1].Q2 - perBagData.frame_Att[1].Q3*perBagData.frame_Att[1].Q3);
	return true;
}

bool ParseZY3Aux::getZY302Aux(string sAux,vector<Orbit_Ep>& allEp,vector<Attitude>& allAtt,vector<LineScanTime>& allTime)
{
	if(sAux.empty()==true)
	{
		printf("=>the input aux file for ZY302 is empty!\n");
		return false;
	}

	FILE *fpAux = fopen(sAux.c_str(),"rb");
	if(!fpAux)
	{
		printf("=>failed to open the aux file for ZY302(%s)\n",sAux.c_str());
		return false;
	}

	//read all content to the memory；
	_fseeki64 (fpAux, 0, SEEK_END);   ///move the file to the end;
	long long nfileSize=_ftelli64(fpAux); ///get the file size;

	printf("=>the total size of the aux is: %d byte\n",nfileSize);

	unsigned char *pData = new unsigned char[nfileSize];
	_fseeki64(fpAux,0,SEEK_SET);
	if(fread(pData,sizeof(unsigned char),nfileSize,fpAux) != nfileSize)
	{
		delete []pData,pData = NULL;
		printf("failed to read all content to the memory!\n");
		fclose(fpAux);
		return false;
	}
	fclose(fpAux);

	int perFrameLength = 56; //56 byte for one frame;
	int n56 = int(nfileSize/perFrameLength);

	int nFrames = 15; //15 frames for one bag;
	int bagLength = nFrames*16/*perFrameLength*/;

	unsigned char* pBagData = new unsigned char[bagLength];

	LineScanTime perTime;

	FrameData_ZY3 perFrameData,preData;
	int addBegin = 0;
	for (int i = 0;i<n56;)
	{
//		printf("%d/%d.......\n",i,n56);
		if(pData[i*perFrameLength+12]==0x0 && i+nFrames-1<n56) //the beginning of the whole bag;
		{

			for (int j = 0;j<nFrames;j++)
			{
				memcpy(pBagData+j*16,pData+i*perFrameLength+8,16);
				i++;
			}

			if(parseZY302FrameData(pBagData,perFrameData)==false)
			{
				printf("failed to parse the %th frame",i);
				continue;
			}

			if(addBegin == 0)  //for the first;
				addBegin++;
			else
			{
				if(preData.frame_Ep.UTC != perFrameData.frame_Ep.UTC)
					allEp.push_back(preData.frame_Ep);  // only one orbit_ep;

				if(preData.frame_Att[0].UTC != perFrameData.frame_Att[0].UTC)
				{
					allAtt.push_back(preData.frame_Att[0]); // only one attitude;
					allAtt.push_back(preData.frame_Att[1]);  // only one attitude;
				}

			}
			preData = perFrameData;	

			double freq = perFrameData.frame_clock.clockFreq;

			int detalTime = 0;
			bool bAddOneSec = false;
			int preCount = perFrameData.frame_timeIndex[0]-1;
			for (int j = 0;j<nFrames;j++)
			{
				if(perFrameData.frame_timeIndex[j]<preCount && bAddOneSec == false)  // can only add 1s in the same bag;
				{
					detalTime = 1.;
					bAddOneSec = true;
				}

				preCount = perFrameData.frame_timeIndex[j];

				perTime.lineNumber = perFrameData.frame_lineIndex[j];
				
				perTime.lineTimeUT = perFrameData.frame_timeIndex[j]/freq+detalTime + perFrameData.frame_clock.intUTC;

				allTime.push_back(perTime);
			}

		}
		else
			i++;
	}
	
	delete []pBagData,pBagData = NULL;
	delete []pData,pData = NULL;
	printf("=>sucess to parse aux!\n");
	return true;
}

//////////////////////////////////////////////////////////////////////////
//功能：读取资三01星姿态txt数据
//输入：sAtt，ZY3-01姿态数据
//输出：姿态vector
//注意：by JYH
//日期：2016.12.06
//////////////////////////////////////////////////////////////////////////
bool ParseZY3Aux::ReadZY301AttTXT(string sAtt,vector<Attitude> &arr_att)
{
	if(sAtt.empty())
		return false;
	FILE *fp = fopen(sAtt.c_str(),"r");
	if(!fp)
		return false;

	arr_att.clear();
	int num;
	Attitude att;
	char c_read[1024];
	string tmpStr;
	fgets(c_read, 1024, fp);
	if (tmpStr.assign(c_read) == "\n")
	{
		while (!feof(fp))
		{
			if (fgets(c_read, 1024, fp) == NULL)
				continue;
			tmpStr.assign(c_read);
			if (tmpStr.find("groupNumber", 0) != -1)
			{
				sscanf(tmpStr.c_str(), "%*s%*s%d", &num);
				if (num <= 0)
				{
					printf("=>zero quaternion!\n");
					fclose(fp);
					return false;
				}
			}

			if (tmpStr.find("attData", 0) != -1)
			{
				while (fgets(c_read, 1024, fp) != NULL)
				{
					tmpStr.assign(c_read);
					if (tmpStr.find("}") != -1)
						break;
					if (tmpStr.find("timeCode", 0) != -1)
						sscanf(tmpStr.c_str(), "%*s%*s%lf", &att.UTC);
					if (tmpStr.find("q1 ", 0) != -1)
						sscanf(tmpStr.c_str(), "%*s%*s%lf", &att.Q1);
					if (tmpStr.find("q2 ", 0) != -1)
						sscanf(tmpStr.c_str(), "%*s%*s%lf", &att.Q2);
					if (tmpStr.find("q3 ", 0) != -1)
						sscanf(tmpStr.c_str(), "%*s%*s%lf", &att.Q3);
					if (tmpStr.find("q4 ", 0) != -1)
						sscanf(tmpStr.c_str(), "%*s%*s%lf", &att.Q0);
				}
				//att.Q0 = sqrt(1-att.Q1*att.Q1-att.Q3*att.Q3-att.Q2*att.Q2);
				arr_att.push_back(att);
			}
		}
	}
	else
	{
		rewind(fp);
		fscanf(fp, "%d\n", &num);
		for (int i=0;i<num;i++)
		{
			fscanf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\n", &att.UTC, &att.Q1, &att.Q2, &att.Q3, &att.Q0);
			arr_att.push_back(att);
		}
	}
	fclose(fp);
	return true;
}

//////////////////////////////////////////////////////////////////////////
//功能：读取资三01星轨道txt数据
//输入：sOrb，ZY3-01姿态数据
//输出：姿态vector
//注意：
//日期：2016.12.06
//////////////////////////////////////////////////////////////////////////
bool  ParseZY3Aux::ReadZY301OrbTXT(string sOrb,vector<Orbit_Ep> &arr_Orb)
{
	if(sOrb.empty())
		return false;
	FILE *fp = fopen(sOrb.c_str(),"r");
	if(!fp)
		return false;

	arr_Orb.clear();
	int num;
	Orbit_Ep Orb;
	char c_read[1024];
	string tmpStr;
	while(!feof(fp))
	{
		if(fgets(c_read,1024,fp) == NULL)
			continue;
		tmpStr.assign(c_read);
		if(tmpStr.find("groupNumber",0) != -1)
		{
			sscanf(tmpStr.c_str(),"%*s%*s%d",&num);
			if(num <= 0)
			{
				printf("=>zero quaternion!\n");
				fclose(fp);
				return false;
			}
		}

		if(tmpStr.find("gpsData",0) != -1)
		{
			while(fgets(c_read,1024,fp) != NULL)
			{
				tmpStr.assign(c_read);
				if(tmpStr.find("}") !=-1)
					break;
				if(tmpStr.find("timeCode",0) != -1)
					sscanf(tmpStr.c_str(),"%*s%*s%lf",&Orb.UTC);
				if(tmpStr.find("PX ",0) != -1)
					sscanf(tmpStr.c_str(),"%*s%*s%lf",&Orb.X);
				if(tmpStr.find("PY ",0) != -1)
					sscanf(tmpStr.c_str(),"%*s%*s%lf",&Orb.Y);
				if(tmpStr.find("PZ ",0) != -1)
					sscanf(tmpStr.c_str(),"%*s%*s%lf",&Orb.Z);
				if(tmpStr.find("VX ",0) != -1)
					sscanf(tmpStr.c_str(),"%*s%*s%lf",&Orb.Xv);
				if(tmpStr.find("VY ",0) != -1)
					sscanf(tmpStr.c_str(),"%*s%*s%lf",&Orb.Yv);
				if(tmpStr.find("VZ ",0) != -1)
					sscanf(tmpStr.c_str(),"%*s%*s%lf",&Orb.Zv);
			}
			arr_Orb.push_back(Orb);
		}
	}

	fclose(fp);
	return true;
}

//////////////////////////////////////////////////////////////////////////
//功能：读取资三01星行时txt数据，顺便读取input文件的起始终止行
//输入：sTime，ZY3-01行时数据；sInput，起始终止行文件
//输出：所需要的行时vector
//注意：
//日期：2016.12.15
//////////////////////////////////////////////////////////////////////////
bool  ParseZY3Aux::ReadZY301TimeTXT(string sTime,string sInput,vector<LineScanTime> &arr_Time)
{
	if(sTime.empty())
		return false;
	FILE *fp1 = fopen(sTime.c_str(),"r");
	FILE *fp2 = fopen(sInput.c_str(),"r");
	if(!fp1||!fp2)
		return false;
	arr_Time.clear();
	
	long startLine,endLine;
	TiXmlDocument doc(sInput);
	doc.LoadFile();
	// 获得根元素
	TiXmlElement *RootElement = doc.RootElement();
	TiXmlElement *FirstNode = RootElement->FirstChildElement();
	TiXmlElement *SecNode = FirstNode->FirstChildElement();
	// 开始行
	string nodename = SecNode->Value();		
		while(nodename!="BeginLine")
		{
			SecNode = SecNode->NextSiblingElement();
			nodename = (char*)SecNode->Value();
		}
		nodename = SecNode->GetText();
		sscanf(nodename.c_str(), "%ld", &startLine);
		// 结束行
		nodename = SecNode->Value();
		while(nodename!="EndLine")
		{
			SecNode = SecNode->NextSiblingElement();
			nodename = (char*)SecNode->Value();
		}
		nodename = SecNode->GetText();
		sscanf(nodename.c_str(), "%ld", &endLine);

	long num;
	LineScanTime lTime;
	fscanf(fp1,"%*s\t%*s\t%*s\n");
	while(!feof(fp1))
	{
		fscanf(fp1,"%d\t%lf\t%*lf\n",&lTime.lineNumber,&lTime.lineTimeUT);
		if (lTime.lineNumber>=startLine&&lTime.lineNumber<=endLine)
		{
			arr_Time.push_back(lTime);
		}
	}

	fclose(fp1);
	fclose(fp2);
	return true;
}

//////////////////////////////////////////////////////////////////////////
//功能：根据工作目录给出各资三数据路径
//输入：auxpath辅助数据路径
//输出：sAtt：姿态数据路径，sOrb：轨道数据路径，sTime：行时数据路径，sInput：起止行文件路径
//注意：
//日期：2016.12.15
//////////////////////////////////////////////////////////////////////////
void ParseZY3Aux::ZY301PATH(string auxpath, string &sAtt, string &sOrb, string &sTime, string &sInput)
{
	sOrb = auxpath + "DX_ZY3_NAD_gps.txt";
	sAtt = auxpath + "DX_ZY3_NAD_att.txt";
	sTime = auxpath + "DX_ZY3_NAD_imagingTime.txt";
	sInput = auxpath + "inputFile_NAD.xml";
}

//////////////////////////////////////////////////////////////////////////
//功能：读取资三02星姿态txt数据
//输入：sAtt，ZY3-02姿态数据
//输出：姿态vector
//注意：
//日期：2017.04.25
//////////////////////////////////////////////////////////////////////////
bool ParseZY3Aux::ReadZY302AttTXT(string sAtt, vector<Attitude> &arr_att)
{
	if (sAtt.empty())
		return false;
	FILE *fp = fopen(sAtt.c_str(), "r");
	if (!fp)
		return false;

	arr_att.clear();
	int num;
	Attitude att;
	char ss[1024];
	fgets(ss,1024, fp);
	fgets(ss, 1024, fp);
	fscanf(fp, "%d\n", &num);//跳过前两行读取第三行
	for (size_t i = 0; i < num; i++)
	{
		fscanf(fp, "%*s\t%lf\t%lf\t%lf\t%lf\t%lf\n", &att.UTC, &att.Q0, &att.Q1, &att.Q2, &att.Q3);
		arr_att.push_back(att);
	}
			
	fclose(fp);
	return true;
}

//////////////////////////////////////////////////////////////////////////
//功能：读取资三02星轨道txt数据
//输入：sOrb，ZY3-02轨道数据
//输出：轨道vector
//注意：这个是定轨数据
//日期：2017.04.25
//////////////////////////////////////////////////////////////////////////
bool  ParseZY3Aux::ReadZY302OrbTXT(string sOrb, vector<Orbit_Ep> &arr_Orb)
{
	if (sOrb.empty())
		return false;
	FILE *fp = fopen(sOrb.c_str(), "r");
	if (!fp)
		return false;
	arr_Orb.clear();

	TiXmlDocument doc(sOrb);
	doc.LoadFile();
	TiXmlElement* rootElement = doc.RootElement();  //ephemlist
	TiXmlElement* pointElement = rootElement->FirstChildElement();
	Orbit_Ep orbTmp;
	pointElement = pointElement->NextSiblingElement();// point元素
	for (; pointElement != NULL; pointElement = pointElement->NextSiblingElement())
	{
		TiXmlElement* locationElement = pointElement->FirstChildElement()->FirstChildElement();
		string	tmp = locationElement->GetText();
		orbTmp.X = atof(tmp.c_str());
		tmp = locationElement->NextSiblingElement()->GetText();
		orbTmp.Y = atof(tmp.c_str());
		tmp = locationElement->NextSiblingElement()->NextSiblingElement()->GetText();
		orbTmp.Z = atof(tmp.c_str());
		tmp = pointElement->FirstChildElement()->NextSiblingElement()->NextSiblingElement()->GetText();
		string timeTmp = tmp.substr(0, 4); int yy = atoi(timeTmp.c_str());
		timeTmp = tmp.substr(4, 2); int mo = atoi(timeTmp.c_str());
		timeTmp = tmp.substr(6, 2); int dd = atoi(timeTmp.c_str());
		timeTmp = tmp.substr(8, 2); int hh = atoi(timeTmp.c_str());
		timeTmp = tmp.substr(10, 2); int mm = atoi(timeTmp.c_str());
		timeTmp = tmp.substr(12, 2); int ss = atoi(timeTmp.c_str());
		double jd0, refMJD;
		Cal2JD(2009, 1, 1, 0, &jd0, &refMJD);
		mBase.FromYMDtoSecond(refMJD,yy,mo,dd,hh,mm,ss, orbTmp.UTC);
		arr_Orb.push_back(orbTmp);
	}
	
	fclose(fp);
	return true;
}

//////////////////////////////////////////////////////////////////////////
//功能：读取资三02星轨道txt数据
//输入：sOrb，ZY3-02轨道数据
//输出：轨道vector
//注意：这个是星上GPS数据
//日期：2017.04.25
//////////////////////////////////////////////////////////////////////////
bool  ParseZY3Aux::ReadZY302OrbTXT2(string sOrb, vector<Orbit_Ep> &arr_Orb)
{
	if (sOrb.empty())
		return false;
	FILE *fp = fopen(sOrb.c_str(), "r");
	if (!fp)
		return false;
	arr_Orb.clear();

	int num;
	Orbit_Ep Orb;
	char ss[1024];
	fgets(ss, 1024, fp);
	fgets(ss, 1024, fp);
	fscanf(fp, "%d\n", &num);//跳过前两行读取第三行
	for (size_t i = 0; i < num; i++)
	{
		fscanf(fp, "%*s\t%lf\t%lf\t%lf\t%lf\t%*lf\t%*lf\t%*lf\n", &Orb.UTC, &Orb.X, &Orb.Y, &Orb.Z);
		arr_Orb.push_back(Orb);
	}

	fclose(fp);
	return true;
}

//////////////////////////////////////////////////////////////////////////
//功能：读取资三02星行时txt数据，顺便读取input文件的起始终止行
//输入：sTime，ZY3-01行时数据；//输出：所需要的行时vector
//注意：
//日期：2017.04.25
//////////////////////////////////////////////////////////////////////////
bool  ParseZY3Aux::ReadZY302TimeTXT(string sTime, vector<LineScanTime> &arr_Time)
{
	if (sTime.empty())
		return false;
	FILE *fp1 = fopen(sTime.c_str(), "r");
	if (!fp1)
		return false;
	arr_Time.clear();	

	long num;
	LineScanTime lTime;
	fscanf(fp1, "%*s\t%*s\t%*s\n");
	fscanf(fp1, "%*s\t%*s\t%*d\t%*d\t%*d\n");
	fscanf(fp1, "%d\n",&num);
	while (!feof(fp1))
	{
		fscanf(fp1, "%d\t%lf\t%*lf\t%*lf\t%*lf\t%*lf\t%*d\t%*d\n", &lTime.lineNumber, &lTime.lineTimeUT);
		arr_Time.push_back(lTime);		
	}

	fclose(fp1);
	return true;
}

//////////////////////////////////////////////////////////////////////////
//功能：根据工作目录给出各资三数据路径
//输入：auxpath辅助数据路径
//输出：sAtt：姿态数据路径，sOrb：轨道数据路径，sTime：行时数据路径
//注意：
//日期：2017.04.25
//////////////////////////////////////////////////////////////////////////
void ParseZY3Aux::ZY302PATH(string auxpath, string &sAtt, string &sOrb, string &sTime)
{	
	string tmp = auxpath.substr(auxpath.rfind('\\'));
	sOrb = auxpath + tmp + "GPSclip.xml";
	//sOrb = auxpath + tmp + ".EPH";
	sAtt = auxpath + tmp + ".ATT";
	sTime = auxpath + tmp + ".IT";
}

bool ParseZY3Aux::ReadZY3SimTimeTXT(string sTime, vector<LineScanTime>& arr_Time)
{
	if (sTime.empty())
		return false;
	FILE *fp1 = fopen(sTime.c_str(), "r");
	if (!fp1)
		return false;
	arr_Time.clear();

	long num;
	LineScanTime lTime;
	fscanf(fp1, "%*s\t%*s\t%*s\n");
	while (!feof(fp1))
	{
		fscanf(fp1, "%d\t%lf\t%*lf\n", &lTime.lineNumber, &lTime.lineTimeUT);
		arr_Time.push_back(lTime);
	}

	fclose(fp1);
	return true;
}

bool ParseZY3Aux::ReadLittleCameraTimeTXT(string sTime, vector<LineScanTime>& arr_Time)
{
	if (sTime.empty())
		return false;
	FILE *fp1 = fopen(sTime.c_str(), "r");
	if (!fp1)
		return false;
	arr_Time.clear();

	long num;
	LineScanTime lTime;
	int year, mon, day, hour, min;
	double sec;
	fscanf(fp1, "%*s\t%*s\t%*s\n\n");
	while (!feof(fp1))
	{
		fscanf(fp1, "%d\t%d-%d-%dT%d:%d:%lf\n", &lTime.lineNumber, &year, &mon, &day, &hour, &min, &sec);
		mBase.FromYMDtoSecond(54832, year, mon, day, hour, min, sec, lTime.lineTimeUT);
		arr_Time.push_back(lTime);
	}

	fclose(fp1);
	return true;
}

//////////////////////////////////////////////////////////////////////////
//功能：根据工作目录给出各资三仿真数据路径
//输入：auxpath辅助数据路径
//输出：sAtt：姿态数据路径，sOrb：轨道数据路径，sTime：行时数据路径，sCam：相机参数路径
//注意：
//作者：GZC
//日期：2017.08.14
//////////////////////////////////////////////////////////////////////////
void ParseZY3Aux::ZY3SimPATH(string auxpath, string & sAtt, string & sOrb, string & sTime, string &sCam)
{	
	sOrb = auxpath + "\\GPS_Error.txt";	
	sAtt = auxpath + "\\ATT_Error.txt";
	sTime = auxpath + "\\TimeInter_Error.txt";
	sCam = auxpath + "\\inner_Error_Line.txt";
}

//////////////////////////////////////////////////////////////////////////
//功能：根据工作目录给出各资三真实数据路径
//输入：auxpath辅助数据路径
//输出：sAtt：姿态数据路径，sOrb：轨道数据路径，sTime：行时数据路径，sCam：相机参数路径
//注意：
//作者：GZC
//日期：2017.08.22
//////////////////////////////////////////////////////////////////////////
void ParseZY3Aux::ZY3RealPATH(string auxpath, string & sAtt, string & sOrb, string & sTime, string &sCam)
{
	sOrb = auxpath + "\\GPS.txt";
	sAtt = auxpath + "\\ATT.txt";
	sTime = auxpath + "\\TimeInter_Error.txt";
	sCam = auxpath + "\\inner_Error_Line.txt";
}

//////////////////////////////////////////////////////////////////////////
//功能：珠海一号辅助数据解析
//输入：const string& caseFileName //需要查找的指定路径名称
//			char *File_ext             //要查找文件扩展名，不含'.'
//输出：vector<string>& ResPath    //结果数组
//注意：
//作者：LLT
//日期：2017.06.17
//////////////////////////////////////////////////////////////////////////
void ParseZY3Aux::search_directory(const string& caseFileName, char *File_ext, vector<string>& ResPath)
{
	if (GetFileAttributes(caseFileName.c_str()) & FILE_ATTRIBUTE_DIRECTORY)
	{
		char path[MAX_PATH];
		WIN32_FIND_DATA fData;
		HANDLE handle;

		string strFile;
		int  pos;
		string  strFileExt;

		sprintf(path, "%s/*", caseFileName.c_str());
		handle = FindFirstFile(path, &fData);

		if (INVALID_HANDLE_VALUE == handle)
		{
			return;
		}

		do {
			if (fData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
			{
				if (!strcmp(fData.cFileName, ".") || !strcmp(fData.cFileName, ".."))
				{
					continue;
				}

				search_directory(caseFileName + string(fData.cFileName) + string("\\"), File_ext, ResPath);
			}
			else
			{
				strFile = fData.cFileName;
				pos = strFile.find_last_of('.');
				if (pos>0)
				{
					strFileExt = strFile.substr(pos + 1);
					if (0 == strcmp(File_ext, _strlwr((char *)strFileExt.c_str())))
					{
						char temp[1024];
						sprintf(temp, "%s%s", caseFileName.c_str(), fData.cFileName);
						ResPath.push_back(temp);
					}
				}
			}
		} while (FindNextFile(handle, &fData));

		FindClose(handle);
	}
	else
	{
		printf("%s\n", caseFileName.c_str());
	}

}

