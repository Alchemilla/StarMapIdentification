#include "ParseOpticalAux.h"

bool parseZY302FrameData(unsigned char*pData,FrameData_ZY302& perBagData)
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

	perBagData.frame_Att[1].UTC = (attData[24]*256+attData[25])*25*1.e-6+attData[26]*256+attData[27]+attData[28]*16777216+attData[29]*65536;

	transType_4byte[0] = attData[31], transType_4byte[1] = attData[30],transType_4byte[2] = attData[33],transType_4byte[3] = attData[32];
	perBagData.frame_Att[1].Q1 = double(*(float*)transType_4byte);

	transType_4byte[0] = attData[35], transType_4byte[1] = attData[34],transType_4byte[2] = attData[37],transType_4byte[3] = attData[36];
	perBagData.frame_Att[1].Q2 = double(*(float*)transType_4byte);

	transType_4byte[0] = attData[39], transType_4byte[1] = attData[38],transType_4byte[2] = attData[41],transType_4byte[3] = attData[40];
	perBagData.frame_Att[1].Q3 = double(*(float*)transType_4byte);

	return true;
}

bool getZY302Aux(string sAux,vector<Orbit_Ep>& allEp,vector<Attitude>& allAtt,vector<LineScanTime>& allTime,string sDir)
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

	FrameData_ZY302 perFrameData,preData;
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


	printf("=>begin to write auxFile!\n");

	string auxName = sAux.substr(sAux.rfind('\\'),sAux.rfind('.')-sAux.rfind('\\'));

	char sEp[1024],sAtt[1024],sUTC[1024];
	sprintf(sEp,"%s%s%s",sDir.c_str(),auxName.c_str(),".EPH");
	sprintf(sAtt,"%s%s%s",sDir.c_str(),auxName.c_str(),".ATT");
	sprintf(sUTC,"%s%s%s",sDir.c_str(),auxName.c_str(),".IT");


	FILE *fpEp = fopen(sEp,"w");
	int epSize = allEp.size();
	fprintf(fpEp,"%lf\t\t%lf\n",allEp[0].UTC,allEp[epSize-1].UTC);
	fprintf(fpEp,"ZY302\n");
	fprintf(fpEp,"%d\n",epSize);
	for (int i = 0;i<epSize;i++)
	{
		fprintf(fpEp,"GPS\t%.15lf\t%.15lf\t%.15lf\t%.15lf\t%.15lf\t%.15lf\t%.15lf\n",allEp[i].UTC,allEp[i].X,allEp[i].Y,allEp[i].Z,allEp[i].Xv,allEp[i].Yv,allEp[i].Zv);
	}
	fclose(fpEp);

	FILE *fpAtt = fopen(sAtt,"w");
	int attSize = allAtt.size();
	fprintf(fpAtt,"%lf\t\t%lf\n",allAtt[0].UTC,allAtt[attSize-1].UTC);
	fprintf(fpAtt,"ZY302\n");
	fprintf(fpAtt,"%d\n",attSize);
	for (int i = 0;i<attSize;i++)
	{
		allAtt[i].Q0 = sqrt(1.-allAtt[i].Q1*allAtt[i].Q1-allAtt[i].Q2*allAtt[i].Q2-allAtt[i].Q3*allAtt[i].Q3);
		fprintf(fpAtt,"Quater\t%.15lf\t%.15lf\t%.15lf\t%.15lf\t%.15lf\n",allAtt[i].UTC,allAtt[i].Q0,allAtt[i].Q1,allAtt[i].Q2,allAtt[i].Q3);
	}
	fclose(fpAtt);

	FILE *fpUTC = fopen(sUTC,"w");
	int timeSize = allTime.size();
	fprintf(fpUTC,"%lf\t\t%lf\n",allTime[0].lineTimeUT,allTime[timeSize-1].lineTimeUT);
	fprintf(fpUTC,"ZY302\n");
	fprintf(fpUTC,"%d\n",timeSize);
//	double preUTC = 0;
	LineScanTime preUTC;
	preUTC.lineTimeUT = 0;
	preUTC.lineNumber = allTime[0].lineNumber-1;
	for (int i = 0;i<timeSize;i++)
	{
		fprintf(fpUTC,"%d\t%.15lf\t%.15lf\n",allTime[i].lineNumber,allTime[i].lineTimeUT,(allTime[i].lineTimeUT-preUTC.lineTimeUT)/(allTime[i].lineNumber-preUTC.lineNumber));
		preUTC = allTime[i];
	}
	fclose(fpUTC);



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
bool ReadZY3_01AttTXT(string sAtt,vector<Attitude> &arr_att)
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

		if(tmpStr.find("attData",0) != -1)
		{
			while(fgets(c_read,1024,fp) != NULL)
			{
				tmpStr.assign(c_read);
				if(tmpStr.find("}") !=-1)
					break;
				if(tmpStr.find("timeCode",0) != -1)
					sscanf(tmpStr.c_str(),"%*s%*s%lf",&att.UTC);
				if(tmpStr.find("q1 ",0) != -1)
					sscanf(tmpStr.c_str(),"%*s%*s%lf",&att.Q1);
				if(tmpStr.find("q2 ",0) != -1)
					sscanf(tmpStr.c_str(),"%*s%*s%lf",&att.Q2);
				if(tmpStr.find("q3 ",0) != -1)
					sscanf(tmpStr.c_str(),"%*s%*s%lf",&att.Q3);
			}
			att.Q0 = sqrt(1-att.Q1*att.Q1-att.Q3*att.Q3-att.Q2*att.Q2);
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
bool  ReadZY3_01OrbTXT(string sOrb,vector<Orbit_Ep> &arr_Orb)
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