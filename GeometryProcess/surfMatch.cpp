#include "surfMatch.h"
#include "GeoReadImage.h"
#include "GeoModelRFM.h"
#include<omp.h>
bool surfMatch_uchar(string sImg1,string sImg2,int nBlocks,double fSigmacondition,int numThread,string sRes)
{
	if(sImg1.empty() == true)
	{
		printf("=>the path of left image is empty!\n");
		return false;
	}
	if(sImg2.empty() == true)
	{
		printf("=>the path of left image is empty!\n");
		return false;
	}

	GDALDataset* poDataset_L = (GDALDataset *)GDALOpen(sImg1.c_str(),GA_ReadOnly);
	if(poDataset_L == NULL)
	{
		printf("=>failed to open the left image(%s)\n",sImg1.c_str());
		return false;
	}

	GDALDataset* poDataset_R = (GDALDataset *)GDALOpen(sImg2.c_str(),GA_ReadOnly);
	if(poDataset_R == NULL)
	{
		GDALClose((GDALDatasetH)poDataset_L);
		printf("=>failed to open the right image(%s)\n",sImg2.c_str());
		return false;
	}

	int imgW1,imgH1;
	int imgW2,imgH2;

	imgW1  = poDataset_L->GetRasterXSize();
	imgH1 = poDataset_L->GetRasterYSize();

	imgW2 = poDataset_R->GetRasterXSize();
	imgH2 = poDataset_R->GetRasterYSize();

	double dMinMax[2];
	poDataset_L->GetRasterBand(1)->ComputeRasterMinMax(true,dMinMax);
	ushort minData1 = dMinMax[0];
	ushort maxData1 = dMinMax[1];

	poDataset_R->GetRasterBand(1)->ComputeRasterMinMax(true,dMinMax);
	ushort minData2 = dMinMax[0];
	ushort maxData2 = dMinMax[1];

//	int bandnum = poDataset_L->GetRasterCount();
	GDALDataType dt = poDataset_L->GetRasterBand(1)->GetRasterDataType();

	ushort *pDataL =new ushort[imgW1*imgH1];
	if(poDataset_L->GetRasterBand(1)->RasterIO(GF_Read, 0, 0, imgW1, imgH1,pDataL,
		imgW1, imgH1, GDT_UInt16,0, 0 )==CE_Failure )
	{
		delete []pDataL,pDataL = NULL;
		GDALClose((GDALDatasetH)poDataset_L);
		GDALClose((GDALDatasetH)poDataset_R);
		printf("=>failed to read left image(%s)\n",sImg1.c_str());
		return false;
	}

	ushort *pDataR =new ushort[imgW2*imgH2];
	if(poDataset_R->GetRasterBand(1)->RasterIO(GF_Read, 0, 0, imgW2, imgH2,pDataR,
		imgW2, imgH2, GDT_UInt16,0, 0 )==CE_Failure )
	{
		delete []pDataL,pDataL = NULL;
		delete []pDataR,pDataR = NULL;
		GDALClose((GDALDatasetH)poDataset_L);
		GDALClose((GDALDatasetH)poDataset_R);
		printf("=>failed to read right image(%s)\n",sImg2.c_str());
		return false;
	}

	string sImg1Rpc = sImg1.substr(0,sImg1.find_last_of('.'))+"_rpc.txt";
	string sImg2Rpc = sImg2.substr(0,sImg2.find_last_of('.'))+"_rpc.txt";

	bool usedDefaultRange = false;
	GeoModelRFM img1RPC,img2RPC;
	if(img1RPC.ReadRFMFile(sImg1Rpc)==false)
		usedDefaultRange = true;
	if(img2RPC.ReadRFMFile(sImg2Rpc)==false)
		usedDefaultRange = true;

	double aveH = (img1RPC.getAveHeight()+img2RPC.getAveHeight())/2.;

	int lineIter = imgH1/nBlocks;
	int sampleIter = imgW1/nBlocks;

	ImgBlockInfo *pBlock = NULL;
	int numBlocks;
	BlocksForImgProcess(0,0,lineIter,sampleIter,imgW1,imgH1,pBlock,numBlocks);

	vector<matchPts> *pSubGCP = new vector<matchPts>[numThread];
	omp_set_num_threads(numThread);
#pragma omp parallel for
	for(int i = 0;i<numBlocks;i++)
	{
		int bSample = pBlock[i].sampleIndex, eSample = pBlock[i].sampleIndex+pBlock[i].width-1;
		int bLine = pBlock[i].lineIndex, eLine = pBlock[i].lineIndex+pBlock[i].height-1;
		double lx[4] = {bSample,eSample,eSample,bSample},ly[4] = {bLine,bLine,eLine,eLine};
		if(pBlock[i].width<=0 || pBlock[i].height<=0)
			continue;
		int bS,eS,bL,eL;
		bS = bL = (imgW2>imgH2?imgW2:imgH2);
		eS = eL = 0;

		for (int j = 0;j<4;j++)
		{
			double lat,lon,h,rx,ry;
			if(img1RPC.FromXY2LatLon(lx[j],ly[j],aveH,lat,lon)==true)
			{
				img2RPC.FromLatLon2XY(lat,lon,aveH,rx,ry);
				bS = bS<rx?bS:rx;
				eS = eS>rx?eS:rx;
				bL = bL<ry?bL:ry;
				eL = eL>ry?eL:ry;
			}
			else
			{
				usedDefaultRange = true;
				break;
			}

		}
		if(usedDefaultRange == true)
		{
			bS = bSample - 200;
			bL = bLine -200;
			eS = eSample +200;
			eL = eLine+200;
		}
		else
		{
			bS = bS - 200;
			bL = bL -200;
			eS = eS +200;
			eL = eL+200;
		}

		bS = bS<0?0:bS; bS = bS>imgW2-1?imgW2-1:bS;
		eS = eS<0?0:eS; eS = eS>imgW2-1?imgW2-1:eS;
		bL = bL<0?0:bL; bL = bL>imgH2-1?imgH2-1:bL;
		eL = eL<0?0:eL; eL = eL>imgH2-1?imgH2-1:eL;

		int w2 = eS-bS; int h2 = eL-bL;
		if(w2<=0 || h2<=0)
			continue;

		Mat img1Part(pBlock[i].height,pBlock[i].width,CV_8UC1),img2Part(h2,w2,CV_8UC1);
		uchar* pxvec1 = img1Part.ptr<uchar>(0);
		for (int line = 0;line<pBlock[i].height;line++)
		{
			int ii = pBlock[i].lineIndex+line;
			for (int sample = 0;sample<pBlock[i].width;sample++)
			{
				int jj = pBlock[i].sampleIndex+sample;

				if(dt==GDT_Byte)
					pxvec1[line*pBlock[i].width+sample] = uchar(pDataL[ii*imgW1+jj]);
				else
				{
					uchar value = uchar (255./(maxData1-minData1)*(pDataL[ii*imgW1+jj]-minData1));
					if(value<0)
						value = 0;
					if(value>255)
						value = 255;
					pxvec1[line*pBlock[i].width+sample] = value;//pDataL[ii*imgW1+jj];
				}
			}
		}

		uchar* pxvec2 = img2Part.ptr<uchar>(0);
		for (int line = 0;line<h2;line++)
		{
			int ii = bL+line;
			for (int sample = 0;sample<w2;sample++)
			{
				int jj = bS+sample;

				if(dt==GDT_Byte)
					pxvec2[line*w2+sample] = uchar(pDataR[ii*imgW2+jj]);
				else
				{
					uchar value = uchar (255./(maxData2-minData2)*(pDataR[ii*imgW1+jj]-minData2));
					if(value<0)
						value = 0;
					if(value>255)
						value = 255;
					pxvec2[line*w2+sample] = value;//pDataL[ii*imgW1+jj];
				}
				//pxvec2[line*w2+sample] = pDataR[ii*imgW2+jj];
			}
		}

		matchPts *pCps = NULL;
		int nCps = 0;
		surfMatch(img1Part,img2Part,pCps,nCps);

		int nThreadID = omp_get_thread_num();
		for (int n = 0;n<nCps;n++)
		{
			pCps[n].xl += pBlock[i].sampleIndex;
			pCps[n].yl+=pBlock[i].lineIndex;
			pCps[n].xr += bS;
			pCps[n].yr+=bL;

			pSubGCP[nThreadID].push_back(pCps[n]);
		}

		if(pCps)
			delete []pCps,pCps = NULL;

	}

	vector<matchPts> allGCP;
	for (int i = 0;i<numThread;i++)
	{
		allGCP.insert(allGCP.end(),pSubGCP[i].begin(),pSubGCP[i].end());
	}

	delMatchAffine(allGCP,fSigmacondition);

	int allSize = allGCP.size();

	FILE *fpRes  =  fopen(sRes.c_str(),"w");
	if(!fpRes)
	{
		printf("=>failed to create result file(%s)\n",sRes.c_str());
		return false;
	}
	fprintf(fpRes, ";\t%d\n", allSize);
	for(int i = 0;i<allSize;i++)
	{
		fprintf(fpRes,"%lf\t%lf\t%lf\t%lf\n",allGCP[i].xl,allGCP[i].yl,allGCP[i].xr,allGCP[i].yr);
	}
	fclose(fpRes);

	delete []pDataL,pDataL = NULL;
	delete []pDataR,pDataR = NULL;
	delete []pBlock,pBlock= NULL;
	delete []pSubGCP,pSubGCP = NULL;
	GDALClose((GDALDatasetH)poDataset_L);
	GDALClose((GDALDatasetH)poDataset_R);
	return true;

}

bool surfMatch(Mat& srcImage1,Mat& srcImage2 ,matchPts*& cps,int& nCps)
{
	if (!srcImage1.data || !srcImage2.data)  
	{  
		printf("=>failed to read image!\n"); 
		return false;  
	}  
	int minHessian = 700;//SURF�㷨�е�hessian��ֵ  
	SurfFeatureDetector detector(minHessian);//����һ��SurfFeatureDetector��SURF�� ������������
	std::vector<KeyPoint> keyPoint1, keyPoints2;//vectorģ���࣬����������͵Ķ�̬����  

	//��3������detect�������SURF�����ؼ�㣬������vector������  
	detector.detect(srcImage1, keyPoint1);
	detector.detect(srcImage2, keyPoints2);

	if(keyPoint1.size()==0)
	{
		printf("=>no keypoints detected!\n");
		return false;
	}
	if(keyPoints2.size()==0)
	{
		printf("=>no keypoints detected!\n");
		return false;
	}
	//��4����������������������
	SurfDescriptorExtractor extractor;
	Mat descriptors1, descriptors2;
	extractor.compute(srcImage1, keyPoint1, descriptors1);
	extractor.compute(srcImage2, keyPoints2, descriptors2);

	BFMatcher matcher;
	vector<DMatch> matches; 
	matcher.match(descriptors1, descriptors2, matches);  

	sort(matches.begin(), matches.end());  //筛选匹配点
	vector< DMatch > good_matches;
	int ptsPairs = (int)(matches.size() * 0.6);//std::min(1000, (int)(matches.size() * 0.6));
	for (int i = 0; i < ptsPairs; i++)
	{
		good_matches.push_back(matches[i]);
	}

	if(cps!=NULL)
		delete []cps,cps = NULL;
	nCps = good_matches.size();
	cps = new matchPts[nCps];

	for (int i = 0;i<nCps;i++)
	{
		cps[i].xl = keyPoint1[good_matches[i].queryIdx].pt.x;
		cps[i].yl = keyPoint1[good_matches[i].queryIdx].pt.y;

		cps[i].xr = keyPoints2[good_matches[i].trainIdx].pt.x;
		cps[i].yr = keyPoints2[good_matches[i].trainIdx].pt.y;
	}

	return true;
}

bool delMatchAffine(vector<matchPts> &allGCP,double fSigmacondition)
{
	int num = allGCP.size();
	if(num<3)
		return false;
	double *xl,*yl,*xr,*yr;
	xl = new double[num];
	yl = new double[num];
	xr = new double[num];
	yr = new double[num];

	for (int i = 0;i<num;i++)
	{
		xl[i] = allGCP[i].xl;
		yl[i] = allGCP[i].yl;
		xr[i] = allGCP[i].xr;
		yr[i] = allGCP[i].yr;
	}

	GeoTranslation affineModel;
	int index = num;

	bool bBread = true;
	do
	{
		if(affineModel.CalAffineParam(xl,yl,xr,yr,index) == false)
		{
			bBread = false;
			break;
		}
		double sigmax = 0,sigmay = 0,sigma = 0,affXr,affYr,*errx,*erry;
		errx = new double[index];
		erry = new double[index];
		for (int i = 0;i<index;i++)
		{
			affineModel.GetValueBaseAffine(xl[i],yl[i],affXr,affYr);
			errx[i] = affXr - xr[i];
			erry[i] = affYr-yr[i];
			sigmax += errx[i]*errx[i]/index;
			sigmay += erry[i]*erry[i]/index;
		}
		sigma = sqrt(sigmax+sigmay);
		sigmax = sqrt(sigmax);
		sigmay = sqrt(sigmay);
		if(sigma<fSigmacondition)
		{
			delete []errx,errx = NULL;
			delete []erry,erry = NULL;
			break;
		}
		double *tmpXl,*tmpYl,*tmpXr,*tmpYr;
		tmpXl = new double[index];
		tmpYl = new double[index];
		tmpXr = new double[index];
		tmpYr = new double[index];
		int index1 = 0;
		for (int i = 0;i<index;i++)
		{
			if(fabs(errx[i])>2*sigmax || fabs(erry[i])>2*sigmay || sqrt(errx[i]*errx[i]+erry[i]*erry[i])>2*sigma)
				continue;
			tmpXl[index1] = xl[i];
			tmpYl[index1] = yl[i];
			tmpXr[index1] = xr[i];
			tmpYr[index1] = yr[i];
			index1++;
		}

		memcpy(xl,tmpXl,sizeof(double)*index1);
		memcpy(yl,tmpYl,sizeof(double)*index1);
		memcpy(xr,tmpXr,sizeof(double)*index1);
		memcpy(yr,tmpYr,sizeof(double)*index1);
		delete []errx,errx = NULL;
		delete []erry,erry = NULL;
		delete []tmpXl,tmpXl = NULL;
		delete []tmpYl,tmpYl = NULL;
		delete []tmpXr,tmpXr = NULL;
		delete []tmpYr,tmpYr = NULL;
		if(index == index1)
			break;
		index = index1;

	} while(1);

	allGCP.clear();
	matchPts perGCP;
	for (int i = 0;i<index;i++)
	{
		perGCP.xl = xl[i];
		perGCP.yl = yl[i];
		perGCP.xr = xr[i];
		perGCP.yr= yr[i];
		allGCP.push_back(perGCP);
	}

	delete []xl,xl = NULL;
	delete []yl,yl = NULL;
	delete []xr,xr = NULL;
	delete []yr,yr = NULL;
	return bBread;
}

bool BlocksForImgProcess(int bline, int bsample, int perBlockLine,
	int perBlockSample, int width, int height, ImgBlockInfo* &pBlock, int& num) //宽度也进行分割;
{
	int totalLines = height / perBlockLine;
	int otherLines = height - totalLines*perBlockLine;

	int totalSamples = width / perBlockSample;
	int otherSamples = width - totalSamples*perBlockSample;

	num = (totalLines + 1)*(totalSamples + 1);
	if (pBlock)
		delete[]pBlock, pBlock = NULL;
	pBlock = new ImgBlockInfo[num];
	for (int i = 0; i < totalLines + 1; i++)
	{
		for (int j = 0; j < totalSamples + 1; j++)
		{
			pBlock[i*(totalSamples + 1) + j].lineIndex = bline + i*perBlockLine;
			pBlock[i*(totalSamples + 1) + j].sampleIndex = bsample + j*perBlockSample;
			pBlock[i*(totalSamples + 1) + j].width = perBlockSample;
			pBlock[i*(totalSamples + 1) + j].height = perBlockLine;
			if (i == totalLines)
			{
				pBlock[i*(totalSamples + 1) + j].width = perBlockSample;
				pBlock[i*(totalSamples + 1) + j].height = otherLines;
			}
			if (j == totalSamples)
			{
				pBlock[i*(totalSamples + 1) + j].width = otherSamples;
				pBlock[i*(totalSamples + 1) + j].height = perBlockLine;
			}
			if (i == totalLines && j == totalSamples)
			{
				pBlock[i*(totalSamples + 1) + j].width = otherSamples;
				pBlock[i*(totalSamples + 1) + j].height = otherLines;
			}

		}

	}
	return true;
}
