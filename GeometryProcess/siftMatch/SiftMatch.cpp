// SiftMatch1.cpp: implementation of the CSiftMatch class.
//
//////////////////////////////////////////////////////////////////////
#include "SiftMatch.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CSiftMatch::CSiftMatch()
{
	m_feat1 = m_feat2 = NULL;
}

CSiftMatch::~CSiftMatch()
{
	if(m_feat1)
		free( m_feat1 );m_feat1=NULL;
	if(m_feat2)
		free( m_feat2 );m_feat2=NULL;
}

bool CSiftMatch::Init(float *block1,long block1width,long block1height,float *block2,long block2width,long block2height)
{
	if (block1 == NULL || block2 == NULL)
		return false;
	m_n2 = sift_featuresblock( block2,block2width,block2height, &m_feat2 );			
	if(m_n2==0) return false;
	m_n1 = sift_featuresblock( block1,block1width,block1height, &m_feat1 );
	if(m_n1==0) return false;
	return true;
}

bool CSiftMatch::kdmatching_once(struct feature* *feat1, struct feature* feat2,int n1,int n2,int &matchednum)
{
	if(n1==0) return false;
	if(n2==0) return false;
	struct feature** nbrs;
	struct feature** nbrs_T;
	struct feature *feat;
	struct kd_node* kd_root2,* kd_root1;
	double d0, d1;
	int i,k2,k1,m=0;;

	kd_root1 = kdtree_build( *feat1, n1 );
	kd_root2 = kdtree_build( feat2, n2 );

	//��1��2
	for( i = 0; i < n1; i++ )
	{
		feat = (*feat1) + i;
		k2 = kdtree_bbf_knn( kd_root2, feat, 2, &nbrs, 200 );
		if( k2 == 2 )
		{
			d0 = descr_dist_sq( feat, nbrs[0] );
			d1 = descr_dist_sq( feat, nbrs[1] );
			//if( d0 < d1 * NN_SQ_DIST_RATIO_THR )
			if( d0 < d1 * 0.6 )
			{
				(*feat1)[i].fwd_match = nbrs[0];
				m++;

			}
		}
		free( nbrs );
	}

	matchednum=m;
	kdtree_release( kd_root2 );kd_root2=NULL;
	kdtree_release( kd_root1 );kd_root1=NULL;
	if(matchednum==0) return false;
	return true;
}

bool CSiftMatch::kdmatching(struct feature* *feat1, struct feature* feat2,int n1,int n2,int &matchednum)
{
	if(n1==0) return false;
	if(n2==0) return false;
	struct feature** nbrs;
	struct feature** nbrs_T;
	struct feature *feat;
	struct kd_node* kd_root2,* kd_root1;
	double d0, d1;
	int i,k2,k1,m=0;;
	
	kd_root1 = kdtree_build( *feat1, n1 );
	kd_root2 = kdtree_build( feat2, n2 );
	
	//��1��2
	for( i = 0; i < n1; i++ )
	{
		feat = (*feat1) + i;
		k2 = kdtree_bbf_knn( kd_root2, feat, 2, &nbrs, 200 );
		if( k2 == 2 )
		{
			d0 = descr_dist_sq( feat, nbrs[0] );
			d1 = descr_dist_sq( feat, nbrs[1] );
			//if( d0 < d1 * NN_SQ_DIST_RATIO_THR )
			if( d0 < d1 * 0.6 )
			{			
				//��1��2 ok
				//��2��1 ��ʼ
				k1 = kdtree_bbf_knn( kd_root1, nbrs[0], 2, &nbrs_T, 200 );
				if( k1 == 2 )
				{
					d0 = descr_dist_sq( nbrs[0], nbrs_T[0] );
					if(nbrs_T[0]->x!=feat->x||nbrs_T[0]->y!=feat->y)
						continue;					
					d1 = descr_dist_sq( nbrs[0], nbrs_T[1] );
					//if( d0 < d1 * NN_SQ_DIST_RATIO_THR )
					if( d0 < d1 * 0.6 )
					{
						//��2��1 ����
						(*feat1)[i].fwd_match = nbrs[0];
						m++;
						
					}
					
				}
				
			}
		}
		free( nbrs );
	}
	
	matchednum=m;
	kdtree_release( kd_root2 );kd_root2=NULL;
	kdtree_release( kd_root1 );kd_root1=NULL;
	if(matchednum==0) return false;
	return true;
}


bool CSiftMatch::ApplyImgMatch(matchPts *&pGCP,int &num)
{
    if(m_n1==0)
	{
		num=0;
		return false;
	}
	int numNodes = 0;
//	if(kdmatching(&m_feat1, m_feat2,m_n1,m_n2,numNodes)==false)
	if(kdmatching_once(&m_feat1, m_feat2,m_n1,m_n2,numNodes)==false)
	{
		num=0;
		return false;
	}
	if(numNodes<=0)
	{
		num = 0;
		return false;
	}
	num = numNodes;

	if(pGCP != NULL)
		delete []pGCP,pGCP = NULL;

	pGCP = new matchPts[num];
	int n = 0;
	for (int i = 0;i<m_n1;i++)
	{
		if(m_feat1[i].fwd_match==NULL)
			continue;
		pGCP[n].xl = m_feat1[i].x;
		pGCP[n].yl = m_feat1[i].y;
		pGCP[n].xr = m_feat1[i].fwd_match->x;
		pGCP[n].yr = m_feat1[i].fwd_match->y;
		n++;
	}

	if(n != num)
	{
		return false;
	}
	free( m_feat1 );m_feat1=NULL;
	free( m_feat2 );m_feat2=NULL;
	return true;
	
}

int siftMatch(string sImg1,string sImg2,int nBlocks,double fSigmacondition,int numThread,string sRes)
{
	if(sImg1.empty() == true)
	{
		printf("=>the path of left image is empty!\n");
		return 0;
	}
	if(sImg2.empty() == true)
	{
		printf("=>the path of left image is empty!\n");
		return 0;
	}

	GDALDataset* poDataset_L = (GDALDataset *)GDALOpen(sImg1.c_str(),GA_ReadOnly);
	if(poDataset_L == NULL)
	{
		printf("=>failed to open the left image(%s)\n",sImg1.c_str());
		return 0;
	}

	GDALDataset* poDataset_R = (GDALDataset *)GDALOpen(sImg2.c_str(),GA_ReadOnly);
	if(poDataset_R == NULL)
	{
		GDALClose((GDALDatasetH)poDataset_L);
		printf("=>failed to open the right image(%s)\n",sImg2.c_str());
		return 0;
	}

	int imgW1,imgH1;
	int imgW2,imgH2;

	imgW1  = poDataset_L->GetRasterXSize();
	imgH1 = poDataset_L->GetRasterYSize();

	imgW2 = poDataset_R->GetRasterXSize();
	imgH2 = poDataset_R->GetRasterYSize();


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
		return 0;
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
		return 0;
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

		float *pData1 = new float[pBlock[i].width*pBlock[i].height];

		for (int line = 0;line<pBlock[i].height;line++)
		{
			int ii = pBlock[i].lineIndex+line;
			for (int sample = 0;sample<pBlock[i].width;sample++)
			{
				int jj = pBlock[i].sampleIndex+sample;

				pData1[line*pBlock[i].width+sample] = pDataL[ii*imgW1+jj];
			}
		}

		float *pData2 = new float[w2*h2];
		for (int line = 0;line<h2;line++)
		{
			int ii = bL+line;
			for (int sample = 0;sample<w2;sample++)
			{
				int jj = bS+sample;

				pData2[line*w2+sample] = pDataR[ii*imgW2+jj];

			}
		}

		matchPts *pCps = NULL;
		int nCps = 0;

		CSiftMatch siftMatch;
		if(siftMatch.Init(pData1,pBlock[i].width,pBlock[i].height,pData2,w2,h2) == false)
		{
			delete []pData1,pData1 = NULL;
			delete []pData2,pData2 = NULL;
			continue;
		}

		if(siftMatch.ApplyImgMatch(pCps,nCps) == false)
		{
			delete []pData1,pData1 = NULL;
			delete []pData2,pData2 = NULL;
			if(pCps!=NULL)
				delete []pCps,pCps = NULL;
			continue;
		}

		int nThreadID = omp_get_thread_num();
		for (int n = 0;n<nCps;n++)
		{
			pCps[n].xl += pBlock[i].sampleIndex;
			pCps[n].yl+=pBlock[i].lineIndex;
			pCps[n].xr += bS;
			pCps[n].yr+=bL;

			pSubGCP[nThreadID].push_back(pCps[n]);

		}

		delete []pData1,pData1 = NULL;
		delete []pData2,pData2 = NULL;

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
		return 0;
	}
	fprintf(fpRes, "; %d\n", allSize);
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
	return allSize;
}


bool siftGPUMatch(string sImg1, string sImg2, int nBlocks, bool usedGeoModel, double fCondition, string sRes)
{
	if (sImg1.empty() == true)
	{
		printf("=>the path of left image is empty!\n");
		return 0;
	}
	if (sImg2.empty() == true)
	{
		printf("=>the path of left image is empty!\n");
		return 0;
	}

	GDALDataset* poDataset_L = (GDALDataset *)GDALOpen(sImg1.c_str(), GA_ReadOnly);
	if (poDataset_L == NULL)
	{
		printf("=>failed to open the left image(%s)\n", sImg1.c_str());
		return 0;
	}

	GDALDataset* poDataset_R = (GDALDataset *)GDALOpen(sImg2.c_str(), GA_ReadOnly);
	if (poDataset_R == NULL)
	{
		GDALClose((GDALDatasetH)poDataset_L);
		printf("=>failed to open the right image(%s)\n", sImg2.c_str());
		return 0;
	}

	int imgW1, imgH1;
	int imgW2, imgH2;

	imgW1 = poDataset_L->GetRasterXSize();
	imgH1 = poDataset_L->GetRasterYSize();

	imgW2 = poDataset_R->GetRasterXSize();
	imgH2 = poDataset_R->GetRasterYSize();


	//	int bandnum = poDataset_L->GetRasterCount();
	GDALDataType dt = poDataset_L->GetRasterBand(1)->GetRasterDataType();
	int GLType;
	switch (dt)
	{
	case GDT_Byte:
		GLType = GL_UNSIGNED_BYTE;
		break;
	case GDT_UInt16:
		GLType = GL_UNSIGNED_SHORT;
		break;
	case GDT_Int16:
		GLType = GL_SHORT;
		break;
	case GDT_UInt32:
		GLType = GL_UNSIGNED_INT;
		break;
	case GDT_Int32:
		GLType = GL_INT;
		break;
	case GDT_Float32:
		GLType = GL_FLOAT;
		break;
	case GDT_Float64:
		GLType = GL_DOUBLE;
		break;
	default:
	{
		GDALClose((GDALDatasetH)poDataset_L);
		GDALClose((GDALDatasetH)poDataset_R);
		printf("=>the data type of image is error!\n");
		return false;
	}
	}

	ushort *pDataL = new ushort[imgW1*imgH1];
	if (poDataset_L->GetRasterBand(1)->RasterIO(GF_Read, 0, 0, imgW1, imgH1, pDataL,
		imgW1, imgH1, GDT_UInt16, 0, 0) == CE_Failure)
	{
		delete[]pDataL, pDataL = NULL;
		GDALClose((GDALDatasetH)poDataset_L);
		GDALClose((GDALDatasetH)poDataset_R);
		printf("=>failed to read left image(%s)\n", sImg1.c_str());
		return 0;
	}

	ushort *pDataR = new ushort[imgW2*imgH2];
	if (poDataset_R->GetRasterBand(1)->RasterIO(GF_Read, 0, 0, imgW2, imgH2, pDataR,
		imgW2, imgH2, GDT_UInt16, 0, 0) == CE_Failure)
	{
		delete[]pDataL, pDataL = NULL;
		delete[]pDataR, pDataR = NULL;
		GDALClose((GDALDatasetH)poDataset_L);
		GDALClose((GDALDatasetH)poDataset_R);
		printf("=>failed to read right image(%s)\n", sImg2.c_str());
		return 0;
	}

	string sImg1Rpc = sImg1.substr(0, sImg1.find_last_of('.')) + "_rpc.txt";
	string sImg2Rpc = sImg2.substr(0, sImg2.find_last_of('.')) + "_rpc.txt";

	bool usedDefaultRange = false;
	GeoModelRFM img1RPC, img2RPC;
	if (img1RPC.ReadRFMFile(sImg1Rpc) == false)
		usedDefaultRange = true;
	if (img2RPC.ReadRFMFile(sImg2Rpc) == false)
		usedDefaultRange = true;

	double aveH = (img1RPC.getAveHeight() + img2RPC.getAveHeight()) / 2.;
	int lineIter = imgH1 / nBlocks;
	int sampleIter = imgW1 / nBlocks;

	ImgBlockInfo *pBlock = NULL;
	int numBlocks;
	BlocksForImgProcess(0, 0, lineIter, sampleIter, imgW1, imgH1, pBlock, numBlocks);

	vector<matchPts> allGCP;
	for (int i = 0; i < numBlocks; i++)
	{
		if (pBlock[i].width == 0 || pBlock[i].height == 0)
			continue;

		int bSample = pBlock[i].sampleIndex, eSample = pBlock[i].sampleIndex + pBlock[i].width - 1;
		int bLine = pBlock[i].lineIndex, eLine = pBlock[i].lineIndex + pBlock[i].height - 1;

		double lx[4] = { bSample,eSample,eSample,bSample }, ly[4] = { bLine,bLine,eLine,eLine };

		int bS, eS, bL, eL;
		bS = bL = (imgH2 > imgW2 ? imgH2 : imgW2);
		eS = eL = 0;

		if (usedGeoModel)
		{
			double lat, lon, h, rx, ry;
			for (int j = 0; j < 4; j++)
			{
				//h = pModel[0].fromxyh2latlon(lx[j],ly[j],0,lat,lon,-1,true);
				//if(fabs(h+99999.)<1.e-6)
				//h = pModel[0].fromxyh2latlon(lx[j],ly[j],0,lat,lon,-1,false);
				img1RPC.FromXY2LatLon(lx[j], ly[j], aveH, lat, lon);

				img2RPC.FromLatLon2XY(lat, lon, aveH, rx, ry);

				bS = bS < rx ? bS : rx;
				eS = eS > rx ? eS : rx;
				bL = bL < ry ? bL : ry;
				eL = eL > ry ? eL : ry;

			}
		}
		else
		{
			bS = bSample - 50;
			bL = bLine - 50;
			eS = eSample + 50;
			eL = eLine + 50;
		}

		bS = bS < 0 ? 0 : bS; bS = bS > imgW2 - 1 ? imgW2 - 1 : bS;
		eS = eS < 0 ? 0 : eS; eS = eS > imgW2 - 1 ? imgW2 - 1 : eS;
		bL = bL < 0 ? 0 : bL; bL = bL > imgH2 - 1 ? imgH2 - 1 : bL;
		eL = eL < 0 ? 0 : eL; eL = eL > imgH2 - 1 ? imgH2 - 1 : eL;

		int w2 = eS - bS; int h2 = eL - bL;

		if (w2 == 0 || h2 == 0)
			continue;

		byte *pDataL = new byte[pBlock[i].width*pBlock[i].height];
		if (poDataset_L->GetRasterBand(1)->RasterIO(GF_Read, pBlock[i].sampleIndex, pBlock[i].lineIndex, pBlock[i].width, pBlock[i].height, pDataL,
			pBlock[i].width, pBlock[i].height, dt, 0, 0) == CE_Failure)
		{
			delete[]pDataL, pDataL = NULL;
			GDALClose((GDALDatasetH)poDataset_L);
			GDALClose((GDALDatasetH)poDataset_R);
			printf("=>failed to read left image(%s)\n", sImg1.c_str());
			return false;
		}

		byte *pDataR = new byte[w2*h2];
		if (poDataset_R->GetRasterBand(1)->RasterIO(GF_Read, bS, bL, w2, h2, pDataR,
			w2, h2, dt, 0, 0) == CE_Failure)
		{
			delete[]pDataL, pDataL = NULL;
			delete[]pDataR, pDataR = NULL;
			GDALClose((GDALDatasetH)poDataset_L);
			GDALClose((GDALDatasetH)poDataset_R);
			printf("=>failed to read left image(%s)\n", sImg2.c_str());
			return false;
		}

		vector<matchPts> cps;
		siftMatch_core(pDataL, pBlock[i].width, pBlock[i].height, pDataR, w2, h2, GLType, cps);

		int numMatch = cps.size();

		for (int n = 0; n < numMatch; n++)
		{
			cps[n].xl += pBlock[i].sampleIndex;
			cps[n].yl += pBlock[i].lineIndex;
			cps[n].xr += bS;
			cps[n].yr += bL;

			allGCP.push_back(cps[n]);

		}

		delete[]pDataL, pDataL = NULL;
		delete[]pDataR, pDataR = NULL;
	}

	//siftMatch(sImgPath[0],sImgPath[1],allGCP);

	delete[]pBlock, pBlock = NULL;
	GDALClose((GDALDatasetH)poDataset_L);
	GDALClose((GDALDatasetH)poDataset_R);

	delMatchAffine(allGCP, fCondition);

	int allSize = allGCP.size();

	//printf("=>total %d matched points after elimination;\n", allSize);

	FILE *fpRes = fopen(sRes.c_str(), "w");
	if (!fpRes)
	{
		printf("=>failed to create result file(%s)\n", sRes.c_str());
		return false;
	}
	fprintf(fpRes, "; %d\n", allSize);
	for (int i = 0; i < allSize; i++)
	{
		fprintf(fpRes, "%f\t%f\t%f\t%f\n", allGCP[i].xl, allGCP[i].yl, allGCP[i].xr, allGCP[i].yr);
	}
	fclose(fpRes);
	return true;
}

//template<class T>
//bool siftGPUMatch(CRPCModel* pModel, string *sImgPath, int nBlocks, bool usedGeoModel, double fCondition, string sRes, T dataType)
//{
//	if (pModel == NULL || sImgPath == NULL)
//	{
//		printf("=>the input image path is empty!\n");
//		return false;
//	}
//	GDALDataset* poDataset_L = (GDALDataset *)GDALOpen(sImgPath[0].c_str(), GA_ReadOnly);
//	if (poDataset_L == NULL)
//	{
//		printf("=>failed to open the left image(%s)\n", sImgPath[0].c_str());
//		return false;
//	}
//	GDALDataset* poDataset_R = (GDALDataset *)GDALOpen(sImgPath[1].c_str(), GA_ReadOnly);
//	if (poDataset_R == NULL)
//	{
//		GDALClose((GDALDatasetH)poDataset_L);
//		printf("=>failed to open the right image(%s)\n", sImgPath[1].c_str());
//		return false;
//	}
//
//	int imgW1, imgH1;
//	int imgW2, imgH2;
//
//	imgW1 = poDataset_L->GetRasterXSize();
//	imgH1 = poDataset_L->GetRasterYSize();
//
//	imgW2 = poDataset_R->GetRasterXSize();
//	imgH2 = poDataset_R->GetRasterYSize();
//
//	int bandnum = poDataset_L->GetRasterCount();
//	GDALDataType dt = poDataset_L->GetRasterBand(1)->GetRasterDataType();
//
//	int GLType;
//	switch (dt)
//	{
//	case GDT_Byte:
//		GLType = GL_UNSIGNED_BYTE;
//		break;
//	case GDT_UInt16:
//		GLType = GL_UNSIGNED_SHORT;
//		break;
//	case GDT_Int16:
//		GLType = GL_SHORT;
//		break;
//	case GDT_UInt32:
//		GLType = GL_UNSIGNED_INT;
//		break;
//	case GDT_Int32:
//		GLType = GL_INT;
//		break;
//	case GDT_Float32:
//		GLType = GL_FLOAT;
//		break;
//	case GDT_Float64:
//		GLType = GL_DOUBLE;
//		break;
//	default:
//	{
//		GDALClose((GDALDatasetH)poDataset_L);
//		GDALClose((GDALDatasetH)poDataset_R);
//		printf("=>the data type of image is error!\n");
//		return false;
//	}
//	}
//
//	int lineIter = imgH1 / nBlocks;
//	int sampleIter = imgW1 / nBlocks;
//
//	ImgBlockInfo *pBlock = NULL;
//	int numBlocks;
//	BlocksForImgProcess(0, 0, lineIter, sampleIter, imgW1, imgH1, pBlock, numBlocks);
//
//	vector<matchPts> allGCP;
//	double aveH = pModel[0].getAveHeight();
//	for (int i = 0; i < numBlocks; i++)
//	{
//		if (pBlock[i].width == 0 || pBlock[i].height == 0)
//			continue;
//
//		int bSample = pBlock[i].sampleIndex, eSample = pBlock[i].sampleIndex + pBlock[i].width - 1;
//		int bLine = pBlock[i].lineIndex, eLine = pBlock[i].lineIndex + pBlock[i].height - 1;
//
//		double lx[4] = { bSample,eSample,eSample,bSample }, ly[4] = { bLine,bLine,eLine,eLine };
//
//		int bS, eS, bL, eL;
//		bS = bL = (imgH2 > imgW2 ? imgH2 : imgW2);
//		eS = eL = 0;
//
//		if (usedGeoModel)
//		{
//			double lat, lon, h, rx, ry;
//			for (int j = 0; j < 4; j++)
//			{
//				//h = pModel[0].fromxyh2latlon(lx[j],ly[j],0,lat,lon,-1,true);
//				//if(fabs(h+99999.)<1.e-6)
//				//h = pModel[0].fromxyh2latlon(lx[j],ly[j],0,lat,lon,-1,false);
//				pModel[0].calclatlon_affine(lx[j], ly[j], pModel[0].getAveHeight(), lat, lon);
//
//				pModel[1].calcxy(lat, lon, pModel[0].getAveHeight(), rx, ry);
//
//				bS = bS < rx ? bS : rx;
//				eS = eS > rx ? eS : rx;
//				bL = bL < ry ? bL : ry;
//				eL = eL > ry ? eL : ry;
//
//			}
//		}
//		else
//		{
//			bS = bSample - 50;
//			bL = bLine - 50;
//			eS = eSample + 50;
//			eL = eLine + 50;
//		}
//
//		bS = bS < 0 ? 0 : bS; bS = bS > imgW2 - 1 ? imgW2 - 1 : bS;
//		eS = eS < 0 ? 0 : eS; eS = eS > imgW2 - 1 ? imgW2 - 1 : eS;
//		bL = bL < 0 ? 0 : bL; bL = bL > imgH2 - 1 ? imgH2 - 1 : bL;
//		eL = eL < 0 ? 0 : eL; eL = eL > imgH2 - 1 ? imgH2 - 1 : eL;
//
//		int w2 = eS - bS; int h2 = eL - bL;
//
//		if (w2 == 0 || h2 == 0)
//			continue;
//
//		T *pDataL = new T[pBlock[i].width*pBlock[i].height];
//		if (poDataset_L->GetRasterBand(1)->RasterIO(GF_Read, pBlock[i].sampleIndex, pBlock[i].lineIndex, pBlock[i].width, pBlock[i].height, pDataL,
//			pBlock[i].width, pBlock[i].height, dt, 0, 0) == CE_Failure)
//		{
//			delete[]pDataL, pDataL = NULL;
//			GDALClose((GDALDatasetH)poDataset_L);
//			GDALClose((GDALDatasetH)poDataset_R);
//			printf("=>failed to read left image(%s)\n", sImgPath[0].c_str());
//			return false;
//		}
//
//		T *pDataR = new T[w2*h2];
//		if (poDataset_R->GetRasterBand(1)->RasterIO(GF_Read, bS, bL, w2, h2, pDataR,
//			w2, h2, dt, 0, 0) == CE_Failure)
//		{
//			delete[]pDataL, pDataL = NULL;
//			delete[]pDataR, pDataR = NULL;
//			GDALClose((GDALDatasetH)poDataset_L);
//			GDALClose((GDALDatasetH)poDataset_R);
//			printf("=>failed to read left image(%s)\n", sImgPath[1].c_str());
//			return false;
//		}
//
//		vector<matchPts> cps;
//		siftMatch_core(pDataL, pBlock[i].width, pBlock[i].height, pDataR, w2, h2, GLType, cps);
//
//		int numMatch = cps.size();
//
//		for (int n = 0; n < numMatch; n++)
//		{
//			cps[n].xl += pBlock[i].sampleIndex;
//			cps[n].yl += pBlock[i].lineIndex;
//			cps[n].xr += bS;
//			cps[n].yr += bL;
//
//			allGCP.push_back(cps[n]);
//
//		}
//
//		delete[]pDataL, pDataL = NULL;
//		delete[]pDataR, pDataR = NULL;
//	}
//
//	//siftMatch(sImgPath[0],sImgPath[1],allGCP);
//
//	delete[]pBlock, pBlock = NULL;
//	GDALClose((GDALDatasetH)poDataset_L);
//	GDALClose((GDALDatasetH)poDataset_R);
//
//	delMatchAffine(allGCP, fCondition);
//
//	int allSize = allGCP.size();
//
//	printf("=>total %d matched points after elimination;\n", allSize);
//
//	FILE *fpRes = fopen(sRes.c_str(), "w");
//	if (!fpRes)
//	{
//		printf("=>failed to create result file(%s)\n", sRes.c_str());
//		return false;
//	}
//	fprintf(fpRes, "%d\n", allSize);
//	for (int i = 0; i < allSize; i++)
//	{
//		fprintf(fpRes, "%f\t%f\t%f\t%f\n", allGCP[i].xl, allGCP[i].yl, allGCP[i].xr, allGCP[i].yr);
//	}
//	fclose(fpRes);
//	return true;
//}

bool siftMatch_core(void* pData1, int w1, int h1, void *pData2, int w2, int h2, int dataType, vector<matchPts>& cps)
{
	SiftGPU sift;
	char *params[] = { "-fo", "-1",  "-v", "0","-mo","1","-cuda","0" };
	int argc = sizeof(params) / sizeof(char*);
	sift.ParseParam(argc, params);
	if (sift.CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED) return 0;

	//	sift.SetTightPyramid();
	vector<float > descriptors1(1), descriptors2(1);
	vector<SiftGPU::SiftKeypoint> keys1(1), keys2(1);
	int num1 = 0, num2 = 0;

	if (sift.RunSIFT(w1, h1, pData1, GL_LUMINANCE, dataType))
	{
		num1 = sift.GetFeatureNum();
		keys1.resize(num1);    descriptors1.resize(128 * num1);

		//reading back feature vectors is faster than writing files
		//if you dont need keys or descriptors, just put NULLs here
		sift.GetFeatureVector(&keys1[0], &descriptors1[0]);
	}

	if (sift.RunSIFT(w2, h2, pData2, GL_LUMINANCE, dataType))
	{
		num2 = sift.GetFeatureNum();
		keys2.resize(num2);    descriptors2.resize(128 * num2);
		sift.GetFeatureVector(&keys2[0], &descriptors2[0]);
	}

	SiftMatchGPU* matcher = CreateNewSiftMatchGPU(4096);
	//matcher->SetLanguage()

	matcher->VerifyContextGL(); //must call once

	matcher->SetDescriptors(0, num1, &descriptors1[0]); //image 1
	matcher->SetDescriptors(1, num2, &descriptors2[0]); //image 2

														//match and get result.    
	int(*match_buf)[2] = new int[num1][2];
	//use the default thresholds. Check the declaration in SiftGPU.h
	int num_match = matcher->GetSiftMatch(num1, match_buf);

	//printf("=>%d sift matches were found!\n", num_match);

	//enumerate all the feature matches'
	matchPts pts;
	for (int i = 0; i < num_match; ++i)
	{
		//How to get the feature matches: 
		//SiftGPU::SiftKeypoint & key1 = keys1[match_buf[i][0]];
		//SiftGPU::SiftKeypoint & key2 = keys2[match_buf[i][1]];
		//key1 in the first image matches with key2 in the second image
		pts.xl = keys1[match_buf[i][0]].x;
		pts.yl = keys1[match_buf[i][0]].y;

		pts.xr = keys2[match_buf[i][1]].x;
		pts.yr = keys2[match_buf[i][1]].y;

		cps.push_back(pts);
	}

	delete[] match_buf;
	delete matcher;

	return true;
}
