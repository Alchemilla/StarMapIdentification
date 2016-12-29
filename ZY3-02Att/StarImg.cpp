//#include "StdAfx.h"
//#include "StarImg.h"
//
//
//StarImg::StarImg(void)
//{
//}
//
//
//StarImg::~StarImg(void)
//{
//}
//
////det_x,det_y,s1,s2,s3,s4
//bool StarImg::CalcFrameDistortion_x0y0ScaleSital(OffsetAngle* ru,string sRes)
//{
//	int nUnknown= 6;
//	double* A_T_P_A = new double[nUnknown*nUnknown];
//	double* A_T_P_L = new double[nUnknown];
//	double* pParams = new double[nUnknown];
//	double *pUnknown = new double[nUnknown];
//	memset(pUnknown,0,sizeof(double)*nUnknown);
//	
//	double Rbody2Wgs84[9],RWgs842body[9],XYZs[3],XYZ[3];
//	double oriXYZ[3],resXYZ[3];
//	double *pRes = new double[nUnknown];
//	int i;
//	int iter = 0;
//	
//	double sigma0 = 9999999999999.,sigma1;
//	double Rbody2cam[9],Rcam2Body[9],Ru_inv[9],Ru[9];
//	m_pGeoModel[0].GetSensorInfor().GetCameraInstall(Rcam2Body);
//	memcpy(Rbody2cam,Rcam2Body,sizeof(double)*9);
//	invers_matrix(Rbody2cam,3);
//
//	bool bConstRu  = false;
//	if(fabs(ru[0].Vphi)<1.e-22 && fabs(ru[0].Vomega)<1.e-22 && fabs(ru[0].Vkappa)<1.e-22)
//		bConstRu = true;
//	double r2,det_x,det_y, X_,Y_,Z_,L,f = m_pGeoModel[0].GetSensorInfor().GetCameraF();
//	LookAngle angle;
//
//	RSProgressEx* ISProgress = new RSProgressEx("畸变解求",NULL);
//	(*ISProgress).Start("畸变解求",100);
//	do 
//	{
//		memset(A_T_P_A,0,sizeof(double)*(nUnknown*nUnknown));
//		memset(A_T_P_L,0,sizeof(double)*(nUnknown));
//
//		sigma1=0.;
//		int total = 0;
//		for ( i = 0;i<m_nImgNum;i++)
//		{
//			if(bConstRu)
//			{
//				rot(ru[i].phi,ru[i].omega,ru[i].kappa,Ru);
//				memcpy(Ru_inv,Ru,sizeof(double)*9);
//				invers_matrix(Ru_inv,3);
//			}
//			for (int j = 0;j<m_pGcpNums[i];j++)
//			{
//				if(!bConstRu)
//				{
//					rot(ru[i].phi+ru[i].Vphi*m_pGCP[i][j].y,ru[i].omega+ru[i].Vomega*m_pGCP[i][j].y,ru[i].kappa+ru[i].Vkappa*m_pGCP[i][j].y,Ru);
//					memcpy(Ru_inv,Ru,sizeof(double)*9);
//					invers_matrix(Ru_inv,3);
//				}
//				
//				double idealx,idealy;
//				m_pGeoModel[i].GetSensorInfor().GetPosInCam(m_pGCP[i][j].x,m_pGCP[i][j].y,idealx,idealy);
//
//				r2 = idealx*idealx+idealy*idealy;
//				
//				det_x = pUnknown[0]+pUnknown[2]*idealx+pUnknown[3]*idealy;//+(pUnknown[6]*r2+pUnknown[7]*r2*r2)*idealx+pUnknown[8]*(r2+2*idealx*idealx)+2*pUnknown[9]*idealx*idealy;
//				det_y = pUnknown[1]+pUnknown[4]*idealx+pUnknown[5]*idealy;//+(pUnknown[6]*r2+pUnknown[7]*r2*r2)*idealy+pUnknown[9]*(r2+2*idealy*idealy)+2*pUnknown[8]*idealx*idealy;
//				
//				m_pGeoModel[i].GetExtOrientationWithoutRu(m_pGCP[i][j].x,m_pGCP[i][j].y,XYZs,Rbody2Wgs84,-1);
//				memcpy(RWgs842body,Rbody2Wgs84,sizeof(double)*9);
//				invers_matrix(RWgs842body,3);
//				
//				oriXYZ[0] = m_pGCP[i][j].X - XYZs[0];
//				oriXYZ[1] = m_pGCP[i][j].Y - XYZs[1];
//				oriXYZ[2] = m_pGCP[i][j].Z - XYZs[2];
//				
//				mult(RWgs842body,oriXYZ,resXYZ,3,3,1);
//				mult(Ru_inv,resXYZ,oriXYZ,3,3,1);
//				mult(Rbody2cam,oriXYZ,XYZ,3,3,1);
//				X_ = XYZ[0];
//				Y_ = XYZ[1];
//				Z_ = XYZ[2];
//				
//				memset(pParams,0,sizeof(double)*nUnknown);
//				
//				pParams[0] = 1;
//				pParams[1] = 0.;
//				pParams[2] = idealx;
//				pParams[3] = idealy;
//				pParams[4] = 0;
//				pParams[5] =0;
//// 				pParams[6] = r2*idealx;
//// 				pParams[7] = r2*r2*idealx;
//// 				pParams[8] = r2+2*idealx*idealx;
//// 				pParams[9] = 2*idealx*idealy;
//
//				L = (idealx - f*X_/Z_-det_x);
//				sigma1 += L*L;
//
//				pNormal(pParams,nUnknown,L,A_T_P_A,A_T_P_L,1.);
//
//				memset(pParams,0,sizeof(double)*nUnknown);
//				pParams[0] = 0;
//				pParams[1] = 1.;
//				pParams[2] = 0;
//				pParams[3] = 0;
//				pParams[4] = idealx;
//				pParams[5] =idealy;
//// 				pParams[6] = r2*idealy;
//// 				pParams[7] = r2*r2*idealy;
//// 				pParams[8] = 2*idealx*idealy;
//// 				pParams[9] = r2+2*idealy*idealy;
//
//				L = (idealy - f*Y_/Z_-det_y);
//				sigma1+=L*L;
//
//				pNormal(pParams,nUnknown,L,A_T_P_A,A_T_P_L,1.0);
//				total++;
//			}
//		}
//
//		sigma1 = sqrt(sigma1/(2.*total));
//		if(sigma1>sigma0)
//			break;
//		sigma0 = sigma1;
//
//		memset(pRes,0,sizeof(double)*nUnknown);
//		GaussExt(A_T_P_A,A_T_P_L,pRes,nUnknown);
//
//		for (int n = 0;n<nUnknown;n++)
//		{
//			pUnknown[n] += pRes[n];
//		}
//
//		iter++;
//		if(iter>10)
//			break;
//		if(!(*ISProgress)(int(iter/10.*100),"畸变解求"))
//			break;
//
//	} while (!IsExitIter(pRes,1e-10,nUnknown));
//	inner.clear();
//	for (i = 0;i<m_pGeoModel[0].GetSensorInfor().GetCCDNums();i++)
//	{
//		for (int n = i*m_pGeoModel[0].GetSensorInfor().GetPixelsPerCCD();n<(i+1)*m_pGeoModel[0].GetSensorInfor().GetPixelsPerCCD();n++)
//		{
//			angle.NumOfCCD = double(n);
//			double idealx,idealy;
//			m_pGeoModel[0].GetSensorInfor().GetPosInCam(n,idealx,idealy);
//
//			px = idealx-pUnknown[4*i] ;
//			py = idealy-pUnknown[4*i+1];
//			r2 = px*px+py*py;
//			
//			det_x = py*pUnknown[i*4+3]+(pUnknown[m_pGeoModel[0].GetSensorInfor().GetCCDNums()*4]*r2+pUnknown[m_pGeoModel[0].GetSensorInfor().GetCCDNums()*4+1]*r2*r2)*px+
//				pUnknown[m_pGeoModel[0].GetSensorInfor().GetCCDNums()*4+2]*(r2+2*px*px)+2*pUnknown[m_pGeoModel[0].GetSensorInfor().GetCCDNums()*4+3]*px*py;
//			det_y = py*pUnknown[i*4+2]+(pUnknown[m_pGeoModel[0].GetSensorInfor().GetCCDNums()*4]*r2+pUnknown[m_pGeoModel[0].GetSensorInfor().GetCCDNums()*4+1]*r2*r2)*py+
//					2*pUnknown[m_pGeoModel[0].GetSensorInfor().GetCCDNums()*4+2]*px*py+pUnknown[m_pGeoModel[0].GetSensorInfor().GetCCDNums()*4+3]*(r2+2*py*py);
//			
//			px -= det_x;
//			py -= det_y;
//			
//			oriXYZ[0] = px;
//			oriXYZ[1] = py;
//			oriXYZ[2] = m_pGeoModel[0].GetSensorInfor().GetCameraF();
//			mult(Rcam2Body,oriXYZ,resXYZ,3,3,1);
//			angle.Phi_X = resXYZ[0]/resXYZ[2];
//			angle.Phi_Y = resXYZ[1]/resXYZ[2];
//
//			inner.push_back(angle);
//		}
//	}
//	DistortionParam disParam;
//	memset(&disParam,0,sizeof(DistortionParam));
//	int CCDNums = m_pGeoModel[0].GetSensorInfor().GetCCDNums();
//	for (i = 0;i<CCDNums;i++)
//	{
//		disParam.det_x0[i] = pUnknown[4*i];
//		disParam.det_y0[i] = pUnknown[4*i+1];
//		disParam.scale[i] = pUnknown[4*i+2];
//		disParam.sital[i] = pUnknown[4*i+3];
//	}
//	disParam.K1 = pUnknown[4*CCDNums];
//	disParam.K2 = pUnknown[4*CCDNums+1];
//	disParam.P1 = pUnknown[4*CCDNums+2];
//	disParam.P2 = pUnknown[4*CCDNums+3];
//	WriteDistortionMdl(disParam,sRes,x0y0scalesitalk1k2p1p2);
//	if(iter<10)
//		(*ISProgress)(100,"指向角平滑模型解算");
//	delete ISProgress;
//	delete []pRes,pRes = NULL;
//	delete []pParams,pParams = NULL;
//	delete []A_T_P_A,A_T_P_A  = NULL;
//	delete []A_T_P_L,A_T_P_L = NULL;
//	delete []pUnknown,pUnknown  = NULL;
//
//	return true;
//}
