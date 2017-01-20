/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "KeyFrameTriangulacion.h"

#include<mutex>

namespace ORB_SLAM2
{

LocalMapping::LocalMapping(Map *pMap, const float bMonocular):
    mbMonocular(bMonocular), mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true)
{}

void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LocalMapping::Run()
{

    mbFinished = false;

    while(1)
    {
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // BoW conversion and insertion in Map
            ProcessNewKeyFrame();

            // Check recent MapPoints
            MapPointCulling();

            // Triangulate new MapPoints
            CreateNewMapPoints();

            if(!CheckNewKeyFrames())
            {
                // Find more matches in neighbor keyframes and fuse point duplications
                SearchInNeighbors();
            }

            mbAbortBA = false;

            if(!CheckNewKeyFrames() && !stopRequested())
            {
                // Local BA
                if(mpMap->KeyFramesInMap()>2)
                    Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpMap);

                // Check redundant local Keyframes
                KeyFrameCulling();
            }

            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
        }
        else if(Stop())
        {
            // Safe area to stop
            while(isStopped() && !CheckFinish())
            {
                usleep(3000);
            }
            if(CheckFinish())
                break;
        }

        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        usleep(3000);
    }

    SetFinish();
}

void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;
}

bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

void LocalMapping::ProcessNewKeyFrame()
{
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }

    // Compute Bags of Words structures
    mpCurrentKeyFrame->ComputeBoW();

    // Associate MapPoints to the new keyframe and update normal and descriptor
    const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    for(size_t i=0; i<vpMapPointMatches.size(); i++){
        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP && !pMP->isBad() && !pMP->IsInKeyFrame(mpCurrentKeyFrame)){
			pMP->AddObservation(mpCurrentKeyFrame, i);
			pMP->UpdateNormalAndDepth();
			pMP->ComputeDistinctiveDescriptors();
        }
    }    

    // Update links in the Covisibility Graph
    mpCurrentKeyFrame->UpdateConnections();

    // Insert Keyframe in Map
    mpMap->AddKeyFrame(mpCurrentKeyFrame);
}

void LocalMapping::MapPointCulling()
{
    // Check Recent Added MapPoints
    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    int nThObs;
    if(mbMonocular)
        nThObs = 2;
    else
        nThObs = 3;
    const int cnThObs = nThObs;

    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPoint* pMP = *lit;
        if(pMP->isBad())
        {
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(pMP->GetFoundRatio()<0.25f )
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
            lit++;
    }
}

void LocalMapping::CreateNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    int nn = 20;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    cout << "CreateNewMapPoints invocado.  KF vecinos:" << vpNeighKFs.size() << endl;
    ORBmatcher matcher(0.6,false);

    cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    KeyFrameTriangulacion &kft1 = *new KeyFrameTriangulacion(mpCurrentKeyFrame);
    /*
    cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
    cv::Mat Tcw1(3,4,CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));

    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;
     */

    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;

    int nnew=0;

    // Triangulate each match
    float umbralCos = 0.9998;
    if(param<998)
    	umbralCos = 0.9 + (float)param/10000;

    // Search matches with epipolar restriction and triangulate
    for(size_t i=0; i<vpNeighKFs.size(); i++){

        if(i>0 && CheckNewKeyFrames())
            return;

        KeyFrame* pKF2 = vpNeighKFs[i];

        // Check first that baseline is not too short
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        cv::Mat vBaseline = Ow2-Ow1;
        const float baseline = cv::norm(vBaseline);

		const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
		const float ratioBaselineDepth = baseline/medianDepthKF2;

		if(ratioBaselineDepth<0.01)
			continue;

        // Compute Fundamental Matrix
        cv::Mat F12 = ComputeF12(mpCurrentKeyFrame,pKF2);

        // Search matches that fullfil epipolar constraint
        vector<pair<size_t,size_t> > vMatchedIndices;
        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices);//,false);

        KeyFrameTriangulacion &kft2 = *new KeyFrameTriangulacion(pKF2);
        //pKF2->inicializarTriangulacion();
        /*
        cv::Mat Rcw2 = pKF2->GetRotation();
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = pKF2->GetTranslation();
        cv::Mat Tcw2(3,4,CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;
         */

        const int nmatches = vMatchedIndices.size();
        cout << "En KF hay matches:" << nmatches << endl;
        for(int ikp=0; ikp<nmatches; ikp++){
        	/* Triangulación.
        	 *
        	 * kp1 y kp2 son los keypoints de cada frame.
        	 * xn1 y xn2 son las coordenadas homogéneas de kp1 y kp2 normalizadas según K (expresadas en distancias focales en lugar de píxeles).
        	 * También son vectores 3D con centro en la cámara, apuntando al keypoint, en el sistema de referencia de la cámara.
        	 * ray1 y ray2 son los "rayos" en el sistema de referencia de orientación del mundo.
        	 *
        	 */
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

			cv::Mat x3D;
			MapPoint::origen origen = MapPoint::normal;
            float cosParallaxRays;

			D(idx1);
			cv::Mat ray1 = kft1.rayo(idx1);
			D(ray1);
			cv::Mat ray2 = kft2.rayo(idx2);
			D(ray2);
			cosParallaxRays = ray1.dot(ray2);
			/*
			const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
			const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];

			// Check parallax between rays
			cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
			cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

			cv::Mat ray1 = Rwc1*xn1;
			cv::Mat ray2 = Rwc2*xn2;
			const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));
			*/
			//cout << "cosParallaxRays" << cosParallaxRays << endl;

			if(cosParallaxRays>0 && cosParallaxRays<0.9998 ){
				// Linear Triangulation Method
				//cv::Mat A(4,4,CV_32F);

				x3D = kft1.triangular(kft2);
				//mpCurrentKeyFrame->A.copyTo(A.rowRange(0,2));
				//pKF2			 ->A.copyTo(A.rowRange(2,4));

				//cout << "mpCurrentKeyFrame->A:" << mpCurrentKeyFrame->A << endl;
				/*
				A.rowRange(0,2) = mpCurrentKeyFrame->A.copyTo(A.rowRange(0,2));
				A.rowRange(3,4) = pKF2->A;
				 */
				/*
				A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
				A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
				A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
				A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);
				 */

				/* Sobre SVD
				 * SVD descompone A = u E vt
				 * E es una matriz cuadrada diagonal.
				 * SVD::compute devuelve w con los elementos de la diagonal de E.
				 *
				 */
				/*
				cv::Mat w,u,vt;
				cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

				x3D = vt.row(3).t();
				 */
				D(x3D);

				// ¿Y esto?  ¿Punto lejano?
				if(x3D.at<float>(3)==0){
					continue;
					// Punto al infinito, originalmente continue para descartarlo
					origen = MapPoint::svdInf;
					x3D = x3D.rowRange(0,3);	// Vector con la dirección del punto en el infinito.  Falta verificar sentido.
					x3D = x3D/norm(x3D) * 1e8;	// Multiplica por 1e7 para enviarlo al quasi-infinito
					//cout << "Creado un punto lejano 3, por SVD:" << mpCurrentKeyFrame->kp.pt << endl;
				} else {
					// Convierte coordenadas del punto triangulado, de homogéneas a euclideanas
					// Euclidean coordinates
					x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

					/*if(cosParallaxRays > umbralCos){	// Umbral arbitrario, empírico, 2,5º.
						origen = MapPoint::umbralCosBajo;
					// ¿Superará el error de reproyección?
					} else {
						origen = MapPoint::normal;
						cout << "punto normal:" << x3D << ", índice:" << idx1 << ", ray1:" << ray1 << endl;
					}*/
				}
			} else continue;/*{
				//No stereo and very low parallax

				// Puntos muy lejanos, proyectarlos al quasi-infinito (QInf)
				origen = MapPoint::umbralCos;

				// Se proyecta según ray1 o ray2, que son paralelos.
				//x3D = ray1*1e6;
				ray1 = ray1 + ray2;
				x3D = ray1/norm(ray1)*1e8;

				// ¿Superará el error de reproyección?
			}*/

			// Activación del usuario para puntos lejanos
			//if(origen != MapPoint::normal && !creacionDePuntosLejanosActivada) continue;

			cv::Mat x3Dt = x3D.t();
			D(x3Dt);


			//Check triangulation in front of cameras
			if(kft1.coordenadaZ(x3Dt)<=0){
				continue;
				if(origen == MapPoint::svdInf || origen == MapPoint::umbralCos)
					// Cambiar el sentido del rayo al infinito y volver a comprobar
					x3Dt = -x3Dt;
				else
					continue;
			}

			if(kft2.coordenadaZ(x3Dt)<=0) continue;
			/*
			float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
			if(z1<=0)
				continue;

			float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
			if(z2<=0)
				continue;
			 */

			//Check reprojection error in first keyframe
			if(!kft1.validarErrorReproyeccion(x3Dt)) continue;
			/*
			const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
			const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
			const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
			const float invz1 = 1.0/z1;

			float u1 = fx1*x1*invz1+cx1;
			float v1 = fy1*y1*invz1+cy1;
			float errX1 = u1 - kp1.pt.x;
			float errY1 = v1 - kp1.pt.y;
			if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1){
				//if(puntoLejano) cout << "Un punto lejano " << puntoLejano << " no superó el error de reproyección en el 1º keyframe " << mpCurrentKeyFrame->mnId << endl;
				continue;
			}*/

			//Check reprojection error in second keyframe
			if(!kft2.validarErrorReproyeccion(x3Dt)) continue;
			/*
			const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
			const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
			const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
			const float invz2 = 1.0/z2;

			float u2 = fx2*x2*invz2+cx2;
			float v2 = fy2*y2*invz2+cy2;
			float errX2 = u2 - kp2.pt.x;
			float errY2 = v2 - kp2.pt.y;
			if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2){
				//if(puntoLejano) cout << "Un punto lejano " << puntoLejano << " no superó el error de reproyección en el 2º keyframe " << pKF2->mnId << endl;
				continue;
			}*/

			//Check scale consistency

			// Verifica que no tenga distancia 0, que no esté sobre el centro de la cámara de ninguna de ambas vistas.

			/*
			cv::Mat normal1 = x3D-Ow1;
			float dist1 = cv::norm(normal1);

			cv::Mat normal2 = x3D-Ow2;
			float dist2 = cv::norm(normal2);
			*/
			//cout << "mutext CreateNewMapPoints terminado" << endl;


            float dist1 = kft1.distancia(x3D),
            	  dist2 = kft2.distancia(x3D);
            if(dist1==0 || dist2==0)
            	// Está sobre el foco de alguno de los dos keyframes
                continue;

            // Verifica que las distancias desde ambas vistas sean consistentes con el nivel de pirámide de sus descriptores.
            const float ratioDist = dist2/dist1;
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kft1.kp.octave]/pKF2->mvScaleFactors[kft2.kp.octave];

            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is succesfull
            /*
             * Creación del MapPoint en coordenadas x3D.
             * mpCurrentKeyFrame será el keyframe de referencia.
             * Lo observan mpCurrentKeyFrame y pKF2.
             */
            MapPoint* pMP = mpCurrentKeyFrame->GetMapPoint(idx1);
            cout << "Nuevo punto 3D en CreateNewMapPoints." << endl;

            // Si el punto existe es porque era candidato, y se debe actualizar.  Si no, se crea uno nuevo.
            /*if(pMP){
            	pMP->SetWorldPos(x3D);
            } else*/ if(mpCurrentKeyFrame->vRgb.size()){
            	pMP = new MapPoint(x3D,mpCurrentKeyFrame,mpMap, mpCurrentKeyFrame->vRgb[idx1]);
            }else
            	pMP = new MapPoint(x3D,mpCurrentKeyFrame,mpMap);

            // Propiedades de punto lejano, si cabe
            /*if(origen != MapPoint::normal){
            	pMP->plOrigen = origen;
            	pMP->plCandidato = true;
            	if(origen == MapPoint::umbralCosBajo)
            		pMP->plLejano = MapPoint::lejano;
            	else
            		pMP->plLejano = MapPoint::muyLejano;
            }

            pMP->plCosOrigen = cosParallaxRays;
             */


            pMP->AddObservation(mpCurrentKeyFrame,idx1);            
            pMP->AddObservation(pKF2,idx2);

            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpMap->AddMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);

            nnew++;
        }
    }
}

void LocalMapping::SearchInNeighbors()
{
    // Retrieve neighbor keyframes
    int nn = 20;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    vector<KeyFrame*> vpTargetKFs;
    for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;

        // Extend to some second neighbors
        const vector<KeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
        for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFrame* pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2);
        }
    }


    // Search matches by projection from current KF in target KFs
    ORBmatcher matcher;
    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;

        matcher.Fuse(pKFi,vpMapPointMatches);
    }

    // Search matches by projection from target KFs in current KF
    vector<MapPoint*> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        KeyFrame* pKFi = *vitKF;

        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

        for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPoint* pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }

    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);


    // Update points
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
            }
        }
    }

    // Update connections in covisibility graph
    mpCurrentKeyFrame->UpdateConnections();
}

cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    // Rotación del keyframe 1 respecto del mundo.
	cv::Mat R1w = pKF1->GetRotation();

    // Traslación del keyframe 1 respecto del mundo.
    cv::Mat t1w = pKF1->GetTranslation();

    // Rotación del keyframe 2 respecto del mundo.
    cv::Mat R2w = pKF2->GetRotation();

    // Traslación del keyframe 2 respecto del mundo.
    cv::Mat t2w = pKF2->GetTranslation();

    // Rotación del keyframe 1 respecto del keyframe 2
    cv::Mat R12 = R1w*R2w.t();

    // Traslación del keyframe 1 respecto del keyframe 2
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    // Matrices de cámara (es siempre la misma)
    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;

    // Cálculo de matriz fundamental a partir de la esencial
    return K1.t().inv()*t12x*R12*K2.inv();
}

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped = false;
    mbStopRequested = false;
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewKeyFrames.clear();

    cout << "Local Mapping RELEASE" << endl;
}

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

void LocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        if(pKF->mnId==0)
            continue;
        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        const int thObs = 3;
        int nRedundantObservations=0;
        int nMPs=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++){
            MapPoint* pMP = vpMapPoints[i];
            if(pMP && !pMP->isBad()){
				nMPs++;
				if(pMP->Observations()>thObs){
					const int &scaleLevel = pKF->mvKeysUn[i].octave;
					const map<KeyFrame*, size_t> observations = pMP->GetObservations();
					int nObs=0;
					for(map<KeyFrame*, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++){
						KeyFrame* pKFi = mit->first;
						if(pKFi==pKF)
							continue;

						const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;
						if(scaleLeveli<=scaleLevel+1){
							nObs++;
							if(nObs>=thObs)
								break;
						}
					}
					if(nObs>=thObs)
						nRedundantObservations++;
                }
            }
        }  

        if(nRedundantObservations>0.9*nMPs)
            pKF->SetBadFlag();
    }
}

cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<
    		 	 	 	 0 ,	-v.at<float>(2),	 v.at<float>(1),
             v.at<float>(2),				 0 ,	-v.at<float>(0),
            -v.at<float>(1),	 v.at<float>(0),  				 0
    );
}

void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(3000);
    }
}

void LocalMapping::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlNewKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();
        mbResetRequested=false;
    }
}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

} //namespace ORB_SLAM
