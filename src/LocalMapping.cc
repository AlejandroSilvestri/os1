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
    ORBmatcher matcher(0.6,false);

    cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();
    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;
    int nnew=0;

    // Triangulate each match
    KeyFrameTriangulacion &kft1 = *new KeyFrameTriangulacion(mpCurrentKeyFrame);

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
        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices);

        KeyFrameTriangulacion &kft2 = *new KeyFrameTriangulacion(pKF2);

        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++){
        	// Índices de keypoints a triangular
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

			MapPoint::origen origen = MapPoint::normal;

			// Rayos versores con centro en foco, apuntando al keypoint, en el sistema de referencia del keyframe
			cv::Mat ray1 = kft1.rayo(idx1);
			cv::Mat ray2 = kft2.rayo(idx2);
            float cosParallaxRays = ray1.dot(ray2);

			cv::Mat x3D, x3Dt;
			if(cosParallaxRays>0 && cosParallaxRays<0.9998 ){
				// Linear Triangulation Method
				x3D = kft1.triangular(kft2);

				// ¿Lambda cero?
				if(x3D.at<float>(3) != 0){
					// Convierte coordenadas del punto triangulado, de homogéneas a euclideanas
					// Euclidean coordinates
					x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
					//origen = MapPoint::normal; // es normal por defecto

					if(cosParallaxRays > umbralCos)	// Umbral arbitrario, controlado por el uusario.  998 o más lo desactiva.
						origen = MapPoint::umbralCosBajo;

				} else {//continue;
					// Lambda cero en coordenadas homogéneas: punto en el infinito.
					origen = MapPoint::svdInf;
					x3D = x3D.rowRange(0,3);	// Vector con la dirección del punto en el infinito.  Falta verificar sentido.
					x3D = x3D/norm(x3D) * 1e8;	// Multiplica por 1e7 para enviarlo al quasi-infinito
					// ¿Superará el error de reproyección?
				}
			} else {//continue;
				// Bajo paralaje, puntos muy lejanos, proyectarlos al quasi-infinito (QInf)
				origen = MapPoint::umbralCos;

				// Se proyecta según ray1 o ray2, que son versores casi paralelos.
				x3D = (ray1+ray2) * 1e8;
				// ¿Superará el error de reproyección?
			}

			// Activación del usuario para puntos lejanos
			if(origen != MapPoint::normal && !creacionDePuntosLejanosActivada) continue;

			x3Dt = x3D.t();

			//Check triangulation in front of cameras
			if(kft1.coordenadaZ(x3Dt)<=0){
				//continue;
				if(origen == MapPoint::svdInf || origen == MapPoint::umbralCos)
					// Cambiar el sentido del rayo al infinito y volver a comprobar
					x3Dt = -x3Dt;
				else
					continue;
			}
			if(kft2.coordenadaZ(x3Dt)<=0) continue;

			//Check reprojection error in first keyframe
			if(!kft1.validarErrorReproyeccion(x3Dt)) continue;

			//Check reprojection error in second keyframe
			if(!kft2.validarErrorReproyeccion(x3Dt)) continue;

			//Check scale consistency

			// Verifica que no tenga distancia 0, que no esté sobre el centro de la cámara de ninguna de ambas vistas.
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

            MapPoint* pMP;
            /* pMP = mpCurrentKeyFrame->GetMapPoint(idx1);

            // Si el punto existe es porque era candidato, y se debe actualizar.  Si no, se crea uno nuevo.
            if(pMP){
            	pMP->SetWorldPos(x3D);
            } else*/

            // Se crea el punto.  Se crea diferente dependiendo de si se dispone o no de la información de color del punto.
            if(mpCurrentKeyFrame->vRgb.size()){
            	pMP = new MapPoint(x3D,mpCurrentKeyFrame,mpMap, mpCurrentKeyFrame->vRgb[idx1]);
            }else
            	pMP = new MapPoint(x3D,mpCurrentKeyFrame,mpMap);

            pMP->AddObservation(mpCurrentKeyFrame,idx1);
            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);

            pMP->AddObservation(pKF2,idx2);
            pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();


            // Propiedades de punto lejano, si cabe.  Se marca como puntoCandidato luego de AddObservations, para evitar su reprocesamiento.
            if(origen != MapPoint::normal){
            	pMP->plOrigen = origen;
            	pMP->plCandidato = true;
            	if(origen == MapPoint::umbralCosBajo)
            		pMP->plLejano = MapPoint::lejano;
            	else
            		pMP->plLejano = MapPoint::muyLejano;
            }

            pMP->plCosOrigen = cosParallaxRays;


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
