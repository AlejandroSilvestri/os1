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

#include "FrameDrawer.h"
#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"

//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/features2d.hpp>

#include <mutex>

namespace ORB_SLAM2
{

/**
 * Constructor, que inicia el contenedor de la imagen a color a mostrar.
 */
FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

/**
 * Genera la imagen para visualización.
 * Sobre el cuadro actual procesado dibuja marcas.
 * En el estado NOT_INITIALIZED dibuja las líneas de flujo óptico consideradas para la triangulación inicial.
 * En el estado OK pone marcas verdes sobre puntos del mapa local visualizados, y marcas azules sobre puntos encontrados para VO
 *
 * Invocado exclusivamente desde Viewer::Run
 */
cv::Mat FrameDrawer::DrawFrame(float radio, int nKF)
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    //vector<int> esPL;
    std::vector<MapPoint*> mapPoints;
    int state; // Tracking state
    nKFPendientes = nKF;	// Cantidad de KF en la cola de LocalMapping, para mostrar con DrawTextInfo.

    vector<int> obs;


    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK){
        	// Se copian porque Update los cambia en cualquier momento, asincrónicamente.
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
            obs = observaciones;
            mapPoints = mvpMapPoints;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }

        cameraPos.copyTo(cameraPosDraw);
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);

    //Draw
    // Dibuja líneas verdes de flujo óptico para inicialización
    if(state==Tracking::NOT_INITIALIZED){ //INITIALIZING
        for(unsigned int i=0; i<vMatches.size(); i++)
            if(vMatches[i]>=0)
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt, cv::Scalar(0,255,255), radio);

    // Dibuja marcas sobre puntos del mapa: verdes en estado normal, azules en VO.
    }else if(state==Tracking::OK){ //TRACKING
        mnTracked=0;
        mnTrackedVO=0;
        candidatos=0;
        const float r = 5;

        // Recorre todos los puntos singulares, pone marcas solamente si tienen un punto de mapa asociado.
        for(unsigned int i=0;i<mapPoints.size();i++){	// Originalmente N en lugar de mapPoints.size().  N no se copió en el mutex, produce error por actualización asincrónica.
        	MapPoint *MP = mapPoints[i];
            if(vbVO[i] || (vbMap[i] && MP)){
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map
                if(vbMap[i]){
                	// Los puntos más nuevos (menos observados) se muestran amarillos, virando al verde cuanto más observados están.
                    cv::Scalar color = MP->color();
                    cv::circle(im, vCurrentKeys[i].pt, 2*radio , color, -1);
                    mnTracked++;
                    if(MP->plCandidato) candidatos++;
                }else{ // This is match to a "visual odometry" MapPoint created in the last frame
                	cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im, vCurrentKeys[i].pt, 2*radio, cv::Scalar(255,0,0), -1);
                    mnTrackedVO++;
                }
            }else{
            	// Agregado por mí
            	// Dibuja circulitos naranja en los puntos singulares
                cv::circle(im, vCurrentKeys[i].pt, radio, cv::Scalar(0,128,255), 1);
            }
        }
    }else  if(state==Tracking::LOST){
    	// Dibuja puntos singulares
        // Recorre todos los puntos singulares, pone marcas solamente si tienen un punto de mapa asociado.
        for(int i=0;i<N;i++)
            cv::circle(im, vCurrentKeys[i].pt, radio, cv::Scalar(0,0,255), 1);
    }

    // Agrega barra de información al pie de la imagen.
    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}


/**
 * Agrega una barra de información det texto al pie de la imagen.
 * El texto de la barra se genera en este método.
 *
 * @param im Imagen original
 * @param nState
 * @param imText Resultado, nueva imagen con la barra de información agregada.  Es un poco más alta que la imagen original, por la barra agregada.
 *
 *
 */
void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;	// Mensaje
    cv::Scalar color = cv::Scalar(0,0,0); // Color del fondo del mensaje, negro por defecto
    if(nState==Tracking::NO_IMAGES_YET){
        s << " WAITING FOR IMAGES";
        color = cv::Scalar(0,32,64);
    }else if(nState==Tracking::NOT_INITIALIZED){
        s << " INICIALIZANDO ";
        color = cv::Scalar(0,96,96);
    }else if(nState==Tracking::OK){
        if(!mbOnlyTracking){
            s << "SLAM |  ";
            color = cv::Scalar(0,mnTracked,0);
        }else{
            s << "LOCALIZACIÓN | ";
            color = cv::Scalar(64,64,0);
        }
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs
				<< ", Matches: " << mnTracked << ", Candidatos: " << candidatos
        		//<<  ", KF pendientes: " << nKFPendientes
        		<< ((nKFPendientes >= 0)? "" : ", LocalMapping ocioso")
        		;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST){
        s << " PERDIDO.  INTENTANDO RELOCALIZAR ";
        color = cv::Scalar(0,0,128);
    }
    else if(nState==Tracking::SYSTEM_NOT_READY){
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    int franjaAlto = textSize.height + 10;
    cv::Mat franja = cv::Mat(franjaAlto, im.cols, im.type(), color);
    franja = color;// Redundante

    imText = cv::Mat(im.rows + franjaAlto, im.cols, im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    franja.copyTo(imText.rowRange(im.rows,imText.rows).colRange(0,im.cols));
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

/**
 * Track invoca este método avisando que se terminó de procesar un cuadro.
 * Cuando se invoque DrawFrame, el objeto FrameDrawer tendrá los datos del último cuadro procesado.
 * Invocada sólo desde Track.
 */
void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;
    mvpMapPoints = pTracker->mCurrentFrame.mvpMapPoints;

    // Datos adicionales para viewer
    observaciones = vector<int>(N,0);
    K = pTracker->mCurrentFrame.mK;
    distCoef = pTracker->mCurrentFrame.mDistCoef;
    cameraPos = pTracker->mCurrentFrame.GetCameraCenter();

    timestamp = pTracker->mCurrentFrame.mTimeStamp;

    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;

                    observaciones[i] = pMP->Observations();
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

void FrameDrawer::onMouse( int event, int X, int Y, int, void* frameDrawer){
	 if(event != cv::EVENT_LBUTTONDOWN) return;

	 FrameDrawer *pFD = (FrameDrawer*) frameDrawer;
	 float x = (float)X / pFD->factorEscalaImagenParaMostrar;
	 float y = (float)Y / pFD->factorEscalaImagenParaMostrar;
	 float radio = 2 / pFD->factorEscalaImagenParaMostrar;
	 /*
	  * FrameDrawer::mvCurrentKeys tiene los keypoints del frame en pantalla, para buscar coincidencia de coordenadas.
	  * FrameDrawer::mvbMap es true si hay un punto asociado en esa posición del vector.
	  * FrameDrawer::mvpMapPoints tiene los puntos del mapa.
	  */

	 // Buscar los keypoints cliqueados, pueden ser varios superpuestos
	 cout << "\nClic x:" << x << ", y:" << y << endl;
	 for(int i=0; i<pFD->N; i++){
		 if(pFD->mvbMap[i]){	// Sólo considerar keypoints asociados a puntos del mapa
			 cv::Point2f pt = pFD->mvCurrentKeys[i].pt;

			 if(fabs(x-pt.x) <= radio  &&  fabs(y-pt.y) <= radio){	// clic en un cuadrado de varios píxeles de lado
				 MapPoint *pMP = pFD->mvpMapPoints[i];
				 //cout << "Punto econtrado" << endl;
				 if(pMP && !pMP->isBad()){
					 // Reporte
					 cv::Mat pos = pMP->GetWorldPos();
					 float distancia = cv::norm(pos - pFD->cameraPosDraw);
					 cout << "Id:"			<< pMP->mnId
						  << ", distancia:"	<< distancia
						  << ", pt:"		<< pt
						  << ", pos:"		<< pos.t() << endl;
					 if(pMP->plCandidato)
						 cout << "candidato, lejano:" << pMP->plLejano
						 	  << "origen:" << pMP->plOrigen
						 	  << ", cos:" << pMP->plCosOrigen
						 	  << endl;
					 else if(pMP->plConfirmacion)
						 cout << "confirmación:" << pMP->plConfirmacion << endl;
				 }
			 }
		 }
	 }
	 cout << endl;
}

} //namespace ORB_SLAM
