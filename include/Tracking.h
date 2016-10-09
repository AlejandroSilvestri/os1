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


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"

#include <mutex>

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

/** Objeto único que se ejecuta en su propio Trhead, y se inicia con Run().
 * Este objeto empaqueta el código que realiza el tracking y se ejecuta en su thread único y exclusivo.
 * .Run() cuelga el listener GrabImage(), que recibe mensajes con imágenes desde el nodo cámara vía ros.
 *
 * Por su naturaleza asincrónica, GrabImage implementa un autómata finito
 * que conduce el hilo por los distintos estados de inicialización, y de relocalización cuando se pierde el tracking.
 * Su variable de estado es mState.
 *
 * En cada ciclo (cada vez que recibe una imagen) crea una instancia de Frame, donde guarda los resultados de la
 * detección de puntos, extracción de descriptores, macheo, vínculo con los puntos 3D, etc.
 *
 * En el thread de Tracking se realiza la triangulación inicial, el SfM, y se generan los KeyFrames.
 * También se dispara la relocalización cuando se la necesita.
 *
 * Construye el mapa local en mvpLocalKeyFrames, un vector de KeyFrames.
 *
 * Convención de prefijos de miembros:
 * Las propiedades adoptan los siguientes prefijos:
 * - m: miembro
 * - mp: miembro puntero
 * - mvp: miembro vector de punteros
 * - mb: miembro booleano
 * - mn: ? se usa en algunos miembros int.
 *
 * La mayoría de los métodos son de uso interno, y muchos de ellos se invocan por única vez,
 * en el sentido que su código no necesita estar en un método, pero está así para mejorar la legibilidad.
 *
 * Sólo otros tres objetos utilizan métodos de tracking:
 * - FramePublisher
 * - LocalMapping
 * - LoopClosing
 *
 * FramPublisher consulta los estados de Tracking mState y mLastState, utiliza su enum de estados,
 * y en Update() toma datos del mCurrentFrame, mInitialFrame y mvIniMatches.
 *
 * LocalMapping::SetTracker y LoopClosing::SetTracker guardan el tracking en mpTracking, y no lo utilizan.
 *
 * La mayoría de los métodos del objeto son partes de un único algoritmo, dividido para mejor comprensión.
 * Estos métodos son invocados solo desde otro método del objeto, y no desde afuera.
 *
 * La siguiente secuencia de métodos corresponden a un único algoritmo que podría estar en una única función:
 *
 * 	GrabImageMonocular: recibe la imagen y la convierte a grises
 * 		Track: ejecuta el autómata finito
 *  		MonocularInitialization: busca maches para triangular con PnP
 * 				CreateInitialMapMonocular: crea el mapa inicial con los puntos triangulados
 * 		 	TrackWithMotionModel: tracking normal, busca macheos según el modelo de movimiento
 *  			UpdateLastFrame: trivial
 * 			TrackLocalMap: agrega puntos nuevos al mapa local
 * 				SearchLocalPoints: actualiza la "visibilidad" (el conteo de vistas) de los puntos, y filtra puntos que ya están en el mapa
 * 				UpdateLocalMap: actualiza el mapa local: puntos y keyframes
 * 					UpdateLocalMapPoints: actualiza el mapa local con los puntos nuevos
 * 					UpdateLocalKeyFrames: actualiza los keyframes locales con los puntos nuevos
 * 			NeedNewKeyFrame: evalúa si hace falta un nuevo keyframe
 * 			CreateNewKeyFrame: crea y registra un nuevo keyframe a partir del frame actual
 * 			TrackReferenceKeyFrame: tracking alternativo, basado en BoW, para cuando falla el tracking normal
 * 			CheckReplacedInLastFrame: actualiza puntos en lastKeyFrame que pudieron haber cambiado por BA, culling u otros motivos
 * 			Relocalization: se invoca si el sistema perdió tracking
 *
 */
class Tracking
{  

public:
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);
    Tracking(System* pSys, ORBVocabulary* pVoc, Map* pMap,
                   KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);


public:

	/** Estado de tracking
	La variable de estado mState del objeto Tracking tiene uno de estos valores.
	*/
    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    /** Estado del autómata finito del tracking.  mState adopta los valores del enum eTrackingState.*/
    eTrackingState mState;

    /** Estado anterior al actual.  Último estado que fue procesado por completo. */
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    /** Cuadro actual.*/
    // Current Frame
    Frame mCurrentFrame;

    /**
     * Imagen del cuadro actual, recién convertida a grises.
     * GrabImageMonocular recibe la imagen como Mat, y si es a color la convierte a grises; la guarda en mImGray.
     */
    cv::Mat mImGray;


    /** Variables de inicialización.  Luego de la inicialización, estos valores están en el Frame.*/
    // Initialization Variables (Monocular)

    /** Vector con los matches anteriores, para inicialización.*/
    std::vector<int> mvIniLastMatches;

    /** Vector con los matches de inicialización.*/
    std::vector<int> mvIniMatches;

    /** Vector con los matches anteriores, para inicialización.*/
    std::vector<cv::Point2f> mvbPrevMatched;

    /** Vector con los puntos 3D iniciales.*/
    std::vector<cv::Point3f> mvIniP3D;

    /** Cuadro en el que se inició el tracking.*/
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    /**
     * Flag indicador de modo Tracking (true) o tracking y mapping (false).
     */
    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset();

    // Agregado: parámetro del trackbar de la ventana que muestra el cuadro actual.
    int param = 100;

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    //void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    /**
     * Flag de odometría visual.
     * Es true cuando no hay macheos con puntos del mapa.
     * El sistema entra en modo de odometría visual para adivinar en qué lugar del mapa está, mientras intenta relocalizar.
     */
    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB

    /**
     * Extractores de cámara.
     * Right se usa solamente en estéreo; monocular usa solamente Left.
     * Ini es una versión más exigente, que durante la inicialización intenta recuperar el doble de features y macheos que lo normal.
     */
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    /**
     * Inicializador, responsable de la triangulación de los primeros puntos del mapa.
     * Sólo se usa en modo monocular.
     * El inicializador es modular, por eso en lugar de estar incorporado en Tracking se proporciona en un objeto Initializer, a ese efecto.
     */
    // Initalization (only for monocular)
    Initializer* mpInitializer;

    //Local Map
    KeyFrame* mpReferenceKF;

    /** Lista del keyframes locales, parte del mapa local. */
    std::vector<KeyFrame*> mvpLocalKeyFrames;

    /** Mapa local, lista de puntos 3D.*/
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    /**Sistema.
     * Única instancia.
     */
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    /** Mapa general.*/
    //Map
    Map* mpMap;

    //Calibration matrix

    /** Matriz intrínseca, de cámara, de calibración K.*/
    cv::Mat mK;

    /** Coeficientes de distorsión.*/
    cv::Mat mDistCoef;


    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    list<MapPoint*> mlpTemporalPoints;


private:
    //bool ReadCameraCalibration(cv::FileStorage fSettings);
    bool _Track_full();
    bool _Track_loc_only();

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
