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


#ifndef SYSTEM_H
#define SYSTEM_H

#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"

namespace ORB_SLAM2
{

/** Visor 3D del sistema, única instancia de Viewer. */
class Viewer;

/** Visor de la imagen procesada, única instancia de FrameDrawer. */
class FrameDrawer;

/** Mapa global de puntos 3D.*/
class Map;

/** Única instancia de Tracking, raíz del sistema que invoca al resto.*/
class Tracking;

/** Única instancia de mapa local.*/
class LocalMapping;

/** Única instancia d*/
class LoopClosing;

/**
 * System se instancia una única vez en main, en la variable SLAM.
 * Sus métodos representan la API del sistema.
 * Luego de la inicialización, el bucle principal de main invoca un método de System para cada cuadro.
 * main controla el origen de imágenes, y lo envía a Track* para su proceso.  Track* es uno de:
 * - TrackMonocular
 * - TrackStereo
 * - TrackRGBD
 *
 * En ese mismo bucle la gui invoca comandos de System, como ActivateLocalizationMode() y DeactivateLocalizationMode().
 * Otros comandos disponibles: Reset, Shutdown, Save*.
 *
 * Las propiedades mp del sistema son punteros a los objetos principales, todos de instancia única, del tipo:
 * - ORBVocabulary
 * - KeyFrameDatabase
 * - Map
 * - FramDrawer
 * - MapDrawer
 *
 * Puntero a los objetos de los hilos, de tipo:
 * - Tracking (está en el mismo hilo que System)
 * - LocalMapping y a su hilo
 * - LoopClosing y a su hilo
 * - Viewer y a su hilo
 *
 * Las propiedades de estado son booleanas:
 * - mbReset
 * - mbActivateLocalizationMode
 * - mbDeactivateLocalizationMode
 *
 * Otras propiedades misceláneas:
 * - mMutexMode
 * - mMutexReset
 * - mSensor: MONOCULAR, STEREO o RGBD
 */
class System
{
public:
    // Input sensor
	/** Tipo de sensor, modo de operación: mono, estéreo o mapa de profundidad.*/
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
    };

public:

    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true);
    System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor=MONOCULAR, Viewer* v=NULL, Map* m=NULL, ORBVocabulary* voc = NULL);

    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    //cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);

    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    //cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp);

    // Proccess the given monocular frame
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode();
    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();

    // Reset the system (clear map)
    void Reset();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();

    // Save camera trajectory in the TUM RGB-D dataset format.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    //void SaveTrajectoryTUM(const string &filename);

    // Save keyframe poses in the TUM RGB-D dataset format.
    // Use this function in the monocular case.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    // Save camera trajectory in the KITTI dataset format.
    // Call first Shutdown()
    // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    //void SaveTrajectoryKITTI(const string &filename);

    Map* GetMap();

    bool SaveMap(const string &filename);

    // TODO: Save/Load functions
    // LoadMap(const string &filename);

    //	enum eTrackingState GetStatus();
    int GetStatus();
    void SetStatus(int status);

    Tracking* GetTracker() {return mpTracker;}



private:

    // Input sensor
	/** Tipo de sensor, modo de operación: mono, estéreo o mapa de profundidad.*/
    eSensor mSensor;

    // ORB vocabulary used for place recognition and feature matching.
    /** Vocabulario ORB BOW.*/
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    /** Base de datos de keyframes, para búsqueda de keyframes por BOW.*/
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    /** Mapa del mundo, con la lista de puntos 3D y de keyframes.*/
    Map* mpMap;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    Viewer* mpViewer;

    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    std::thread* mptViewer;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;

    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
