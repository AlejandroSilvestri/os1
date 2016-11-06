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
	// Para que se vea desde main y viewer
	//mutex mutexVideoEntrada;


	// Input sensor
	/** Tipo de sensor, modo de operación: mono, estéreo o mapa de profundidad.*/
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
    };

public:

    /**
     * Constructor.
     *
     * @param strVocFile Nombre del archivo con el vocabulario BoW, para abrir.
     * @param strSettingsFile Nombre del archivo de configuración, para abrir.
     * @param sensor Tipo de sensor.  MONOCULAR para mono slam.
     * @param bUseViewer Señal para activar el visor.
     *
     */
    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true, cv::VideoCapture* = NULL);

    /** Constructor alternativo, no utilizado en orb-slam2, que permite indicar un mapa, un visor y un vocabulario preexistentes.*/
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

    /**
     * Procesa una imagen de la cámara.
     * Es el método central, invocado repetidamente en el bucle principal de tracking.
     *
     * @param im Imagen de cámara a procesar.
     * @param timestamp Marca temporal, sólo para registro, orb-slam2 no la utiliza.
     */
    // Proccess the given monocular frame
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

    /** Pasa a modo de sólo localización, sin mapeo (sólo tracking).*/
    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode();

    /** Pasa a modo de localización y mapeo.*/
    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();

    /** Reinicia el sistema.  Limpia el mapa.*/
    // Reset the system (clear map)
    void Reset();


    /**
     * Solicita el cierre del sistema.
     * Solicita terminar a todos los hilos, y espera hasta que terminen.
     * Los datos permanecen en memoria.  Esta función se debe invocar antesd e guardar la trayectoria.
     */
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

    /** Devuelve el mapa.  @returns Mapa del mundo.*/
    Map* GetMap();


    // TODO: Save/Load functions
    // LoadMap(const string &filename);

    /** Informa el estado.*/
    //	enum eTrackingState GetStatus();
    int GetStatus();

    /** Establece el estado.*/
    void SetStatus(int status);

    /** Devuelve un puntero al objeto de tracking.*/
    Tracking* GetTracker() {return mpTracker;}


    /* Agregados */

    /** Visor, maneja el visor del mapa y el de cámara.   Usa Pangolin.  Lo hice público para que main pueda interactuar con él a través de System.*/
    // The viewer draws the map and the current camera pose. It uses Pangolin.
    Viewer* mpViewer;

    /** Video de entrada.*/
    cv::VideoCapture* video;

    /** Imagen de entrada.*/
    cv::Mat imagenEntrada;

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

    /** Objecto de tracking.*/
    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

    /** Objeto de mapeo.*/
    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

    /** Objeto de cierre de bucles.*/
    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;

    /** Visor de cámara, que muestra la imagen con los puntos detectados sobre ella.*/
    FrameDrawer* mpFrameDrawer;

    /** Visor del mapa que muestra los puntos 3D y la pose de la cámara.   Usa Pangolin.*/
    MapDrawer* mpMapDrawer;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    /** Hilo de mapeo.*/
    std::thread* mptLocalMapping;
    /** Hilo de cierre de bucle.*/
    std::thread* mptLoopClosing;
    /** Hilo de visor.*/
    std::thread* mptViewer;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;

    // Change mode flags
    std::mutex mMutexMode;
    /** Señal solicitud de modo only tracking, sin mapeo.*/
    bool mbActivateLocalizationMode;
    /** Señal solicitud de modo tracking&mapping.*/
    bool mbDeactivateLocalizationMode;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
