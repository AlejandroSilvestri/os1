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


#ifndef VIEWER_H
#define VIEWER_H

/*
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"
*/
#include <opencv2/videoio.hpp>

#include <string>
#include <mutex>

namespace ORB_SLAM2
{

class Tracking;
class FrameDrawer;
class MapDrawer;
class System;

class Viewer
{
public:
    Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking, const std::string &strSettingPath, cv::VideoCapture* = NULL);	// Agregé el últinmo argumento

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();

    /* Agregados */

    /** Flujo de entrada de video.  NULL si no proviene de un archivo.  Usado para manipular el tiempo con un trackbar.  Definido en el constructor.*/
    cv::VideoCapture* video;

    /**
     * Posición del trackbar de tiempo de entrada, expresado en cuadros.
     * Eco de la posición del trackbar.
     * Sólo se usa cuando la entrada es archivo de video.
     */
    int tiempo = 0;

    /**
     * Duración del archivo de video de entrada, expresado en cuadros.
     * 0 si no la entrada no es de un archivo.
     * Definido en el constructor de viewer.
     */
    int duracion;

    /**
     * Señal para main, indicando que el usuario cambió el tiempo con el trackbar.
     * Viewer.Run la levanta, y main la baja.
     */
    bool tiempoAlterado = false;

    /**
     * Sentido de avance del tiempo. false para adelante, true para reversa.
     */
    bool tiempoReversa = false;

    /** Control de pausa del video.  Es leído desde el bucle principal de main.*/
    bool videoPausado = false;

    /** Señal que alterna mostrando la imagen de entrada como viene o antidistorsionada.*/
    bool mostrarEntradaAntidistorsionada = true;

    /** Modo automatico que pasa a resversa cuando se pierde.  Controlado por el usuario.*/
    bool modoAutomatico = false;

    /** Variable interna que indica el sentido del video cuando actúa la corrección automática.  false es normal para adelante, perdido reversa.  true es lo contrario.*/
    bool sentidoModoAutomatico = false;



private:

    bool Stop();

    System* mpSystem;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    Tracking* mpTracker;

    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;

};

}


#endif // VIEWER_H
	

