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
//#include <opencv2/videoio.hpp>

#include <string>
#include <mutex>

namespace ORB_SLAM2
{

class Tracking;
class FrameDrawer;
class MapDrawer;
class System;
//class Video;

/**
 * Viewer representa la interfaz de usuario.
 *
 * Se compone de dos partes, implementadas en dos clases: FrameDrawer y MapDrawer, ambos singleton.
 *
 * FrameDrawer se encarga de generar la imagen procesada con los puntos del mapa y una barra de estado.  Viewer la muestra con imshow.
 *
 * MapDrawer se encarga de cargar en Pangolin los puntos 3D del mundo, las poses de los keyframes y de la cámara.
 *
 * Estos dos objetos preparan lo que se muestra, la acción de llevarlo a a pantalla ocurre en Viewer::Run mediante:
 * - imshow
 * - pangolin::FinishFrame
 *
 * Pausa:
 * El proceso del visor se puede pausar desde otro hilo:
 * - Viewer::RequestStop para solicitar la pausa
 * - Viewer::isStopped para verificar si está pausado
 * - Viewer::Release para quitar la pausa y reanudar el proceso
 *
 * Durante la pausa el visor no se actualiza.
 *
 *
 * Terminación:
 * El proceso se termina definitivamente para cerrar la aplicación con:
 * - Viewer::RequestFinish para solicitar la terminación
 * - Viewer::isFinished para confirmar que ha terminado
 */
class Viewer
{
public:
	/**
	 * Constructor único del singleton Viewer.
	 * Se ocupa de la interfaz de usuario: presentación en pantalla y de procesar teclas y botones.
	 * Es el consumidor único de los singletons de FrameDrawer y MapDrawer.
	 */
    Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking, const std::string &strSettingPath);	// Agregé el últinmo argumento

    /**
     * Método principal de Viewer, que se ejecuta en un hilo propio.
     * Run entra en un bucle sin fin, del que sale solamente cuando se le solicita terminar con RequestFinish.
     * En este bucle se procesan teclazos y botones de Pangolin, y se presentan en pantalla las imágenes y el mapa.
     */
    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    /**
     * Solicita terminar para cerrar la aplicación.
     * Invocado por otro hilo.
     */
    void RequestFinish();

    /**
     * Solicita pausar.
     * Invocado asincrónicamente por otro hilo, que deberá consultar isStopped para confirmar que Viewer se detuvo.
     * Viewer pausa en el bucle de Run, y reanuda luego de que otro hilo invoque Release.
     *
     */
    void RequestStop();

    /**
     * Informa si Viewer ha terminado, paso previo para cerrar la aplicación.
     *
     * @returns Viewer::mbFinished: true si se está en proceso de terminación.
     */
    bool isFinished();

    /**
     * Devuelve true si Viewer está pausado.
     * Se consulta repetidamente luego de haber solicitado la parada con RequesStop.
     *
     * @returns Viewer::mbStopped, que es true si Viewer::Run ya está en el bucle de pausa.
     */
    bool isStopped();

    /**
     * Solicita salir de una pausa y resumir la operación.
     * Invocado por otro hilo, luego de haber solictado para con RequestStop.
     */
    void Release();

    /* Agregados */

    /**
     * Recibe la duración del video de archivo, y dimensiona el trackbar de tiempo.
     * Si el trackbar no se creó, se limita a actualizar la propiedad duracion.
     *
     * @param T Duración en cantidad de cuadros.  0 para desactivar el trackbar, usualmente para flujos sin barra de tiempo.
     */
    void setDuracion(int T=0);

    /** Flujo de entrada de video.  NULL si no proviene de un archivo.  Usado para manipular el tiempo con un trackbar.  Definido en el constructor.*/
    //cv::VideoCapture* video;

    /** Singleton Video, fuente de entrada de video a procesar. */
    //Video *video;

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
    int duracion = 0;

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
    bool mostrarEntradaAntidistorsionada = false;

    /**
     * Control para mostrar u ocultar la imagen de entrada.
     */
    bool mostrarEntrada = false;

    /** Modo automatico que pasa a resversa cuando se pierde.  Controlado por el usuario.*/
    bool modoAutomatico = false;

    /** Variable interna que indica el sentido del video cuando actúa la corrección automática.  false es normal para adelante, perdido reversa.  true es lo contrario.*/
    bool sentidoModoAutomatico = false;

    /** Señal para que main cargue en mapa en el thread de System y Tracking.*/
    bool cargarMapa = false;

    /** Señal para que main guarde el mapa en el thread de System y Tracking.*/
    bool guardarMapa = false;

    /** Señal para que main abra un archivo de video.*/
    bool abrirVideo = false;

    /**
     * Período mínimo entre cuadros, en segundos.
     * El bucle main se asegura de demorar como mínimo este tiempo entre dos cuadros.
     * El usuario ajusta este valor.
     */
    float T = 0.033;	// 30 fps

private:

    /**
     * Método privado invocado exclusivamente en el bucle principal de Viewer::Run,
     * para ejecutar una pausa si fue solicitada.
     *
     * @returns true si el hilo debe detenerse, false si no.
     *
     * El bucle principal de Run invoca Stop y si devuelve true detiene el hilo mientras isStopped() sera true.
     *
     */
    bool Stop();

    /** Sistema.*/
    System* mpSystem;

    /** Única instancia de FrameDrawer, que se invoca sólo desde Viewer.*/
    FrameDrawer* mpFrameDrawer;

    /** Única instancia de MapDrawer, que se invoca sólo desde Viewer.*/
    MapDrawer* mpMapDrawer;

    /** Tracker.*/
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

    /**
     * Señal de que Viewer está parado.
     * Pero no se lee en ninguna parte del código: no se usa.
     */
    bool mbStopped;

    /**
     * Señal de que se solicitó la parada del Viewer.
     * Cuando pueda parará, y marcará mbStopped
     *
     */
    bool mbStopRequested;
    std::mutex mMutexStop;

    /**
     * Flag que indica si el trackbar de tiempo ya se creó y está disponible.
     * Es usado solamente por la inicialización y por setDuracion, para evitar que este método intente cambiar el tamaño de la barra cuando no se creoó todavía.
     */
    bool iniTrackbar = false;
    std::mutex mutexIniTrackbar;

};

}


#endif // VIEWER_H
	

