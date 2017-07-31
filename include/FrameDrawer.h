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

#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

//#include "Tracking.h"
//#include "MapPoint.h"
//#include "Map.h"

#include <opencv2/core.hpp>
//#include <opencv2/features2d.hpp>
#include <mutex>

using namespace std;
namespace ORB_SLAM2
{

class Tracking;
//class Viewer;
class MapPoint;
class Map;

/**
 * Clase de instancia única que se ocupa de mostrar la imagen de la cámara, con marcas verdes sobre los puntos reconocidos.
 * Utiliza imshow de opencv.  Al pie de la imagen describe el estado del sistema.
 * FrameDrawer se accede desde dos hilos paralelos asincrónicos:
 * - Tracking::Track invoca el método FrameDrawer::Update avisando que hay un nuevo Frame para mostrar.
 * - Viewer::Run invoca el método FrameDrawer::DrawFrame cuando es momento de actualizar la pantalla.
 */
class FrameDrawer
{
public:
	/**
	 * Constructor único.
	 *
	 * @param pMap puntero al mapa, de donde tomará mappoints y keyframes para graficarlos.
	 */
    FrameDrawer(Map* pMap);
    //FrameDrawer();

    /**
     * Invocado sólo por Tracking::Track avisando que hay un nuevo Frame para mostrar.
     *
     * @param pTracker puntero al único Tracker.
     *
     * El parámetro es siempre el mismo, se podría convertir en un parámetro del constructor en lugar de estar acá.
     */
    // Update info from the last processed frame.
    void Update(Tracking *pTracker);

    /**
     * Invocado sólo por Viewer::Run para que envíe la imagen del cuadro actual a la pantalla.
     *
     * Originalmente sin argumentos, uso los argumentos como una vía de enviar datos para mostrar en pantalla o para controlar aspectos gráficos.
     *
     * @param radio Radio del los features a mostrar en pantalla.
     * @param nKF Cantidad de keyframes pendientes de procesamiento en LocalMapping, dato para mostrar en pantalla.
     */
    // Draw last processed frame.
    cv::Mat DrawFrame(float radio = 1, int nKF = 0);

    /** Matriz de calibración del último frame, para undistort.*/
    cv::Mat K;

    /** Coeficientes de distorsión para undistort.*/
    cv::Mat distCoef;

    /**
     * Callback, listener del clic de mouse en ventana de opencv.
     *
     * Función estática para que se pueda pasar como puntero a función.
     *
     * @param event Evento de mouse, sólo interesa clic.
     * @param x Coordenada x del píxel donde actúa el mouse.
     * @param y Coordenada y del píxel donde actúa el mouse.
     * @param instancia Puntero al singleton FrameDrawer para que la función estática pueda acceder a sus miembros.
     *
     * Doc: http://docs.opencv.org/3.1.0/d7/dfc/group__highgui.html#ga89e7806b0a616f6f1d502bd8c183ad3e
     * Ejemplo del uso: https://github.com/opencv/opencv/blob/master/samples/cpp/ffilldemo.cpp
     *
     */
    static void onMouse( int event, int x, int y, int, void* instancia);

    /**
     * Copia de la variable de Viewer::Run, para corregir la posición del clic en FrameDrawer::onMouse.
     */
    float factorEscalaImagenParaMostrar;

    /**
     * Marca temporal del Frame
     */
    double timestamp = 0;


protected:

    /** Agrega una barra debajo de la imagen y escribe en ella el estado del sistema.*/
    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

    /** Imagen gris del cuadro actual.*/
    // Info of the frame to be drawn
    cv::Mat mIm;

    /** Cantidad de puntos singulares.*/
    int N;

    /** Puntos singulares del cuadro actual.*/
    vector<cv::KeyPoint> mvCurrentKeys;

    //@{
    /** Vectores booleanos que indican si el punto singular está siendo trackeado.
     * mvbMap indica si el punto singular corresponde a un punto del mapa.
     * mvbVO indica si el punto singular corresponde a un punto no agregado al mapa, para odometría visual.
     */
    vector<bool> mvbMap, mvbVO;
    vector<int> esPuntoLejano;
    //@}

    /**Copia de la señal de sólo tracking (true) o tracking & mapping (false).*/
    bool mbOnlyTracking;

    //@{
    /**
     * Cantidad de macheos con puntos 3D.
     * mnTracked indica la cantidad de macheos entre puntos singulares del cuadro y puntos del mapa.
     * mnTrackedVO indica la cantidad de macheos entre puntos singulares del cuadro y puntos 3D de odometría visual, no agregados al mapa.
     */
    int mnTracked, mnTrackedVO, candidatos;
    //@}

    /** Puntos singulares de inicialización.*/
    vector<cv::KeyPoint> mvIniKeys;

    /** Macheos de inicialización.*/
    vector<int> mvIniMatches;

    /** Copia del estado del autómata finito del sistema.*/
    int mState;

    /**Mapa mundi.*/
    Map* mpMap;

    std::mutex mMutex;

    /** Cantidad de observaciones de cada punto del mapa.*/
    vector<int> observaciones;

    /**
     * Vector de puntos 3D del mapa asociados a los puntos singulares.
     * Este vector tiene la misma longitud que mvKeys y mvKeysUn.
     * Las posiciones sin asociación son NULL.
     * Copiado de mCurrentFrame en FrameDrawer::Update.
     */
    std::vector<MapPoint*> mvpMapPoints;

    /**
     * Coordenadas del frame
     */
    cv::Mat cameraPos, cameraPosDraw;

    /*
     * Auxiliar para mostrar en pantalla
     */
    int nKFPendientes;
};

} //namespace ORB_SLAM

#endif // FRAMEDRAWER_H
