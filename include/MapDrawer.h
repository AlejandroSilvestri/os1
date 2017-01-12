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

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include"Map.h"
#include"MapPoint.h"
#include"KeyFrame.h"
#include<pangolin/pangolin.h>

#include<mutex>

namespace ORB_SLAM2
{

/**
 * Esta clase contiene los métodos para dibujar en pangolin.
 * Dibuja la cámara, los keyframes y los puntos 3D del mapa.
 * No los dibuja directamente sobre Pangolin, sino que usa glew.
 * Viewer::Run invoca estos métodos, y vuelca lo dibujado a Pangolin.
 *
 */
class MapDrawer
{
public:
    /**
     * Constructor que toma sus parámetros directamente desde el archivo de configuración.
     *
     * @param pMap Mapa del mundo.
     * @param strSettingPath Ruta del archivo de configuración.
     *
     * La configuración se limita a aspectos gráficos: anchos de trazos, tamaños de cámara y keyframes, expresados en píxeles, para su visualización.
     * Los parámetros son:
     *
     * - Viewer.KeyFrameSize: tamaño de las marcas de keyframes
     * - Viewer.KeyFrameLineWidth: ancho de las líneas al dibujar keyframes
     * - Viewer.GraphLineWidth: ancho de las líneas del grafo de keyframes
     * - Viewer.PointSize: tamaño de los puntos del mapa
     * - Viewer.CameraSize: tamaño del dibujo de la cámara
     * - Viewer.CameraLineWidth: ancho de línea de la cámara
     *
     *
     * Invocado sólo desde el constructor de System.
     */
	MapDrawer(Map* pMap, const string &strSettingPath);

    /** Mapa del mundo.*/
    Map* mpMap;

    /**
     * Dibuja en pantalla todos los puntos 3D del mapa.
     * Dibua todos en color negro, y luego en rojo los de referencia (los visualizados por la cámara).
     * Invocado en cada iteración de Viewer::Run.
     *
     * @param color true para ver los puntos del mapa a color.
     *
     * Invocado sólo desde Viewer::Run, en cada iteración.
     */
    void DrawMapPoints(bool color = false);

    /**
     * Dibuja los keyframes y el grafo en pantalla.
     *
     * @param bDrawKF true para dibujar keyframes.
     * @param bDrawGraph true para dibujar el grafo.
     *
     * Invocado sólo desde Viewer::Run, en cada iteración.
     */
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);

    /**
     * Dibuja la cámara en la pose dada.
     *
     * @param Twc Pose de la cámara.
     *
     * Invocado sólo desde Viewer::Run, en cada iteración.
     */
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);

    /**
     * Informa la pose de la cámara al objeto MapDrawer, que la copia a MapDrawer::mCameraPose.
     *
     * @param Tcw Pose en el mundo, matriz de rototraslación en coordenadas homogéneas.
     *
     * Invocado sólo desde Tracking::Track y Tracking::CreateInitialMapMonocular.
     */
    void SetCurrentCameraPose(const cv::Mat &Tcw);

    /**
     * Informa Twc, la pose actual del mundo respecto de la cámara, en formato pangolin.
     *
     * Calcula la pose actual invertida en el formato paonglin::OpenGlMatrix
     *
     * @param M Matriz de pose Twc convertida a matriz opengl.
     *
     * La pose de la cámara se encuentra convenientemente actualizada en la propiedad mCameraPose.
     *
     * Invocado sólo desde Viewer::Run.
     */
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

    /**
     * Similar a MapDrawer::GetCurrentOpenGLCameraMAtriz, informa la pose del mundo respecto del visor, en formato pangolin.
     * En este caso el visor p (de pájaro) no coincide con la cámara.  Se proporciona un argumento de rototraslación entre el visor y la cámara.
     *
     * @param T Rototraslación, pose de la cámara respecto del visor.
     * @param M Matriz resultado, pose del mundo respecto del visor, en formato pangolin.
     *
     * Invocado sólo desde Viewer::Run.
     *
     */
    void GetCurrentOpenGLCameraMatrixModified(cv::Mat &T, pangolin::OpenGlMatrix &M);

private:

    /** Tamaño de los keyframe a dibujar, definido en el archivo de configuración.*/
    float mKeyFrameSize;

    /** Ancho del trazo para dibujar keyframes, definido en el archivo de configuración.*/
    float mKeyFrameLineWidth;

    /** Ancho del trazo del grafo, definido en el archivo de configuración.*/
    float mGraphLineWidth;

    /** Tamaño de los puntos del mapa, definido en el archivo de configuración.*/
    float mPointSize;

    /** Tamaño de la cámara a dibujar, definido en el archivo de configuración.*/
    float mCameraSize;

    /** Ancho del trazo para dibujar la cámara, definido en el archivo de configuración.*/
    float mCameraLineWidth;

    /**
     * Pose de la cámara.
     *
     * Permanentemente actualizada con MapDrawer::SetCurrentCameraPose invocado por Viewer::Run.
     */
    cv::Mat mCameraPose;

    /**
     * mutex que evita que cambios en la pose de la cámara cuando se le están extrayendo los vectores rotación y traslación.
     */
    std::mutex mMutexCamera;
};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
