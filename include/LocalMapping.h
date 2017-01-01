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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"

#include <mutex>


namespace ORB_SLAM2
{

class Tracking;
class LoopClosing;
class Map;

/**
 * LocalMapping tiene una única instancia ejecutando el método Run en su propio thread.
 * Se encarga de mantener el mapa local, agregando keyframes cuando Tracking los crea,
 * y buscando nuevos puntos triangulando desde keyframes vecinos.
 *
 * Tiene dos conjuntos de métodos: uno para computación del mapa local, otro para interacción con el thread.
 * Estos últimos manejan autómatas finitos de interacción, usualmente iniciados desde afuera,
 * como por ejemplo vía RequestReset, RequestStop o RequestFinish.
 *
 * Los métodos de computación son protegidos e invocados desde Run.
 *
 * No hay una entidad formal que represente el "mapa local".
 * El mapa local es un sobconjunto del mapa global, con raíz en el keyframe de referencia,
 * involucrando a los vecinos en el grafo.
 *
 *
 * Métodos públicos para interacción con el thread
 *
 * El método Run tiene un bucle infinito, que se ejecuta en su propio thread.  Las comunicaciones con él son asincrónicas:
 *
 * - LocalMapping::RequestStop solicita que pare, que haga una pausa.  LocalMapping::isStopped se puede consultar para confirmar que paró.
 * - Run entra en un bucle esperando que la señal de reanudamiento.
 * - LocalMapping::Release solicita que reanude luego de una pausa.
 * - LocalMapping::RequestReset reinicializa el objeto limpiando variables.  Se invoca desde Tracking::Reset.  El reseteo es asincrónico, no hay señal que indique que ya ocurrió.
 * - LocalMapping::RequestFinish solicita terminar.  Se puede consultar con LocalMapping::isFinished.  En la práctica no hace nada.
 *
 *
 */
class LocalMapping
{
public:
	/**
	 * Constructor de la única instancia de LocalMapping.
	 * @param mMap Mapa global.
	 * @param bMonocular Señal que indica que el sistema es monocular.
	 */
    LocalMapping(Map* pMap, const float bMonocular);

    /** Registra el LoopCloser, la única instancia en el sistema..*/
    void SetLoopCloser(LoopClosing* pLoopCloser);

    /** Registra el tracker, la única instancia en el sistema.*/
    void SetTracker(Tracking* pTracker);

    /**
     * Bucle principal del thread de mapeo local.
     * Recibe y procesa pedidos de control, como reset, finish, etc.
     * Cuando hay keyframes en la lista de nuevos keyframes, los procesa disparando las tareas principales del mapeo en este orden:
     *
     * -ProcessNewKeyFrame
     * -MapPointCulling
     * -CreateNewMapPoints
     * -SearchInNeighbors
     * -Optimizer::LocalBundleAdjustment
     * -KeyFrameCulling
     *
     * Si hay varios nuevos keyframes en la lista, procesa de a uno por bucle.
     * Algunas de las tareas principales listadas no se ejecutan hasta que se vacía la lista de nuevos keyframes.
     */
    // Main function
    void Run();

    /**
     * Método usado por el tracker para solicitar la inclusión de un nuevo keyframe.
     * Este método agrega el keyframe a la cola LocalMapping.mlNewKeyFrames, que se procesa en un hilo aparte, en LocalMapping.Run por LocalMapping.ProcessNewKeyFrame.
     *
     * Invocado sólo desde Tracking.
     */
    void InsertKeyFrame(KeyFrame* pKF);

    // Thread Synch
    /**
     * Solicitud de pausa.
     * Otros hilos solicitan detener momentáneamente LocalMapping para interactuar con el mapa, generalmente para BA.
     * Se invoca también cuando se pasa a modo solo tracking, sin mapeo.
     * Luego de solicitar la parada, esperan que se cumpla consultando LocalMapping::isStopped.
     * Para reanudar el mapeo, hay que invocar LocalMapping::Release.
     */
    void RequestStop();

    /** Solicitud de reinicialización.  Invocado sólo por Tracking::Reset.*/
    void RequestReset();


    /** Invocado en el bucle principal de Run, para procesar pedidos de parada.  Si hay un pedido de parada, devuelve true.*/
    bool Stop();

    /** Invocado desde otros hilos, limpia el buffer de keyframes nuevos y reanuda el mapeo.*/
    void Release();

    /** Informa el si LocalMapping está parado.*/
    bool isStopped();

    /** Informa si se ha solicitado una parada.*/
    bool stopRequested();

    /** Informa si LocalMapping acepta nuevos keyframes.*/
    bool AcceptKeyFrames();

    /** Establece la aceptación de nuevos keyframes.*/
    void SetAcceptKeyFrames(bool flag);

    /** Establece la señal de no parar, que ignora solicitudes de parada.  Invocado desde Tracking.*/
    bool SetNotStop(bool flag);

    /** Solicita interrupción de BA.  Invocado desde Tracking.*/
    void InterruptBA();

    /** Solicita terminar para cerrar el sistema.*/
    void RequestFinish();

    /** Informa si LocalMapping ha terminado.*/
    bool isFinished();

    /** Informa la cantidad de keyframes en la cola para agregarse al mapa.*/
    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

protected:

    /**
     * Consulta la lista de nuvos keyframes mlNewKeyFrames.
     * @returns true si hay keyframes en la lista.
     */
    bool CheckNewKeyFrames();

    /**
     * Actualiza LocalMapping.mpCurrentKeyFrame, y quita el nuevo keyframe de la lista LocalMapping.mlNewKeyFrames.
     * Para cada punto 3D visualizado computa BoW y recomputa descriptores, normal y profundidad.
     *
     * Invocado sólo desde LocalMapping.Run.
     */
    void ProcessNewKeyFrame();

    /**
     * El ciclo de mapeo local invoca periódicamente este método, que busca macheos candidatos para su triangulación y agregado al mapa.
     * La búsqueda se hace triangulando puntos del keyframe actual y todos sus vecinos en el grafo.
     * Invoca a SearchForTriangulation para obtener los pares macheados.
     * Luego evalúa el paralaje para descartar el punto.
     * Finalmente triangula son SVD y lo agrega al mapa.
     * Realiza esta operación para el keyframe actual, comparado con cada uno de sus vecinos en el grafo.
     */
    void CreateNewMapPoints();

    /**
     * Elimina por varios motivos puntos recién agregados.
     * Los elimina si el punto es malo, o muy poco observado.
     * Se invoca desde el bucle principal de LocalMapping::Run.
     */
    void MapPointCulling();


    /**
     * Recorre los keyframes vecinos buscando puntos para fusionar.
     *
     * Es el único lugar que invoca ORBmatcher::Fuse
     * Recorre los vecinos de primer y segundo orden.
     * SearchInNeighbors se ejecuta sólo LocalMapping::Run apenas termina de procesar todos los nuevos keyframes.
     *
     */
    void SearchInNeighbors();

    /**
     * Elimina keyframes redundantes, que no agregan información.
     * Busca en el mapa local solamente y elimina los keyframes con más del 90% de sus puntos observados por otros keyframes.
     * Se invoca desde el bucle principal de LocalMapping::Run.
     */
    void KeyFrameCulling();

    /**
     * Computa la matriz fundamental F del keyframe 1 respecto del keyframe 2.
     * @param pKF1 Primer keyframe
     * @param pKF2 Segundo keyframe
     * @returns La matriz fundamental F12.
     *
     * Invocado sólo desde CreateNewMapPoints.
     */
    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    /**
     * Genera una matriz antisimétrica a partir de un vector.
     * Una matriz es antisimétrica cuando su traspuesta es su negativa.
     *
     * @param v Vector 1x3.
     * @returns Matriz antisimétrica de 3x3.
     */
    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    /** Señal de modo monocular.*/
    bool mbMonocular;

    /** Procesa la solicitud de reinicio.*/
    void ResetIfRequested();

    /** Reinicio solicitado.*/
    bool mbResetRequested;
    std::mutex mMutexReset;

    /** Informa si terminó.*/
    bool CheckFinish();

    /** Marca las señales como terminado.*/
    void SetFinish();

    /** Terminación solicitada.*/
    bool mbFinishRequested;

    /** Terminado.*/
    bool mbFinished;
    std::mutex mMutexFinish;

    /** Mapa del mundo.*/
    Map* mpMap;

    /** Cerrador de bucles.*/
    LoopClosing* mpLoopCloser;

    /** Tracker.*/
    Tracking* mpTracker;

    /** Buffer de solicitud de nuevos keyframes.*/
    std::list<KeyFrame*> mlNewKeyFrames;

    /** Keyframe actual, el último keyframe nuevo que se ha procesado.*/
    KeyFrame* mpCurrentKeyFrame;

    /** Lista de nuevos puntos agregados al mapa, que serán revisados por LocalMapping::MapPointCulling*/
    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    /** BA abortado.*/
    bool mbAbortBA;

    /** LocalMapping parado.*/
    bool mbStopped;

    /** Parada solicitada.*/
    bool mbStopRequested;

    /** Señal de no parar.*/
    bool mbNotStop;
    std::mutex mMutexStop;

    /** Señal para aceptar nuevos keyframes.*/
    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
