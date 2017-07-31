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
 * Los métodos de computación son protegidos e invocados desde LocalMapping::Run.
 *
 * No hay una entidad formal que represente el "mapa local".
 * El mapa local es un sobconjunto del mapa global, con raíz en el keyframe de referencia,
 * involucrando a los vecinos en el grafo.
 *
 *
 * <h3>Métodos públicos para interacción con el thread</h3>
 *
 * El método Run tiene un bucle infinito, que se ejecuta en su propio thread.  Las comunicaciones con él son asincrónicas:
 *
 * - LocalMapping::RequestStop solicita que pare, que haga una pausa.
 *   + LocalMapping::Run entra en un bucle esperando que la señal de reanudamiento.
 *   + LocalMapping::isStopped se puede consultar para confirmar que paró.
 * - LocalMapping::Release solicita que reanude luego de una pausa.
 * - LocalMapping::RequestReset reinicializa el objeto limpiando variables.
 *   + Se invoca desde Tracking::Reset.  El reseteo es asincrónico, no hay señal que indique que ya ocurrió.
 * - LocalMapping::RequestFinish solicita terminar.  Se puede consultar con LocalMapping::isFinished.  En la práctica no hace nada.
 *
 * \sa Run
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

    /**
     * Registra el LoopCloser, la única instancia en el sistema.
     *
     * Se registra en LocalMapping::mpLoopCloser.
     *
     * Invocado sólo por el constructor de System.
     */
    void SetLoopCloser(LoopClosing* pLoopCloser);

    /** Registra el tracker, la única instancia en el sistema.*/
    void SetTracker(Tracking* pTracker);

    /**
     * Bucle principal del thread de mapeo local.
     * Recibe y procesa pedidos de control, como reset, finish, etc.
     * Cuando hay keyframes en la lista de nuevos keyframes, los procesa disparando las tareas principales del mapeo en este orden:
     *
     * - LocalMapping::ProcessNewKeyFrame
     *   + Inicia el proceso de los keyframes insertados asincŕonicamente vía LocalMapping::InsertKeyFrame
     *   + Computa Bow y algunas propiedades del keyframe
     * - LocalMapping::MapPointCulling
     * - LocalMapping::CreateNewMapPoints
     *
     *   Sólo cuando ya no hay keyframes pendientes de proceso, procede con:
     * - LocalMapping::SearchInNeighbors
     *   + Busca puntos para fusionar, en keyframes vecinos.
     * - Optimizer::LocalBundleAdjustment
     * - LocalMapping::KeyFrameCulling
     *
     * Si hay varios nuevos keyframes en la lista, procesa de a uno por bucle.
     * Cuando termina, duerme 3 segundos.
     */
    // Main function
    void Run();

    /**
     * Método usado por el tracker para solicitar la inclusión de un nuevo keyframe.
     * Este método agrega el keyframe a la cola LocalMapping.mlNewKeyFrames, que se procesa en un hilo aparte,
     * en LocalMapping.Run por LocalMapping.ProcessNewKeyFrame.
     *
     * Invocado sólo desde Tracking.
     */
    void InsertKeyFrame(KeyFrame* pKF);

    // Thread Synch
    /**
     * Solicitud de pausa.
     *
     * Otros hilos solicitan detener momentáneamente LocalMapping para interactuar con el mapa, generalmente para BA.
     *
     * Se invoca también cuando se pasa a modo solo tracking, sin mapeo.
     * Luego de solicitar la parada, esperan que se cumpla consultando LocalMapping::isStopped.
     *
     * Para reanudar el mapeo, hay que invocar LocalMapping::Release.
     *
     * Invocado por varios hilos:
     * - System::TrackMonocular
     * - main, antes de serializar el mapa
     * - LoopClosing::CorrectLoop
     * - LoopClosing::RunGlobalBundleAdjustment
     */
    void RequestStop();

    /**
     * Solicitud de reinicialización.
     *
     * Este método se invoca desde otro hilo, el reseteo de LocalMapping se hace de manera asincrónica.
     *
     * No obstante, el método no termina hasta que confirma la conclusión del reseteo.
     *
     * Levanta el flag mbResetRequested, consultado en el bucle principal de LocalMapping
     * para invocar LocalMapping::ResetIfRequested, que al terminar baja el flag.
     *
     *
     * Invocado sólo por Tracking::Reset.
     */
    void RequestReset();


    /**
     * Invocado en el bucle principal de Run, para procesar pedidos de parada.  Si hay un pedido de parada, devuelve true.
     */
    bool Stop();

    /**
     * Invocado desde otros hilos, limpia el buffer de keyframes nuevos y reanuda el mapeo.
     *
     * Otros hilos pausan LocalMapping vía LocalMapping::RequestStop, y lo reaundan con este método sincrónico.
     *
     * Al terminar este método LocalMapping ya está corriendo nuevamente.
     */
    void Release();

    /** Informa el si LocalMapping está parado.*/
    bool isStopped();

    /** Informa si se ha solicitado una parada.*/
    bool stopRequested();

    /**
     * Informa si LocalMapping acepta nuevos keyframes.
     * Indica que LocalMapping está ocioso.
     * En rigor siempre acepta nuevos keyframes, que van a la cola para ser procesados.
     * Informa el valor de LocalMapping::mbAcceptKeyFrames.
     *
     * Sólo invocado por Tracking::NeedNewKeyFrame.
     */
    bool AcceptKeyFrames();

    /**
     * Establece la aceptación de nuevos keyframes.
     *
     * Este médoto escribe en LoalMapping::mbAcceptKeyFrames.
     *
     * Es invocado dos veces en cada bucle de Run: uno para parar la aceptación mientras el hilo está ocupado,
     * otro para liberarla antes de dormir unos segundos.
     */
    void SetAcceptKeyFrames(bool flag);

    /**
     * Establece la señal de no parar, que ignora solicitudes de parada.
     *
     * @param flag true para activar la señal de no parar, false para desactivarla.
     * @returns Informa si pudo ajustar la señal de no parar.
     *
     * Si ya está parado no deja activar esta señal.
     *
     * Ajusta mbNonStopFlag.
     *
     * Invocado sólo desde Tracking::CreateNewKeyFrame, para activar y luego desactivar la señal.
     *
     *
     */
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

    /**
     * Parámetro controlado por barra de desplazamiento de ventana de cuadro.
     * En este momento usada para controlar el coseno umbral para la triangulación de puntos.
     * El coseno será 0.9 + param/10000
     */
    int param = 1000;

    /**
     * Marca que activa la creación de puntos lejanos.
     *
     * Controlada desde Viewer, por el usuario.
     */
    bool creacionDePuntosLejanosActivada = false;

protected:

    /**
     * Consulta la lista de nuvos keyframes mlNewKeyFrames.
     * @returns true si hay keyframes en la lista.
     */
    bool CheckNewKeyFrames();

    /**
     * Crea un keyframe a partir del cuadro actual LocalMapping::mpCurrentKeyFrame.
     *
     * Computa BoW para todos los puntos singulares del cuadro actual y quita el nuevo keyframe de la lista LocalMapping.mlNewKeyFrames.
     *
     * Agrega el keyframe al mapa, y como observación a cada punto 3D a la vista, para los que recomputa descriptores, normal y profundidad.
     *
     * Invocado sólo desde LocalMapping.Run.
     *
     * \sa Run
     */
    void ProcessNewKeyFrame();

    /**
     * El ciclo de mapeo local invoca periódicamente este método, que busca macheos candidatos para su triangulación y agregado al mapa.
     *
     * La búsqueda se hace triangulando puntos del keyframe actual y todos sus vecinos en el grafo.
     *
     * Invoca a ORBmatcher::SearchForTriangulation para obtener los pares de keypoints macheados.
     *
     * Luego evalúa el paralaje para descartar el punto.
     *
     * Finalmente triangula con SVD y lo agrega al mapa.
     *
     * Realiza esta operación para el keyframe actual, haciendo par con cada uno de sus vecinos en el grafo.
     *
     * Es el único lugar del código que agrega puntos al mapa.  Invocado sólo desde LocalMapping::Run.
     *
     * Descripción de los pasos:
     * - Recorre todos los keyframes vecinos.
     * - Descarta el keyframe vecino si no se distancia al menos un 1% de la profundidad mediana de la escena.
     * - Computa la matriz fundamental para machear pares sobre la línea epipolar, con ORBmatcher::SearchForTriangulation.
     * - Recorre los pares macheados.
     * - Calcula los rayos 3D del par de puntos macheado.
     * - Exige un paralaje de cos<0.9998.  Extrañamente exige que el coseno de ambos rayos no sea negativo (esto puede ser un error).
     * - Triangula los rayos por SVD, obteniendo las coordenadas del nuevo punto.
     * - Si la 4ª coordenada homogénea es cero, se descarta el punto en el infinito.
     * - Se comprueba que el punto esté delante de ambas cámaras.  De lo contrario se descarta.
     * - Que el error de reproyección sobre ambos keyframes sea menor a 5.991 píxel^2.
     * - Que el punto no se encuentre sobre el foco de alguna cámara.
     * - Que los niveles de pirámides del punto en cada cámara sea consistente con las distancias del punto 3D a las cámaras.
     *
     * Luego de superar todas las comprobaciones, se:
     * - agrega al mapa
     * - agrega a las observaciones de los keyframes
     * - computa un descriptor distintivo
     * - calcula la profundidad o rango de visión, la normal, y el cos visual de originación
     * - agrega a la lista de puntos nuevos, que serán revisados más tarde por LocalMapping::MapPointCulling
     *
     * \sa Run
     */
    void CreateNewMapPoints();

    /**
     * Elimina por varios motivos puntos recién agregados.
     *
     * Los elimina si el punto es malo, o muy poco observado.
     *
     * Se invoca desde el bucle principal de LocalMapping::Run.
     * \sa Run
     */
    void MapPointCulling();


    /**
     * Recorre los keyframes vecinos buscando puntos para fusionar.
     *
     * Es el único lugar que invoca ORBmatcher::Fuse.
     *
     * Recorre los vecinos de primer y segundo orden.
     *
     * SearchInNeighbors se ejecuta sólo LocalMapping::Run apenas termina de procesar todos los nuevos keyframes.
     *
     * \sa Run
     */
    void SearchInNeighbors();

    /**
     * Elimina keyframes redundantes, que no agregan información.
     *
     * Busca en el mapa local solamente y elimina los keyframes con más del 90% de sus puntos observados por otros keyframes.
     *
     * Se invoca desde el bucle principal de LocalMapping::Run.
     * \sa Run
     */
    void KeyFrameCulling();

    /**
     * Computa la matriz fundamental F del keyframe 1 respecto del keyframe 2.
     * @param pKF1 Primer keyframe
     * @param pKF2 Segundo keyframe
     * @returns La matriz fundamental F12.
     *
     * Invocado sólo desde LocalMapping::CreateNewMapPoints.
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

    /**
     * Procesa la solicitud de reinicio.
     *
     * Invocado en el bucle principal, si el flag mbResetRequested está levantado, procede al reseteo y luego lo baja.
     *
     * Invocado sólo desde LocalMapping::Run.
     */
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

    /**
     * Cerrador de bucles.
     *
     * Se usa en el bucle principal para enviar nuevos keyframes, luego de procesador por LocalMapping.
     */
    LoopClosing* mpLoopCloser;

    /** Tracker.*/
    Tracking* mpTracker;

    /**
     * Buffer de solicitud de nuevos keyframes.
     *
     * Cola FIFO, los nuevos pedidos se agregan al final con push_back,
     * y se procesa el keyframe obtenido de front.
     */
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

    /** Pausa solicitada.*/
    bool mbStopRequested;

    /** Señal de no parar.*/
    bool mbNotStop;

    /**
     *
     */
    std::mutex mMutexStop;

    /**
     * Señal para aceptar nuevos keyframes.
     * Se escribe con LocalMapping::SetAcceptKeyFrames, y se lee con LocalMapping::AcceptKeyFrames.
     * Leída sólo por Tracking::NeedNewKeyFrame.
     * Escrita en el bucle principal de LocalMapping::Run,
     * de modo que se pone en false al comienzo (indicando que el mapeo no acepta nuevos keyframes),
     * y en true al final, justo antes de dormir por 3 s.
     */
    bool mbAcceptKeyFrames;

    /**
     * Mutex para acceder a LocalMapping::mbAcceptKeyFrames.
     *
     * Evita escritura y lectura concurrete.
     *
     * Da la impresión de que podría quitarse este mutex sin ningún efecto.
     */
    std::mutex mMutexAccept;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
