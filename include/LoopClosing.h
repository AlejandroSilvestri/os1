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

#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "Tracking.h"

#include "KeyFrameDatabase.h"

#include <thread>
#include <mutex>
#include "../Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;


/**
 * Cierre de bucles.
 * Este objeto singleton ejecuta el método LoopClosing::Run en un thread exclusivo.
 *
 * Cada vez que se agrega un keyframe al mapa (LoopClosing::CheckNewKeyFrames), intenta detectar un bucle (LoopClosing::DetectLoop).
 * Si lo detecta, computa su pose (LoopClosing::ComputeSim3), corrige el mapa (LoopClosing::CorrectLoop) usando en la etapa final LoopClosing::SearchAndFuse.
 *
 * El cierre de bucle se realiza según se describe en el paper Scale Drift-Aware Large Scale Monocular SLAM, Strasdat et al, 2010.
 *
 *
 * Terminación del hilo, para cerrar la aplicación:
 * 1. Un hilo externo invoca LoopClosing::RequestFinish, que marca mbFinishedRequested
 * 2. En el bucle principal de LoopClosing::Run, LoopClosing::CheckFinish informa que se ha solicitado la finalización y sale del bucle
 * 3. Antes de terminar el hilo, LoopClosing::Run invoca LoopClosing::SetFinish(), que marca LoopClosing::mbFinished.
 * 4. LoopClosing::isFinished() informa esa marca a otros hilos que pregunten.
 *
 * Reset:
 * Ante una mala inicialziación, Tracking::Reset invoca LoopClosing::RequestReset, que bloquea el hilo de Tracking hasta que concluye la reinicialización de LoopClosing.
 *
 */
class LoopClosing
{
public:

    /**
     * Par de entero y conjunto de keyframes.
     */
	typedef pair<set<KeyFrame*>,int> ConsistentGroup;

	/**
	 * Mapa de keyframe a pose sim3.
	 *
	 * El mapa se ordena por el puntero a keyframe.  Se provee un allocator propio de Eigen.
	 *
	 */
    typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;

public:

    /**
     * Único constructor invocado por el constructor de System para crear el singleton.
     */
    LoopClosing(Map* pMap, KeyFrameDatabase* pDB, ORBVocabulary* pVoc);//,const bool bFixScale);

    /**
     * Registra el tracker del sistema.
     * @param pTracker puntero al tracker singleton del sistema.
     *
     * Invocado sólo por System durante su construcción.
     *
     * El puntero al tracker no se usa.
     */
    void SetTracker(Tracking* pTracker);

    /**
     * Registra el mapeador local en LoopClosing::mpLocalMapper.
     */
    void SetLocalMapper(LocalMapping* pLocalMapper);

    /**
     * Función que se ejecuta en el hilo dedicado al cierre de bucles, y alberga el bucle principal de este proceso.
     */
    // Main function
    void Run();

    /**
     * Agrega nuevos keyframes a la cola de proceso para buscar bucles.
     *
     * @param pKF nuevo keyframe a procesar
     */
    void InsertKeyFrame(KeyFrame *pKF);

    /**
     * Solicita reinicializar el proceso.
     *
     * Invocado por Tracking cuando falla la inicialización del mapa.
     */
    void RequestReset();

    /**
     * Bundle adjustment sobre todo el mapa luego de cerrar el bucle.
     *
     * Invocado sólo desde LoopClosing::CorrectLoop
     */
    // This function will run in a separate thread
    void RunGlobalBundleAdjustment(unsigned long nLoopKF);

    /**
     * @returns LoopClosing::mbRunningGBA
     *
     * Invocado sólo por LoopClosing::CorrectLoop y System::Shutdown.
     */
    bool isRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }

    /**
     * @returns LoopClosing::mbFinishedGBA
     *
     * Sólo invocado por LoopClosing::CorrectLoop
     */
    bool isFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }   

    /**
     * Otro hilo solicita terminar LoopClosing.
     */
    void RequestFinish();

    /**
     * Informa a otros hilos que LoopClosing ha terminado.
     */
    bool isFinished();

protected:

    /**
     * Indica si hay keyframes en la cola esperando a ser procesados.
     *
     * @returns true si hay keyframes en la cola.
     */
    bool CheckNewKeyFrames();

    /**
     * Procesa los keyframes de la cola, buscando bucles.
     *
     * Produce una lista LoopClosing::mvpEnoughConsistentCandidates.
     */
    bool DetectLoop();

    /**
     * Procesa la lista de candidatos intentando corregir su pose.
     * Si el encastre no es perfecto, los candidatos se descartan.
     * Los encastres existosos se utilizan luego para corregir el bucle.
     */
    bool ComputeSim3();

    /**
     * Proyecta los puntos observados en la vecindad del keyframe del bucle,
     * sobre el keyframe actual y vecinos usando las poses corregidas.
     * Fusiona duplicados.
     *
     * @param CorrectedPosesMap Mapa de poses de los keyframes del mapa de covisibilidad del keyframe actual, que cierra el bucle.
     *
     * Invocado sólo desde LoopClosing::CorrectLoop.
     *
     */
    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap);

    /**
     * Corrige el bucle con los encastres elaborados por LoopClosing::ComputeSim3.
     *
     * Luego de cerrar el bucle ejecuta un BA global LoopClosing::RunGlobalBundleAdjustment.
     *
     * Pasos de la corrección de bucle:
     *
     * 1- Reposicionar el mapa de covisibilidad (puntos y keyframes) del keyframe actual (ya reposicionado para cerrar el bucle).
     * 2- Fusionar puntos coincidentes con el otro extremo del bucle.
     * 3- Replantear el mapa de covisibilidad luego de la fusión, incorporando el otro extremo del bucle.
     * 4- Optimizer::OptimizeEssentialGraph corrige el bucle desparramando el error .
     */
    void CorrectLoop();

    /**
     * Reinicializa el objeto si hay una solicitud pendiente para hacerlo.
     * Invocado en cada iteración del bucle principal de LoopClosing::Run.
     */
    void ResetIfRequested();

    /**
     * Marca de solicitud de reinicialización.
     * Sólo el tracker solicita la reinicialización del singleton LoopClosing, cuando falló la inicialización del mapa.
     */
    bool mbResetRequested;

    /**
     * Mutex para acceder a LoopClosing::mbResetRequested
     */
    std::mutex mMutexReset;

    /**
     * @returns LoopClosing::mbFinishRequested
     * En el bucle principal de LoopClosing::Run,
     * LoopClosing::CheckFinish informa si se ha solicitado la finalización.
     */
    bool CheckFinish();

    /**
     * Hace true LoopClosing::mbFinished, para consultas de otros hilos.
     */
    void SetFinish();

    /**
     * Marca que se ha solicitado la terminación de LoopClosing.
     */
    bool mbFinishRequested;

    /**
     * Marca de proceso terminado.
     */
    bool mbFinished;

    /**
     * mutex para acceder a LoopClosing::mbFinished.
     */
    std::mutex mMutexFinish;

    /** Puntero al mapa singleton. */
    Map* mpMap;

    /** Puntero al tracker singleton.  No se usa.*/
    Tracking* mpTracker;

    /** Puntero a la base de datos de keyframes singleton.*/
    KeyFrameDatabase* mpKeyFrameDB;

    /** Puntero al vocabulario BoW singleton.*/
    ORBVocabulary* mpORBVocabulary;

    /**
     * Puntero al mapeador local singleton.
     * El proceso de cierre de bucle interactía con el mapeador local solamente para solicitar pausa y reanudación,
     * de manera de no interferir en el mapa.
     */
    LocalMapping *mpLocalMapper;

    /**
     * Lista de nuevos keyframes a analizar para buscar bucles.
     *
     * LocalMapping crea keyframes y los agrega a esta lista mediante LoopClosing::InsertKeyFrame.
     * LoopClosing::DetectLoop los procesa en orden de llegada.
     */
    std::list<KeyFrame*> mlpLoopKeyFrameQueue;

    /**
     * mutex para acceso a la cola de nuevos keyframes LoopClosing::mlpLoopKeyFrameQueue
     */
    std::mutex mMutexLoopQueue;

    // Loop detector parameters
    float mnCovisibilityConsistencyTh;

    // Loop detector variables
    /**
     * Keyframe "actual", extraído de la cola de keyframes a procesar.
     * Efímero.
     */
    KeyFrame* mpCurrentKF;

    /**
     * Keyframe que machea con LoopClosing::mpCurrentKF.
     * LoopClosing::ComputeSim3 encuentra el keyframe y lo registra en esta variable.
     * Efímero.
     */
    KeyFrame* mpMatchedKF;

    /**
     * Vector de keyframes y su grupo de pertenencia.
     * Efímero.
     */
    std::vector<ConsistentGroup> mvConsistentGroups;

    /**
     * Lista de keyframes candidatos a cerrar un bucle.
     * Vector efímero.
     */
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;

    /**
     * Conjunto de keyframes conectados por el grafo de covisibilidad.
     * Se usa en un cierre de bucle, para corregir sus poses y sus grafos.
     * Vector efímero utilizado solamente en LoopClosing::CorrectLoop.
     */
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;

    /**
     * Conjunto de puntos macheados para el bucle, en LoopClosing::ComputeSim3.
     * LoopClosing::CorrectLoop lo utiliza para cerrar el bucle.
     * Es un vector efímero.
     */
    std::vector<MapPoint*> mvpCurrentMatchedPoints;

    /**
     * Conjunto de puntos observados por el keyframe actual y sus vecinos, a considerar y fusionar en el cierre de bucle.
     *
     * Se define completamten en LoopClosing::ComputeSim3, y se utiliza en LoopClosing::SearchAndFuse.
     * Es un vector efímero.
     */
    std::vector<MapPoint*> mvpLoopMapPoints;

    /**
     * Pose de la cámara respecto del mundo, propuesta para el keyframe actual al cerrar el bucle, por LoopClosing::Computesim3,
     * que la utiliza para buscar más macheos con ORBmatcher.SearchByProjection.
     */
    cv::Mat mScw;

    /**
     * Pose de la cámara respecto del mundo en formato g2o, de donde se obtiene LoopClosing::mScw.
     */
    g2o::Sim3 mg2oScw;

    long unsigned int mLastLoopKFid;

    // Variables related to Global Bundle Adjustment
    /**
     * Marca que indica que se está ejecutando un Global Bundle Adjustement.
     * Se marca al final de Loopclosing::CorrectLoop y se pone en false al final de Loopclosing::RunGlobalBundleAdjustment.
     * Se informa mediante Loopclosing::isRunningGBA.
     */
    bool mbRunningGBA;

    /**
     * Marca exactamente contraria a mbRunningGBA.
     * No agrega información.
     */
    bool mbFinishedGBA;

    /**
     * Marca para solicitar abortar el Global Bundle Adjustment, porque se requiere iniciar otro con mejores datos.
     */
    bool mbStopGBA;

    /**
     * mutex para acceder a mbRunningGBA y a mbFinishedGBA.
     * LoopClosing::RunGlobalBundleAdjustment detiene las consultas isFinishedGBA e isRunningGBA mientras propaga el resultado
     * del GBA a los nuevos keyframes que pudieran haber surgido durante su cómputo.
     *
     * Se asume que estas consultas "is" las realiza el mapeador local.
     */
    std::mutex mMutexGBA;

    /**
     * Hilo para el Global Bundle Adjustment.
     */
    std::thread* mpThreadGBA;

    /**
     * Siempre false en monocular.
     */
    // Fix scale in the stereo/RGB-D case
    //bool mbFixScale;
};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
