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

#ifndef MAP_H
#define MAP_H

//#include "MapPoint.h"
//#include "KeyFrame.h"
//#include "System.h"
#include "ORBVocabulary.h"
#include <set>
#include <mutex>
#include <boost/serialization/access.hpp>
#include "Serializer.h"



namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;
class ORBextractor;
class KeyFrameDatabase;

/**
 * Mapa del mundo, de puntos y keyframes.
 * El mapa del mundo consiste en un conjunto de puntos 3D, de puntos 3D de referencia, y de keyframes que los observan.
 * ORB-SLAM utiliza un único mapa que se crea en main.cc, se pasa a los constructores de los 3 threads (tracking, loopClosing, localMapping),
 * al constructor del MapPublisher y al framePublisher.
 * El mapa tiene un flag mbMapUpdates, protegido, que indica si el mapa está actualizado o, por el contrario, hay algún BA en proceso.
 * localMapping y loopClosing son los dos threads que llevan a cabo BA, y al terminar corrigen el flag con SetFlagAfterBA().
 * El flag puede estar en false por períodos muy breves en otras actualizaciones, como por ejemplo al agregar o eliminar un punto.
 */
class Map
{
public:
	/** Constructor, que inicializa propiedades.  El mapa comienza "desactualizado" con mbMapUpdated==false. */
	Map();

	/** Agrega un nuevo keyframe al mapa.
	 *
	 * Registra el nuevo keyframe en el vector de keyframes del mapa,
	 * y actualiza el registro mnMaxKFid del id del último keyframe.
	 *
	 * Se invoca sólo desde Tracking::CreateInitialMapMonocular y LocalMapping::ProcessNewKeyFrame.
	 */
    void AddKeyFrame(KeyFrame* pKF);

    /** Agrega un mapa 3D al mapa.*/
    void AddMapPoint(MapPoint* pMP);

    /**
     * Elimina un determinado punto del mapa.
     *
     * Quita el punto del contenedor mspMapPoints.
     * Debería luego eliminar el objeto punto, pero no lo hace para evitar conflictos de concurrencia.
     * Esta función es final, delega la responsabilidad de la coherencia de datos a la función que la invoca.
     *
     *
     * @param pMP Punto del mapa a eliminar.
     *
     * Ejecuta mspMapPoints.erase(pMP)
     *
     * Se invoca sólo desde MapPoint::Replace y MapPoint::SetBadFlag.
     */
    void EraseMapPoint(MapPoint* pMP);

    /** Elimina un determinado keyframe del mapa.*/
    void EraseKeyFrame(KeyFrame* pKF);

    /** Agrega un punto al conjunto de puntos de referencia.
     * Esta función se invoca exclusivamente desde Tracing::UpdateReference(), para registrar aquí los puntos del mapa local,
     * solamente para su visualización.
     */
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);

    /** Devuelve un vector de keyframes con todos los keyframes del mapa.*/
    std::vector<KeyFrame*> GetAllKeyFrames();

    /** Devuelve un vector de puntos, con todos los puntos del mapa.*/
    std::vector<MapPoint*> GetAllMapPoints();

    /**
     * Devuelve un vector de puntos a graficar.
     * Invocado sólo por MapDrawer::DrawMapPoints, estos puntos se grafican en rojo, representando el mapa local.
     */
    std::vector<MapPoint*> GetReferenceMapPoints();

    /** Informa la cantidad de puntos en el mapa.*/
    long unsigned int MapPointsInMap();

    /** Informa la cantidad de keyframes en el mapa.*/
    long unsigned  KeyFramesInMap();

    /** Informa el id del último keyframe agregado.*/
    long unsigned int GetMaxKFid();

    /**
     * Reinicia el mapa.
     * Elimina puntos y keyframes del mapa (es decir, del sistema), y limpia los vectores de puntos y keyframes.
     */
    void clear();


    /**
     * Indica si el punto o el keyframe está en el mapa
     * Agregado por mí.
     */
    bool isInMap(KeyFrame *KF);
    bool isInMap(MapPoint *MP);

	/**
	 * Registro de los keyframes iniciales.
	 * El único keyframe en este vector lo agrega Tracking::CreateInitialMapMonocular.  El el keyframe inicial.
	 * Es iterado por LoopClosing::RunGlobalBundleAdjustment.
	 */
    vector<KeyFrame*> mvpKeyFrameOrigins;

    /**
     * Mutex para actualización del mapa.
     *
     * Todos los procesos de alto nivel que devienen en modificaciones al mapa usan este mutex.
     * - Tracking::Track se ejecuta completo con este mutex.
     * - LoopClosing::CorrectLoop esa este mutex cuando detectó el bucle y está por proceder a la corrección del mapa
     * - Optimizer::LocalBundleAdjustment usa este mutex.
     * - ...
     *
     */
    std::mutex mMutexMapUpdate;

    /** Mutex para agregado de nuevos puntos al mapa.*/
    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;



    /** Puntos lejanos.*/
    std::set<MapPoint*> puntosLejanos;

protected:
    /** Puntos del mapa.*/
    std::set<MapPoint*> mspMapPoints;

    /** KeyFrames del mapa.*/
    std::set<KeyFrame*> mspKeyFrames;

    /**
     * Puntos de referencia en el mapa.
     * Son los puntos del mapa local, para graficar en rojo.
     * Tracking::UpdateReference lo genera, y MapDrawer::DrawMapPoints lo consume.
     * Por cada cuadro que se procesa, se actualiza el mapa local y se copia este vector al mapa.
     * Es un vector efímero.
     */
    std::vector<MapPoint*> mvpReferenceMapPoints;

    /**
     * Id del último keyframe agregado.
     *
     * Se usa para evitar agregar un keyframe al mapa dos veces.
     *
     * Difiere de KeyFrame::nextID en que pueden existir keyframes que todavía no están en el mapa.
     *
     * Invocado desde Map::AddKeyFrame, entre otros.
     */
    long unsigned int mnMaxKFid;

    /** Mutext del mapa.*/
    std::mutex mMutexMap;

	/** Serialización agregada para guardar y cargar mapas.*/
	friend class boost::serialization::access;
	friend class Serializer;
	template<class Archivo> void serialize(Archivo&, const unsigned int);
	// Fin del agregado para serialización
};

} //namespace ORB_SLAM

#endif // MAP_H
