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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"
#include "Serializer.h"

#include <opencv2/core/core.hpp>
#include <mutex>
#include <boost/serialization/access.hpp>

namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class Frame;


/**
 * Cada instancia representa un mapa 3D en el sistema de referencia del mapa.
 * MapPoint es mucho más que las coordenadas del punto 3D.  Contiene, por ejemplo,  la lista de keyframes que lo observan.
 * Implementa un ABC (alta, baja, consulta) de "observaciones" (keyframes), y otro de descriptores asociados.
 * Incluso elije el mejor descriptor, el más cercano al resto de sus descriptores.
 * Entre otras propiedades, un MapPoint tiene una normal que intenta indicar desde qué lado es observable.
 * También tiene un rango de distancias en el que es observable.  Desde muy cerca o muy lejos los descriptores no machean.
 * Este rango permite reducir el ámbito de macheo para aumentar la performance.
 * El keyframe de referencia es el informado en el constructor, presumiblemente el primer keyframe que observa el punto.
 * Si se elimina el keyframe de referencia, se adopta el siguiente de la lista.
 * El punto lleva la cuenta de la cantidad de veces que "debería haber sido observado" según la pose de la cámara,
 * de la cantidad de veces que efectivamente fue hallado (su descriptor fue macheado).
 *
 * Hay tres conjuntos de propiedades efímeras para uso específico de tracking, loop closing y local mapping.
 * Estas propiedades guardan valores asociados al punto y de corta duración, válidas a lo largo de un procedimiento.
 * No son multihilo, los procedimientos comienza asginando algún valor que consumen más adelante en el mismo procedimiento, y carece de sentido al finalizar.
 */
class MapPoint
{
public:
	/**
	 * Constructor que toma los valores argumentos.
	 *
	 * @param Pos Posición en el mapa.
	 * @param pRefKF Keyframe de referencia, el keyframe actual al momento de crear el punto.
	 * @param pMap Mapa al que pertenece el punto.  Hay un único mapa en ORB-SLAM.
	 * @param rgb_ Color del punto.  Opcional, negro si no se proporciona.
	 *
	 * Este constructor se invoca sólo desde LocalMapping::CreateNewMapPoints para el agregado de puntos del SLAM.
	 *
	 * Único constructor utilizado.
	 *
	 */
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap, cv::Vec3b rgb = 0);

    /**
     * Asigna al punto las coordenadas 3D argumento.
     *
     * @param Pos Vector 3x1 float con la posición que se copiará a MapPoint::mWorldPos.
     */
    void SetWorldPos(const cv::Mat &Pos);

    /**
     * Devuele un Mat con las coordenadas del punto.
     *
     * @returns Vector de posición del punto.
     */
    cv::Mat GetWorldPos();

    /**
     * Vector normal promedio de las observaciones.
     * Los puntos suelen estar en una superficie, y sólo pueden ser observados de un lado.
     * Cada punto aproxima su vector normal como promedio de las observaciones.
     *
     * @returns Vector normal del punto.
     */
    cv::Mat GetNormal();

    /**
     * Devuelve el keyframe de referencia.
     *
     * @returns Keyframe de referencia.
     *
     * Devuelve mpRefKF.
     */
    KeyFrame* GetReferenceKeyFrame();

    /**
     * Establece el keyframe de referencia mpRefKF.
     *
     * @param pRefKF Keyframe de referencia.
     *
     * Este método no se utiliza.
     */
    void SetReferenceKeyFrame(KeyFrame* pRefKF);

    /**
     * Devuelve todas las observaciones del punto.  Cada observación consiste de un mapa id a keyframe.
     *
     * Devuelve MapPoint::mObservations
     */
    std::map<KeyFrame*,size_t> GetObservations();

    /** Informa la cantidad de observaciones que registra el punto.*/
    int Observations();

    /** Cada vez que el punto es observado desde un keyframe, se registra esa observación.
     * @param pKF Keyframe que observa el punto.
     * @param idx índice del keyframe que observa el punto.
     */
    void AddObservation(KeyFrame* pKF,size_t idx);

    /**
     * Elimina del punto el registro que indicaba que fue observado por ese keyframe.
     *
     * Elimina el keyframe del mapa mObservations y decrementa la cantidad de observaciones del punto.
     * Si el eliminado es el keyframe de referencia, elige otro.
     * Si el punto queda observado por sólo 2 keyframes, lo elimina con SetBadFlag.
     *
     * @param pKF Keyframe a eliminar.
     *
     * Si pKF no observa el punto, no hace nada.
     */
    void EraseObservation(KeyFrame* pKF);

    /**
     * Índice de este punto en el vector de features del keyframe.  -1 si el keyframe no observa este punto.
     *
     * @param pKF Keyframe a testear, para ver si observa o no el punto.
     *
     * Devuelve el índice idx, de modo que este punto es pKF->mvpMapPoints[idx]
     */
    int GetIndexInKeyFrame(KeyFrame* pKF);

    /**
     * Busca si este punto es observado por el keyframe argumento.
     *
     * @param pKF Keyframe donde buscar el punto.
     * @returns true si encontró el punto.
     */
    bool IsInKeyFrame(KeyFrame* pKF);

    /**
     * Elimina el punto, marcándolo como malo.
     * Lo retira del mapa.
     * No destruye el punto en sí, para evitar conflictos entre hilos, pero debería.
     * Este flag es efímero, aunque por algún error algunos puntos marcados como malos y retirados del mapa perduran en otros contenedores.
     *
     * Con mutex marca mbBad y elimina el mapa mObservations.
     * Fuera del mutex recorre los keyframes de mObservations para eliminar el punto de sus macheos.
     * Finalmente elimina el punto del registro del mapa.
     */
    void SetBadFlag();

    /**
     * Informa el flag mBad.
     *
     * Todos los iteradores consultan este flag antes de considerar el punto.
     * Sobre un vector, preguntan primero si el puntero es nulo, y enseguida si isBad.
     */
    bool isBad();

    /**
     * Reemplaza este punto por el punto argumento, en una fusión.
     *
     * @param pMP Punto argumento que reemplazará al actual.
     *
     * Las observaciones de este punto se agregan al punto argumento.  También se suman las visualizaciones y encuentros.
     *
     * Actualiza todas las visualizaciones en los keyframes que obervan este punto, para que observen al punto argumento.
     *
     * Invocado por tres métodos que fusionan puntos: LoopCosing::CorrectLoop, LoopClosing::SearchAndFuse y ORBmatcher::Fuse.
     */
    void Replace(MapPoint* pMP);
    MapPoint* GetReplaced();

    /** Incrementa el contador de cantidad de veces que fue observado el punto.*/
    void IncreaseVisible(int n=1);

    /** Incrementa el contador de cantidad de veces que fue hallado el punto.*/
    void IncreaseFound(int n=1);

    /** Porcentaje de veces que el punto fue detectado, sobre el total de veces que estuvo en el fustrum.*/
    float GetFoundRatio();

    /** Cantidad de veces que el punto fue encontrado.*/
    inline int GetFound(){
        return mnFound;
    }


    /**
     * Elige el mejor descriptor entre todos los keyframes que observan el punto.
     * Recupera todos los descriptores, computa las distancias entre todas las combinaciones,
     * y elige el descriptor con menor distancia promedio al resto.
     * Guarda el valor en mDescriptor.
     */
    void ComputeDistinctiveDescriptors();

    /**
     * Devuelve el mejor descriptor del punto 3D.
     * Un punto 3D tiene varias observaciones, y por lo tanto varios descriptores.
     * MapPoint::ComputeDistinctiveDescriptors calcula el que mejor lo representa.
     *
     * @returns Descriptor del punto 3D.
     */
    cv::Mat GetDescriptor();

    /**
     * Recalcula el vector normal y la profundidad de visibilidad a partir de las observaciones.
     */
    void UpdateNormalAndDepth();

    /**
     * Calcula la distancia mínima a la que puede ser observado el punto.
     * Se calcula como 80% de la menor distancia en que fue efectivamente observado.
     *
     * @returns Distancia mínima de observación.
     */
    float GetMinDistanceInvariance();

    /**
     * Calcula la distancia máxima a la que puede ser observado el punto.
     * Se calcula como el 127% de la mayor distancia en que fue efectivamente observado.
     *
     * @returns Distancia máxima de observación.
     */
    float GetMaxDistanceInvariance();


    /**
     * Predice la escala (el nivel de la pirámide) en que se encontrará el punto, a partir de la distancia de observación.
     *
     * @param currentDist Distancia actual.
     * @param logScaleFactor Factor de escala.
     * @returns Nivel de la pirámide.
     */
    int PredictScale(const float &currentDist, const float &logScaleFactor);


	/**
	 * Color sugerido para graficación.
	 * Invocado por MapDrawer y FrameDrawer
	 */

	cv::Scalar color();

	/**
	 * Informa si el punto es quasi-infinito.
	 * @param true si es quasi-infinito, false si es normal.
	 */
	inline bool esQInf();



public:
    /** Identificador del punto.  Único para cada instancia.*/
    long unsigned int mnId;

    /** Identificador para el próximo nuevo punto.*/
    static long unsigned int nNextId;

    /**
     * Id del primer keyframe que observa este punto, cuyo valor es asignado en el constructor y nunca modificado.
     *
     * Es el id del keyframe de referencia, pero es efímero y tiene utilidad hasta que se genera el 4º keyframe posterior al de referencia.
     * En el caso que se elimine el keyframe de referencia, la referencia cambia, pero se preserva este id.
     *
     * Es utilizado solamente en LocalMapping::MapPointCulling para evitar descartar puntos desde el keyframe que lo generó ni de los dos siguientes.
     *
     * Se puede reconstruir luego de una serialiación haciendo cero su valor.
     */
    long int mnFirstKFid;

    /** Id del primer frame que observa este punto.*/
    long int mnFirstFrame;

    /** Cantidad de veces que fue observado.*/
    int nObs;

    ///@{
    /**
     * Variables efímeras usadas por Tracking.
     *
     * Varias se escriben solamente en Frame::IsInFrustum y se utiliza en OrbMatcher::SearchByProjection
     */
    //@{
    /** Variables usadas por Tracking.*/
    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    //float mTrackProjXR;
    bool mbTrackInView;
    int mnTrackScaleLevel;

    /**
     * Coseno de triangulación.
     * Efímero.
     * Frame::IsInFustrum lo registra, ORBmatcher::SearchByProjection lo consume.
     */
    float mTrackViewCos;

    /**
     * Registra el id del último cuadro que incorporó a este punto en su mapa local.
     *
     * Variable utilizada en Tracking::UpdateLocalPoints.
     *
     * Se inicializa en 0 en el constructor, y se escribe y lee sólo en Tracking::UpdateLocalPoints.
     * Luego de la carga se puede inicializar en cero.
     *
     */
    long unsigned int mnTrackReferenceForFrame;

    /**
     * Registra el id del último frame que observó el punto.
     *
     * Utilizada en Tracking sólo para distinguir si se está observando en el frame actual o no.
     *
     * No es necesario serializar, el constructor lo inicializa en cero.
     */
    long unsigned int mnLastFrameSeen;

    ///@}
    //@}
    ///@{
    /** Variables efímeras usadas por LocalMapping.*/
    //@{
    /** Variables usadas por LocalMapping.*/
    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;
    ///@}

    ///@{
    /** Variables efímeras usadas por LoopClosing.*/
    //@{
    /** Variables usadas por LoopClosing.*/
    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    /**
     * Id del último keyframe que corrigió la posición del punto.
     * Inicializado en 0 en la construcción.
     * Escrito por CorrectLoop, que luego invoca OptimizeEssentialGraph y lo lee.
     */
    long unsigned int mnCorrectedByKF;

    /*
     * Escrito por CorrectLoop, que luego invoca OptimizeEssentialGraph y lo lee.
     */
    long unsigned int mnCorrectedReference;    
    cv::Mat mPosGBA;

    /**
     * RunBundleAdjustment lo asigna y luego lo consume.
     */
    long unsigned int mnBAGlobalForKF;
    ///@}
    //@}

    cv::Vec3b rgb = 0;

    static std::mutex mGlobalMutex;


    // Puntos lejanos

    /**
     * Sobre puntos lejanos
     *
     * Las propiedades de puntos lejanos tienen el prefijo pl.
     *
     * Los puntos normales o normalizados tienen plLejano = cercano.
     * Los puntos normales desde su origen tienen plLejano = cercano, plCandidato = false y plOrigen = normal.
     * Marca que distingue el punto lejano.
     * Principalmente para limitar el mapa de covisibilidad, y quizás el bundle adjustment.
     * 0. normal
     * 1. lejano triangulado
     * 2. muy lejano por COS
     * 3. muy lejano por SVD
     */

	/**
	 * true si es candidato a punto lejano.
	 * No se debe considerar en PoseOptimization ni extender el grafo de covisibilidad.
	 */
	bool plCandidato = false;

	/**
	 * Clasificación del punto:
	 * - cercano para los puntos estándares de orb-slam2
	 * - lejano para los puntos triangulados con poco paralaje
	 * - muy lejano para los puntos en el infinito
	 */
	enum lejania {
		cercano,	// 0, no lejano
		lejano,
		muyLejano
	};
	lejania plLejano = cercano;

	/**
	 * Método de triangulación del punto candidato:
	 * - na No Aplica
	 * - umbralCosBajo para triangulaciones ordinarias interceptadas en un umbral ad hoc
	 * - umbralCos para triangulaciones con cos de paralaje superior a 0.9998.  Se envían al infinito.
	 * - svdInf para infinitos según SVD
	 */
	enum origen {normal, umbralCosBajo, umbralCos, svdInf};
	origen plOrigen = normal;

	/**
	 * Método que confirmó el punto candidato (dejó de ser candidato):
	 * - na No Aplica, puntos no cofirmados.
	 * - tiangulacionLejanaSinParalaje: triangulación con keyframes no vecinos, sin paralaje, envía el punto al infinito
	 * - triangulacionLejana: triangulación con keyframes no vecinos, con paralaje, le da coordenadas al punto
	 * - localBA: Optimizer::LocalBundleAdjustment le dio coordenadas al punto candidato
	 * - observacionParalaje: detección de paralaje cuando se agrega una observación con MapPoint::AddObservation: paralaje en keyframes vecinos.
	 * - retriangulación: LocalMapping::CreateNewMapPoints le dio coordenadas al punto candidato, por retriangulación
	 */
	enum confirmacion {na, tiangulacionLejanaSinParalaje, triangulacionLejana, localBA, observacionParalaje, retriangulacion};
	confirmacion plConfirmacion = na;

	/**
	 * Registra el cos del paralaje cuando se crea el punto.
	 */
	float plCosOrigen = 0.0;

	/**
	 * Menos coseno (mayor paralaje) obtenido luego de varias observaciones.
	 */
	float plCosMenor;


protected:    

    /**
     * Posición en coordenadas absolutas del mapa global.
     * Mat vertical de opencv de 3x1 (3 filas, una columna).
     */
	// Position in absolute coordinates
	cv::Mat mWorldPos;

	/**
	 * Keyframes que observan este punto, y sus índices.
	 *
	 * El mapa tiene como índice punteros a keyframes que observan este punto.
	 * El dato secundario asociado al índice es el índice del vector KeyFrame::mvpMapPoints, cuyo elemento es un puntero a este punto.
	 */
	// Keyframes observing the point and associated index in keyframe
	std::map<KeyFrame*,size_t> mObservations;

	/** Vector normal, computado como el promedio de las direcciones de todas las vistas del punto.*/
	// Mean viewing direction
	cv::Mat mNormalVector;

	/** Mejor descriptor del punto.*/
	// Best descriptor to fast matching
	cv::Mat mDescriptor;

	/**
	 * Keyframe de referencia.
	 *
	 * Comienza siendo el keyframe que creó el punto, aunque rota si ese keyframe se elimina.
	 * Utilizado al optimizar el grafo esencial y en BA para cerrar bucles.
	 */
	// Reference KeyFrame
	KeyFrame* mpRefKF;

	/** Cantidad de veces que el punto fue captado por la cámara, independientemente de si se lo detectó.
	 *  Es la cantidad de veces que "debería haber sido visto", según la pose de la cámara.
	 */
	// Tracking counters
	int mnVisible;

	/** Cantidad de veces que fue visible, y pudo ser detectado.*/
	int mnFound;

	/**
	 * Flag de borrado.
	 * Los puntos eliminados se quitan de las visualizaciones en keyframes, y del mapa.
	 * La instancia en sí no se elimina, no hay control de quién puede estar apuntándola.
	 */
	// Bad flag (we do not currently erase MapPoint from memory)
	bool mbBad;

	/**
	 * Punto que reemplazó a éste en una fusión.
	 * Éste quedó marcado como malo.
	 */
	MapPoint* mpReplaced;

	/* Rango de distancias (mínima y máxima) en la que puede ser observado y reconocido por su descriptor.*/
	// Scale invariance distances
	/** Distancia mínima en la que puede ser observado y reconocido por su descriptor.*/
	float mfMinDistance;

	/** Distancia máxima en la que puede ser observado y reconocido por su descriptor.*/
	float mfMaxDistance;

	/** Mapa al que pertenece.
	 * Hay un único mapa, el mapa global.
	 */
	Map* mpMap;

	std::mutex mMutexPos;
	std::mutex mMutexFeatures;

	/**
	 * Constructor por defecto de MapPoint para el serializador.
	 *
	 * Se encarga de inicializar las variables const, para que el compilador no chille.
	 */
	MapPoint();
	friend class boost::serialization::access;
	friend class Serializer;

	/**
	 * Serializador de MapPoint
	 * Se invoca al serializar Map::mspMapPoints y KeyFrame::mvpMapPoints, cuyos mapPoints nunca tienen mbBad true.
	 * La serialización de MapPoint evita punteros para asegurar el guardado consecutivo de todos los puntos antes de proceder con los KeyFrames.
	 * Esto evita problemas no identificados con boost::serialization, que cuelgan la aplicación al guardar.
	 *
	 * Versionado:
	 * La versión 1 reconstruye mRefKF, y ya no guarda mnFirstKFid.
	 */
	template<class Archivo> void serialize(Archivo&, const unsigned int);
	// Fin del agregado para serialización
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
