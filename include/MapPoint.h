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

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include<opencv2/core/core.hpp>
#include<mutex>

namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class Frame;


/** Cada instancia representa un mapa 3D en el sistema de referencia del mapa.
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
	MapPoint(const cv::Mat &Pos, int FirstKFid, int FirstFrame, Map* pMap);

	/**
	 * Constructor que toma los valores argumentos.
	 *
	 * @param Pos Posición en el mapa.
	 * @param pRefKF Keyframe de referencia.
	 * @param pMap Mapa al que pertenece el punto.  Hay un único mapa en ORB-SLAM.
	 *
	 */
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

    /**
     * Asigna al punto las coordenadas 3D argumento.
     *
     * @param Pos Vector con la posición que se copiará al punto.
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
     */
    KeyFrame* GetReferenceKeyFrame();

    /**
     * Establece el keyframe de referencia.
     *
     * @param pRefKF Keyframe de referencia.
     */
    void SetReferenceKeyFrame(KeyFrame* pRefKF);

    /** Devuelve todas las observaciones del punto.  Cada observación consiste de un mapa id a keyframe.*/
    std::map<KeyFrame*,size_t> GetObservations();

    /** Informa la cantidad de observaciones que registra el punto.*/
    int Observations();

    /** Cada vez que el punto es observado desde un keyframe, se registra esa observación.
     * @param pKF Keyframe que observa el punto.
     * @param idx índice del keyframe que observa el punto.
     */
    void AddObservation(KeyFrame* pKF,size_t idx);

    /** Elimina del punto el registro que indicaba que fue observado por ese keyframe.  */
    void EraseObservation(KeyFrame* pKF);

    /**
     * Id del keyframe si éste observa el punto, -1 si no.
     *
     * @param pKF Keyframe a testear, para ver si observa o no el punto.
     *
     * Devuelve el índice del keyframe argumento si el mismo está en el registro de keyframes que observan al punto.  Si no devuelve -1.
     */
    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    /** Elimina el punto, marcándolo como malo.*/
    void SetBadFlag();

    /** Informa el flag mBad.  Todos los iteradores consultan este flag antes de considerar el punto.*/
    bool isBad();

    /**  Toma las propiedades del punto argumento.*/
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

public:
    /** Identificador del punto.  Único para cada instancia.*/
    long unsigned int mnId;

    /** Identificador para el próximo nuevo punto.*/
    static long unsigned int nNextId;

    /** Id del primer keyframe que observa este punto.
     * Usualmente es el keyframe de referencia.
     * En el raro caso en que se elimine el keyframe de referencia, la referencia cambia, pero se preserva este id.
     */
    long int mnFirstKFid;

    /** Id del primer frame que observa este punto.*/
    long int mnFirstFrame;

    /** Cantidad de veces que fue observado.*/
    int nObs;

    ///@{
    /** Variables efímeras usadas por Tracking.*/
    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;
    ///@}

    ///@{
    /** Variables efímeras usadas por LocalMapping.*/
    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;
    ///@}

    ///@{
    /** Variables efímeras usadas por LoopClosing.*/
    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;
    ///@}

    static std::mutex mGlobalMutex;

protected:    

    /** Posición en coordenadas absolutas del mapa global.
     * Mat de opencv.
     */
	// Position in absolute coordinates
	cv::Mat mWorldPos;

	/** Keyframes que observan este punto, y sus índices.*/
	// Keyframes observing the point and associated index in keyframe
	std::map<KeyFrame*,size_t> mObservations;

	/** Vector normal, computado como el promedio de las direcciones de todas las vistas del punto.*/
	// Mean viewing direction
	cv::Mat mNormalVector;

	/** Mejor descriptor del punto.*/
	// Best descriptor to fast matching
	cv::Mat mDescriptor;

	/** Keyframe de referencia.*/
	// Reference KeyFrame
	KeyFrame* mpRefKF;

	/** Cantidad de veces que el punto fue captado por la cámara, independientemente de si se lo detectó.
	 *  Es la cantidad de veces que "debería haber sido visto", según la pose de la cámara.
	 */
	// Tracking counters
	int mnVisible;

	/** Cantidad de veces que fue visible, y pudo ser detectado.*/
	int mnFound;

	/** Flag de borrado.  Los puntos eliminados no se quitan del vector, para evitar rearmar el vector.*/
	// Bad flag (we do not currently erase MapPoint from memory)
	bool mbBad;
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
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
