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
 * Hay tres conjuntos de propiedades para uso específico de tracking, loop closing y local mapping.
 */
class MapPoint
{
public:
	MapPoint(const cv::Mat &Pos, int FirstKFid, int FirstFrame, Map* pMap);
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();

    cv::Mat GetNormal();
    KeyFrame* GetReferenceKeyFrame();
    void SetReferenceKeyFrame(KeyFrame* pRefKF);

    std::map<KeyFrame*,size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF,size_t idx);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapPoint* pMP);    
    MapPoint* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
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

    int nObs;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;


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
