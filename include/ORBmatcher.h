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


#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"MapPoint.h"
#include"KeyFrame.h"
#include"Frame.h"


namespace ORB_SLAM2
{

/**
 *
 * ORBmatcher empaqueta todos los métodos de macheo de descriptores.
 * Las pocas propiedades son protegidas, corresponden a la configuración establecida en la construcción.
 * No confundir con la clase ORBextractor, de nombre similar, que empaqueta todos los métodos de extracción de descriptores.
 * Todos los métodos son invocados desde otros objetos.
 * Solamente DescriptorDistance, que computa la distancia entre dos descriptores con el algoritmo de Stanford,
 * es invocado también internamente por prácticamente todos los otros métodos de ORBMatcher.
 *
 * Mientras ORBextractor se ocupa de detectar puntos singulares y extraer descriptores ORB,
 * ORBmatcher se ocupa de machear de diversas maneras:
 *
 * - SearchByProjection parte de una pose, proyecta los puntos del mapa local que deberían ser visibles, y realiza un macheo circular.
 * - SearchByBoW machea primero por BoW, y luego por descriptores cuando hay varios candidatos con el mismo BoW.
 * - SearchForInitialization machea para encontrar los primeros puntos del mapa.
 * - SearchForTriangulation machea para encontrar nuevos puntos 3D.
 * - SearchBySim3 machea para evaluar candidatos a cierre de bucle.
 * - Fuse fusiona puntos del mapa duplicados.
 */
class ORBmatcher
{    
public:

    ORBmatcher(float nnratio=0.6, bool checkOri=true);

    // Computes the Hamming distance between two ORB descriptors
    static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

    // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
    // Used to track the local map (Tracking)
    int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th=3);

    // Project MapPoints tracked in last frame into the current frame and search matches.
    // Used to track from previous frame (Tracking)
    int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono);

    // Project MapPoints seen in KeyFrame into the Frame and search matches.
    // Used in relocalisation (Tracking)
    int SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*> &sAlreadyFound, const float th, const int ORBdist);

    // Project MapPoints using a Similarity Transformation and search matches.
    // Used in loop detection (Loop Closing)
    int SearchByProjection(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> &vpPoints, std::vector<MapPoint*> &vpMatched, int th);

    // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
    // Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
    // Used in Relocalisation and Loop Detection
    int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches);
    int SearchByBoW(KeyFrame *pKF1, KeyFrame* pKF2, std::vector<MapPoint*> &vpMatches12);

    // Matching for the Map Initialization (only used in the monocular case)
    int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize=10);

    // Matching to triangulate new MapPoints. Check Epipolar Constraint.
    int SearchForTriangulation(KeyFrame *pKF1, KeyFrame* pKF2, cv::Mat F12,
                               std::vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo);

    // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]
    // In the stereo and RGB-D case, s12=1
    int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th);

    // Project MapPoints into KeyFrame and search for duplicated MapPoints.
    int Fuse(KeyFrame* pKF, const vector<MapPoint *> &vpMapPoints, const float th=3.0);

    // Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.
    int Fuse(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint);

public:

    /**
     * Umbral bajo, estricto, para distancias entre descriptores.
     * `bestDist<=TH_LOW` es la chequeo habitual para aceptar o rechazar la mejor distancia luego de comparar varios descriptores candidatos.
     * Este umbral se usa sólo dentro del objeto, para todas los macheos excepto para machear descriptores de cuadros con descriptores de mapa.
     * TH_LOW se define en 50 (hasta 50 de los 256 bits del descriptor pueden ser diferentes del descriptor de referencia).
     */
    static const int TH_LOW;

    /**
     * Umbral alto, laxo, para distancias entre descriptores.
     * `bestDist<=TH_HI` es la chequeo habitual para aceptar o rechazar la mejor distancia luego de comparar varios descriptores candidatos.
     * Este umbral se usa sólo dentro del objeto en ComputeSim3 y SearchByProjection para tracking.
     * TH_HIGH se define en 100 (hasta 100 de los 256 bits del descriptor pueden ser diferentes del descriptor de referencia).
     */
    static const int TH_HIGH;

    /**
     * Tamaño del array de histograma.
     */
    static const int HISTO_LENGTH;


protected:

    bool CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &F12, const KeyFrame *pKF);

    float RadiusByViewingCos(const float &viewCos);

    void ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

    /**
     * Nearest neighbor ratio.
     * Al buscar los dos mejores distancias, se procura que la mejor sea mayor a un porcentaje (mfNNratio) de la segunda.
     * `bestDist1<mfNNratio*bestDist2`
     * Es 0.6 por defecto.
     */
    float mfNNratio;

    /**
     * Señal interna de configuración definida en la construcción del objeto, que indica si se debe chequear la orientación
     * antes de comparar descriptores.
     * La orientación dice de qué lado es visible el descriptor, de modo que con esta verificación se evita comparar los puntos 3D observados desde el lado contrario.
     * Es `true` por defecto.
     */
    bool mbCheckOrientation;
};

}// namespace ORB_SLAM

#endif // ORBMATCHER_H
