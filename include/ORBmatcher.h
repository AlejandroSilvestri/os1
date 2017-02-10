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
 * Hay 4 SearchByProjection especializados en:
 * 	- puntos rastreados
 * 	- puntos proyectados no rastreados
 * 	- candidatos para relocalización
 * 	- candidatos para cierre de bucle
 * - SearchByBoW machea primero por BoW, y luego por descriptores cuando hay varios candidatos con el mismo BoW.
 * - SearchForInitialization machea para encontrar los primeros puntos del mapa.
 * - SearchForTriangulation machea para encontrar nuevos puntos 3D.
 * - SearchBySim3 machea para evaluar candidatos a cierre de bucle.
 * - Fuse fusiona puntos del mapa duplicados.
 */
class ORBmatcher
{    
public:

	/**
	 * Constructor que guarda los argumentos en propiedades.
	 * @param nnratio Nearest Neighbour ratio.  Indica la relación entre el mejor y peor candidato a brindar como resultado.
	 * @param checkOri Check Orientation.  true para comprobar la orientación de cada punto antes de evaluar el macheo (true por defecto).  Además lleva un histórico de orientaciones.
	 *
	 */
	ORBmatcher(float nnratio=0.6, bool checkOri=true);

    // Computes the Hamming distance between two ORB descriptors
    static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

    /**
     * Macheo en ventana cuadrada entre puntos singulares detectados en el cuadro actual, y la proyección de los puntos del mapa que deberían ser vistos.
     * Evita reprocesar los puntos que ya fueron vistos en LastFrame y que han sido asignados al cuadro actual en TrackWithMotionModel.
     *
     * Estos puntos tienen la marca efímera MapPoint::mbTrackInView.
     *
     * Agrega los puntos nuevos al registro de puntos vistos del cuadro actual mvpMapPoints.
     *
     * @param F Cuadro actual
     * @param vpMapPoints Mapa local
     * @param th Threshold, umbral, base para calcular el radio de búsqueda.  Es 1, excepto luego de una relocalización, que es 5.
     * El radio se obtiene multiplicando th * ORBmatcher::RadiusByViewingCos, que normalmente es 4, pero es 2,5 cuando no hay paralaje.
     *
     * Invocado exclusivamente desde Tracking::SearchLocalPoints.
     */
    // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
    // Used to track the local map (Tracking)
    int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th=3);

    /**
     * Macheo en ventana cuadrada entre descriptores de puntos singulares del cuadro actual
     * con los puntos singulares del cuadro anterior asociados a algún punto del mapa local.
     *
     * Crea la lista de puntos del mapa local vistos desde el cuadro actual CurrentFrame.mvpMapPoints.
     *
     * @param CurrentFrame Cuadro actual, con sus puntos singulares detectados.
     * @param LastFrame Cuadro anterior, con sus puntos singulares y sus puntos de mapa local asociados.
     * @param th Umbral que determina el radio del área donde machear.  El radio dependerá el nivel de la pirámide donde se busque.
     *
     * Se invoca con 15, y si se consiguieron pocos macheos se repite con 30.
     *
     * 1. Aplica el modelo de movimiento para pronosticar una pose.
     * 2. Proyecta puntos del mapa detectados en LastFrame.
     * 3. Machea descriptores entre los puntos singulares de CurrentFrame encontrados en una región circular con centro en la proyección.
     * 4. Asocia los puntos 3D macheados a los puntos singulares en CurrentFrame.
     *
     * Este método no corrige pose, sólo asocia puntos singulares macheados en una región circular.  No verifica coherencia.
     *
     * Invocado exclusivamente desde Tracking::TrackWithMotionModel, que a partir de este macheo computa la pose del cuadro actual.
     */
    // Project MapPoints tracked in last frame into the current frame and search matches.
    // Used to track from previous frame (Tracking)
    int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th);

    /**
     * Busca en el cuadro actual los puntos vistos en el keyFrame candidato para relocalización.
     *
     * @param CurrentFrame Cuadro actual.
     * @param pKF KeyFrame candidato para relocalización.
     * @param AlreadyFound Mapa de puntos encontrados.
     * @param th Umbral, radio patrón para el macheo circular.  El radio computado en píxeles varía para cada punto singular según el nivel de la pirámide.
     * @param ORBdist Distancia máxima aceptada entre descriptores.  Si el mejor macheo tiene distancia mayor, se descarta.
     *
     * Invocado exclusivamente desde Tracking::Relocalization.
     */
    // Project MapPoints seen in KeyFrame into the Frame and search matches.
    // Used in relocalisation (Tracking)
    int SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*> &sAlreadyFound, const float th, const int ORBdist);

    /**
     * Ante una detección de bucle con varios macheos y una pose propuesta, este método aumenta la cantidad de macheos
     * comparando contra otros puntos que deberían ser vistos desde el keyframe actual.
     *
     * @param pKF Keyframe actual, candidato a cerrar bucle.
     * @param Scw Rototraslación y escala (sim3), transformación de pose propuesta para el cierre de bucle.
     * @param vpPoints Puntos posiblemente observados según la pose calculada, a considerar en el cierre de bucle.
     * @param vpMatched Puntos macheados en la detección de bucle.
     * @param th Umbral.
     *
     * Invocado exclusivamente desde LoopClosing::ComputeSim3.
     */
    // Project MapPoints using a Similarity Transformation and search matches.
    // Used in loop detection (Loop Closing)
    int SearchByProjection(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> &vpPoints, std::vector<MapPoint*> &vpMatched, int th);

    /**
     * Machea puntos del cuadro actual con los de un keyframe.
     * Macheo de fuerza bruta, primero por BoW y luego por descriptores.
     *
     * @param pKF Keyframe candidato a explicar el cuadro actual.  Es el keyframe de referencia para tracking, o uno de varios candidatos en relocalización.
     * @param F Cuadro actual que se intenta ubicar.
     * @param vpMapPointMatches Resultado, puntos 3D macheados.
     * @returns Cantidad de puntos macheados.
     *
     * Invocado sólo por Tracking::Relocalization y Tracking::TrackByReferencieKeyFrame.
     */
    // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
    // Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
    // Used in Relocalisation and Loop Detection
    int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches);

    /**
     * Evalúa keyframe candidato para cierre de bucle.
     * Machea por BoW y luego por descriptores.
     * @param pKF1 Keyframe actual.
     * @param pKF2 Keyframe candidato al cierre de bucle.
     * @param vpMatches12 Resultado de la búsqueda, puntos del mapa macheados.  Es un vector alineado con el vector de puntos de pKF1, conteniendo los puntos macheados por BoW y descriptores, observador por pKF2.
     * @returns Cantidad de macheos.
     *
     * Invocado sólo desde LoopClosing::ComputeSim3.
     */
    int SearchByBoW(KeyFrame *pKF1, KeyFrame* pKF2, std::vector<MapPoint*> &vpMatches12);

    /**
     * Macheo para inicialización.
     * Este método busca macheos en una ventana holgada, procurando una elevada cantidad de macheos que luego serán considerados para la triangulación de los primeros puntos del mapa.
     * El método se invoca de manera repetida y sucesiva, cuadro por cuadro, hasta que se logra inicializar, o hasta que la cantidad de macheo se reduce demasiado.
     * @param F1 Cuadro inicial del proceso de inicialización del mapa.
     * @param F2 Cuadro actual.
     * @param vbPrevMatched Coordenadas de los puntos singulares macheados en el cuadro anterior (es decir, la última vez que se invocó este método).
     * @param vnMatches12
     * @param windowSize Tamaño del área de búsqueda, windowSize es la longitud del lado del área cuadrada.  Siempre es 100.
     * @returns Cantidad de macheos obtenidos.
     *
     * Invocado sólo por Tracking::MonocularInitialization.
     */
    // Matching for the Map Initialization (only used in the monocular case)
    int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize=10);

    /**
     * Machea puntos singulares entre el keyframe actual y un vecino,
     * con el propósito posterior de triangular nuevos puntos y agregarlos al mapa.
     * Para cada punto singular del primer keyframe, el macheo recorre la línea epipolar del segundo keyframe.
     * Usa la matriz fundamental para aplicar geomtría epipolar, una manera de comprobar la compatibilidad geométrica del macheo.
     * Además reduce el esfuerzo de macheo comparando por BoW antes de calcular distancias de descriptores.
     *
     * @param pKF1 Keyframe actual.
     * @param pKF2 Keyframe vecino, anterior a pKF1.
     * @param F12 Matriz fundamental de pKF1 respecto de pKF2.
     * @param vMatchedPairs Resultado del algoritmo, los pares macheados.  Si tenía algo, lo borra.
     * @returns Cantidad de macheos encontrados.
     *
     * Primero utiliza KeyFrame::mFeatVec para encontrar conjuntos de puntos singulares con el mismo BoW.
     *
     * Luego, en estos conjuntos compara distancias por "fuerza bruta".
     *
     * Finalmente realiza una verificación epipolar.
     *
     * Invocado sólo desde LocalMapping::CreateNewMapPoints
     */
    // Matching to triangulate new MapPoints. Check Epipolar Constraint.
    int SearchForTriangulation(KeyFrame *pKF1, KeyFrame* pKF2, cv::Mat F12,
                               std::vector<pair<size_t, size_t> > &vMatchedPairs);//, const bool bOnlyStereo);

    /**
     * Compara dos keyframes candidatos al cierre de bucle y buscando aumentar los puntos macheados por aspecto.
     *
     * Se proporcionan los keyframes, un vector de puntos observador por un keyframe y machedos en el otro por aspecto,
     * y la transformación de similaridad que corrige la pose entre ellos.
     *
     * @param pKF1 Keyframe actual.
     * @param pKF2 Keyframe candidato a cerrar el bucle.
     * @param vpMatches12 Dato y resultado.  Puntos 3D vistos desde pKF2 y macheados por BoW y descriptores en pKF1.  Vector alineado con los de pKF1.
     * @param s12 Escala de pFK1 respecto de pKF2.
     * @param R12 Matriz rotación de pFK1 respecto de pKF2.
     * @param t12 Traslación de pFK1 respecto de pKF2.
     * @param th Radio en píxeles donde buscar puntos singulares para machear.
     * @returns Cantidad de puntos macheados y explicados por la Sim3.
     *
     * Como resultado de la operación se modifica el argumento vpMatches, que se ve aumentado con nuevos macheos encontrados.
     *
     * Invocado sólo desde LoopClosing::ComputeSim3.
     */
    // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]
    // In the stereo and RGB-D case, s12=1
    int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th);

    /**
     * Proyecta puntos del mapa sobre un keyframe y busca puntos duplicados, como parte del mapeo local.
     * Fusiona puntos del mapa que corresponden al mismo punto real.
     * Se utiliza dos veces: proyectando los puntos del keyframe actual sobre los keyframes vecinos, y viceversa.
     * @param pKF Keyframe donde se proyectarán los puntos del mapa.
     * @param vpMapPoints Puntos del mapa observados en un keyframe vecino.
     * @param th Radio de búsqueda circular.
     * @returns Cantidad de puntos fusionados.
     *
     * Invocado dos veces sólo desde LocalMapping::SearchInNeighbors.
     */
    // Project MapPoints into KeyFrame and search for duplicated MapPoints.
    int Fuse(KeyFrame* pKF, const vector<MapPoint *> &vpMapPoints, const float th=3.0);

    /**
     * Fusiona los puntos del mapa para cerrar un bucle.
     *
     * @param pKF Keyframe actual o vecino de un extremo del bucle cerrado.
     * @param Scw Pose sim3 de la cámara respecto del mundo.  Este argumento requiere mayor investigación.
     * @param vpPoints Puntos del mapa observados por el keyframe del otro extremo del bucle, y sus vecinos.
     * @param th Radio para el macheo circular.
     * @param vpReplacePoint Resultado del método, puntos fusionados.
     * @returns Cantidad de puntos fusionados.
     *
     * Invocado sólo desde LoopClosing::SearchAndFuse
     */
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

    /** Comprobación por línea epipolar.
     * Comprueba si un puntos singulares de una pose, están en (o muy cerca) de la línea epipolar de otro punto singular de otra pose.
     *
     * @param kp1 Punto singular de la pose 1.
     * @param kp2 Punto singular de la pose 2.
     * @param  F12 Matriz fundamental entre ambas poses.
     * @param pKF1 Keyframe 1 (pose 1).
     * @param pKF2 Keyframe 2 (pose 2).
     * @returns Señal que indica que los puntos tienen correspondencia epipolar.
     *
     * Invocada solo de SearchForTriangulation, con puntos singulares macheados.
     *
     */
    bool CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &F12, const KeyFrame *pKF);

    /**
     * Empaqueta la decisión del radio en búsqueda en ventana caudrada, según el coseno de la triangulación.
     * A mayor paralaje, menor radio, la búsqueda será más estricta.
     * @param viewCos Coseno del ángulo de visión del punto.
     *
     * Invocado sólo desde ORBmatcher::SearchByProjection entre keyframe y mapa.
     */
    float RadiusByViewingCos(const float &viewCos);

    /**
     * Computa 3 máximos.
     * Recorre los vectores del histograma, comparando sus longitudes, y devuelve los índices de los tres vectores de mayor cantidad de elementos.
     *
     * @param histo Histograma, array de vectores de int.  Siempre creado como `vector<int> rotHist[HISTO_LENGTH];`.
     * @param L Longitud, cantidad de elementos de histo.  Siempre es HISTO_LENGTH.
     * @param ind1 Índice del histotrama de mayor longitud.  Valor resultado, uno de los 3 máximos.  Siempre se pasa inicializado en -1.
     * @param ind2 Índice del histotrama de segunda mayor longitud.  Valor resultado, uno de los 3 máximos.  Siempre se pasa inicializado en -1.
     * @param ind3 Índice del histotrama de tercera mayor longitud.  Valor resultado, uno de los 3 máximos.  Siempre se pasa inicializado en -1.
     *
     * Este método se invoca sólo desde otros métodos de la misma clase.  Todos ellos inicializan `int1=int2=int3=-1`.
     */
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
