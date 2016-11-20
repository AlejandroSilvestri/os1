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
#ifndef INITIALIZER_H
#define INITIALIZER_H

#include<opencv2/opencv.hpp>
#include "Frame.h"


namespace ORB_SLAM2
{

/**
 * Inicializador de mapa en modo monocular.
 * Procura la triangulación de los primeros puntos del mapa.
 *
 */
// THIS IS THE INITIALIZER FOR MONOCULAR SLAM. NOT USED IN THE STEREO OR RGBD CASE.
class Initializer
{
    typedef pair<int,int> Match;

public:

    /**
     * Constructor que toma ReferenceFrame como referencia para los sucesivos intentos de inicialización.
     *
     * @param ReferenceFrame Cuadro de referencia inicial, el primero de las dos vistas necesarias para la triangulación.
     * @param sigma
     * @param iterations Cantidad máxima de iteraciones para cada intento de triangulación.
     */
    // Fix the reference frame
    Initializer(const Frame &ReferenceFrame, float sigma = 1.0, int iterations = 200);

    /**
     * Computa en paralelo una matriz fundamental y una homografía.
     * Selecciona el modelo con mejor resultado e intenta obtener el movimiento y la estructura SFM (structure from motion).
     * Intenta triangular los primeros puntos del mapa.
     *
     * @param CurrentFrame Cuadro actual.
     * @param vMatches12 Macheos.
     * @param R21 Matriz rotación de la segunda cámara respecto de la primera, resultado de la inicialización.
     * @param t21 Vector traslación de la segunda cámara respecto de la primera, resultado de la inicialización.
     * @param vP3D Puntos 3D triangulados.
     * @param vbTriangulated Indicador de inliers (true) u outliers (false).
     * @returns true si logró la triangulación, false si no.
     */
    // Computes in parallel a fundamental matrix and a homography
    // Selects a model and tries to recover the motion and the structure from motion
    bool Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12,
                    cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated);


private:

    /**
     * Busca la homografía entre los dos cuadros macheados, usando RANSAC.
     * Junto con FindFundamental, son los dos métodos de inicialización que intentan obtener la pose inicial.
     * Los tres argumentos sólo reciben el resultado.
     * @param vbMatchesInliers Vector booleano cuyos elementos indican si los puntos singulares correspondiente son inliers.
     * @param score Puntaje de la homografía.  Mayor, mejor.
     * @param H21 Matriz de transformación homográfica obtenida.
     *
     */
    void FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21);

    /**
     * Computa la matriz fundamental entre los dos cuadros macheados, usando RANSAC.
     * Junto con FindHomography, son los dos métodos de inicialización que intentan obtener la pose inicial.
     * Los tres argumentos reciben el resultado.
     * @param vbInliers
     * @param score
     * @param F21 Matriz fundamental.
     */
    void FindFundamental(vector<bool> &vbInliers, float &score, cv::Mat &F21);

    /**
     * Computa una homografía para una dada correspondencia de puntos normalizados en dos imágenes.
     *
     * @param vP1 Puntos singulares de un cuadro.
     * @param vP2 Puntos singulares correspondientes en el otro cuadro.
     * @returns Matriz de homografía.
     *
     * Invocado sólo por Initializer::FindHomography.
     */
    cv::Mat ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);

    /**
     * Computa la matriz fundamental para una dada correspondencia de puntos normalizados en dos imágenes, mediante SVD.
     * @param vP1 Puntos singulares de un cuadro.
     * @param vP2 Puntos singulares correspondientes en el otro cuadro.
     * @returns Matriz de homografía.
     *
     * Invocado sólo por Initializer::FindFundamental.
     */
    cv::Mat ComputeF21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);

    /**
     * Evalúa la calidad de la homografía, haciendo una verificación epipolar.
     *
     * @param H21 Matriz de transformación homográfica.
     * @param H12 Inversa de H21.
     * @param vbMatchesInliers Resultado, marca de puntos singulares que pasaron el test.
     * @param sigma Sigma establece el umbral de prueba.
     * @returns Resultado de la evaluación.  Mayor, mejor.
     *
     * Invocado sólo desde Initializer::FindHomography.
     */
    float CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma);

    /**
     * Evalúa la calidad de la matriz fundamental, haciendo una verificación epipolar.
     *
     * @param F21 Matriz fundamental.
     * @param vbMatchesInliers Resultado, marca de puntos singulares que pasaron el test.
     * @param sigma Sigma establece el umbral de prueba.
     * @returns Resultado de la evaluación.  Mayor, mejor.
     *
     * Invocado sólo desde Initializer::FindFundamental.
     */
    float CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma);

    /**
     * Verifica las 4 hipótesis de rototraslación obtenidas de la matriz fundamental.
     * Si hay suficientes inliers y paralaje, usa esa pose para inicializar.
     *
     * @param vbMatchesInliers Marca de maches válidos, inliers.
     * @param F21 Matriz fundamental.
     * @param K Matriz de cámara, intrínseca o de calibración.
     * @param R21 Resultado: matriz de rotación de una cámara respecto de la otra.
     * @param t21 Resultado: vector traslación de una cámara respecto de la otra.
     * @param vP3D Puntos 3D.
     * @param vbTriangulated
     * @param minParallax Umbral mínimo de paralaje: 1.0.
     * @param minTriangulated Cantidad mínima de puntos triangulados: 50.
     * @returns true si eligió el candidato para la inicialización.
     *
     * Invocado sólo desde Initializer::initialize.
     */
    bool ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

    /**
     * Verifica las 8 hipótesis de rototraslación de Faugeras obtenidas de la homografía.
     * Si hay suficientes inliers y paralaje, usa esa pose para inicializar.
     *
     * @param vbMatchesInliers Marca de maches válidos, inliers.
     * @param F21 Matriz fundamental.
     * @param K Matriz de cámara, intrínseca o de calibración.
     * @param R21 Resultado: matriz de rotación de una cámara respecto de la otra.
     * @param t21 Resultado: vector traslación de una cámara respecto de la otra.
     * @param vP3D Puntos 3D.
     * @param vbTriangulated
     * @param minParallax Umbral mínimo de paralaje: 1.0.
     * @param minTriangulated Cantidad mínima de puntos triangulados: 50.
     * @returns true si eligió el candidato para la inicialización.
     *
     * Invocado sólo desde Initializer::initialize.
     */
    bool ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

    /**
     * Triangula los puntos 3D partir de sus vistas desde ambas cámaras.
     *
     * @param kp1 Punto singular observado desde la cámara 1.
     * @param kp2 Punto singular observado desde la cámara 2.
     * @param P1 Matriz de proyección de la cámara 1: K[I|0].  Matriz de 4x3 basada en la matriz de cámara.
     * @param P2 Matriz de proyección de la cámara 2: K[R|t].  Matriz de 4x3 basada en la matriz de cámara y la pose relativa a la cámara 1.
     * @param x3D Resultado, coordenadas euclideanas del punto 3D triangulado por SVD, en el sistema de coordenadas de la cámara 1.
     *
     * Usa cv::SVD para triangular.
     *
     * La matriz de proyección P de una cámara es una matriz de 4x3 (3 filas, 4 columnas), que combina sus matrices intrínseca y extrínseca.
     * P = K[R|t], siendo K la matriz de cámara, R y t la rototraslación de la cámara respecto de la referencia.
     * Premultiplicando una coordenada 3D homogénea (vector de 4 dimensiones), se obtiene su proyección en coordenadas homogéneas en píxeles.
     *
     * Invocado sólo desde Initializer::CheckRT.
     */
    void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

    /**
     * Produce puntos con las coordenadas normalizadas de los puntos singulares.
     * Normalizar consiste en colocar el origen en el punto medio, y que las distancias promedio al origen sean 1.
     *
     * @param vKeys Puntos singulares a normalizar.
     * @param vNormalizedPoints Resultado, puntos con coordenadas normalizadas.
     * @param T Resultado, matriz homogénea de 3x3, transformación de normalización.
     *
     * Invocado sólo desde Initializer::FindHomography y Initializer::FindFundamental.
     *
     */
    void Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);

    /**
     * Verifica la pose para confirmar hipótesis de H o F.
     * Invoca repetidamente Initializer::Triangulate para obtener los puntos 3D que proyectan los puntos singulares.
     * Los marca como "buenos" si no tienen coordenadas en infinito, están delante de las cámaras,
     * el error de reproyección no supera el umbral argumento y tienen bajo paralaje (cosParallax<0.99998).
     *
     *
     * @param R Matriz rotación propuesta.
     * @param t Vector traslación propuesto.
     * @param vKeys1 Puntos singulares en el cuadro 1.
     * @param vKeys2 Puntos singulares en el cuadro 2.
     * @param vMatches12 Macheos entre puntos singulares.
     * @param vnInliers Señal de inlier.
     * @param K Matriz de cámara.
     * @param vP3D Resultado, puntos 3D triangulados.
     * @param th2 Umbral cuadrado, siempre 4*sigma2, máximo error de reproyección admitido para que el punto se considere bueno.
     * @param vbGood Resultado, señal de punto triangulado "bueno", correspondientes a vKeys1, vKeys2, vMatches12, vnInliers y vP3D.
     * @param parallax Resultado, paralaje promedio.
     * @returns Cantidad de puntos buenos.
     *
     * Invocado sólo desde Initializer::ReconstructF e Initializer::ReconstructH.
     */
    int CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<Match> &vMatches12, vector<bool> &vbInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax);

    /**
     * Descompone la matriz esencial E en matrices de rotación y vector traslación.
     *
     * @param E Matriz esencial.
     * @param R1 Resultado, matriz de rotación 1 (primera hipótesis).
     * @param R2 Resultado, matriz de rotación 2 (segunda hipótesis).
     * @param t Resultado, vector traslación para una hipótesis, su negativo para la otra.
     *
     * Invocada sólo desde Initializer::ReconstructF.
     */
    void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);


    /** Puntos singulares del cuadro 1.*/
    // Keypoints from Reference Frame (Frame 1)
    vector<cv::KeyPoint> mvKeys1;

    /** Puntos singulares del cuadro 2.*/
    // Keypoints from Current Frame (Frame 2)
    vector<cv::KeyPoint> mvKeys2;

    /** Macheos.*/
    // Current Matches from Reference to Current
    vector<Match> mvMatches12;

    /** Marcas de macheos..*/
    vector<bool> mvbMatched1;

    /** Matriz de calibración, intrínseca o de cámara.*/
    // Calibration
    cv::Mat mK;

    /** Desvío estándar y variancia.*/
    // Standard Deviation and Variance
    float mSigma, mSigma2;

    /** Cantidad máxima de iteraciones al aplicar RanSaC.*/
    // Ransac max iterations
    int mMaxIterations;

    /** Conjuntos para Ransac.*/
    // Ransac sets
    vector<vector<size_t> > mvSets;

};

} //namespace ORB_SLAM

#endif // INITIALIZER_H
