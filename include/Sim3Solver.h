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


#ifndef SIM3SOLVER_H
#define SIM3SOLVER_H

#include <opencv2/opencv.hpp>
#include <vector>

#include "KeyFrame.h"



namespace ORB_SLAM2
{

/**
 * Solucionador de transformación sim3 para cerrar bucles.
 * Para dos keyframes, presenta la transformación sim3 (rotación, traslación y escala) que mejor explica las poses.
 * También indica si no encontró una buena explicación.
 *
 * Esta clase se instancia exclusivamente en LoopClosing::ComputeSim3(), en este contexto:
 * Cuando se detecta la posibilidad de cierre de bucles, varios keyframes se presentan como candidatos.
 * Se crea un solucionador sim3 para evaluar la calidad del cierre propuesto por cada keyframe candidato.
 * Se utilizan los métodos:
 * - SetRansacParameters
 * - iterate
 * - GetEstimatedRotation
 * - GetEstimatedTranslation
 * - GetEstimatedScale
 *
 * Se inicializa el solucionador con los parámetros RANSAC, se lo ejecuta con iterate,
 * y luego se evalúa rotación, traslación y escala resultante.
 *
 */
class Sim3Solver
{
public:

	/**
	 * Constructor del solucionador sim3, a partir de dos keyframes y un vector de puntos macheados.
	 *
	 * @param pKF1 Un keyframe extremo a cerrar en bucle.  Se usa el keyframe actual.
	 * @param pKF2 Otro keyframe extremo a cerrar en bucle.  Se usa el keyframe detectado, el del otro extremo.
	 * @param vpMatched12 Vector de puntos del mapa que se observan en ambos keyframes, sobre los que se aplicará la transformación de similaridad.
	 *
	 */
	Sim3Solver(KeyFrame* pKF1, KeyFrame* pKF2, const std::vector<MapPoint*> &vpMatched12, const bool bFixScale = true);

    /**
     * Configuración de Ransac.
     *
     * @param probability
     * @param minInliers Mínima cantidad de inliers para considerar exitora una iteración.
     * @param maxIterations Máxima cantidad de iteraciones, luego de las cuales aborta.
     */
	void SetRansacParameters(double probability = 0.99, int minInliers = 6 , int maxIterations = 300);

	/**
	 * Ejecuta el número máximo de iteraciones Ransac.
	 *
     * @param vbInliers Resultado, marca la posición de los elementos inliers encontrados.
     * @param nInliers Restulado, cantidad de inliers encontrados.
     * @returns Pose de la relocalización, o una matriz vacía si no falla.
     *
   	 * No utilizado en orb-slam2.  En su lugar se usa Sim3solver::iterate.
	 */
	cv::Mat find(std::vector<bool> &vbInliers12, int &nInliers);

    /**
     * Ejecuta n iteraciones de Ransac.
     *
     * @param nIterations Cantidad de iteraciones a ejecutar.  Termina antes si se alcanza PnPsolver::mRansacMaxIts en total.
     * @param bNoMore Resultado, señal que indica que se alcanzaron las PnPsolver::mRansacMaxIts iteraciones en total.
     * @param vbInliers Resultado, marca la posición de los elementos inliers encontrados.
     * @param nInliers Restulado, cantidad de inliers encontrados.
     * @returns Pose de la relocalización, o una matriz vacía si no falla.
     *
     * Usado sólo en LoopClosing::ComputeSim3.  También se invoca desde el método no utilizado find.
     *
     *
     */
	cv::Mat iterate(int nIterations, bool &bNoMore, std::vector<bool> &vbInliers, int &nInliers);

    /** Obtiene la matriz rotación 3D estimada para el cierre de bucle.  @returns Matriz de rotación 3D, de 3x3.*/
	cv::Mat GetEstimatedRotation();
	/** Obtiene la traslación estimada para el cierre de bucle.  @returns Vector traslación 3d, de 3x1.*/
    cv::Mat GetEstimatedTranslation();
    /** Obtiene la escala de acople estimada para el cierre de bucle.  @returns Factor de escala para el cierre de bucle.*/
    float GetEstimatedScale();


protected:

    void ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C);

    /**
     * Implementación de Horn 1987, para computar la transformación de similaridad.
     *
     * @param P1
     * @param P2
     */
    void ComputeSim3(cv::Mat &P1, cv::Mat &P2);

    /** Establece las marcas de inliers, reproyectando y comparando el error con un umbral.*/
    void CheckInliers();

    /**
     * Proyecta puntos 3D expresados en coordenadas del mundo, sobre una cámara.
     *
     * @param vP3Dw Puntos 3D a proyectar.
     * @param vP2D Resultado, proyecciones 2D de los puntos 3D.
     * @param Tcw Pose de la cámara respecto del mundo.
     * @param K Matriz intrínseca de la cámara.
     *
     * Utilizado sólo en Sim3Solver::CheckInliers.
     */
    void Project(const std::vector<cv::Mat> &vP3Dw, std::vector<cv::Mat> &vP2D, cv::Mat Tcw, cv::Mat K);

    /**
     * Proyecta puntos 3D expresados en las coordenadas de la cámara.
     *
     * @param vP3Dw Puntos 3D a proyectar.
     * @param vP2D Resultado, proyecciones 2D de los puntos 3D.
     * @param K Matriz intrínseca de la cámara.
     *
     * Invocado sólo desde el constructor.
     */
    void FromCameraToImage(const std::vector<cv::Mat> &vP3Dc, std::vector<cv::Mat> &vP2D, cv::Mat K);


protected:

    // KeyFrames and matches
    KeyFrame* mpKF1;
    KeyFrame* mpKF2;

    std::vector<cv::Mat> mvX3Dc1;
    std::vector<cv::Mat> mvX3Dc2;
    std::vector<MapPoint*> mvpMapPoints1;
    std::vector<MapPoint*> mvpMapPoints2;
    std::vector<MapPoint*> mvpMatches12;
    std::vector<size_t> mvnIndices1;
    std::vector<size_t> mvSigmaSquare1;
    std::vector<size_t> mvSigmaSquare2;
    std::vector<size_t> mvnMaxError1;
    std::vector<size_t> mvnMaxError2;

    int N;
    int mN1;

    // Current Estimation
    cv::Mat mR12i;
    cv::Mat mt12i;
    float ms12i;
    cv::Mat mT12i;
    cv::Mat mT21i;
    std::vector<bool> mvbInliersi;
    int mnInliersi;

    // Current Ransac State
    int mnIterations;
    std::vector<bool> mvbBestInliers;
    int mnBestInliers;
    cv::Mat mBestT12;
    cv::Mat mBestRotation;
    cv::Mat mBestTranslation;
    float mBestScale;

    // Scale is fixed to 1 in the stereo/RGBD case
    bool mbFixScale;

    // Indices for random selection
    std::vector<size_t> mvAllIndices;

    // Projections
    std::vector<cv::Mat> mvP1im1;
    std::vector<cv::Mat> mvP2im2;

    // RANSAC probability
    double mRansacProb;

    // RANSAC min inliers
    int mRansacMinInliers;

    // RANSAC max iterations
    int mRansacMaxIts;

    // Threshold inlier/outlier. e = dist(Pi,T_ij*Pj)^2 < 5.991*mSigma2
    float mTh;
    float mSigma2;

    // Calibration
    cv::Mat mK1;
    cv::Mat mK2;

};

} //namespace ORB_SLAM

#endif // SIM3SOLVER_H
