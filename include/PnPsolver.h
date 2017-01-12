/**
* This file is part of ORB-SLAM2.
* This file is a modified version of EPnP <http://cvlab.epfl.ch/EPnP/index.php>, see FreeBSD license below.
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

/**
* Copyright (c) 2009, V. Lepetit, EPFL
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* The views and conclusions contained in the software and documentation are those
* of the authors and should not be interpreted as representing official policies,
*   either expressed or implied, of the FreeBSD Project
*/

#ifndef PNPSOLVER_H
#define PNPSOLVER_H

#include <opencv2/core/core.hpp>
#include "MapPoint.h"
#include "Frame.h"

namespace ORB_SLAM2
{

/**
 * Solucionador Point n-Points EPnP.
 * Utiliza RANSAC para buscar la pose de un keyframe que mejor explican una serie de puntos observados, de posición conocida en el mapa.
 *
 * Esta clase se instancia repetidas veces solamente en Tracking::Relocalisation(), este este contexto:
 * Se evalúan varios keyframes candidatos, que obsevan presumiblemente los mismos puntos observados por el frame actual.
 * Para cada uno se correlacionan los puntos observados con BoW, del keyframe candidato se obtienen sus correspondientes coordenadas 3D,
 * y se ejecuta el solucionador para conseguir la pose que mejor explica la observación,
 *
 */
class PnPsolver {
 public:
  /*
   * Constructor, a partir de un cuadro y un vector de puntos 3D.
   * - Inicializa en cero la mayoría de los atributos.
   * - Asigna el tamaño definitivo a los vectores, y los carga con datos del cuadro argumento F.
   * - Ajusta los parámetros Ransac por defecto.
   * - Copia de F los parámetros intrínsecos.
   *
   * @param F Cuadro a relocalizar.
   * @param vpMapPointMatches Puntos 3D macheados con un keyframe.  Se guarda en el atributo PnPsolver::mvpMapPointMatches.
   *
   * Invocado sólo desde Tracking::Relocalization.
   */
  PnPsolver(const Frame &F, const vector<MapPoint*> &vpMapPointMatches);

  /** Destructor por defecto.*/
  ~PnPsolver();

  /**
   * Ajusta los parámetros de Ransac.
   * Todos los parámetros tienen un valor por defecto, aunque orb-slam2 no los usa.
   *
   * @param probability Mínima probablidad de acierto aceptada.  Este valor se registra en PnPsolver::mRansacProb
   * @param minInliers Mínima cantidad de inliers aceptados.  Este valor se registra en PnPsolver::mRansacMinInliers
   * @param maxIterations Máxima cantidad de iteraciones permitidas para encontrar la solución.  Si la alcanza, desiste.  Este valor se registra en PnPsolver::mRansacMaxIts
   * @param minSet Cantidad mínima de puntos a considerar en cada iteración.  Este valor se registra en PnPsolver::mRansacMinSet
   * @param epsilon   Este valor se registra en PnPsolver::mRansacEpsilon
   * @param th2 Umbral al caudrado.
   *
   * Invocado con parámetros por defecto por el constructor de PnPsolver, y por Tracking::Relocalization con parámetros ad hoc.
   *
   * En orb-slam2 toda vez que se construye una instancia, se invoca dos veces consecutivas este método:
   * primero en el constuctor con valores por defecto, e inmediatamente después con parámetros ad hoc.
   *
   */
  void SetRansacParameters(double probability = 0.99, int minInliers = 8 , int maxIterations = 300, int minSet = 4, float epsilon = 0.4,
                           float th2 = 5.991);

  /**
   * Ejecuta el número máximo de iteraciones Ransac.
   *
   * @param vbInliers Resultado, marca la posición de los elementos inliers encontrados.
   * @param nInliers Restulado, cantidad de inliers encontrados.
   * @returns Pose de la relocalización, o una matriz vacía si no falla.
   *
   * No utilizado en orb-slam2.  En su lugar se usa PnPsolver::iterate.
   */
  cv::Mat find(vector<bool> &vbInliers, int &nInliers);

  /**
   * Ejecuta n iteraciones de Ransac.
   *
   * @param nIterations Cantidad de iteraciones a ejecutar.  Termina antes si se alcanza PnPsolver::mRansacMaxIts en total.
   * @param bNoMore Resultado, señal que indica que se alcanzaron las PnPsolver::mRansacMaxIts iteraciones en total.
   * @param vbInliers Resultado, marca la posición de los elementos inliers encontrados.
   * @param nInliers Restulado, cantidad de inliers encontrados.
   * @returns Pose de la relocalización, o una matriz vacía si no falla.
   *
   * Usado sólo en Tracking::Relocalization.  También se invoca desde el método no utilizado find.
   */
  cv::Mat iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers);

 private:

  /** Reproyecta según la pose calculada y vuelve a marcar como inliers solamente a los que tienen un error menor que maxError.*/
  void CheckInliers();
  /** */
  bool Refine();

  // Functions from the original EPnP code
  /** Función del código original EPnP.*/
  void set_maximum_number_of_correspondences(const int n);
  /** Función del código original EPnP.*/
  void reset_correspondences(void);
  /** Función del código original EPnP.*/
  void add_correspondence(const double X, const double Y, const double Z,
              const double u, const double v);

  /** Función del código original EPnP.*/
  double compute_pose(double R[3][3], double T[3]);

  /** Función del código original EPnP.*/
  void relative_error(double & rot_err, double & transl_err,
              const double Rtrue[3][3], const double ttrue[3],
              const double Rest[3][3],  const double test[3]);

  /** Función del código original EPnP.*/
  void print_pose(const double R[3][3], const double t[3]);
  /** Función del código original EPnP.*/
  double reprojection_error(const double R[3][3], const double t[3]);

  /** Función del código original EPnP.*/
  void choose_control_points(void);
  /** Función del código original EPnP.*/
  void compute_barycentric_coordinates(void);
  /** Función del código original EPnP.*/
  void fill_M(CvMat * M, const int row, const double * alphas, const double u, const double v);
  /** Función del código original EPnP.*/
  void compute_ccs(const double * betas, const double * ut);
  /** Función del código original EPnP.*/
  void compute_pcs(void);

  /** Función del código original EPnP.*/
  void solve_for_sign(void);

  /** Función del código original EPnP.*/
  void find_betas_approx_1(const CvMat * L_6x10, const CvMat * Rho, double * betas);
  /** Función del código original EPnP.*/
  void find_betas_approx_2(const CvMat * L_6x10, const CvMat * Rho, double * betas);
  /** Función del código original EPnP.*/
  void find_betas_approx_3(const CvMat * L_6x10, const CvMat * Rho, double * betas);
  /**
   * Función del código original EPnP.
   *
   * Función final de la siguiente cadena de Invocaciones:
   ORB_SLAM2::PnPsolver::qr_solve(CvMat *, CvMat *, CvMat *) : void
	ORB_SLAM2::PnPsolver::gauss_newton(const CvMat *, const CvMat *, double *) : void
		ORB_SLAM2::PnPsolver::compute_pose(double (*)[3], double *) : double (3 matches)
			ORB_SLAM2::PnPsolver::Refine() : bool
			ORB_SLAM2::PnPsolver::iterate(int, bool &, vector<bool,allocator<bool>> &, int &) : Mat
				ORB_SLAM2::Tracking::Relocalization() : bool
   */
  void qr_solve(CvMat * A, CvMat * b, CvMat * X);

  /** Función del código original EPnP.*/
  double dot(const double * v1, const double * v2);
  /** Función del código original EPnP.*/
  double dist2(const double * p1, const double * p2);

  /** Función del código original EPnP.*/
  void compute_rho(double * rho);
  /** Función del código original EPnP.*/
  void compute_L_6x10(const double * ut, double * l_6x10);

  /** Función del código original EPnP.*/
  void gauss_newton(const CvMat * L_6x10, const CvMat * Rho, double current_betas[4]);
  /** Función del código original EPnP.*/
  void compute_A_and_b_gauss_newton(const double * l_6x10, const double * rho,
				    double cb[4], CvMat * A, CvMat * b);

  /** Función del código original EPnP.*/
  double compute_R_and_t(const double * ut, const double * betas,
			 double R[3][3], double t[3]);

  /** Función del código original EPnP.*/
  void estimate_R_and_t(double R[3][3], double t[3]);

  /** Función del código original EPnP.*/
  void copy_R_and_t(const double R_dst[3][3], const double t_dst[3],
		    double R_src[3][3], double t_src[3]);

  /** Función del código original EPnP.*/
  void mat_to_quat(const double R[3][3], double q[4]);


  /** Parámetros de cámara.*/
  double uc, vc, fu, fv;

  /** */
  double * pws, * us, * alphas, * pcs;
  /** */
  int maximum_number_of_correspondences;
  /** */
  int number_of_correspondences;

  /** */
  double cws[4][3], ccs[4][3];
  /** */
  double cws_determinant;

  /**
   * Puntos 3D macheados recibidos por el constructor.
   *
   * Esta propiedad apenas se usa un par de veces para medir su tamaño.
   * Su contenido es volcado a mvP3Dw por el constructor, directamente desde el parámetro.
   * mvP3Dw es un vector de puntos 3D con las posiciones de los mappoints.
   */
  vector<MapPoint*> mvpMapPointMatches;

  /** Coordenadas 2D de los puntos singulares.*/
  // 2D Points
  vector<cv::Point2f> mvP2D;

  /** Factor de escala al cuadrado, correspondiente al punto 2D.*/
  vector<float> mvSigma2;

  /** Coordenadas 3D de los puntos 3D.*/
  // 3D Points
  vector<cv::Point3f> mvP3Dw;

  /** Índices de los puntos singulares.*/
  // Index in Frame
  vector<size_t> mvKeyPointIndices;

  // Current Estimation
  /** */
  double mRi[3][3];
  /** */
  double mti[3];
  /** */
  cv::Mat mTcwi;
  /** Marca de inlier.*/
  vector<bool> mvbInliersi;
  /** Cantidad de inliers.*/
  int mnInliersi;

  /** Cantidad de iteraciones Ransac realizadas.*/
  // Current Ransac State
  int mnIterations;

  /** Marca los mejores inliers.*/
  vector<bool> mvbBestInliers;

  /** Cantidad de "mejores inliers".*/
  int mnBestInliers;

  /** Mejor pose.*/
  cv::Mat mBestTcw;

  /** Pose refinada.*/
  // Refined
  cv::Mat mRefinedTcw;

  /** Marca los inliers refinados.*/
  vector<bool> mvbRefinedInliers;

  /** Cantidad de inliers refinados.*/
  int mnRefinedInliers;

  /** Cantidad de correspondencias.*/
  // Number of Correspondences
  int N;

  /** Índice de las correspondencias en los vectores apareados de puntos 3D y demás.*/
  // Indices for random selection [0 .. N-1]
  vector<size_t> mvAllIndices;

  /** Mínima probablidad de acierto aceptada.*/
  // RANSAC probability
  double mRansacProb;

  /** Mínima cantidad de inliers aceptados.*/
  // RANSAC min inliers
  int mRansacMinInliers;

  /** Máxima cantidad de iteraciones permitidas para encontrar la solución.  Si la alcanza, desiste.*/
  // RANSAC max iterations
  int mRansacMaxIts;

  /** */
  // RANSAC expected inliers/total ratio
  float mRansacEpsilon;

  /** */
  // RANSAC Threshold inlier/outlier. Max error e = dist(P1,T_12*P2)^2
  float mRansacTh;

  /** Cantidad mínima de puntos a considerar en cada iteración.*/
  // RANSAC Minimun Set used at each iteration
  int mRansacMinSet;

  /** Máximo error de reproyección según el nivel de escala de la pirámide.*/
  // Max square error associated with scale level. Max error = th*th*sigma(level)*sigma(level)
  vector<float> mvMaxError;

};

} //namespace ORB_SLAM

#endif //PNPSOLVER_H
