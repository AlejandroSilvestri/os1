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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include "../Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

class LoopClosing;

/**
 * Optimizer concentra todas las operaciones con g2o.
 * Esta clase no tiene propiedades, sino solamente un conjunto métodos estáticos o de clase.
 * Optimizer no se instancia, funciona como un espacio de nombres.
 * Reune las seis funciones implementadas con el framwork g2o, que incluyen bundle adjustment, pose optimization y graph optimization.
 *
 */
class Optimizer
{
public:

	/**
	 * Bundle adjusment sobre los keyframes y puntos el mapa pasados como argumentos.
	 * Toma todos los keyframes y todos los puntos del mapa, para ejecutar un BA.
	 * @param vpKFs Vector de keyframes.  Cada keyframe contiene los puntos 2d visualizados, y su relación con los puntos del mapa.
	 * @param vpMP Vector de puntos del mapa.
	 * @param nIterations Cantidad de iteraciones máximas para el BA.
	 * @param pbStopFlag Señal para forzar la parada del optimizador.
     * @param nLoopKF
     * @param bRobust Señal que solicita un evaluador robusto (que admite outliers) en lugar de uno estricto (que asume que todos los puntos son válidos).
	 *
	 * Este método se invoca solamente desde GlobalBundleAdjustment.
	 *
	 */
	void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);


    /**
     * Ejecuta BundleAdjstment tomando los datos del mapa.
     * Toma todos los keyframes y todos los puntos del mapa, para ejecutar un BA.
     * @param pMap Mapa, de donde tomar todos los keyframes y los puntos.
     * @param nIterations Cantidad de iteraciones máximas para el BA, pasado tal cual a BundleAdjustment.
     * @param pbStopFlag Señal para forzar la parada del optimizador, pasado tal cual a BundleAdjustment.
     * @param nLoopKF
     * @param bRobust Señal que solicita un evaluador robusto (que admite outliers) en lugar de uno estricto (que asume que todos los puntos son válidos).
     *
     * Este método se invoca solamente desde Tracking::createInitialMap al inicio del tracking, cuando el mapa es pequeno.
     */
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);


    /** Bundle adjusment local a partir de un keyframe.
     * @param pKF Keyframe inicial, usualmente Tracking::mpCurrentKeyFrame.
     * @param pbStopFlag Señal para forzar la parada del optimizador.
     *
     * El BA local toma el keyframe de referencia (usualmente el actual).
     * A partir de él forma un vector de keyframes covisibles con GetVectorCovisibleKeyFrames(),
     * y un vector de puntos del mapa vistos por ellos.  Estos keyframes y puntos del mapa serán modificados por el BA.
     * Finalmente crea un vector keyframes fijos (no afectados por el BA), con los otros keyframes que también observan esos puntos.
     * Con estos datos ejecuta un BA usando g2o.
     *
     * Éste es el Bundle adjustment periódico del tracking.
     *
     * Este método se invoca solamente desde LocalMapping::Run().
     *
     * El optimizador se arma así:
     *
    		typedef BlockSolver< BlockSolverTraits< Eigen::Dynamic, Eigen::Dynamic > > BlockSolverX;
        	g2o::SparseOptimizer optimizer;
        	optimizer.setAlgorithm(
        		new g2o::OptimizationAlgorithmLevenberg(
        			new g2o::BlockSolverX(
        				new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>()
        			)
        		)
        	);

     * PoseMatrixType es MatrixXD, es decir de elementos double y dimensiones por definir.
     * Este código significa: optimizador espaciado, con algoritmo LM, solucionador de bloque para poses de n dimensiones
     * y puntos de m dimensiones, usando Cholesky espaciado de la librería Eigen.
     *
     */
    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap);






    /**
     * Cómputo de la pose de un cuadro.
     *
     * @param pFrame
     */
    /**
     * Calcula la pose del frame a partir de los puntos observados y sus posiciones en el mapa.
     * La pose es una matriz de rototraslación de 4x4 en coordenadas homogéneas.
     * Usa la pose del cuadro pFrame->mTcw como estimación inicial, usualmente la estimada por el modelo de movimiento, aunque puede también ser recuperada por relocalización.
     * Éste es el único método que calcula la pose final de un cuadro.  Se invoca para cada cuadro.
     * Las poses de los cuadros son establecidas principalmente por este método, aunque también
     * otros métodos (de Tracking) inicializan la pose, la copian o estiman por modelo de movimiento.
     *
     * @param pFrame Cuadro con macheos contra puntos del mapa, cuya pose se quiere calcular.  La pose se guarda en pFrame->mTcw.  En OrbSlam siempre es el cuadro actual.
     * @returns La cantidad de correspondencias optimizadas (macheos sobrevivientes, inliers).
     *
     * El optimizador se arma así, idéntico al de LocalBundleAdjustment:
     *
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(
    	    new g2o::OptimizationAlgorithmLevenberg(
    		    new g2o::BlockSolverX(
    			    new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>()
    		    )
    	    )
        );
     */
    int static PoseOptimization(Frame* pFrame);





    /** Optimiza el grafo esencial para cerrar un bucle.
     * Al detectar un cierre de bucle, se unen los extremos.
     * Este método realiza un "pose graph optimization"; ejecuta un BA para repartir el error cuadrático medio a lo largo del grafo esencial.
     * @param pMap Mapa completo.
     * @param pLoopKF Keyframe del extremo anterior del bucle.
     * @param pCurKF Keyframe actual, del extremo posterior del bucle.
     * @param Scurw Transformación Sim3 necesaria para adaptar el extremo posterior al anterior.  No utilizado.
     * @param NonCorrectedSim3 KeyframeAndPose de los keyframes actual y vecinos, versión sin transformar sim3.
     * @param CorrectedSim3 KeyframeAndPose de los keyframes vecinos, versión corregida por sim3.
     * @param LoopConnections Conjunto de keyframes conectados gracias al cierre del bucle.
     *
     * Este método se invoca exclusivamente en la etapa final de LoopClosing::CorrectLoop().
     *
     * Carga en el optimizador todos los keyframes del mapa, marcando como fijo solamente al del extremo anterior del bucle.
     * Carga todos los ejes del mapa: las conexiones entre keyframes, agregando las nuevas conexiones del bucle informadas en loopConnections.
     * Ejecuta 20 iteraciones, y vuelca el resultado a las poses de los keyframes y de las posiciones de los puntos, recomputando normal y profundidad.
     *
     *
     * El optimizador se arma así:
     *
    		typedef  BlockSolver< BlockSolverTraits< 7, 3 > > BlockSolver_7_3;
    		g2o::SparseOptimizer optimizer;
    		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
    			new g2o::BlockSolver_7_3(
    				new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>()
    			)
    		);
    		solver->setUserLambdaInit(1e-16);
    		optimizer.setAlgorithm(solver);

     * PoseMatrixType significa que para el hessiano de puntos se usan matrices double de 3x3, y para el hessiano de poses matrices double de 7x7.
     * Este código significa: optimizador espaciado, con algoritmo LM, solucionador de bloque para poses de 7 dimensiones
     * y puntos de 3 dimensiones, usando Cholesky espaciado de la librería Eigen.
     *
     * A diferencia de PoseOptimization y LocalBundleAdjustment, se usa un BlockSolver_7_3 en lugar del BlockSolverX, y se le ajusta el LambdaInit.
     *
     */
    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                       const bool &bFixScale);




    /** Determina la transformación sim3 entre dos keyframes, que mejor explica un macheo.
     * Dados dos keyframes que presumiblemente observan lo mismmo y son candidatos a cierre de bucle,
     * y dada una serie de puntos macheados observados por ambos keyframes,
     * OptimizeSim3() calcula la transformación de similaridad sim3 que compatibiliza ambas visualizaciones y permite cerrar el bucle.
     *
     * ESte método se invoca exclusivamente desde LoopClosing::ComputeSim3().
     *
     * El optimizador se arma así, idéntico al de LocalBundleAdjustment:
     *
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(
        	new g2o::OptimizationAlgorithmLevenberg(
        		new g2o::BlockSolverX(
        			new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>()
        		)
        	)
        );

     *
     */
    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);
};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
