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
 *
 * Esta clase no tiene propiedades, sino solamente un conjunto métodos estáticos o de clase.
 * Optimizer no se instancia, funciona como un espacio de nombres.
 *
 * Concentra todas las funciones implementadas con el framework g2o, que incluyen bundle adjustment, pose optimization y graph optimization.
 *
 * Los métodos de bundle adjustment utilizan g2o de la misma manera, descrita en Optimizer::LocalBundleAdjustment.
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
	 * Este método se invoca solamente desde Optimizer::GlobalBundleAdjustment.
	 *
	 *
	 * <h3>Funcionamiento</h3>
	 * El optimizador se arma así, exactamente igual que Optimizer::LocalBundleAdjustment:
     *
    		typedef BlockSolver< BlockSolverTraits<6, 3> > BlockSolver_6_3;
        	g2o::SparseOptimizer optimizer;
        	optimizer.setAlgorithm(
        		new g2o::OptimizationAlgorithmLevenberg(
        			new g2o::BlockSolver_6_3(
        				new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>()
        			)
        		)
        	);

     * BlockSolver_6_3::PoseMatrixType es MatrixXD ?, es decir de elementos double y dimensiones por definir.
     * Este código significa: optimizador espaciado, con algoritmo LM, solucionador de bloque para poses de 6 dimensiones
     * y puntos de 3 dimensiones, usando Cholesky espaciado de la librería Eigen.
     *
     * Vértices:
     * - g2o::VertexSE3Expmap: pose de keyframe
     * - g2o::VertexSBAPointXYZ: posición del mappoint
	 *
	 * Ejes:
	 * - g2o::EdgeSE3ProjectXYZ
	 *   - vértice 0: mappoint
	 *   - vértice 1: keyframe
	 *   - medición: coordenadas del keypoint
	 *   - información: invSigma2 de la octava del keypoint
	 *
	 */
	void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);


    /**
     * Ejecuta BundleAdjustment sobre todo el mapa.
     * Toma todos los keyframes y todos los puntos del mapa, para ejecutar un BA.
     * @param pMap Mapa, de donde tomar todos los keyframes y los puntos.
     * @param nIterations Cantidad de iteraciones máximas para el BA, pasado tal cual a BundleAdjustment.
     * @param pbStopFlag Señal para forzar la parada del optimizador, pasado tal cual a BundleAdjustment.
     * @param nLoopKF
     * @param bRobust Señal que solicita un evaluador robusto (que admite outliers) en lugar de uno estricto (que asume que todos los puntos son válidos).
     *
     * Este método se invoca solamente desde Tracking::CreateInitialMapMonocular al inicio del tracking, cuando el mapa es pequeno,
     * y desde LoopClosing::RunGlobalBundleAdjustment para corregir el mapa luego de cerrar un bucle.
     */
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);


    /**
     * Bundle adjusment local a partir de un keyframe.
     *
     * @param pKF Keyframe inicial, usualmente Tracking::mpCurrentKeyFrame.
     * @param pbStopFlag Señal para forzar la parada del optimizador.
     * @param pMap Mapa del mundo.
     *
     * El BA local toma el keyframe de referencia (usualmente el actual).
     * A partir de él forma un vector de keyframes covisibles con KeyFrame::GetVectorCovisibleKeyFrames(),
     * y un vector de puntos del mapa vistos por ellos.  Estos keyframes y puntos del mapa serán modificados por el BA.
     * Finalmente crea un vector de keyframes fijos (no afectados por el BA), con los otros keyframes que también observan esos puntos.
     * Con estos datos ejecuta un BA usando g2o.
     *
     * Éste es el Bundle adjustment periódico del tracking.
     *
     * Este método se invoca solamente desde LocalMapping::Run().
     *
     * El optimizador se arma así:
     *
        	g2o::SparseOptimizer optimizer;
        	optimizer.setAlgorithm(
        		new g2o::OptimizationAlgorithmLevenberg(
        			new g2o::BlockSolver_6_3(
        				new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>()
        			)
        		)
        	);

     * BlockSolver_6_3::PoseMatrixType es MatrixXD ?, es decir de elementos double y dimensiones por definir.
     * Este código significa: optimizador espaciado, con algoritmo LM, solucionador de bloque de ejes que unen poses de 6 dimensiones
     * y puntos de 3 dimensiones, usando Cholesky espaciado de la librería Eigen.
     *
     * Las poses de los keyframes y las posiciones de los puntos 3D constituyen los vértices:
     *
			g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
			vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
        	vSE3->setId(id del vertex);
        	vSE3->setFixed(true o false);
			optimizer.addVertex(vSE3);

			g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        	vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        	vPoint->setId(id del vertex);
        	vPoint->setMarginalized(true);
        	optimizer.addVertex(vPoint);
     *
     * Las poses de los keyframes se expresan como g2o::VertexSE3Expmap, mapas exponenciales del grupo de simetría del espacio euclideano de 3 dimensiones (que tiene 6 grados de libertad).
     * Se fijan las poses de los keyframes que observan a los puntos locales, sin pertenecer al grafo de covisibilidad.
     *
     * Las poses d elos puntos se expresan como g2o::VertexSBAPointXYZ, puntos xyz para BA espaciado.
     *
     * Los ejes que proyectan uno al otro son g2o::EdgeSE3ProjectXYZ;
     *
     * Los ejes unen keyframes con puntos, un eje para cada observación de cada punto:
     *
			g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id del vertex KeyFrame)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id del vertex MapPoint)));
            e->setMeasurement(coordenadas del punto singular);
            e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);	// invSigma2 correspondiente a la observación de ese punto en ese keyframe
            e->setRobustKernel(new g2o::RobustKernelHuber);
            rk->setDelta(sqrt(5.991));

            e->fx = pKFi->fx;
            e->fy = pKFi->fy;
            e->cx = pKFi->cx;
            e->cy = pKFi->cy;

            optimizer.addEdge(e);

     * La matriz de información es la identidad (2x2) multiplicada por invSigma2,
     * la inversa cuadrada del factor de escala que corresponde al nivel de pirámide en el que se observó el punto.
     * Esto significa que a mayor nivel de la pirámide, más difusa es la posición del punto.
     *
     * Aplica a la observación del punto (el punto singular 2D).
     *
     * El factor de escala se aplica a las coordenadas en píxeles: un factor de escala 1,2 significa un error de 1,2 píxeles en la medición.
     *
     *
     * Vértices:
     * - g2o::VertexSE3Expmap: pose de keyframe
     * - g2o::VertexSBAPointXYZ: posición del mappoint
	 *
	 * Ejes:
	 * - g2o::EdgeSE3ProjectXYZ
	 *   - vértice 0: mappoint
	 *   - vértice 1: keyframe
	 *   - medición: coordenadas del keypoint
	 *   - información: invSigma2 de la octava del keypoint
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
     * El optimizador se arma así:
     *
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(
    	    new g2o::OptimizationAlgorithmLevenberg(
    		    new g2o::BlockSolverX(
    			    new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>()
    		    )
    	    )
        );
     *
     * Único vértice:
     * - g2o::VertexSE3Expmap: pose del frame
     *
     * Ejes de un sólo vértice común:
     * - g2o::EdgeSE3ProjectXYZOnlyPose
     * - g2o::RobustKernelHuber, delta sqrt(5.991)
     * - medición: coordenadas del keypoint
     * - información: invSigma2 de la octava
     * - xW: coordenadas del mappoint
     * - matriz de calibración
     */
    int static PoseOptimization(Frame* pFrame);





    /** Optimiza el grafo esencial para cerrar un bucle.
     * El cierre de bucle se realiza según se describe en el paper Scale Drift-Aware Large Scale Monocular SLAM, Strasdat et al, 2010.
     *
     * La corrección minimizando el error de las transformaciones sim3 se describe en IV.B.Loop Closure Correction (24).
     *
     * Realiza una corrección de pose solamente (en sim3) minimizando la diferencia (o error)
     * entre las transformaciones originales con drifting y las propuestas que cierran el bucle,
     * sin considerar los puntos observados.
     *
     * Al detectar un cierre de bucle, se unen los extremos.
     *
     * Este método realiza un "pose graph optimization"; ejecuta un BA para repartir el error cuadrático medio a lo largo del grafo esencial.
     * @param pMap Mapa completo.
     * @param pLoopKF Keyframe del extremo anterior del bucle.
     * @param pCurKF Keyframe actual, del extremo posterior del bucle.
     * @param NonCorrectedSim3 KeyframeAndPose de los keyframes actual y vecinos, versión sin transformar sim3.
     * @param CorrectedSim3 KeyframeAndPose de los keyframes vecinos, versión corregida por sim3.
     * @param LoopConnections Conjunto de keyframes conectados gracias al cierre del bucle.  Para cada keyframe que cierra el bucle, asocia un conjunto de keyframes asociados del otro lado del bucle.
     * @param bFixScale false para monocular.  Si es true computa sobre 6 grados de libertad, si es false sobre 7 grados de libertad (porque incluye escala).
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
     *
     * Vértices:
     * - g2o::VertexSim3Expmap: pose de keyframe
     *
     * Ejes:
     * - g2o::EdgeSim3: ejes entre dos poses de keyframes
     * - medición: g2o::Sim3, pose relativa entre dos keyframes
     * - información: 1
     */
    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                       const bool &bFixScale);




    /**
     * Depura la transformación sim3 (espacio de transformaciones de similaridad, de 7 dimensiones) entre dos keyframes, que mejor explica un macheo.
     * Dados dos keyframes que presumiblemente observan lo mismmo y son candidatos a cierre de bucle,
     * y dada una serie de puntos macheados observados por ambos keyframes,
     * OptimizeSim3() calcula la transformación de similaridad sim3 que compatibiliza ambas visualizaciones y permite cerrar el bucle.
     *
     * @param pKF1 KeyFrame actual, candidato a cerrar el bucle.
     * @param pKF2 KeyFrame preexistente, el otro extremo el bucle.
     * @param vpMatches1 Puntos 3D observados desde ambos keyframes.
     * @param g2oS12 Estimación inicial y resultado, transformación sim3 que alinea los keyframes.
     * @param th2 Umbral, siempre 10.
     * @param bFixScale Siempre false para monocular.
     *
     * ESte método se invoca exclusivamente desde LoopClosing::ComputeSim3().
     *
     * El optimizador se arma así:
     *
				g2o::SparseOptimizer optimizer;
				optimizer.setAlgorithm(
					new g2o::OptimizationAlgorithmLevenberg(
						new g2o::BlockSolverX(
							new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>()
						)
					)
				);

     *
     * Vértices:
     * - g2o::VertexSim3Expmap: pose sim3 entre ambos keyframe.  Único vertex a optimizar
     * - g2o::VertexSBAPointXYZ: posición de los mappoints
     *
     * Ejes:
     * - g2o::EdgeSim3ProjectXYZ: eje del keyframe a un mappoint
     * - g2o::EdgeInverseSim3ProjectXYZ: eje de un mappoint al keyframe
     *
     *
     * Las poses son g2o::VertexSim3Expmap en lugar de VertexSE3Expmap,
     * pues son poses de similaridad (sim3) en lugar de rototraslación (SE3, Special Euclidean).
     * Expmap son mapas exponenciales.
     *
     * Las posiciones de los puntos son g2o::VertexSBAPointXYZ, como en todos los BA.
     *
     * Los ejes van de a pares, en ambos sentidos, y son de tipos g2o::EdgeSim3ProjectXYZ y g2o::EdgeInverseSim3ProjectXYZ.
     *
     * OptimizeSim3 construye un vertex VertexSim3Expmap, consituido por una transformación de similaridad inicial que explica las poses entre dos keyframes.
     * Es el único vertex a optimizar.
     *
     * El resto de los vertex son fijos, corresponden a los puntos 3D observados desde el pKF2 (el keyframe preexistente).
     *
     * De este modo la transformación resultante es la que se aplicará al keyframe actual para cerrar el bucle.
     *
     *
     *
     */
    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2);//, const bool bFixScale);
};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
