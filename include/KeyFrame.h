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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "../Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "../Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "Serializer.h"

#include <mutex>
#include <boost/serialization/access.hpp>

class KeyFrameTriangulacion;


namespace ORB_SLAM2
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

/**
 * Cuadro clave, keyframe.
 * ORB-SLAM identifica algunos cuadros como clave y los agrega al mapa.
 * Un KeyFrame tiene más datos que un Frame.  Al construirse copia los datos del Frame.
 * Los Frame son efímeros, los KeyFrame perduran en el mapa.
 * LocalMapping triangula nuevos puntos del mapa exclusivamente a partir de keyframes, no de frames.
 * Tracking::CreateNewKeyFrame tiene la exclusividad de la creación de keyframes.
 *
 * Matrices
 *
 * - E: matriz esencial, 3x3
 * - F: matriz fundamental, 3x3
 * - H: homografía, 3x3
 * - K: matriz intrínseca, de cámara o de calibración, 3x3
 * - R: matriz de rotación, 3x3, derivada de una pose
 * - T: transformación, matriz homogénea de 4x4, de rototraslación, para expresar poses
 *
 * Subíndices usuales:
 * https://github.com/raulmur/ORB_SLAM2/issues/226
 *
 * - Tcw: "pose respecto de Cámara, de World".  Vc = Tcw . Vw : convierte Vw en coordenadas del mundo en Vc en coordenadas de la cámara.
 * - F21: "matriz fundamental del cuadro 1 respecto de 2"
 *
 * Vectores:
 *
 * - t: traslación, derivada de una pose
 *
 * Origen de coordenadas:
 *
 * El origen de coordenadas por defecto es el mundo, cuyo origen está dado por la pose de la primer cámara,
 * y la escala por la distancia entre las cámaras de inicialización que triangularon los primeros puntos.
 *
 *
 * Grafos
 *
 * Los keyframes se ordenan en tres grafos:
 * - árbol de expansión (spanning tree) mínimo con raíz en el primer keyframe.
 * - grafo de covisibilidad, consistente en ejes ponderados, entre keyframes.
 * - grafo esencial, árbol de expansión con una poda del grafo de covisibilidad
 *
 * El árbol de expansión conecta todos los keyframes sin redundancia.
 * Cada keyframe registra su padre en KeyFrame::mpParent y sus hijos en KeyFrame::mspChildrens.
 * Se administra con:
 * - KeyFrame::AddChild
 * - KeyFrame::EraseChild
 * - KeyFrame::ChangeParent
 * - KeyFrame::GetChilds
 * - KeyFrame::GetParent
 * - KeyFrame::hasChild
 *
 * El grafo de covisibilidad consiste en una conexión desde un keyframe hacia cada keyframe que observa algún punto en común.
 * Las conexiones (ejes del grafo) son ponderadas y se registran en
 * - KeyFrame::mConnectedKeyFrameWeights
 * - KeyFrame::mvpOrderedConnectedKeyFrames
 * - KeyFrame::mvOrderedWeight
 *
 * y se administran con
 * - KeyFrame::AddConnection
 * - KeyFrame::EraseConnection
 * - KeyFrame::GetVectorCovisibleKeyframes
 * - KeyFrame::GetCovisiblesByWight
 *
 * El grafo esencial está formado por el árbol de expansión, las conexiones fuertes de covisibilidad, y las conexiones de bucle.
 * - KeyFrame::GetBestCovisibleKeyFrames
 * - KeyFrame::AddLoopEdge
 * - KeyFrame::GetLoopEdges
 *
 * El grafo esencial se utiliza en Optimizer::OptimizeEssentialGraph.
 */
class KeyFrame
{
public:
	/**
	 * Constructor que copia los datos del frame que se convierte en keyframe.
	 *
	 * @param F Frame de referencia, que se erige en KeyFrame
	 * @param pMap Mapa local donde se encuentra este KeyFrame, necesario solamente para quitar el KeyFrame del mapa.
	 * @param pKFDB Base de datos de keyframes donde se registran todos los keyframes, necesario solamente para quitar este KeyFrame de la base de datos.  Se registra en mpKeyFrameDB.
	 *
	 * Este constructor copia todos los valores del Frame F.
	 * Se invoca sólo desde:
	 * - Tracking::CreateInitialMapMonocular para crear los dos primeros keyframes, a partir de los dos cuadros usados en la triangulación de los primeros puntos del mapa.
	 * - Tracking::CreateNewKeyFrame para crear todos los demás keyframes, siempre a partir del cuadro actual mCurrentFrame.
	 *
	 */
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    // Pose functions
    /**
     * Establece Tcw, la pose del keyframe.
     *
     * @param Tcw Matriz de pose de la cámara respecto del mundo.
     *
     * Este método también calcula otras formas de expresión de la pose:
     * - Twc
     * - Ow
     *
     * Invocado sólo desde:
     * - constructor
     * - optimizaciones y correcciones de pose de BA local
     * - cierre de bucle
     * - al crear el mapa inicial
     *
     */
    void SetPose(const cv::Mat &Tcw);

    /** Lee Tcw, la pose del keyframe. @returns Tcw, la pose.*/
    cv::Mat GetPose();

    /** Lee Twc, la matriz inversa de la pose. @returns Twc, la inversa de la pose.*/
    cv::Mat GetPoseInverse();

    /** Lee el vector centro de cámara, igual al vector traslación para monocular. @returns el vector de posición de la cámara, igual a la traslación.  Es diferente sólo en estéreo.*/
    cv::Mat GetCameraCenter();

    /**
     * Lee la matriz rotación 3D, obtenida de Tcw.
     * @returns R, la matriz rotación 3D de 3x3, parte de la pose.
     *
     * La matriz rotación rota un vector
     */
    cv::Mat GetRotation();

    /** Lee el vector traslación, obtenido de Tcw.*/
    cv::Mat GetTranslation();

    /**
     * Computa BoW para los descriptores del keyframe.
     *
     * Genera los mapas KeyFrame::mBowVec y KeyFrame::mFeatVec, el primero con los BoW y sus pesos,
     * el segundo con la lista de puntos singulares correspondiente a cada BoW.
     *
     * Frame::ComputeBoW hace exactamente lo mismo, pero no se ejecuta para todos los cuadros,
     * solamente para el cuadro actual cuando está perdido y se necesita relocalizar.
     * El constructor de KeyFrame copia los mapas de BoW, y este método evita recomputarlos si casualmente ya se computaron para el cuadro.
     *
     * Es invocado sólo desde:
     * - LocalMapping::ProcessNewKeyFrame
     * - Tracking::CreateInitialMapMonocular
     *
     *
     */
    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    /**
     * Conecta el keyframe con otro en el grafo de covisibilidad.
     * El grafo de covisibilidad es un mapa de pares keyframe y peso KeyFrame::mConnectedKeyFrameWeights.
     *
     * @param pKF Keyframe a conectar.
     * @param weight Peso o ponderación de la conexión.
     *
     * AddConnection agrega otro keyframe al grafo de covisibilidad de este keyframe.
     *
     * Invocado sólo desde KeyFrame::UpdateConnections.
     */
    void AddConnection(KeyFrame* pKF, const int &weight);

    /**
     * Elimina del grafo de covisibilidad la conexión con el keyframe dado.
     *
     * @param pKF Keyframe a desconectar.
     *
     * Invocado sólo desde KeyFrame::SetBadFlag.
     */
    void EraseConnection(KeyFrame* pKF);

    /**
     * Releva la covisibilidad y crea las conexiones en el grafo de covisibilidad.
     *
     * Crea KeyFrame::mvpOrderedConnectedKeyFrames, KeyFrame::mvOrderedWeights y KeyFrame::mConnectedKeyFrameWeights.
     *
     * Recorre todos los mapPoints vistos por este keyframe, y releva todos los otros keyframes que los observan,
     * creando así el mapa de covisibilidad KeyFrame::mConnectedKeyFrameWeights.
     *
     * Se agrega a sí mismo al mapa de cada keyframe relevado, invocando sus respectivos métodos KeyFrame::AddConnection.
     * Este accionar debería garantizar la doble referencia.
     *
     * Es invocado por:
     *
     * - ORB_SLAM2::Tracking::CreateInitialMapMonocular() : void (2 matches), al crear lso primeros keyframes del mapa inicial.
     * - ORB_SLAM2::LocalMapping::ProcessNewKeyFrame() : void, cuando se crea el KeyFrame (de manera asincrónica).
     * - ORB_SLAM2::LocalMapping::SearchInNeighbors() : void.
     * - ORB_SLAM2::LoopClosing::CorrectLoop() : void (3 matches), al cerrar un bucle se debe rehacer el grafo de covisibilidad.
     *
     * Quizás el cierre de bucle podría interferir con una llamada simultánea de parte de LocalMapping.  Por eso emplea mutex.
     *
     * Respecto del posibles solapamiento son SetBadFlag:
     *
     * LoopClosing marca el keyframe con SetNotErase.  Esa marca perdura en SearchInNeighbors, por lo que no hay interferencia posible.
     * ProcessNewKeyFrame crea el keyframe, es imposible que en ese momento se marque como malo.  Tampoco hay interferencia posible.
     *
     */
    void UpdateConnections();

    /**
     * Actualiza los mejores covisibles.
     *
     * El grafo de covisibilidad es un mapa.
     * Este método produce dos vectores alineados:
     * - KeyFrame::mvpOrderedConnectedKeyFrames con los keyframes
     * - KeyFrame::mvOrderedWeights con sus respectivos pesos
     *
     * Sólo usa datos de KeyFrame::mConnectedKeyFrameWeights.
     *
     *
     * Invocado cada vez que se actualiza el grafo (cada que que se agrega o elimina una conexión), desde:
     * - KeyFrame::AddConnection
     * - KeyFrame::EraseConnection
     */
    void UpdateBestCovisibles();


    /**
     * Devuelve un conjunto de keyframes conectados a éste por el grafo esencial.
     */
    std::set<KeyFrame *> GetConnectedKeyFrames();

    /** Devuelve el vector de keyframes covisibles KeyFrame::mvpOrderedConnectedKeyFrames .*/
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();

    /**
     * Devuelve los primeros y mejores N elementos del vector de keyframes covisibles KeyFrame::mvpOrderedConnectedKeyFrames.
     * @param N Cantidad máxima de keyframes a devolver.
     *
     * Ordena las conexiones por peso.
     */
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);

    /** Devuelve los mejores elementos del vector de keyframes covisibles, con peso mejor al de referencia.  @param w Peso de referencia.*/
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);

    /** Informa el peso de la conexión de un determinado keyframe covisible. @param pKF Keyframe cuyo peso se quiere conocer.  @returns Peso del keyframe.*/
    int GetWeight(KeyFrame* pKF);

    // Spanning tree functions

    /**
     * Agrega un keyframe hijo al grafo de keyframes KeyFrame::mspChildrens.
     * @param pKF Keyframe hijo a agregar.
     *
     * Invocado sólo desde
	 * - ORB_SLAM2::KeyFrame::ChangeParent
	 * - ORB_SLAM2::KeyFrame::UpdateConnections
     */
    void AddChild(KeyFrame* pKF);

    /** Borra un keyframe hijo del grafo KeyFrame::mspChildrens. @param pKF Keyframe hijo a borrar.*/
    void EraseChild(KeyFrame* pKF);

    /**
     * Cambia el padre del keyframe en el grafo.
     *
     * @param pKF Nuevo keyframe padre.
     *
     * Este método cambia el valor de KeyFrame::mpParent, y se registra como hijo del nuevo padre.
     *
     * No se desregistra del viejo padre, que usualmente dejó de existir.
     *
     * Invocado sólo desde ORB_SLAM2::KeyFrame::SetBadFlag(), al remendar el grafo.
     */
    void ChangeParent(KeyFrame* pKF);

    /** Devuelve los hijos de este keyframe. @returns Keyframes hijos.*/
    std::set<KeyFrame*> GetChilds();

    /** Informa el keyframe padre.  @returns Keyframe padre.*/
    KeyFrame* GetParent();

    /** Indica si un keyframe es hijo de éste.  @param pKF Keyframe presunto hijo.  @returns true si es hijo, false si no.*/
    bool hasChild(KeyFrame* pKF);

    // Loop Edges
    /**
     * Agrega un eje al grafo de covisibilidad, a partir de una detección de bucle.
     *
     * @param pKF Keyframe con el que tender el eje.
     *
     * Sólo invocado por LoopClosing::CorrectLoop().
     */
    void AddLoopEdge(KeyFrame* pKF);

    /** Informa los keyframes conectados con ejes de bucle. @returns Keyframes conectados por ejes de bucles.*/
    std::set<KeyFrame*> GetLoopEdges();

    // MapPoint observation functions
    /**
     * Agrega un punto 3D a la lista de puntos observados.
     *
     * @param pMP Punto 3D a agregar al keyframe.
     * @param idx Índice del punto en el vector de puntos 3D, también índice del punto singular asociado.
     *
     *
     */
    void AddMapPoint(MapPoint* pMP, const size_t &idx);

    /**
     * Reemplaza el punto del índice por NULL en el vector mvpMapPoints.
     *
     * @param idx Índice del punto a eliminar.
     *
     * Es una función final, se limita a reemplazar el elemento indicado por NULL.
     *
     * Invocada sólo pro MapPoint::Replace y MapPoint::SetBadFlag.
     *
     */
    void EraseMapPointMatch(const size_t &idx);

    /**
     * Quita el punto de la lista de puntos observados por este keyfame.
     * Reemplaza el punto por NULL en el vector mvpMapPoints.
     *
     * @param pMP Punto 3D a eliminar del vector de puntos 3D observados.
     *
     * Si no lo encuentra no hace nada.
     * Para borrar le punto, el keyframe debe poder encontrarse en sus observaciones.  Si no se encuentra, el punto no se borra.
     *
     * Es una función final, no procura coherencia con otros contenedores.
     *
     * Invocado sólo desde ORB_SLAM2::Optimizer::LocalBundleAdjustment para eliminar outliers.
     */
    void EraseMapPointMatch(MapPoint* pMP);

    /**
     * Reemplaza un elemento de los puntos observador por un nuevo punto 3D.
     *
     * @param idx Índice del punto a reemplazar.
     * @param pMP Nuevo punto que reemplaza al anterior.
     *
     */
    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);

    /**
     * Devuelve los puntos 3D observados por el keyframe.
     * Recorre el vector KeyFrame::mvpMapPoints y genera un set, copiando los elementos no NULL.
     * @returns set de puntos 3D.
     */
    std::set<MapPoint*> GetMapPoints();

    /**
     * Devuelve una copia del vector de puntos 3D.
     *
     * @returns Copia de KeyFrame::mvpMapPoints.
     *
     * Es ampliamente utilizado.
     */
    std::vector<MapPoint*> GetMapPointMatches();


    /**
     * Indica la cantidad de puntos 3D que tengan como mínimo una cantidad dada de observaciones.
     *
     * @param minObs Cantidad mínima de observaciones
     * @returns Cantidad de puntos 3D con al menos esa cantidad mínima de observaciones.
     */
    int TrackedMapPoints(const int &minObs);

    /**
     * Devuelve el punto 3D a partir del índice.
     *
     * @param idx Índice para el vector KeyFrame::mvpMapPoints.
     * @returns Punto 3D.
     */
    MapPoint* GetMapPoint(const size_t &idx);

    // KeyPoint functions

    /**
     * Filtra puntos singulares que estén en el cuadrado de centro x,y y lado 2 r.
     *
     * @param x Coordenada del centro del cuadrado.
     * @param y Coordenada del centro del cuadrado.
     * @param r Longitud de medio lado del cuadrado.
     * @returns Vector de índices de los puntos singulares en el área cuadrada.
     */
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    //cv::Mat UnprojectStereo(int i);

    // Image

    /**
     * Verifica si la coordenada está dentro del área de la imagen.
     * @param x Coordenada x.
     * @param y Coordenada y.
     * @returns true si la coordenada está dentro del área de la imagen, false si no.
     */
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes

    /**
     * Previene la eliminación del keyframe mientras se utiliza en una detección de bucle.
     *
     * Hace mbNotErase = true.
     *
     * LoopClosing previene el borrado de los keyframes considerados marcándolos con KeyFrame::SetNotErase antes de procesarlos,
     * y liberándolos al final con KeyFrame::SetErase.
     *
     * Invocado sólo desde LoopClosing::DetectLoop y LoopClosing::ComputeSim3.
     * DetectLoop lo invoca sólo sobre mpCurrentKF
     */
    void SetNotErase();

    /**
     * Libera el keyframe previamente marcado con KeyFrame::SetNotErase.
     *
     * Hace mbNotErase = false, sólo si el keyframe no está conectado por ejes de bucle a otros keyframes.
     *
     * Aprovecha el momento y verifica si el keyframe está marcado para borrado, en cuyo caso invoca SetBadFlag.
     *
     * SetErase es un paso anterior a SetBadFlag, invocado solamente desde LoopClosing.
     * Invocado repetidamente sólo desde LoopClosing::DetectLoop y LoopClosing::ComputeSim3 para liberar el keyframe.
     *
     * DetectLoop sólo lo invoca sobre mpCurrentKF.
     *
     * Estos dos métodos marcan keyframes para no borrar y los liberan al final con KeyFrame::SetErase,
     * excepto que el keyframe termine formando parte de un cierre de bucle, en cuyo caso quedará marcado para siempre.
     */
    void SetErase();

    // Set/check bad flag
    /**
     * Intenta eliminar el keyframe y marcarlo como malo.  Con esta marca los algoritmos lo ignorarán.
     *
     * Se se permite eliminar el keyframe:
     * - se retira del mapa
     * - se retira de KeyFrameDatabase
     * - se remienda el grafo de keyframes eligiendo nuevos padres para los hijos.
     * - se quita de las observaciones de cada MapPoint de mvpMapPoints
     * - se marca mbBad
     *
     *
     * El objeto en sí no se elimina nunca, aunque debería.  No se libera porque no tiene modo de asegurar que nadie lo está accediendo.
     * Una manera sería crear un vector de keyframes huérfanos con un timestamp de su deceso, y comparar con timestamps de hilos que podrían usarlo.
     *
     * El flag mbBad sirve cuando se accede a un keyframe que justo en ese momento se está borrando desde otro hilo.
     *
     * Si se previene la eliminación de ese keyframe (mbNotErase == true), se marca para su eliminación inmediata apenas se libere:
     * KeyFrame::Erase libera el keyframe, verifica la marca mbToBeErase, y si es true vuelve a invocar SetBadFlag.
     * Este sistema de prevención de eliminación se usa solamente durante el proceso de detección de bucles, de manera temporal.
     * Cuando un keyframe queda enganchado en un bucle, este sistema prevendrá su eliminación para siempre.
     *
     * No tiene efecto sobre el keyframe inicial, con id 0.
     *
     *
     * SetBadFlag bloquea mMutexConnections y mMutexFeatures, y marca mbBad antes de desbloquear.
     * Curiosamente los métodos que utilizan este lock no consultan la marca mbBad antes de proceder.
     * Esto podría explicar el bug por el que perduran en el grafo algunos keyframes marcados como malos.
     *
     *
     * Invocado sólo por LocalMapping::KeyFrameCulling y KeyFrame::SetErase.
     *
     * TODO:
     * Convendría realizar la eliminación del keyframe del mapa antes que nada, de manera reducir la probabilidad de que otros hilos lo encuentren.
     * Del mismo modo, se debería marcar como Bad desde el principio.
     *
     */
    void SetBadFlag();

    /**
     * Consulta la marca mbBad.
     * true si es malo.
     * Cuando el keyframe es malo, se lo ignora sin excepciones en orb-slam2.
     * Equivaldría a eliminar la instancia, pero eso puede romper algún puntero efímero.
     * La serialización es un buen momento para quitar los malos.
     *
     * Si esta marca es true, el keyframe se ha eliminado de Map, KeyFrameDatabase, de las observaciones de todos los MapPoint y del spanning tree.
     *
     */
    bool isBad();

    /**
     *  Computa la profundidad del punto mediano de la escena.
     *  La escena es el conjunto de puntos 3D observador por el keyframe.
     *
     *  La profundidad de un punto 3D respecto de una pose de cámara es la distancia de ese punto al plano paralelo a la imagen de la cámara, que pasa por su foco.
     *
     *  Calcula la profundidad para cada punto del mundo, los ordena por profundidad y devuelve la profundidad del punto 1/q.
     *
     *  Fallaría si no hubiera ningún punto 3D en la escena.
     *
     *  @param q Fraccionador, q>=1, que elige qué profundidad devolver.  1 devuelve la del más lejano, infinito la del más cercano, 2 la mediana.
     *  @returns Profundidad del punto elegido.
     *
     *  Se invoca desde dos lugares, siempre con el argumento q=2.
     *
     *  Desde Tracking::CreateInitialMapMonocular se usa esta profundidad para establecer la unidad de medida del mundo.
     *  Desde LocalMapping::CreateNewMapPoints se usa para descartar la triangulación de un par de keyframes.
     */
    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    /**
     * Compara dos pesos.
     * @param a Peso a.
     * @param b Peso b.
     * @returns a>b.
     */
    static bool weightComp( int a, int b){
        return a>b;
    }

    /**
     * Compara Id.  Menor id, mayor antigüedad.
     * @param pKF1 keyframe 1.
     * @param pKF2 keyframe 2.
     * @returns true si pKF1 es más antiguo que pKF2.
     */
    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }

    /**
     * Reconstruye las observaciones de los puntos observados por el keyframe.
     *
     * Usado exclusivamente en la serialización, para reconstruir datos redundantes.
     *
     * Los keyframes registran los puntos obervados, y éstos registran los keyframes que los observan.
     * Sólo los primeros se serializan, los segundos se reconstruyen con este método.
     *
     * Invocado sólo por Serializer::mapLoad
     */
    void buildObservations();


    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:
    /** Contador para autonumeración de ids.*/
    static long unsigned int nNextId;

    /** Id del keyFrame.  El id es un número de secuencia.*/
    long unsigned int mnId;

    /** Id del frame asociado.  El id es un número de secuencia de los frames.*/
    const long unsigned int mnFrameId;

    /** Time stamp.*/
    const double mTimeStamp;

    /** Dimensiones de la grilla del frame para acelerar macheo.  Están dadas por mnGridCols y mnGridRows.*/
    // Grid (to speed up feature matching)
    const int mnGridCols;

    /** Dimensiones de la grilla del frame para acelerar macheo.  Están dadas por mnGridCols y mnGridRows.*/
    const int mnGridRows;

    /** La inversa del ancho de la grilla en píxeles.  Obtenido del Frame asociado.*/
    const float mfGridElementWidthInv;

    /** La inversa del alto de la grilla en píxeles.  Obtenido del Frame asociado.*/
    const float mfGridElementHeightInv;

    // Variables used by the tracking

    /** Variable efímera usadas por Tracking.*/
    long unsigned int mnTrackReferenceForFrame;

    /** Variable efímera usadas por Tracking.*/
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping

    /** Variable efímera usadas por el mapeo local.*/
    long unsigned int mnBALocalForKF;

    /** Variable efímera usadas por el mapeo local.*/
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    /** Variable efímera usadas por la base de datos de keyframes.*/
    long unsigned int mnLoopQuery;

    /** Variable efímera usadas por la base de datos de keyframes.*/
    int mnLoopWords;

    /** Variable efímera usadas por la base de datos de keyframes.*/
    float mLoopScore;

    /** Variable efímera usadas por por la base de datos de keyframes.*/
    long unsigned int mnRelocQuery;

    /** Variable efímera usadas por la base de datos de keyframes.*/
    int mnRelocWords;

    /** Variable efímera usadas por la base de datos de keyframes.*/
    float mRelocScore;

    // Variables used by loop closing
    /** Variable usada en cierre de bucle.*/
    cv::Mat mTcwGBA;
    /** Variable usada en cierre de bucle.*/
    cv::Mat mTcwBefGBA;
    /** Variable efímera usadas por  cierre de bucle.*/
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    /** Parámetros de calibración.*/
    const float fx, fy, cx, cy, invfx, invfy;//, mbf, mb, mThDepth;

    // Number of KeyPoints
    /**
     * Cantidad de puntos singulares.
     *
     * Tamaño de los vectores alineados KeyFrame::mKeys, KeyFrame::mKeysUn y KeyFrame::mDescriptors.
     */
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)

    /** Puntos singulares visualizados por el keyframe.*/
    const std::vector<cv::KeyPoint> mvKeys;

    /**
     * Puntos singulares con coordenadas "antidistorsionadas".
     *
     * Los elementos se corresponden con los de mvKeys.
     *
     *
     */
    const std::vector<cv::KeyPoint> mvKeysUn;

    /** Descriptores.  Se corresponden con los de mvKeys.*/
    const cv::Mat mDescriptors;

    /** Color de los keypoints.  Para visualización solamente.  Vector cargado en el constructor de keyframe, alineado con mvKeys*/
    vector<cv::Vec3b> vRgb;

    //BoW
    /**
     * Vector de BoW obtenidos de los descriptores del keyframe.  ComputeBoW llena este vector.
     *
     * BowVector es un mapa de Word Id (unsigned int) a Word value (double), que representa un peso.
     *
     * Este peso se utiliza solamente para relocalización y cierre de bucle,
     * siempre a través de ORBVocabulary::score, que compara dos BowVector completos.
     *
     * Word Id es la palabra BoW.
     */
    DBoW2::BowVector mBowVec;

    /**
     * Vector de Features de DBoW2.
     * KeyFrame::ComputeBoW llena este vector.
     *
     * FeatureVector es un mapa de nodos (NodeId, entero) con un vector de índices de puntos singulares asociados.
     *
     * A este vector no se accede por el índice de un punto singular, sino al revés:
     * se accede por NodoId, y se obtienen los índices de todos los puntos singulares con ese NodeId.
     *
     * Cada BoW tiene un NodeId biunívoco.  Pero hay NodeId sin BoW, pues no son hojas del árbol de vocabulario.
     * Sin embargo ORBVocabulary::transform sólo registra nodos hoja, es decir con BoW asociado.
     * mFeacVec y mBowVec tienen el mismo tamaño.  Si bien se pueblan alineados, al ser mapas el orden se altera.
     *
     */
    DBoW2::FeatureVector mFeatVec;

    /**
     * Vector alineado de BoW.
     *
     * Cada elemento es un BoW, correspondiente a un punto singular.
     *
     * DBoW2::WordId es unsigned int.
     *
     * Generado por la versión modificada de KeyFrame::ComputeBow.
     *
     * Usado exclusivamente para puntos lejanos.
     */
    vector<DBoW2::WordId> bows;

    /**
     * Vector alineado de pesos de BoW.
     *
     * Cada elemento es un peso, correspondiente al BoW del punto singular.
     *
     * DBoW2::WordValue es double.
     *
     * Generado por la versión modificada de KeyFrame::ComputeBow.
     *
     * Usado exclusivamente para puntos lejanos.
     */
    vector<DBoW2::WordValue> bowPesos;




    // Pose relative to parent (this is computed when bad flag is activated)
    /**
     * Pose de cámara relativa a su padre.
     *
     * Se computa cuando el keyframe se marca como malo, y no lo usa nadie.
     *
     * En OS1 se desactivó.
     */
    //cv::Mat mTcp;

    // Scale
    /** Cantidad de niveles de la pirámide.*/
    const int mnScaleLevels;

    /** Factor de escala entre dos niveles consecutivos de la pirámide.*/
    const float mfScaleFactor;

    /** Logaritmo del factor de escala.*/
    const float mfLogScaleFactor;

    /** Factores de escala absolutos para cada nivel de la pirámide.*/
    const std::vector<float> mvScaleFactors;

    /** Sigma cuadrado (cuadrado de KeyFrame::mvScaleFactors) para cada nivel de la pirámide.*/
    const std::vector<float> mvLevelSigma2;

    /** Inversa de sigma cuadrado para cada nivel de la pirámide.*/
    const std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    /** Vértices de la imagen antidistorsionada: mnMinX, mnMinY, mnMaxX, mnMaxY.*/
    const int mnMinX;

    /** Vértices de la imagen antidistorsionada: mnMinX, mnMinY, mnMaxX, mnMaxY.*/
    const int mnMinY;

    /** Vértices de la imagen antidistorsionada: mnMinX, mnMinY, mnMaxX, mnMaxY.*/
    const int mnMaxX;

    /** Vértices de la imagen antidistorsionada: mnMinX, mnMinY, mnMaxX, mnMaxY.*/
    const int mnMaxY;

    /**
     * Matriz K de calibración de cámara.
     */
    const cv::Mat mK;


    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // SE3 Pose and camera center

    /**
     * Matriz de 4x4 de rototraslación en coordenadas homogéneas, que expresa la pose del keyframe.
     * SetPose determina su valor, y GetPose lo lee.
     * GetRotation y GetTraslation obtienen la matriz de rotación y el vector traslación en coordenadas euclideanas.
     *
     * Tcw es la transformación de coordenadas del mundo (w) a coordenadas de la cámara (c).
     * Su inversa es Twc.
     */
    cv::Mat Tcw;

    /**
     * Matriz de 4x4 de rototraslación en coordenadas homogéneas, con la transformación inversa de Tcw.
     * Convierte un punto del sistema de referencia de la cámara al mundo.
     * GetPoseInverse lee su valor.
     */
    cv::Mat Twc;

    /**
     * Centro de la cámara, posición de la cámara.
     * Se obtiene con GetCameraCenter.
     * Vector de 3x1.
     */
    cv::Mat Ow;

    /**
     * Puntos del mapa asociado a los puntos singulares.
     */
    // MapPoints associated to keypoints
    std::vector<MapPoint*> mvpMapPoints;

    // BoW
    /** Base de datos de keyframes, donde se encuentra este keyframe.
     * Se recibe como argumento del constructor.
     * Se utiliza exclusivamente en SetBadFlag, para quitar el propio keyFrame de la base de datos.
     * Esa base de datos se consulta con un descriptor BoW para conseguir la lista de keyframes que lo contienen.
     */
    KeyFrameDatabase* mpKeyFrameDB;


    /** Vocabulario BoW para ORB.
     * Representa un diccionario de clasificación que devuelve un BoW a partir de un descriptor ORB.
     */
    ORBVocabulary* mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    /**
     * Mapa de covisibilidad, que vincula los keyframes covisibles con sus pesos.
     * Generado completamente vía KeyFrame::UpdateConnections, que invoca a KeyFrame::AddConnection.
     * Se genera a partir de las observaciones de los puntos del mapa del keyframe.
     * Al generarse se actualiza el mismo mapa de los otros keyframes.
     */
    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;

    /**
     * Keyframes covisibles ordenados por peso, actualizado vía KeyFrame::UpdateBestCovisibles.
     *
     * Este vector está alineado con KeyFrame::mvOrderedWeights, con los respectivos pesos de cada conexión.
     */
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;

    /**
     * Pesos de los keyframes de KeyFrame::mvpOrderedConnectedKeyFrames.
     */
    std::vector<int> mvOrderedWeights;

    // Spanning Tree and Loop Edges
    /**
     * Estado del autómata de inicialización.
     * true al construirse el keyframe, false a partir de que se le agrega la primer conexión,
     * excepto que sea el keyframe de id 0, que no tiene con quién conectarse y esta marca permanece siempre en true.
     */
    bool mbFirstConnection;

    /**
     * KeyFrame padre en el grafo.
     * Los keyframes se construyen con mpParent = NULL,
     * KeyFrame::UpdateConnections adopta la primer conexión como padre.
     * El primer keyframe no tiene padre.
     */
    KeyFrame* mpParent;

    /**
     * KeyFrames hijos en el grafo, que conforman el spanning tree del grado esencial.
     *
     * Se va poblando a medida que otros keyframes identifican a éste como padre.
     *
     * La identificación ocurre en el primer llamado a UpdateConnections apenas se crea el keyframe,
     * que recorre los keyframes con puntos covisibles, y elige como padre al que comparte más puntos.
     *
     * Esto implica que es posible reconstruir este set ejecutando UpdateConnections en cada KeyFrame.
     *
     * mspChildrens se usa con los siguientes métodos:
     * - KeyFrame::AddChild
     * - KeyFrame::EraseChild
     * - KeyFrame::GetChilds
     * - KeyFrame::hasChild
     * - KeyFrame::SetBadFlag
     *
     * El grafo esencial dispone de estos otros métodos para su administración:
     * - KeyFrame::GetParent
     * - KeyFrame::ChangeParent
     * - KeyFrame::UpdateConnections
     *
     */
    std::set<KeyFrame*> mspChildrens;

    /**
     * KeyFrames que participan de un extremo de un bucle.
     *
     * LoopClosing::CorrectLoop es el único método que agrega estos ejes,
     * y que los consume indirectamente invocando Optimizer::OptimizeEssentialGraph.
     *
     * No es efímero.
     *
     * Todos los keyframes de este set están marcados para no borrar con KeyFrame::SetNotErase.
     * Esto significa que no pueden ser malos.
     *
     */
    std::set<KeyFrame*> mspLoopEdges;

    // Bad flags
    /**
     * Señal de no borrar.
     * Se hace true cuando se le agrega un eje al grafo, o con SetNotErase() cuando se usa el keyframe en detección de bucle.
     * Se hace false con SetErase(), sólo si no tiene algún eje de bucle.
     * Los keyframes se marcan para no borrar momentáneamente cuando se consideran en un bucle, y se desmarcan cuando falla el cierre.
     * Si el bucle se cierra, los keyframes invoclucrados quedan marcados para siempre.
     *
     */
    bool mbNotErase;

    /**
     * Marca el keyframe para ser borrado.
     * Los keyframes se construyen con este bit en false.
     * Se pone true solamente con SetBadFlag, cuando mbNotErase es true.
     * Este flag indica que el keyframe está pendiente de borrado, no se pudo borrar porque tenía eje de bucle
     * o estaba siendo usado en una detección de bucle en ese momento.
     *
     * Reconstruíble: true si tiene ejes de bucle.
     */
    bool mbToBeErased;

    /**
     * Flag de eliminación.
     *
     * Cuando KeyFrame::SetBadFlag no puede elmimnar un keyframe porque está marcado con KeyFrame::mbSetNotErase,
     * pone en true este flag, para reintentar la eliminación apenas se quite la marca mencionada.
     *
     * Todas las actividades que envuelven un keyframe consultan primero este flag antes de proceder, mediante KeyFrame::IsBad.
     *
     */
    bool mbBad;    

    /** Mapa donde se encuentra el keyFrame.
     * Se recibe como argumento del constructor.
     * Se utiliza exclusivamente en SetBadFlag, para quitar el propio keyFrame del mapa.
     */
    Map* mpMap;

    /**
     * mutex de acceso a la pose Tcw.
     *
     * KeyFrame::SetPose realiza varias operaciones matriciales.
     *
     * Este mutex se incluye en los métodos que clonan matrices de pose.
     * Las matrices de pose no se deben acceder durante esta operación, pues la clonación como las operaciones matriciales no son monolíticas.
     */
    std::mutex mMutexPose;

    /**
     * mutex de acceso al grafo de conexiones.
     *
     * KeyFrame::UpdateBestCovisibles y KeyFrame::SetBadFlag operan en bucle construyendo o remendando el grafo de conexiones.
     *
     * Durante estas operaciones se deben prevenir modificaciones a variables del grafo.
     * Este mutex está aplicado en todos los métodos que deben esperar la finalización de las operaciones mencionadas.
     *
     *
     *
     * Agregado mío:
     * Para prevenir que el keyframe se siga usando y registrando justo cuando se está marcando como malo con SetBadFlag,
     * los siguientes métodos que utilizan este mutex y registran el keyframe en algún lugar,
     * sólo procederán luego de verificar la marca KeyFrame::mbBad.
     * Los métodos con este agregado son:
     *
     * - KeyFrame::AddChild, evita que el keyframe vuelva al grafo de covisibilidad
     * - KeyFrame::AddConnection, evita que el keyframe vuelva al grafo de conexiones
     * - KeyFrame::UpdateConnections, evita volver a cargar mConnectedKeyFrameWeights, mvpOrderedConnectedKeyFrames y mvOrderedWeights
     *
     * - KeyFrame::UpdateBestCovisibles nunca se invocará si se acaba de ejecutar SetBadFlag
     *
     * No se incluyen los invocados por LoopClosing, pues éstos tienen la potestad de exigir que el KeyFrame no se elimine.
     *
     *
     */
    std::mutex mMutexConnections;

    /**
     * mutex de acceso a mvpMapPoints.
     *
     * Controla el acceso de escritura al vector mvpMapPoints.
     * Previene que se lo modifique mientras se lo copia con KeyFrame::GetMapPointMatches.
     *
     * KeyFrame::SetBadFlag usa este mutex.
     *

     */
    std::mutex mMutexFeatures;

    /**
     * Constructor por defecto para KeyFrame
     * Se ocupa de inicializar los atributos const, para que el compilador no chille.
     * Entre ellos inicializa los atributos no serializables (todos punteros a singletons).
     * Luego serialize se encarga de cambiarle los valores, aunque sean const.
     */
	KeyFrame();
    friend class boost::serialization::access;
	friend class Serializer;

	/**
	 * Serializador para KeyFrame.
	 * Invocado al serializar Map::mspKeyFrames.
	 * No guarda mpKeyFrameDB, que se debe asignar de otro modo.
	 *
	 */
	template<class Archivo> void serialize(Archivo&, const unsigned int);
	// Fin del agregado para serialización

public:
	/**
	 * Functor de comparación para ordenar un set de KeyFrame*, por su mnId.
	 *
	 * Se usa en el mapa de keyframes, para que se serialicen ordenadamente.
	 */
	struct lessPointer{
		bool operator()(const KeyFrame* k1, const KeyFrame* k2) const{
			return k1->mnId < k2->mnId;
		}
	};

};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
