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

#include "MapPoint.h"
#include "../Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "../Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"

#include <mutex>


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
 *
 * - Tcw: "pose de Cámara respecto de World"
 * - F21: "matriz fundamental del cuadro 2 respecto de 1"
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
 *
 */
class KeyFrame
{
public:
	/**
	 * Constructor que copia los datos del frame que se convierte en keyframe.
	 *
	 * @param F Frame de referencia, que se erige en KeyFrame
	 * @param pMap Mapa local donde se encuentra este KeyFrame, necesario solamente para quitar el KeyFrame del mapa.
	 * @param pKFDB Base de datos de keyframes donde se registran todos los keyframes, necesario solamente para quitar este KeyFrame de la base de datos.
	 *
	 * Este constructor copia todos los valores del Frame F.
	 *
	 * pMap es un puntero al mapa donde está este keyFrame, y se utiliza sólo para cuando se requiera eliminar del mapa.  Se registra en mpMap.
	 *
	 * pKFDB es la base de datos de keyframes.  Se registra en mpKeyFrameDB.
	 *
	 */
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    // Pose functions
    /** Establece Tcw, la pose del keyframe.*/
    void SetPose(const cv::Mat &Tcw);

    /** Lee Tcw, la pose del keyframe. @returns Tcw, la pose.*/
    cv::Mat GetPose();

    /** Lee Twc, la matriz inversa de la pose. @returns Twc, la inversa de la pose.*/
    cv::Mat GetPoseInverse();

    /** Lee el vector centro de cámara, igual al vector traslación para monocular. @returns el vector de posición de la cámara, igual a la traslación.  Es diferente sólo en estéreo.*/
    cv::Mat GetCameraCenter();

    //cv::Mat GetStereoCenter();

    /**
     * Lee la matriz rotación 3D, obtenida de Tcw.
     * @returns R, la matriz rotación 3D de 3x3, parte de la pose.
     *
     * La matriz rotación rota un vector
     */
    cv::Mat GetRotation();

    /** Lee el vector traslación, obtenido de Tcw.*/
    cv::Mat GetTranslation();

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
     * Invocado sólo desde KeyFrame::UpdateConnections.
     */
    void AddConnection(KeyFrame* pKF, const int &weight);

    /**
     * Elimina del grafo de covisibilidad la conexión con el keyframe dado.
     *
     * @param pKF Keyframe a desconectar.
     */
    void EraseConnection(KeyFrame* pKF);

    /**
     * Releva la covisibilidad y crea las conexiones en el grafo de covisibilidad.
     *
     * Es invocado de varios módulos, entre ellos desde ProcessNewKeyFrame.
     */
    void UpdateConnections();

    /**
     * Actualiza los mejores covisibles.
     *
     * El grafo de covisibilidad es un mapa.
     * Este método produce dos vectores alineados, KayFrame.mvpOrderedConnectedKeyFrames con los keyframes,
     * y KeyFrame::mvOrderedWeights con sus respectivos pesos.
     *
     *
     * Invocado cada vez que se actualiza el grafo (cada que que se agrega o elimina una conexión).
     */
    void UpdateBestCovisibles();


    /** Devuelve un conjunto de keyframes conectados a éste por el grafo esencial.*/
    std::set<KeyFrame *> GetConnectedKeyFrames();

    /** Devuelve el vector de keyframes covisibles KeyFrame::mvpOrderedConnectedKeyFrames .*/
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();

    /** Devuelve los primeros y mejores N elementos del vector de keyframes covisibles KeyFrame::mvpOrderedConnectedKeyFrames .  @param N Cantidad máxima de keyframes a devolver.*/
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);

    /** Devuelve los mejores elementos del vector de keyframes covisibles, con peso mejor al de referencia.  @param w Peso de referencia.*/
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);

    /** Informa el peso de la conexión de un determinado keyframe covisible. @param pKF Keyframe cuyo peso se quiere conocer.  @returns Peso del keyframe.*/
    int GetWeight(KeyFrame* pKF);

    // Spanning tree functions

    /** Agrega un keyframe hijo al grafo KeyFrame::mspChildrens. @param pKF Keyframe hijo a agregar.*/
    void AddChild(KeyFrame* pKF);

    /** Borra un keyframe hijo del grafo KeyFrame::mspChildrens. @param pKF Keyframe hijo a borrar.*/
    void EraseChild(KeyFrame* pKF);

    /** Cambia el padre del keyframe.  @param pKF Nuevo keyframe padre.*/
    void ChangeParent(KeyFrame* pKF);

    /** Devuelve los hijos de este keyframe. @returns Keyframes hijos.*/
    std::set<KeyFrame*> GetChilds();

    /** Informa el keyframe padre.  @returns Keyframe padre.*/
    KeyFrame* GetParent();

    /** Indica si un keyframe es hijo de éste.  @param pKF Keyframe presunto hijo.  @returns true si es hijo, false si no.*/
    bool hasChild(KeyFrame* pKF);

    // Loop Edges
    /** Agrega un eje al grafo de covisibilidad, a partir de una detección de bucle.  @param pKF Keyframe con el que tender el eje.*/
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

    /** Reemplaza el punto del índice por NULL.  @param idx Índice del punto a eliminar.*/
    void EraseMapPointMatch(const size_t &idx);

    /** Reemplaza el punto por NULL.  @param pMP Punto 3D a eliminar del vector de puntos 3D observados.*/
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
     * Recorre el vector y genera un set, eliminando los índices NULL.
     * @returns Puntos 3D.
     */
    std::set<MapPoint*> GetMapPoints();

    /** Devuelve el vector de puntos 3D.  @returns KeyFrame::mvpMapPoints.*/
    std::vector<MapPoint*> GetMapPointMatches();


    /**
     * Indica la cantidad de puntos 3D que tengan como mínimo una cantidad dada de observaciones.
     *
     * @param minObs Cantidad mínima de observaciones
     * @returns Cantidad de puntos 3D con al menos esa cantidad mínima de observaciones.
     */
    int TrackedMapPoints(const int &minObs);

    /** Devuelve el punto 3D a partir del índice.  @param idx Índice.  @returns Punto 3D.*/
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

    /** Marca el keyframe como imborrable.*/
    void SetNotErase();

    /** Elimina el keyframe.*/
    void SetErase();

    // Set/check bad flag
    /** Marca el keyframe como malo.  Los algoritmos lo ignorarán.*/
    void SetBadFlag();

    /** Consulta la marca mbBad.  true si es malo.*/
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

    /** Variables usadas por tracking.*/
    long unsigned int mnTrackReferenceForFrame;

    /** Variables usadas por tracking.*/
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping

    /** Variables usadas por el mapeo local.*/
    long unsigned int mnBALocalForKF;

    /** Variables usadas por el mapeo local.*/
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    /** Variables usadas por la base de datos de keyframes.*/
    long unsigned int mnLoopQuery;

    /** Variables usadas por la base de datos de keyframes.*/
    int mnLoopWords;

    /** Variables usadas por la base de datos de keyframes.*/
    float mLoopScore;

    /** Variables usadas por la base de datos de keyframes.*/
    long unsigned int mnRelocQuery;

    /** Variables usadas por la base de datos de keyframes.*/
    int mnRelocWords;

    /** Variables usadas por la base de datos de keyframes.*/
    float mRelocScore;

    // Variables used by loop closing
    /** Variable usada en cierre de bucle.*/
    cv::Mat mTcwGBA;
    /** Variable usada en cierre de bucle.*/
    cv::Mat mTcwBefGBA;
    /** Variable usada en cierre de bucle.*/
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    /** Parámetros de calibración.*/
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    /** Cantidad de puntos singulares.*/
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)

    /** Puntos singulares visualizados por el keyframe.*/
    const std::vector<cv::KeyPoint> mvKeys;

    /** Puntos singulares con coordenadas "desdistorsionadas".  Los elementos se corresponden con los de mvKeys.*/
    const std::vector<cv::KeyPoint> mvKeysUn;

    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvDepth; // negative value for monocular points

    /** Descriptores.  Se corresponden con los de mvKeys.*/
    const cv::Mat mDescriptors;

    //BoW
    /** Vector de BoW obtenidos de los descriptores del keyframe.  ComputeBoW llena este vector.*/
    DBoW2::BowVector mBowVec;
    /** Vector de Features de DBoW2.  ComputeBoW llena este vector.*/
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    /** Pose de cámara relativa a su padre. */
    cv::Mat mTcp;

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
     * Tcw es la transformación de coordenadas de la cámara (c) a coordenadas del mundo (w).
     * Su inversa es Twc.
     */
    cv::Mat Tcw;

    /**
     * Matriz de 4x4 de rototraslación en coordenadas homogéneas, con la transformación inversa de Tcw.
     * GetPoseInverse lee su valor.
     */
    cv::Mat Twc;

    /**
     * Centro de la cámara.
     * Se obtiene con GetCameraCenter.
     */
    cv::Mat Ow;

    //cv::Mat Cw; // Stereo middel point. Only for visualization

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

    /** Mapa de covisibilidad, que vincula los keyframes covisibles con sus pesos.  Actualizado vía KeyFrame::UpdateConnections.*/
    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;

    /** Keyframes covisibles ordenados por peso, actualizado vía KeyFrame::UpdateBestCovisibles.*/
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;

    /** Pesos de los keyframes de KeyFrame::mvpOrderedConnectedKeyFrames.*/
    std::vector<int> mvOrderedWeights;

    // Spanning Tree and Loop Edges
    /**
     * Estado del autómata de inicialización.
     * true al construirse el keyframe, false a partir de que se le agrega la siguiente conexión.
     */
    bool mbFirstConnection;

    /**
     * KeyFrame padre en el grafo.
     */
    KeyFrame* mpParent;

    /**
     * KeyFrames hijos en el grafo.
     */
    std::set<KeyFrame*> mspChildrens;

    /**
     * KeyFrames que participan de un extremo de un bucle.
     */
    std::set<KeyFrame*> mspLoopEdges;

    // Bad flags
    /** Señal de no borrar.*/
    bool mbNotErase;

    /** Marca el keyframe para ser borrado.*/
    bool mbToBeErased;

    /**
     * Flag de eliminación.
     * Cuando se elimina el KeyFrame, en lugar de quitarlo del vector de keyframe del mapa y de KeyFrameDatabase,
     * se pone en true este flag.
     * Todas las actividades que envuelven un keyframe consultan primero este flag antes de proceder.
     *
     */
    bool mbBad;    

    /** Dato auxiliar para visualización solamente.*/
    float mHalfBaseline; // Only for visualization

    /** Mapa donde se encuentra el keyFrame.
     * Se recibe como argumento del constructor.
     * Se utiliza exclusivamente en SetBadFlag, para quitar el propio keyFrame del mapa.
     */
    Map* mpMap;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
