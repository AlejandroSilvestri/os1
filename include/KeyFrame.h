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
 */
class KeyFrame
{
public:
	/**
	 * Constructor que copia los datos del frame que se convierte en keyframe.
	 * @param F Frame de referencia, que se erige en KeyFrame
	 * @param pMap Mapa local donde se encuentra este KeyFrame, necesario solamente para quitar el KeyFrame del mapa.
	 * @param pKFDB Base de datos de keyframes donde se registran todos los keyframes, necesario solamente para quitar este KeyFrame de la base de datos.
	 * Este constructor copia todos los valores del Frame F.
	 *  pMap es un puntero al mapa donde está este keyFrame, y se utiliza sólo para cuando se requiera eliminar del mapa.  Se registra en mpMap.
	 *  pKFDB es la base de datos de keyframes.  Se registra en mpKeyFrameDB.
	 *
	 */
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    // Pose functions
    /** Establece Tcw, la pose del keyframe.*/
    void SetPose(const cv::Mat &Tcw);

    /** Lee Tcw, la pose del keyframe.*/
    cv::Mat GetPose();

    /**Lee Twc, la matriz inversa de la pose.*/
    cv::Mat GetPoseInverse();

    /** Lee el vector centro de cámara, igual al vector traslación para monocular.*/
    cv::Mat GetCameraCenter();

    //cv::Mat GetStereoCenter();

    /** Lee la matriz rotación 3D, obtenida de Tcw.*/
    cv::Mat GetRotation();

    /** Lee el vector traslación, obtenido de Tcw.*/
    cv::Mat GetTranslation();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    /**
     * Conecta el keyframe con otro en el grafo de covisibilidad.
     *
     * @param pKF Keyframe a conectar.
     * @param weight Peso o ponderación de la conexión.
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
     */
    void UpdateConnections();

    /**
     * Actualiza los mejores covisibles.
     *
     *
     * Invocado cada vez que se actualiza el grafo (cada que que se agrega o elimina una conexión).
     */
    void UpdateBestCovisibles();


    std::set<KeyFrame *> GetConnectedKeyFrames();
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);
    int GetWeight(KeyFrame* pKF);

    // Spanning tree functions
    void AddChild(KeyFrame* pKF);
    void EraseChild(KeyFrame* pKF);
    void ChangeParent(KeyFrame* pKF);
    std::set<KeyFrame*> GetChilds();
    KeyFrame* GetParent();
    bool hasChild(KeyFrame* pKF);

    // Loop Edges
    void AddLoopEdge(KeyFrame* pKF);
    std::set<KeyFrame*> GetLoopEdges();

    // MapPoint observation functions
    void AddMapPoint(MapPoint* pMP, const size_t &idx);
    void EraseMapPointMatch(const size_t &idx);
    void EraseMapPointMatch(MapPoint* pMP);
    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);
    std::set<MapPoint*> GetMapPoints();
    std::vector<MapPoint*> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
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
     *  Computa la profundidad de la escena.
     *  @param q
     *  @returns
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
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
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
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    /** Pose de cámara relativa a su padre. */
    cv::Mat mTcp;

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvLevelSigma2;
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

    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
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
    bool mbNotErase;
    bool mbToBeErased;

    /**
     * Flag de eliminación.
     * Cuando se elimina el KeyFrame, en lugar de quitarlo del vector de keyframe del mapa y de KeyFrameDatabase,
     * se pone en true este flag.
     * Todas las actividades que envuelven un keyframe consultan primero este flag antes de proceder.
     *
     */
    bool mbBad;    

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
