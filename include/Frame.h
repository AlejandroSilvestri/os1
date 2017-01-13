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

#ifndef FRAME_H
#define FRAME_H

#include <vector>

#include "MapPoint.h"
#include "../Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "../Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
//#include "KeyFrame.h"
//#include "ORBextractor.h"

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;
class ORBextractor;

/**
 * Frame representa un cuadro, una imagen, con los puntos singulares detectados.
 *
 * Frame representa una toma de cámara, con su imagen, los puntos 2D detectados, sus descriptores, su mapeo en 3D, su pose y demás.
 *
 * La posición del cuadro en el mapa se obtiene con Frame::GetCameraCenter.  La orientación con Frame::GetRotationInverse.
 *
 * Tracking utiliza 3 Frames:
 * - Frame::mCurrentFrame
 * - Frame::mInitialFrame
 * - Frame::mLastFrame
 *
 * Cada cuadro tiene su propia matriz K de calibración.
 * Usualmente es la misma matriz (los mismos valores) para todos los cuadros y keyframes.
 *
 * El constructor clona el Mat K, haciendo una copia propia en Frame::mK.
 * Luego analiza la imagen, pero no la guarda.  La imagen se pierde cuando el constructor termina.
 * Parte de este análisis consiste en detectar puntos singulares, extraer sus descriptores y clasificarlos.
 *
 * Puntos singulares, descriptores, BoW y puntos 3D rastreados se registran en vectores paralelos y se explican en Frame::N.
 *
 * El sistema de coordenadas y el significado de las matrices de posición se explica en Frame::mTcw.
 *
 * Esta clase no determina la pose del cuadro.  Su pose es registrada por Tracking y Optimizer.
 *
 * \sa SetPose
 */
class Frame
{
public:
	/**
	 * El constructor sin argumentos crea un Frame sin inicialización.
	 * El único dato inicializado es nNextId=0.
	 * No usado.
	 */
    Frame();

    /**
     * Constructor de copia, clona un frame.
     *
     */
    // Copy constructor.
    Frame(const Frame &frame);

    /**
     * Constructor que crea un Frame y lo llena con los argumentos.
     * @param timeStamp Marca de tiempo, para registro.  ORB-SLAM no la utiliza.
     * @param extractor Algoritmo extractor usado para obtener los descriptores.  ORB-SLAM utiliza exclusivamente el extractor BRIEF de ORB.
     *
     * Se distinguen dos modos de cámara: normal si se proporcionan los coeficientes de distorsión, o fisheye sin coeficientes si se proporciona noArray().
     *
     * Invocado sólo desde Tracling::GrabImageMonocular.
     */
    // Constructor for Monocular cameras.
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);
    //Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor, ORBVocabulary* voc, const float &thDepth);

    /**
     * Procede con la extracción de descriptores ORB.
     *
     * @param flag false para monocular, o para cámara izquierda.  true para cámara derecha.  Siempre se invoca con false.
     * @param im Imagen sobre la que extraer los descriptores.
     *
     * Los descriptores se conservan en Frame::mDescriptors.
     *
     * Invocado sólo desde el constructor.
     */
    // Extract ORB on the image. 0 for left image and 1 for right image.
    void ExtractORB(int flag, const cv::Mat &im);

    /**
     * Computa BoW para todos los descriptores del cuadro.
     * Los guarda en la propiedad mBowVec, que es del tipo BowVector.
     */
    // Compute Bag of Words representation.
    void ComputeBoW();

    /**
     * Registra la pose.
     *
     * Usado por varios métodos para establecer o corregir la pose del cuadro.
     *
     * Luego de establecer la nueva pose se recalculan sus diferentes representaciones con UpdatePoseMatrices, como el vector traslación o la matriz rotación.
     *
     * @param Tcw Nueva pose, matriz de rototraslación en coordenadas homogéneas, de 4x4.
     *
     * Registra la pose en Frame::mTcw, que correspone a la pose del origen del mundo en el sistema de coordenadas de la cámara.
     *
     * Este método se invoca por Tracking con posiciones aproximadas para inicializar y rastrear,
     * y por Optimizer::PoseOptimization, el único que registra la pose optimizada.
     */
    // Set the camera pose.
    void SetPose(cv::Mat Tcw);

    /**
     * Calcula las matrices de posición mRcw, mtcw y mOw a partir de la pose mTcw.
     * Estas matrices son una manera de exponer la pose, no se utilizan en la operación de ORB-SLAM.
     * UpdatePoseMatrices() extrae la inforamción de mTcw, la matriz que combina la pose completa.
     *
     */
    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    /** Devuelve el vector de la posición de la cámara, el centro de la cámara.*/
    // Returns the camera center.
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    /**
     * Devuelve la orientación en el mapa.
     *
     * Es la inversa de la rotación mRwc.
     *
     * @returns mRwc.t()
     *
     * No dispone de GetRotation para devolver la rotación sin invertir mRcw.
     */
    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    /**
     * Indica si un determinado punto 3D se encuentra en el subespacio visual (frustum) del cuadro.
     * El subespacio visual es una pirámide de base cuadrilátera, cuyos vértices son los del cuadro pero antidistorsionados.
     * viewingCosLimit es una manera de limitar el alcance del frustum.
     */
    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    /**
     * Calcula las coordenadas de la celda en la grilla, a la que pertenece un punto singular.
     * Informa las coordenadas en los argumentos posX y posY pasados por referencia.
     * Devuelve true si el punto está en la grilla, false si no.
     * @param kp Punto singular "desdistorsionado".
     * @param posX Coordenada X de la celda a la que pertence el punto.
     * @param posY Coordenada Y de la celda a la que pertence el punto.
     *
     */
    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    /**
     * Selecciona los puntos dentro de una ventana cuadrada de centro x,y y lado r.
     * Recorre todos los niveles del frame filtrando los puntos por coordenadas.
     * Se utiliza para reducir los candidatos para macheo.
     */
    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

public:
	/** Vocabulario BOW para clasificar descriptores.*/
    // Vocabulary used for relocalization.

    /** "Algoritmo" para extraer los descriptores.  El framework permite al programador probar diferentes extractores; ORB-SLAM utiliza solamente ORBextractor.*/
    ORBVocabulary* mpORBvocabulary;

    /** Extractor usado para extraer descriptores.*/
    // Feature extractor. The right is used only in the stereo case.
    ORBextractor* mpORBextractorLeft;//, *mpORBextractorRight;

    /**
     * Time stamp de la captura de la imagen.
     * Sólo para propósitos de registro e investigación, el algoritmo no lo utiliza.
     */
    // Frame timestamp.
    double mTimeStamp;

	/**
	 * Matriz K intrínseca, de cámara.
	 * Coeficientes de distorsión mDistCoef, parámetros intrínsecos fx, fy, cx, cy.
	 */
    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;

	/** Parámetros intrínsecos fx, fy, cx, cy.*/
	/** Parámetro intrínseco.*/
    static float fx;
	/** Parámetro intrínseco.*/
    static float fy;
	/** Parámetro intrínseco.*/
    static float cx;
	/** Parámetro intrínseco.*/
    static float cy;
	/** Inversa del parámetro intrínseco.*/
    static float invfx;
	/** Parámetro intrínseco.*/
    static float invfy;

	/** Coeficientes de distorsión de cámara mDistCoef.*/
    cv::Mat mDistCoef;

    /** Modo de cámara, 0 normal, 1 fisheye sin distorsión.*/
    int camaraModo;

    /** No usado en monocular.*/
    // Stereo baseline multiplied by fx.
    float mbf;

    /** No usado en monocular.*/
    // Stereo baseline in meters.
    float mb;

    /** No usado en monocular.*/
    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;

	/**
	 * Cantidad de puntos singulares.
	 *
	 * Es el tamaño de los vectores apareados:
	 * - mvKeys
	 * - mvKeysUn
	 * - mDescriptors
	 * - mBowVec
	 * - mFeatVec
	 * - mvpMapPoints
	 * - mvbOutlier
	 *
	 *
	 * Todos éstos se pasan al keyframe.
	 */
    // Number of KeyPoints.
    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    /**
     * Vector de puntos singulares obtenidos por el detector, tal como los devuelve opencv.
     *
     * Sus coordenadas están en píxeles, en el sistema de referencia de la imagen.
     */
    std::vector<cv::KeyPoint> mvKeys;

	/**
	 * Vector de puntos antidistorsionados, mvKeys corregidos según los coeficientes de distorsión.
	 *
	 * Este vector está apareado con mvKeys, ambos de tamaño N.
	 *
	 * Sus coordenadas están en píxeles, en el sistema de referencia de la imagen antidistorsionada.
	 * Los puntos se obtienen con cv::undistortPoints, reaplicando la matriz K de cámara.
	 */
    std::vector<cv::KeyPoint> mvKeysUn;

    /** -1 para monocular.  Se pasa en el constructor de copia de Frame.*/
    //std::vector<float> mvDepth;

    // Bag of Words Vector structures.
    /** Vector BoW correspondiente a los puntos singulares.*/
    DBoW2::BowVector mBowVec;
    /** Vector "Feature" correspondiente a los puntos singulares.*/
    DBoW2::FeatureVector mFeatVec;

	/** Descriptores ORB en el formato Mat, tal como los devuelve opencv.  mDescritorRight no se usa, se pasa en el constructor de copia.*/
    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors;//, mDescriptorsRight;

	/** Vector de puntos 3D del mapa asociados a los puntos singulares.
	Este vector tiene la misma longitud que mvKeys y mvKeysUn.
	Las posiciones sin asociación son NULL.
	*/
    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<MapPoint*> mvpMapPoints;

	/** Flag que indica si hay outliers asociados en mvpMapPoints.*/
    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;

	/** mfGridElementWidthInv es la inversa del ancho de la celda en píxeles.
	 * Los puntos de la imagen antidistorsionada se dividen en celdas con una grilla de FRAME_GRID_COLS por FRAME_GRID_ROWS,
	 * para reducir la complejidad del macheo.
	 */
    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;

	/** mfGridElementHeightInv es la inversa del altoo de la celda en píxeles.
	 * Los puntos de la imagen antidistorsionada se dividen en celdas con una grilla de FRAME_GRID_COLS por FRAME_GRID_ROWS,
	 * para reducir la complejidad del macheo.
	 */
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

	/**
	 * Pose de la cámara.
     * Matriz de 4x4 de rototraslación en coordenadas homogéneas.
     * Es cv:Mat(4,4,CV_32F), y sus elementos son float.
     *
     * La posición de la cámara (vector traslación) se obtiene de la última columna de esta matriz.
     *
     * La unidad de medida se establece durante la inicialización igual a la profundidad media de la escena.
     * Esto significa que la unidad dependerá de los puntos 3D triangulados en la inicialización.
     *
     * El sistema de coordenadas 3D de una cámara se relaciona con el 2D de su proyección
     * manteniendo paralelos los ejes X e Y, y estableciendo Z hacia adelante.
     * De este modo X apunta a la derecha e Y apunta hacia abajo (en esto es contrario al sistema de coordenadas estándar).
     * Los vectores 3D homogéneos tienen la disposición tradicional:
     *
     * V = [vx, vy, vz, 1]t
     *
     * Las poses son matrices de 4x4 como ésta, sus subíndices indican referencia y sujeto.  Tcw es T respecto de cámara, de world.
     * Las poses se combinan así:
     *
     * Tca = Tba * Tcb
     *
     * Su valor se actualiza mediante SetPose, que además extrae la matriz rotación y el vector traslación con UpdatePoseMatrices.
     * Estos datos extraídos se calculan para presentación, pero no son usados por el algoritmo, que sólo utiliza mTcw.
     *
     * Frame no calcula mTcw.  Esta matriz es calculada y pasada al cuadro con SetPose, en:
     * - Tracking, copiando de un cuadro anterior, inicializando con la pose cartesiana, o estimando por modelo de movimiento.
     * - Optimizer::PoseOptimization, invocado desde varios métodos de Tracking.  Aquí es donde se hace el verdadero cálculo de pose.
     * -
	 */
    // Camera pose.
    cv::Mat mTcw;

	/** Id incremental con el valor para el frame siguiente.
	 * Es una variable de clase que lleva la cuenta del número de id.
	 * El id de un Frame se asigna con
	 *     mnId=nNextId++;
	 */
    // Current and Next Frame id.
    static long unsigned int nNextId;

	/** Id incremental que identifica a este frame.*/
    long unsigned int mnId;

	/** KeyFrame de referencia.*/
    // Reference Keyframe.
    KeyFrame* mpReferenceKF;

	/** Cantidad de niveles en la pirámide.*/
    // Scale pyramid info.
    int mnScaleLevels;

	/** Factor de escala entre niveles de la pirámide.*/
    float mfScaleFactor;

    /** Factor de escala logarítmico.*/
    float mfLogScaleFactor;

	/** Vector de factores de escala de cada nivel de la pirámide.*/
    vector<float> mvScaleFactors;
	/** Inversa de mvScaleFactors.*/
    vector<float> mvInvScaleFactors;
    /** Cuadrado de mvScaleFactos.*/
    vector<float> mvLevelSigma2;
    /** Inversa de mvLevelSigma2.*/
    vector<float> mvInvLevelSigma2;


	/** Vértices de la imagen antidistorsionada: mnMinX, mnMinY, mnMaxX, mnMaxY.*/
    // Undistorted Image Bounds (computed once).
    static float mnMinX;
	/** Vértices de la imagen antidistorsionada: mnMinX, mnMinY, mnMaxX, mnMaxY.*/
    static float mnMaxX;
	/** Vértices de la imagen antidistorsionada: mnMinX, mnMinY, mnMaxX, mnMaxY.*/
    static float mnMinY;
	/** Vértices de la imagen antidistorsionada: mnMinX, mnMinY, mnMaxX, mnMaxY.*/
    static float mnMaxY;

	/**
	 * Esta variable se pone en uno con un argumento del constructor solamente cuando se crea el primer Frame.
	 * true para cálculos de variables de clase, que no cambian luego.
	 */
    static bool mbInitialComputations;

    /**
     * Calcula los puntos singulares de mvKeysUn.
     * Antidistorsiona los puntos detectados, que están en mvKeys, y los guarda en mvKeysUn en el mismo orden.
     * Si no hay distorsión, UndistortKeyPoints retorna rápidamente unificando mvKeysUn = mvKeys en un mismo vector.
     */
    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    /**
     * Antidistorsiona puntos según el modelo de cámara fisheye de proyección equidistante sin coeficientes de distorsión.
     * Usa mK, la matriz intrínseca, para pasar los puntos distorsionados a escala focal, y luego de antidistorsionarlos pasarlos de nuevo a escala de imagen.
     *
     * @param puntos Mat de nx2 float, cada par corresponde a un punto.  Entrada y salida.  Los puntos se corrigen y el resultado se guarda en el mismo lugar.
     *
     * Sólo usado desde Frame::UndistortKeyPoints y Frame::ComputeImageBounds
     */
    void antidistorsionarProyeccionEquidistante(cv::Mat &puntos);

    /**
     * Calcula los vértices del cuadro andistorsionado.
     * Define mnMinX, mnMaxX, mnMinY, mnMaxY.
     * Si no hay distorsión, el resultado es trivial con origen en (0,0).
     *
     * @param imLeft Imagen, solamente a los efectos de medir su tamaño con .rows y .cols .
     *
     * Invocado sólo desde el constructor.
     */
    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    /**
     * Asigna los puntos singulares a sus celdas de la grilla.
     * La imagen de divide en una grilla para detectar puntos de manera más homogénea.
     * Luego de "desdistorsionar" las coordenadas de los puntos singulares detectados,
     * este método crea un vector de puntos singulares para cada celda de la grillam, y lo puebla.
     */
    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();


private:

    // Rotation, translation and camera center

    /**
     * Matriz R de rotación del mundo respecto de la cámara.
     * Se actualiza con UpdatePoseMatrices().
     */
    cv::Mat mRcw;

    /**
     * Vector t de traslación del origen del mundo en el sistema de referencia de la cámara.
     * Se actualiza con UpdatePoseMatrices().
     */
    cv::Mat mtcw;

    /**
     * Matriz de rotación inversa, de la cámara respecto del mundo.
     * Se actualiza con UpdatePoseMatrices().
     */
    cv::Mat mRwc;

    /**
     * Vector centro de cámara, posición de la cámara respecto del mundo.
     *
     * Es privado, se informa con Frame::GetCameraCenter.
     *
     * Matriz de 3x1 (vector vertical).
     * Vector de traslación invertido.  mtcw es el vector traslación del origen del mundo en el sistema de referencia de la cámara.
     * Se actualiza con UpdatePoseMatrices().
     */
    cv::Mat mOw; //==mtwc
};

}// namespace ORB_SLAM

#endif // FRAME_H
