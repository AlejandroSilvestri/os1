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
 * Frame representa una toma de cámara, con su imagen, los puntos 2D detectados, sus descriptores, su mapeo en 3D y demás.
 * Tracking utiliza 3 Frames:
 * - mCurrentFrame
 * - mInitialFrame
 * - mLastFrame
 *
 * Cada cuadro tiene su propia matriz K de calibración.
 * Usualmente es la misma matriz (los mismos valores) para todos los cuadros y keyframes.
 * El constructor clona el Mat K, haciendo una copia propia.
 *
 * El constructor analiza la imagen, pero no la guarda.  La imagen se pierde cuando el constructor termina.
 * Parte de este análisis consiste en detectar puntos singulares, extraer sus descriptores y clasificarlos
 */
class Frame
{
public:
    Frame();

    // Copy constructor.
    Frame(const Frame &frame);

    // Constructor for stereo cameras.
    //Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for RGB-D cameras.
    //Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for Monocular cameras.
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);
    //Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor, ORBVocabulary* voc, const float &thDepth);

    // Extract ORB on the image. 0 for left image and 1 for right image.
    void ExtractORB(int flag, const cv::Mat &im);

    // Compute Bag of Words representation.
    void ComputeBoW();

    // Set the camera pose.
    void SetPose(cv::Mat Tcw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    /** Devuelve el vector de la posición de la cámara, el centro de la cámara.*/
    // Returns the camera center.
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    /** Devuelve la inversa de la rotación.*/
    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    //void ComputeStereoMatches();

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    //void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    //cv::Mat UnprojectStereo(const int &i);

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

	/** Cantidad de puntos singulares.*/
    // Number of KeyPoints.
    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    /** Vector de puntos singulares obtenidos por el detector, tal como los devuelve opencv.*/
    std::vector<cv::KeyPoint> mvKeys/*, mvKeysRight*/;

	/** Vector de puntos antidistorsionados, mvKeys corregidos según los coeficientes de distorsión.*/
    std::vector<cv::KeyPoint> mvKeysUn;

    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    /** -1 para monocular.*/
    std::vector<float> mvuRight;
    /** -1 para monocular.*/
    std::vector<float> mvDepth;

    // Bag of Words Vector structures.
    /** Vector BoW correspondiente a los puntos singulares.*/
    DBoW2::BowVector mBowVec;
    /** Vector "Feature" correspondiente a los puntos singulares.*/
    DBoW2::FeatureVector mFeatVec;

	/** Descriptores ORB en el formato Mat, tal como los devuelve opencv.*/
    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors, mDescriptorsRight;

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

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();


private:

    // Rotation, translation and camera center

    /** Matriz R de rotación.  Se actualizan con UpdatePoseMatrices().*/
    cv::Mat mRcw;

    /** Vector t de traslación.  Se actualizan con UpdatePoseMatrices().*/
    cv::Mat mtcw;

    /** Matriz de rotación inversa, del mundo respecto de la cámara.*/
    cv::Mat mRwc;

    /** Vector centro de cámara, igual al vector traslación.*/
    cv::Mat mOw; //==mtwc
};

}// namespace ORB_SLAM

#endif // FRAME_H
