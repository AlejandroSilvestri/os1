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
#include "ORBextractor.h"

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;
class ORBextractor;

/**
 * Frame represents a frame, an image, with feature detection.
 *
 * Frame represents a camara shoot, with its image, 2D points detected, its descriptors, its 3D mapping, its, etc.
 *
 * The frame position in the map is obtained with Frame::GetCameraCenter. The orientation with Frame::GetRotationInverse.
 *
 * Tracking uses 3 Frames:
 * - Frame::mCurrentFrame
 * - Frame::mInitialFrame
 * - Frame::mLastFrame
 *
 * Each frame has its own K calibration matrix.
 * It's usually the same matrix (same values) for all frames and keyframes.
 *
 * The constructor clones the K Mat, generating an own copy in Frame::mK.
 * It then analyzes de image, but does not save it. The image will be lost when the constructor finishes.
 * Part of this analysis consists in feature detection, extract its descriptors and classify them.
 *
 * Keypoints, descriptors, BoW and tracked 3D points are registered in parallel vectors and explained in Frame::N.
 *
 * The coordinate system and the position matrices meaning is explained in Frame::mTcw.
 *
 * This class does not determine the frame position. Its position is tracked by Tracking and Optimizer.
 *
 * \sa SetPose
 */
class Frame
{
public:
	/**
	 * The constructor with no arguments creates a Frame with no initialization. The only initialated data is nNextId=0.
	 * Not used.
	 */
    Frame();

    /**
     * Copy constructor, clones a frame.
     *
     */
    // Copy constructor.
    Frame(const Frame &frame);

    /**
     * Constructor that crates a Frame and fills it with arguments.
     * @param timeStamp Marks time, for record.  ORB-SLAM does not use it.
     * @param extractor Extractor algorithm used to obtain descriptors.  ORB-SLAM uses the ORB extractor from BRIEF exclusively.ing
     *
     * Two camera modes are distinguished: normal if distortion coeficients are delivered, or fisheye without coeficients if noArray() is delivered.
     *
     * Invoked just from Tracling::GrabImageMonocular.
     */
    // Constructor for Monocular cameras.
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf);//, const float &thDepth);
    //Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor, ORBVocabulary* voc, const float &thDepth);

    /**
     * Proceeds with the extraction of ORB descriptors.
     *
     * @param flag false for monocular, or for left camera.  true for right camera.  Always invoke with false.
     * @param im Image above to which extract descriptors.
     *
     * Descriptors are conserved in Frame::mDescriptors.
     *
     * Invoked only from the constructor.
     */
    // Extract ORB on the image. 0 for left image and 1 for right image.
    void ExtractORB(int flag, const cv::Mat &im);

    /**
     * Computing BoW for all frame descriptors.
     * Its saved in the mBowVec property, BowVectorn type, and in mFeatVec, FeatureVector type.
     *
     * If mBowVec is not empty the method returns without doing anything, avoiding the rebound.
     *
     * Its computed BoW for a frame when set up in keyframe, and when to relocate.
     *
     * DBoW2 is documented in http://webdiis.unizar.es/~dorian/doc/dbow2/annotated.html
     *
     *
     */
    // Compute Bag of Words representation.
    void ComputeBoW();

    /**
     * Registers pose.
     *
     * Used by various methods to establish or correct the frame pose.
     *
     * After establishing the new pose the different representations are recalculated with UpdatePoseMatrices, such as the traslation or rotation vector.
     *
     * @param Tcw New pose, rototraslation matrix in homogeneous coordinates, of 4x4.
     *
     * Registers the pose in Frame::mTcw, that corresponds to the world origin pose in the camera coordinate system.
     *
     * This method is invoked by Tracking with aproximate positions to initialize and track,
     * and by Optimizer::PoseOptimization, the only one that tracks the optimized pose.
     */
    // Set the camera pose.
    void SetPose(cv::Mat Tcw);

    /**
     * Calculates the position matrices mRcw, mtcw y mOw from mTcw pose.
     * This matrices are a way to espose the pose, are not used in the ORB-SLAM operation.
     * UpdatePoseMatrices() extracts the mTcw information, the matrix that combines the complete pose.
     *
     */
    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    /** Brings back the camera position vector, the camera center.*/
    // Brings back the camera center.
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    /**
     * Brings back the map orientation.
     *
     * Its the rotation mRwc inverse.
     *
     * @returns mRwc.t()
     *
     * Does not has GetRotation to bring back the rotation without inverting mRcw.
     */
    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    /**
     * Indicates if a specific 3D point is found in the frame visual subspace (frustum).
     * The visual subspace is a quadrilateral base pyramid, whichs vertices are those from the frame but undistorted.
     * viewingCosLimit is a way to limitate the frustum reach.
     */
    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    /**
     * Calculates the cell coordinates in the grid, to which it belongs a keypoint.
     * Informs the coordinates in posX and posY arguments.
     * Gives back true if the point is in the grid, false if not.
     * @param kp "Undistorted" keypoint.
     * @param posX X coordinate of the cell to which the point belongs.
     * @param posY Y coordinate of the cell to which the point belongs.
     *
     */
    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    /**
     * It selects the point inside a square window of center x,y and radio r (side 2r).
     *
     * It goes through all frame levels filtering the points by coordinates.
     *
     * Its used to reduce candidates by matching.
     *
     * @param x Coordinate x from the area center
     * @param y Coordinate y from the area center
     * @param r square area radius
     * @param minLevel Pyramid minimum level where to search for keypoints.  Negative if there's no minimum.
     * @param maxLevel Pyramid maximum level where to search for keypoints.  Negative if there's no maximum.
     *
     * Invoked only by various ORBmarcher methods. 
     */
    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

public:
	/** BOW vocabulary to classify descriptors.*/
    // Vocabulary used for relocalization.

    /** "Algorithm" to extract descriptors.  The framework allows the programmer to try different extractors; ORB-SLAM uses only ORBextractor.*/
    ORBVocabulary* mpORBvocabulary;

    /** Extractor used to extract descriptors.*/
    // Feature extractor. The right is used only in the stereo case.
    ORBextractor* mpORBextractorLeft;

    /**
     * Time stamp of the image capture.
     * Only for research and register purposes, the algorithm does not use it.
     */
    // Frame timestamp.
    double mTimeStamp;

	/**
	 * Camera intrinsec K Matrix.
	 * Distrortion mDistCoef coefficients, intrinsic parameters fx, fy, cx, cy.
	 */
    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;

	/** Intrinsic parameters fx, fy, cx, cy.*/
	/** Intrinsic parameter.*/
    static float fx;
	/** Intrinsic parameter.*/
    static float fy;
	/** Intrinsic parameter.*/
    static float cx;
	/** PIntrinsic parameter.*/
    static float cy;
	/** Intrinsic parameter inverse.*/
    static float invfx;
	/** Intrinsic parameter.*/
    static float invfy;

	/** mDistCoef camera distortion coeficients.*/
    cv::Mat mDistCoef;

    /** Camera mode, 0 normal, 1 fisheye without distortion.*/
    int camaraModo;

    /** Not used in monocular.*/
    // Stereo baseline multiplied by fx.
    float mbf;

    /** Not used in monocular.*/
    // Stereo baseline in meters.
    float mb;

    /** Not used in monocular.*/
    // Thereshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;

	/**
	 * Amount of keypoints.
	 *
	 * It's de size of paired vectors:
	 * - mvKeys
	 * - mvKeysUn
	 * - mDescriptors
	 * - mBowVec
	 * - mFeatVec
	 * - mvpMapPoints
	 * - mvbOutlier
	 *
	 *
	 * All these are passed to keyframe.
	 */
    // Number of KeyPoints.
    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    /**
     * Keypoins vector obtained from detector, as returned by opencv.
     *
     * Its coordinates are in pixels, in the image reference system.
     */
    std::vector<cv::KeyPoint> mvKeys;

	/**
	 * Undistorted points' vector, mvKeys corrected according to the distortion coeficients.
	 *
	 * This vector is paired with mvKeys, both of N size.
	 *
	 * Its coordinates are in pixels, in the undistorted image system of reference.
	 * Points are obtained with cv::undistortPoints, reapplying the camera k matrix.
	 */
    std::vector<cv::KeyPoint> mvKeysUn;

    /** -1 for monocular.  It's passed in the Frame copy constructor.*/
    //std::vector<float> mvDepth;

    // Bag of Words Vector structures.
    /**
     * Vector BoW corresponding to skeypoints.
     *
     * BowVector is a Word Id map (unsigned int) a Word value (double), that represents a weight.
     */
    DBoW2::BowVector mBowVec;

    /**
     * "Feature" vector corresponding to keypoints.
     */
    DBoW2::FeatureVector mFeatVec;

	/** ORB descriptors in Mat format, as given back by opencv.  mDescritorRight is not used, its passed in the copy constructor.*/
    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors;//, mDescriptorsRight;

	/** 3D map points vector associated to keypoints.
	This vector has the same length as mvKeys and mvKeysUn.
	Positions with no association are NULL.
	*/
    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<MapPoint*> mvpMapPoints;

	/** Flag that indicates if there's associated outliers in mvpMapPoints.*/
    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;

	/** mfGridElementWidthInv is the cell width inverse in pixels.
	 * Undistorted image points are divided in cells with a FRAME_GRID_COLS by FRAME_GRID_ROWS grid,
	 * to reduce matching complexity.
	 */
    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;

	/** mfGridElementHeightInv is the cell height inverse in pixels.
	 * Undistorted image points are divided in cells with a FRAME_GRID_COLS by FRAME_GRID_ROWS grid,
	 * to reduce matching complexity.
	 */
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

	/**
	 * Camera pose.
     * 4x4 rototraslation matrix in homogeneous coordinates
     * It's cv:Mat(4,4,CV_32F), and its elements are float.
     *
     * Camera position (traslation vector) is obtained from the last matrix column.
     *
     * Metric unit is set during the initializatin, same as the average scene depth.
     * This means that the unit will depend of 3D points triangulated in initialization.
     *
     * A camera 3D coordinate system is related with its 2D projection.
     * keeping the X and Y axes parallel, and establishing Z forwards.
     * This way, X points right and Y point down (it's contrary to the standard coordinate system).
     * Homogeneous 3D vectors have the traditinoal disposition:
     *
     * V = [vx, vy, vz, 1]t
     *
     * Poses are 4x4 matrices like this one, its subindexes indicate reference and subject.  Tcw is T regarding the camera, from world.
     * Poses are combined like this:
     *
     * Tca = Tba * Tcb
     *
     * Its value is updated through SetPose, which in addition extracts the rotation matrix and the traslation vector with UpdatePoseMatrices.
     * These extracter data are calculated for presentation, but are not used by the algorithm, that only uses mTcw.
     *
     * Frame does not calculate mTcw.  This matrix is calculated and passed to the frame with SetPose, in:
     * - Tracking, copying from a previous frame, initializing with the cartesion pose, or estimating by movement model.
     * - Optimizer::PoseOptimization, invoked from various Tracking methods.  Here's where the real pose calculus us made.
     * -
	 */
    // Camera pose.
    cv::Mat mTcw;

	/** Id incremental with the value for next frame.
	 * Is a class variable that keeps the id number count.
	 * The Frame id is assigned with
	 *     mnId=nNextId++;
	 */
    // Current and Next Frame id.
    static long unsigned int nNextId;

	/** Id incremental that identifies this frame.*/
    long unsigned int mnId;

	/** Reference KeyFrame.*/
    // Reference Keyframe.
    KeyFrame* mpReferenceKF;

	/** Amount of pyramid levels.*/
    // Scale pyramid info.
    int mnScaleLevels;

	/** Scale factor between pyramid levels.*/
    float mfScaleFactor;

    /** Logarithmic scale factor.*/
    float mfLogScaleFactor;

	/** Scale factor vector of each pyramid level.*/
    vector<float> mvScaleFactors;
	/** mvScaleFactors inverse.*/
    vector<float> mvInvScaleFactors;
    /** mvScaleFactos square.*/
    vector<float> mvLevelSigma2;
    /** mvLevelSigma2 inverse.*/
    vector<float> mvInvLevelSigma2;


	/** Undistorted image vertices: mnMinX, mnMinY, mnMaxX, mnMaxY.*/
    // Undistorted Image Bounds (computed once).
    static float mnMinX;
	/** Undistorted image vertices: mnMinX, mnMinY, mnMaxX, mnMaxY.*/
    static float mnMaxX;
	/** Undistorted image vertices: mnMinX, mnMinY, mnMaxX, mnMaxY.*/
    static float mnMinY;
	/** Undistorted image vertices: mnMinX, mnMinY, mnMaxX, mnMaxY.*/
    static float mnMaxY;

	/**
	 * This variables is one with a contructor argument only when the first Frame is created.
	 * true for class variable calculus, that do not change later.
	 */
    static bool mbInitialComputations;

    /**
     * Calculates mvKeysUn keypoints.
     *
     * Undistorted detected points, that are in mvKeys, and keeps them in mvKeysUn in the same order.
     * If there's no distortion, UndistortKeyPoints goes back quikly unifying mvKeysUn = mvKeys in a same vector.
     */
    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    /**
     * Undistorted points according to the fisheye camera equidistant projection with no distortion coeficients model.
     * Uses mK, intrinsec matrix, to pass the distortione points to focal scale, and after undistortioning them, pass them once again to image scale.
     *
     * @param  Mat point of nx2 float, each pair corresponds to a point.  Input and output.  Points are corrected and the result kept in the same place.
     *
     * Used only form Frame::UndistortKeyPoints and Frame::ComputeImageBounds
     */
    void antidistorsionarProyeccionEquidistante(cv::Mat &puntos);

    /**
     * Calculates the undistorted frame's vertices.
     * Defines mnMinX, mnMaxX, mnMinY, mnMaxY.
     * If there's no distortion, the result is trivial with origin in (0,0).
     *
     * @param imLeft Imagen, only to the effects of measuring its size with .rows y .cols .
     *
     * Invoked only from the constructor.
     */
    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    /**
     * Assignes keypoints to its grid cells.
     * The image is divided in a grid to detect point in a more homogeneous way.
     * The keypoint detecter is later 'undistorted',
     * this method creates a keypoint vector for each grid cell, and tests it.
     */
    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();


private:

    // Rotation, translation and camera center

    /**
     * World rotation R Matrix  regarding the camera.
     * It's updated with UpdatePoseMatrices().
     */
    cv::Mat mRcw;

    /**
     * World origin traslation vector t in the camera system of reference.
     * It's updated with UpdatePoseMatrices().
     */
    cv::Mat mtcw;

    /**
     * Inverse rotation matrix of the camera regarding the world.
     * It's updated with UpdatePoseMatrices().
     */
    cv::Mat mRwc;

    /**
     * Center camera vector, camera position regarding the world.
     *
     * It's private, informed with Frame::GetCameraCenter.
     *
     * 3x1 matrix (vertical vector).
     * Inverted traslation vector. mtcw is the world origin traslation vector in the camera system of reference.
     * It's updated with UpdatePoseMatrices().
     */
    cv::Mat mOw; //==mtwc
};

}// namespace ORB_SLAM

#endif // FRAME_H
