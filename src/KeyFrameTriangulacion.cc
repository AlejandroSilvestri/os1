/*
 * KeyFrameTriangulacion.cc
 *
 *  Created on: 19/1/2017
 *      Author: alejandro
 */

#include "KeyFrame.h"	// Incluye MapPoint.h, que incluye opencv
#include "KeyFrameTriangulacion.h"

using namespace cv;
namespace ORB_SLAM2{

KeyFrameTriangulacion::KeyFrameTriangulacion(KeyFrame *pKF):
	kf(*pKF), fx(kf.fx), fy(kf.fy), cx(kf.cx), cy(kf.cy), invfx(kf.invfx), invfy(kf.invfy),
	Tcw(kf.Tcw), Ow(kf.Ow)
{
	Tcw.rowRange(0,3).colRange(0,3).copyTo(Rcw);
	Tcw.rowRange(0,3).col(3).copyTo(tcw);
	Rwc = Rcw.t();
}

Mat KeyFrameTriangulacion::rayo(int indice){
	// Código extraído de LocalMapping::CreateNewMapPoints
	kp = kf.mvKeysUn[indice];
	xn = (Mat_<float>(3,1) << (kp.pt.x-cx)*invfx, (kp.pt.y-cy)*invfy, 1.0);
	D(Rwc); D(xn);
	Mat rayo = Rwc * xn;
	return rayo/norm(rayo);
}

Mat KeyFrameTriangulacion::triangular(KeyFrameTriangulacion &kft){
	Mat A(4,4,CV_32F);
	A.row(0) = xn.at<float>(0)*Tcw.row(2)-Tcw.row(0);
	A.row(1) = xn.at<float>(1)*Tcw.row(2)-Tcw.row(1);
	A.row(2) = kft.xn.at<float>(0)*kft.Tcw.row(2)-kft.Tcw.row(0);
	A.row(3) = kft.xn.at<float>(1)*kft.Tcw.row(2)-kft.Tcw.row(1);
	D(A);
	Mat w,u,vt;
	SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

	return vt.row(3).t();
}

float KeyFrameTriangulacion::errorReproyeccion(cv::Mat x3Dt){
	const float x1 = Rcw.row(0).dot(x3Dt)+tcw.at<float>(0);
	const float y1 = Rcw.row(1).dot(x3Dt)+tcw.at<float>(1);
	const float invz1 = 1.0/z;

	float u1 = fx*x1*invz1+cx;
	float v1 = fy*y1*invz1+cy;
	float errX1 = u1 - kp.pt.x;
	float errY1 = v1 - kp.pt.y;

	return errX1*errX1+errY1*errY1;
}



}// ORB_SLAM2
