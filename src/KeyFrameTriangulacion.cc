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
	kf(*pKF), fx(kf.fx), fy(kf.fy), cx(kf.cx), cy(kf.cy), invfx(kf.invfx), invfy(kf.invfy)
{
	Tcw = kf.GetPose();
	Rcw = kf.GetRotation();
	Rwc = Rcw.t();
	Ow = kf.GetCameraCenter();
	Tcw.rowRange(0,3).col(3).copyTo(tcw);
}

KeyFrameTriangulacion::KeyFrameTriangulacion(KeyFrame *pKF1, int indice1, KeyFrame *pKF2,int indice2)
:KeyFrameTriangulacion(pKF1)
{
	setKeyPoint(indice1);

	setKeyFrame2(pKF2);
	setKeyPoint2(indice2);

	triangular();
}

int KeyFrameTriangulacion::triangular(){
	// Triangular puntos y guardar resultados parciales en propiedades
	ray = rayo();
	kft2->ray = kft2->rayo();
	cosParalaje = kft2->ray.dot(ray);
	if(cosParalaje < umbralCosParalaje){
		x3D = triangular(*kft2);
		if(x3D.at<float>(3) != 0){
			x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
			inf = 0;	// normal
		} else {
			x3D = x3D.rowRange(0,3);	// Vector con la dirección del punto en el infinito.  Falta verificar sentido.
			x3D = x3D/norm(x3D) * 1e8;	// Multiplica por 1e7 para enviarlo al quasi-infinito
			inf = 2;	// infitito por SVD
		}
	} else {
		x3D = (ray+kft2->ray) * 1e8;
		inf = 1;	// infinito por umbral coseno
	}

	// Verificaciones
	Mat x3Dt = x3D.t();
	if(coordenadaZ(x3Dt)<=0){
		if(inf)
			x3Dt = -x3Dt;
		else
			return error = 1;	// Rechazado por estar detrás de cámara
	}

	if(kft2->coordenadaZ(x3Dt)<=0)
		return error = 1;	// Rechazado por estar detrás de cámara

	if(!validarErrorReproyeccion(x3Dt) || !kft2->validarErrorReproyeccion(x3Dt))
		return error = 2;

	if(!validarDistancias())
		return error = 3;

	return error = 0;
}

Mat KeyFrameTriangulacion::rayo(int indice){
	setKeyPoint(indice);//kp = kf.mvKeysUn[indice];
	return rayo();
}

Mat KeyFrameTriangulacion::rayo(){
	// Código extraído de LocalMapping::CreateNewMapPoints
	xn = (Mat_<float>(3,1) << (kp.pt.x-cx)*invfx, (kp.pt.y-cy)*invfy, 1.0);
	Mat rayo = Rwc * xn;
	return rayo/norm(rayo);
}

Mat KeyFrameTriangulacion::triangular(KeyFrameTriangulacion &kft){
	Mat A(4,4,CV_32F);
	A.row(0) = xn.at<float>(0)*Tcw.row(2)-Tcw.row(0);
	A.row(1) = xn.at<float>(1)*Tcw.row(2)-Tcw.row(1);
	A.row(2) = kft.xn.at<float>(0)*kft.Tcw.row(2)-kft.Tcw.row(0);
	A.row(3) = kft.xn.at<float>(1)*kft.Tcw.row(2)-kft.Tcw.row(1);

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

bool KeyFrameTriangulacion::validarDistancias(){
    const float dist1 = distancia(x3D),
    	  dist2 = kft2->distancia(x3D);
    if(dist1==0 || dist2==0)
		return false;
    const float ratioDist = dist2/dist1;
    const float ratioOctave = kf.mvScaleFactors[kp.octave]/kft2->kf.mvScaleFactors[kft2->kp.octave];

    const float ratioFactor = 1.5f*kf.mfScaleFactor;
    if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
    	return false;

    return true;
}


}// ORB_SLAM2
