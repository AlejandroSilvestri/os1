/*
 * Video.cpp
 *
 *  Created on: 27 jun. 2017
 *      Author: alejandro
 */

#include "Video.h"
#include <iostream>

using namespace std;

namespace ORB_SLAM2{

Video::Video(){
	// Crea la imagen negra, de dimensiones arbitrarias, para el flujo NEGRO
	imNegra = cv::Mat::zeros(640, 480, CV_8UC3);
}

void Video::Run(){

	// Bucle principal de procesamiento de video de entrada
	while(true){
		cv::Mat im;

		switch(flujo){
		case NEGRO:
			im = imNegra;
			break;

		case CAM:
			camara.read(im);
			break;

		case VIDEO_RT:
			posCuadro = video.get(cv::CAP_PROP_POS_FRAMES);
			video.read(im);
			break;

		case VIDEO:
			posCuadro = video.get(cv::CAP_PROP_POS_FRAMES);
			video.read(im);
			if(imagenDisponible == true){	// Único flujo: No Tiempo Real
				//cout << "VIDEO, antes de mutex" << endl;
				// Al haber imagen disponible, espera que se consuma para continuar.
				unique_lock<mutex> lock(mutexEspera);
				//cout << "VIDEO, después del mutexEspera, antes del wait." << endl;
				conVarEspera.wait(lock);	// Este lock se destraba sólo con getImage, que además deja imagenDisponible en false.
				//cout << "VIDEO, después del wait." << endl;
			}
			break;
		}


		{	// Actualizar propiedades con mutex
			unique_lock<mutex> lock(mutexImagen);
			imagenDisponible = true;
			imagen = im;
		}
	}
}

cv::Mat Video::getImagen(int delta){
	// mutex para propiedades imagen e imagenDisponible
	unique_lock<mutex> lock(mutexEspera);
	unique_lock<mutex> lock2(mutexImagen);

	cv::Mat im = imagen;
	imagenDisponible = false;

	//lock.release();
	conVarEspera.notify_all();	// Destraba la lectura de video en Run while.

	if(flujo == VIDEO && delta != 1)
		video.set(cv::CAP_PROP_POS_FRAMES, posCuadro + delta);
	return im;
}


bool Video::abrirCamara(int cam = 0){
	// Abrir cámara
	camara.open(cam);
	posCuadro = 0;
	cantidadCuadros = 0;
	return setFlujo(CAM);
}

bool Video::abrirVideo(std::string ruta){
	// Abrir video
	video.open(ruta);

	char fourcc[] = "    ";	// 4 bytes para un int, más un 5º byte con \0 para tratarlo como char*.
	*((int*)fourcc) = static_cast<int>(video.get(cv::CAP_PROP_FOURCC));
	cout << "FOURCC: " << fourcc << endl;

	posCuadro = 0;
	cantidadCuadros = video.get(cv::CAP_PROP_FRAME_COUNT);

	return setFlujo(VIDEO);
}

/**
 * TODO:
 * Antes de cambiar el flujo, hay que verificar que su VideoCapture esté abierto.
 * Si no lo está, no se cambia el flujo y se devuelve false.
 */
bool Video::setFlujo(flujos tipoDeFlujo){
	flujo = tipoDeFlujo;

	// por si Run está bloqueado en VIDEO
	conVarEspera.notify_all();

	return false;
}

void Video::setCuadroPos(int pos){
	if(flujo == VIDEO || flujo == VIDEO_RT){
		video.set(cv::CAP_PROP_POS_FRAMES, pos);
		posCuadro = pos;
	}
}

}	//namespace ORB_SLAM2
