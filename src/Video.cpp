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
	// Inicialización de imagen, hasta que algún flujo la cambie
	imagen = imNegra = cv::Mat::zeros(480, 640, CV_8UC3);
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
				// Al haber imagen disponible, espera que se consuma para continuar.
				unique_lock<mutex> lock(mutexEspera);
				conVarEspera.wait(lock);	// Este lock se destraba sólo con getImage, que además deja imagenDisponible en false.
			}
			break;
		}

		if(im.rows){// Problema conocido, suele enviar una imagen vacía en la primera lectura
			// Actualizar propiedades con mutex
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

	conVarEspera.notify_all();	// Destraba la lectura de video en Run while.

	if(flujo == VIDEO && delta != 1)
		video.set(cv::CAP_PROP_POS_FRAMES, posCuadro + delta);
	return im;
}


bool Video::abrirCamara(int cam){

	if(cam<0){
		// Buscar cámara:
		// Cantidad de índices a probar, empezando por nIndices+1, y en el rango 0 a nIndices-1
		const int nIndices = 3;
		int i;
		for(i=0; i<nIndices; i++){
			cam = (i + camaraIndice + 1) % nIndices;

			try{camara.open(cam);}
			catch(const exception& e){cout << e.what() << endl;};

			if(camara.isOpened())
				// Camara encontrada
				break;
		}
		if(i >= nIndices){
			// No se encontró ninguna cámara.  Se termina con error.  Se establece flujo negro.
			setFlujo(NEGRO);
			alto = imNegra.rows;
			ancho = imNegra.cols;
			cout << "No se encontró ninguna cámara." << endl;
			return true;
		}

	} else
		camara.open(cam);

	// Abrir cámara
	camaraIndice = cam;
	posCuadro = 0;
	cantidadCuadros = 0;
	ancho = camara.get(cv::CAP_PROP_FRAME_WIDTH);
	alto = camara.get(cv::CAP_PROP_FRAME_HEIGHT);

	cout << //"Índice de cámara " << cam <<
			"\nResolución " << ancho <<
			" x " << alto <<
			//"\nCola " << camara.get(cv::CAP_PROP_GSTREAMER_QUEUE_LENGTH) <<
			"\nFPS " << camara.get(cv::CAP_PROP_FPS) <<
			//"\nFOURCC" << camara.get(cv::CAP_PROP_FOURCC) <<
			//"\nFormato Mat " << camara.get(cv::CAP_PROP_FORMAT) <<
			endl << endl;

	return setFlujo(CAM);
}

bool Video::abrirVideo(std::string ruta){
	// Abrir video
	video.open(ruta);

	char fourcc[] = "    ";	// 4 bytes para un int, más un 5º byte con \0 para tratarlo como char*.
	*((int*)fourcc) = static_cast<int>(video.get(cv::CAP_PROP_FOURCC));

	posCuadro = 0;
	cantidadCuadros = video.get(cv::CAP_PROP_FRAME_COUNT);
	ancho = video.get(cv::CAP_PROP_FRAME_WIDTH);
	alto = video.get(cv::CAP_PROP_FRAME_HEIGHT);

	cout << "FOURCC: " << fourcc << "\n" << ancho << " x " << alto << " píxeles." <<endl;
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
