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

/**
 * @file main
 * main prepara el "sensor" de imágenes monocular e inicia el bucle principal, enviando una imagen Mat por vez.
 * Previamente inicializa el sistema al construir el objeto SLAM, pasándole como argumentos las rutas al vocabulario y configuración,
 * y el modo MONOCULAR.
 */


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <System.h>
#include <Viewer.h>
#include <Tracking.h>
#include <FrameDrawer.h>
#include <Video.h>
#include <Osmap.h>

using namespace std;

ORB_SLAM2::System *Sistema;

int main(int argc, char **argv){

	cout	<< "Iniciando ORB-SLAM.  Línea de comando:" << endl
			<< "os1 [archivo de configuración yaml [ruta al archivo de video]]\nSin argumentos para usar la webcam, con configuración en webcam.yaml" << endl;

    // Parámetros de la línea de comando

    char* rutaConfiguracion = NULL;
    char* rutaVideo = NULL;
	char archivoConfiguracionWebcamPorDefecto[] = "webcam.yaml";	// Configuración por defecto, para webcam.

	switch(argc){
	case 1:	// Sin argumentos, webcam por defecto y webcam.yaml como configuración
		rutaConfiguracion = archivoConfiguracionWebcamPorDefecto;
		cout << "Sin argumentos, webcam con esta configuración: " << rutaConfiguracion << endl;
		break;

	case 2:	// Un argumento, archivo de configuración  NO IMPLEMENTADO
		rutaConfiguracion = argv[1];
		break;

	case 3:	// Dos argumentos, archivo de configuración y ruta de video
		rutaConfiguracion = argv[1];
		rutaVideo = argv[2];	// Segundo argumento
		cout << "Dos argumentos: " << rutaConfiguracion << ", " << rutaVideo << endl;
		break;

	}


	// Inicializa el sistema SLAM.
    // Mi versión de archivo binario con el vocabulario, que carga mucho más rápido porque evita el análisis sintáctico.
    ORB_SLAM2::System SLAM("orbVoc.bin", rutaConfiguracion,ORB_SLAM2::System::MONOCULAR,true);

    // Puntero global al sistema singleton
    Sistema = &SLAM;

    // Imagen de entrada
    cv::Mat im;

    ORB_SLAM2::Viewer* visor = SLAM.mpViewer;
	SLAM.mpSerializer = new ORB_SLAM2::Osmap(SLAM);

    // Arranca el hilo de Video
    ORB_SLAM2::Video video;
    SLAM.mpVideo = &video;
    thread* ptVideo = new thread(&ORB_SLAM2::Video::Run, &video);
    pthread_setname_np(ptVideo->native_handle(), "VideoPlayer");

	// Indica si la entrada de video corresponde a un archivo, y por lo tanto su base de tiempo es controlable
	bool videoEsArchivo = rutaVideo;
    if(videoEsArchivo){
		video.abrirVideo(rutaVideo);
		visor->setDuracion(video.cantidadCuadros);
	}else{
		// No hay parámetros, no hay video, sólo webcam.
		video.abrirCamara();
		visor->setDuracion();
	}




    while(true){

        // Leer nuevo cuadro, controlar el tiempo del video
    	if(video.flujo == ORB_SLAM2::Video::VIDEO || video.flujo == ORB_SLAM2::Video::VIDEO_RT){
    		if(visor->tiempoAlterado){
				// El usuario movió el trackbar: hay que cambiar el frame.
				video.setCuadroPos(visor->tiempo);
				visor->tiempoAlterado = false;	// Bajar la señal.
			} else if(visor->tiempoReversa && !visor->videoPausado){
				// La película va marcha atrás
				if(video.posCuadro<2){
					// Si llega al inicio del video, recomienza hacia adelante
					video.setCuadroPos(0);
					visor->tiempoReversa = false;
				}
			}
    	}

    	// t1 está en segundos, con doble precisión.  El bucle para inicialización demora entre 1 y 2 centésimas de segundo.
    	//std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        if(video.imagenDisponible)
        	SLAM.TrackMonocular(
        		video.getImagen(
        			visor->videoPausado? 0 :
        			visor->tiempoReversa? -1 :
        			1
        		),
				(double)video.posCuadro	// timestamp
			);


    	// Ver si hay señal para cargar el mapa, que debe hacerse desde este thread
    	if(visor->cargarMapa){
    		visor->cargarMapa = false;

        	char charchivo[1024] = {};
        	FILE *f = popen("zenity --file-selection --file-filter=\"*.yaml\" --title=\"Cargar mapa\"", "r");
        	if(f){
				fgets(charchivo, 1024, f);
				cout << charchivo << endl;
				if(charchivo[0]){
					std::string nombreArchivo(charchivo);
					nombreArchivo.pop_back();	// Quita el \n final
					cout << "Abriendo archivo " << nombreArchivo << endl;
					SLAM.mpSerializer->mapLoad(nombreArchivo);
					cout << "Mapa cargado." << endl;
				}
        	}
    	}
    	if(visor->guardarMapa){
    		visor->guardarMapa = false;

        	char charchivo[1024] = {};
			cout << charchivo << endl;
        	FILE *f = popen("zenity --file-selection --save --confirm-overwrite --filename=mapa --file-filter=\"*.yaml\" --title=\"Guardar mapa\"", "r");
        	if(f){
				fgets(charchivo, 1024, f);
				if(charchivo[0]){
					std::string nombreArchivo(charchivo);
					nombreArchivo.pop_back();	// Quita el \n final
					cout << "Guardando archivo " << nombreArchivo << endl;
					SLAM.mpSerializer->options.set(ORB_SLAM2::Osmap::ONLY_MAPPOINTS_FEATURES, visor->opcionesMapa & 1);
					SLAM.mpSerializer->options.set(ORB_SLAM2::Osmap::NO_FEATURES_DESCRIPTORS, visor->opcionesMapa & 2);
					SLAM.mpSerializer->mapSave(nombreArchivo);
					cout << "Mapa guardado." << endl;
				}
        	}
    	}


    	// Abrir un archivo de video
    	if(visor->abrirVideo){
    		visor->abrirVideo = false;
        	char charchivo[1024];
        	FILE *f = popen("zenity --file-selection --title\"Abrir video\"", "r");
        	fgets(charchivo, 1024, f);
        	if(charchivo[0]){
				std::string nombreArchivo(charchivo);
				nombreArchivo.pop_back();	// Quita el \n final
				cout << "Abriendo video " << nombreArchivo << endl;
				video.abrirVideo(nombreArchivo);
				cout << "Video abierto." << endl;

				// Abrir archivo de calibración, que es igual al de configuración pero sólo se leen los parámetros de cámara
				Sistema->mpTracker->ChangeCalibration(
						// Hay que cambiar la extensión del nombre del archivo de video por .yaml
						(nombreArchivo.erase(nombreArchivo.find_last_of('.'))).append(".yaml")
				);
        	}

    	}

    	// Abrir una webcam
    	if(visor->abrirCamara){
    		visor->abrirCamara = false;
    		video.abrirCamara();
    		Sistema->mpTracker->ChangeCalibration(archivoConfiguracionWebcamPorDefecto);//"webcan.yaml");
    	}
    }
    cout << "Invocando shutdown..." << endl;

    // Stop all threads
    SLAM.Shutdown();

    cout << "Terminado." << endl;

    return 0;
}
