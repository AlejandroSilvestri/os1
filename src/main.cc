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
#include <Serializer.h>


using namespace std;

ORB_SLAM2::System *Sistema;

int main(int argc, char **argv){

	cout	<< "Iniciando ORB-SLAM.  Línea de comando:" << endl
			<< "os1 [archivo de configuración yaml [ruta al archivo de video]]\nSin argumentos para usar la webcam, con configuración en webcam.yaml" << endl;

    cv::VideoCapture video,					// Entrada de video desde un archivo
    			 	 webcam;				// Entrada de video desde una webcam
    cv::VideoCapture* videoEntrada = NULL;	// Entrada de video seleccionada, una de las de arriba

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
	    //cv::FileStorage fSettings(rutaConfiguracion, cv::FileStorage::READ);
	    //rutaVideo = fSettings["rutaVideo"];// No sé cargar una string de yaml
		break;

	case 3:	// Dos argumentos, archivo de configuración y ruta de video
		rutaConfiguracion = argv[1];
		rutaVideo = argv[2];	// Segundo argumento
		cout << "Dos argumentos: " << rutaConfiguracion << ", " << rutaVideo << endl;
		break;

	}

	// Indica si la entrada de video corresponde a un archivo, y por lo tanto su base de tiempo es controlable
	bool videoEsArchivo = rutaVideo;
    if(videoEsArchivo){
		video.open(rutaVideo);
		videoEntrada = &video;
		char fourcc[] = "    ";
		int* fcc;
		fcc = (int*)fourcc;
		*fcc = static_cast<int>(video.get(cv::CAP_PROP_FOURCC));
		cout << "FOURCC: " << fourcc << endl;

	}else{
		// No hay parámetros, no hay video, sólo webcam.
		webcam.open(0);
		videoEntrada = &webcam;
	}

    // Inicializa el sistema SLAM.
    // Mi versión de archivo binario con el vocabulario, que carga mucho más rápido porque evita el análisis sintáctico.
    ORB_SLAM2::System SLAM("orbVoc.bin", rutaConfiguracion,ORB_SLAM2::System::MONOCULAR,true, videoEntrada);
	// Versión original que carga el vocabulario de un archivo de texto
    //ORB_SLAM2::System SLAM("../Archivos/ORBvoc.txt", rutaConfiguracion,ORB_SLAM2::System::MONOCULAR,true);

    // Puntero global al sistema singleton
    Sistema = &SLAM;

    // Imagen de entrada
    cv::Mat im;

    ORB_SLAM2::Viewer* visor = SLAM.mpViewer;
	// Número de cuadro procesado, monótonamente creciente, para espaciar mensajes de consola.
    int n = 0;
    while(true){
        // Leer nuevo cuadro
    	bool hayImagen;

    	if(videoEsArchivo){
    		if(visor->tiempoAlterado){
				// El usuario movió el trackbar: hay que cambiar el frame.
				videoEntrada->set(cv::CAP_PROP_POS_FRAMES, visor->tiempo);
				visor->tiempoAlterado = false;	// Bajar la señal.
			} else if (visor->tiempoReversa && !visor->videoPausado){
				// La película va marcha atrás
				int pos = videoEntrada->get(cv::CAP_PROP_POS_FRAMES);
				videoEntrada->set(cv::CAP_PROP_POS_FRAMES, pos>=2? pos-2 : 0);
				if(pos<2)
					// Si llega al inicio del video, recomienza hacia adelante
					visor->tiempoReversa = false;
			}
    	}

   		if(!visor->videoPausado)
   			hayImagen = videoEntrada->read(im);

    	// t1 está en segundos, con doble precisión.  El bucle para inicialización demora entre 1 y 2 centésimas de segundo.
    	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        if(hayImagen)
        	SLAM.TrackMonocular(im,0);
        else
        	cout << "No hay imagen en bucle nº " << n << endl;	// No pasa nunca

    	// Ver si hay señal para cargar el mapa, que debe hacerse desde este thread
    	if(visor->cargarMapa){
    		visor->cargarMapa = false;

        	// Limpia el mapa de todos los singletons
    		SLAM.mpTracker->Reset();

    	    // Espera a que se detenga LocalMapping y  Viewer
    		SLAM.mpLocalMapper->RequestStop();
    		SLAM.mpViewer	  ->RequestStop();
    		while(!SLAM.mpLocalMapper->isStopped()) usleep(1000);
    		while(!SLAM.mpViewer	 ->isStopped()) usleep(1000);

        	char archivo[] = "mapa.bin";
        	//SLAM.mpMap->load(archivo);
        	SLAM.serializer->mapLoad(archivo);
        	cout << "Mapa cargado." << endl;

        	SLAM.mpTracker->mState = ORB_SLAM2::Tracking::LOST;

        	// Reactiva viewer.  No reactiva el mapeador, pues el sistema queda en sólo tracking luego de cargar.
        	SLAM.mpViewer->Release();

        	// Por las dudas, es lo que hace Tracking luego que el estado pase a LOST.
        	// Como tiene un mutex, por las dudas lo invoco después de viewer.release.
        	SLAM.mpFrameDrawer->Update(SLAM.mpTracker);

    	}
    	if(visor->guardarMapa){
    		visor->guardarMapa = false;

    	    // Espera a que se detenga LocalMapping
    		SLAM.mpLocalMapper->RequestStop();
    		SLAM.mpViewer	  ->RequestStop();	// No parece que sea necesario para guardar, sino sólo para cargar, pues al guardar no se modifica el mapa.

    		while(!SLAM.mpLocalMapper->isStopped()) usleep(1000);
    		while(!SLAM.mpViewer	 ->isStopped()) usleep(1000);

        	char archivo[] = "mapa.bin";
        	//SLAM.mpMap->save(archivo);
        	SLAM.serializer->mapSave(archivo);
        	cout << "Mapa guardado." << endl;

        	// Reactiva viewer.  No reactiva el mapeador, pues el sistema queda en sólo tracking luego de cargar.
        	SLAM.mpViewer->Release();
    	}

        // Stop cronómetro para medir duración del procesamiento
        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(std::chrono::steady_clock::now() - t1).count();

        // Delay para 30 fps, período de 0.033 s
        if(ttrack < 0.033)
        	usleep((0.033-ttrack)*1e6);

    }
    cout << "Invocando shutdown." << endl;

    // Stop all threads
    SLAM.Shutdown();

    cout << "Terminado." << endl;

    return 0;
}
