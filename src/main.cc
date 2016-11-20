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


using namespace std;

/** \mainpage ORB-SLAM2 comentado y modificado, fork personal os1
 * \section intro_sec Introducción a ORB-SLAM2
 * ORB-SLAM2 es la versión 2 de ORB-SLAM, publicada al año siguiente del original por los mismos autores.
 * Se trata de una prueba de concepto, una aplicación que demuestra el funcionamiento de ORB-SLAM.
 * La aplicación tiene una interfaz de usuario mínima y opcional.
 * ORB-SLAM2 está pensado para ser usado como parte de un proyecto mayor, conectado por ROS.
 *
 * \section conf Configuración para el primer uso
 * Para poder compilar este código con Eclipse CDT, se requiere tener instalado OpenCV 2.4.3 o superior, incluyendo 3.1,
 * Pangolin, Blas, Lapack, Eigen3.  Consultar https://github.com/raulmur/ORB_SLAM2 2.Prerequisites.
 * En Eclipse se deben cargar estas librerías (Properties for os1, Settings, g++ Linker Libraries):
 *
 * - opencv_core, usada para Mat
 * - pangolin, para la visualización del mapa
 * - opencv_features2d, para FAST
 * - opencv_highgui, para visualizar el cuadro de la cámara
 * - pthread, para disparar hilos
 * - opencv_videoio, para obtener imágenes de la webcam o de archivo
 * - opencv_imgcodecs, para obtener imágenes de archivo de video
 * - opencv_imgproc
 * - opencv_calib3d
 * - GL
 * - GLU
 *
 * Carpeta de Eigen en Properties for os1, c/c++ General Settings, Path and Symbols, Includes:
 * /usr/local/include/eigen3
 *
 * Carpeta de Eigen en Properties for os1, c/c++ General Settings, Path and Symbols, Symbols, G++:
 * - _cplusplus 201103L
 * - ArchivoBowBinario
 * - COMPILEDWITHC11

 *
 * \section desc Secuencia inicial del algoritmo ORB-SLAM2
 * Como es habitual, la ejecución inicia por main, que:
 * - lee los parámetros de la línea de comando de ejecución
 * - crea el objeto SLAM, única instancia de la clase System, cuyo constructor
 * 	- carga el vocabulario BoW
 * 	- carga el archivo de configuración y lo parsea
 * 	- crea hilos para los cuatro procesos principales y paralelos:
 * 		- LoopClosing
 * 		- LocalMapping
 * 		- Viewer
 * 		- Tracking (que se ejecuta en el hilo de main).
 * - entra en el bucle principal while(true) que ejecutará el proceso entero por cada cuadro de la cámara
 *
 *	Este bucle principal es un algoritmo largo, separado en métodos por prolijidad, la mayoría de los cuales se invocan desde un único lugar.
 *	La secuencia de invocaciones es:
 *	1- System::TrackMonocular, pasando la imagen a color de la cámara
 *	2- Tracking::GrabImageMonocular, pasando la imagen Mat en escala de grises, crea el currentFrame a partir de la imagen
 *	3- Tracking::Track, autómata finito que despacha métodos de acuerdo al estado.
 *	4- Tracking::TrackWithMotionModel, invocado en el estado OK
 *
 * \section clas Clasificación de las clases
 *	ORB-SLAM2 tiene un archivo .h y otro .cc para cada clase, todas declaradas en el espacio de nombres ORB-SLAM2.
 *	Adopta los siguientes sufijos para nombrar las propiedades miembros de clases, explicitando su tipo:
 *	- m: miembro.  Todas las propiedades comienzan con m.
 *	- p: puntero
 *	- v: vector
 *	- b: booleano
 *	- n: entero
 *	- f: float
 *	Hay excepciones, probablemente por error.
 *
 *	Ejemplos:
 *	- mvpMapPoints es un miembro vector de punteros.
 *	- mnId es un miembro entero Id.
 *
 *
 *	Hay varios tipos de clase:
 *	- Clases que se instancian repetidas veces:
 *		- Frame
 *		- KeyFrame
 *		- MapPoint
 *		- Initializer
 *		- ORBextractor
 *		- ORBmatcher
 *	- Clases que se instancian una única vez y se asocian a un thread:
 *		- Tracking
 *		- LocalMapping
 *		- Viewer
 *		- LoopClosing
 *	- Clases que se instancian una única vez y perduran:
 *		- System
 *		- Map
 *		- KeyFrameDatabase
 *		- MapDrawer
 *		- FrameDrawer
 *	- Clases que no se instancian, no tienen propiedades, son repositorios de métodos de clase:
 *		- Converter
 *		- Optimizer
 *
 * Excepciones:
 * ORBVocabulary.h es una excepción a esta regla, no define una clase sino un simple typedef.
 * ORBExtractor.h define dos clases, incluyendo ExtractorNode.
 * La carpeta Thirparty contiene versiones podadas de DBoW2 y g2o con estilos propios.
 *
 *
 * \section conc Conceptos de ORB-SLAM y sus clases
 *
 * Frame:
 * A partir de cada imagen de la cámara se crea un objeto Frame efímero.
 * Usualmente perduran el cuadro actual y el anterior solamente, habiendo sólo dos instancias de esta clase simultáneamente en el sistema.
 * El objeto Frame tiene los puntos singulares detectados, su versión antidistorsionada, sus descriptores y los puntos del mapa asociados.
 * La clase proporciona los métodos para su construcción a partir de la imagen.
 *
 * KeyFrame:
 * Los KeyFrame se crean a partir de un Frame cuando el sistema entiende que éste aporta información al mapa.
 * Mientras un Frame es efímero, un KeyFrame es de larga vida, es la manera en que un Frame se convierte en parte del mapa.
 * Cuando se crea un KeyFrame, copia la información principal del Frame actual, y computa más datos, como por ejemplo los BoW de cada descriptor.
 * La documentación de KeyFrame explica la notación de matrices utilizadas en ésta y otras clases.
 *
 * MapPoint:
 * Punto 3D del mapa del mundo.  No sólo tiene las coordenadas del punto, sino también la lista de KeyFrames que lo observan,
 * la lista de descriptores asociados al punto (un descriptor por cada KeyFrame que lo observa), entre otros.
 *
 * Map:
 * Mapa del mundo.  Tiene una lista de MapPoitns, una lista de KeyFrames, y métodos para la administración del mapa.
 *
 *
 *
 * \section hilos Descripción de los hilos
 * ORB-SLAM tiene cuatro hilos paralelos, cada uno con su propio bucle.
 *
 * - Tracking es el objeto preponderante del hilo principal, que se encarga de procesar cada imagen que llega,
 * detectando puntos singulares, calculando descriptores, macheando con el cuadro anterior
 * tratando de identificar los puntos conocidos del mapa.  Decide cuándo agregar un nuevo Keyframe.  No agrega nuevos puntos al mapa.
 * - LocalMapping procura agregar puntos al mapa cada vez que se agrega un KeyFrame.  Agrega puntos por triangulación con KeyFrames vecinos.
 * También optimiza el mapa quitando keyframes redundantes.
 * - LoopClosing se aboca a comparar la observación actual con el mapa, procurando alguna pose que explique la observación actual, detectando así bucles.
 * En ese caso procede con el cierre del bucle.
 * -Viewer maneja las dos visualizaciones: la del mapa y la de la imagen procesada.
 *
 *
 */

/**
 * @file main
 * main prepara el "sensor" de imágenes monocular e inicia el bucle principal, enviando una imagen Mat por vez.
 * Previamente inicializa el sistema al construir el objeto SLAM, pasándole como argumentos las rutas al vocabulario y configuración,
 * y el modo MONOCULAR.
 */

ORB_SLAM2::System* Sistema;

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
    //ORB_SLAM2::System::sistema = &SLAM;
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

        // Start cronómetro para medir duración del procesamiento
        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(std::chrono::steady_clock::now() - t1).count();

    	// Ver si hay señal para cargar el mapa, que debe hacerse desde este thread
    	if(visor->cargarMapa){
    		visor->cargarMapa = false;

        	// Limpia el mapa de todos los singletons
    		SLAM.mpTracker->Reset();

    	    // Espera a que se detenga LocalMapping
    		SLAM.mpLocalMapper->RequestStop();
    		while(!SLAM.mpLocalMapper->isStopped()) usleep(1000);

        	char archivo[] = "mapa.bin";
        	SLAM.mpMap->load(archivo);
        	cout << "Mapa cargado." << endl;

        	SLAM.mpTracker->mState = ORB_SLAM2::Tracking::LOST;

        	// Por las dudas, es lo que hace Tracking luego que el estado pase a LOST
        	SLAM.mpFrameDrawer->Update(SLAM.mpTracker);

    	}

        // Delay para 30 fps, período de 0.033 s
        if(ttrack < 0.033)
        	//std::this_thread::sleep_for(std::chrono::milliseconds((0.033-ttrack)*1e3));
        	usleep((0.033-ttrack)*1e6);

    }
    cout << "Invocando shutdown." << endl;

    // Stop all threads
    SLAM.Shutdown();

    cout << "Terminado." << endl;

    return 0;
}
