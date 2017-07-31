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
 * \mainpage ORB-SLAM2 comentado y modificado, fork personal os1
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
 *	1. System::TrackMonocular, pasando la imagen a color de la cámara
 *	2. Tracking::GrabImageMonocular, pasando la imagen Mat en escala de grises, crea el currentFrame a partir de la imagen
 *	3. Tracking::Track, autómata finito que despacha métodos de acuerdo al estado.
 *	4. Tracking::TrackWithMotionModel, invocado en el estado OK
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
 *	- Clases que se instancian repetidas veces y perduran:
 *		- Frame: hay 3 instancias: Tracking::mCurrentFrame, Tracking::mLastFrame y Tracking::mInitialFrame.
 *		- KeyFrame: varias instancias registradas en Map::mspKeyFrames
 *		- MapPoint: varias instancias registradas en Map::mspMapPoints
 *	- Clases efímeras que se instancias repetidas veces y se destruyen:
 *		- ExtractorNode: varios nodos creados en ORBextractor.
 *		- Initializer
 *		- ORBextractor: creado para la extracción de puntos singulares y descriptores, y luego destruído.
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
 * Cada clase se define en su propio archivo .h homónimo, excepto ORBExtractor.h define también ExtractorNode.
 * El único archivo include/ *.h que no define una clase es ORBVocabulary.h, que contiene un simple typedef.
 *
 * La carpeta Thirparty contiene versiones podadas de DBoW2 y g2o con estilos propios.
 *
 *
 * \section conc Conceptos de ORB-SLAM y sus clases
 *
 * \subsection Frame Frame
 *
 * A partir de cada imagen de la cámara se crea un objeto Frame efímero.
 *
 * Usualmente perduran el cuadro actual y el anterior solamente, habiendo sólo dos instancias de esta clase simultáneamente en el sistema.
 *
 * El objeto Frame tiene los puntos singulares detectados, su versión antidistorsionada, sus descriptores y los puntos del mapa asociados.
 *
 * La clase proporciona los métodos para su construcción a partir de la imagen.
 *
 * Hay sólo tres instancias de esta clase:
 * - Tracking::mCurrentFrame con el cuadro más reciente
 * - Tracking::mLastFrame con el cuadro inmediato anterior al más reciente
 * - Tracking::mInitialFrame con el cuadro inicial, que participó en la inicialización del mapa
 *
 * Mientras las instancias mCurrentFrame y mLastFrame perduran, su contenido no lo hace, sino que rota.
 * Cada nueva imagen se vuelca sobre mCurrentFrame, reemplazando a la anterior.
 *
 *
 * \subsection KeyFrame KeyFrame
 *
 * Los KeyFrame se crean a partir de un Frame cuando el sistema entiende que éste aporta información al mapa.
 *
 * Un KeyFrame es de larga vida, es la manera en que un Frame se convierte en parte del mapa.
 *
 * Cuando se crea un KeyFrame, copia la información principal del Frame actual, y computa más datos, como por ejemplo los BoW de cada descriptor.
 *
 * Los keyframes se registran en Map::mspKeyFrames.
 *
 * La documentación de KeyFrame explica la notación de matrices de pose utilizadas en KeyFrame y en Frame.
 *
 *
 *
 * \subsection MapPoint MapPoint
 * Punto 3D del mapa del mundo.  No sólo tiene las coordenadas del punto, sino también la lista de KeyFrames que lo observan,
 * la lista de descriptores asociados al punto (un descriptor por cada KeyFrame que lo observa), entre otros.
 *
 * Los mapPoints se registran en Map::mspMapPoints.
 *
 *
 * \subsection Map Map
 * Mapa del mundo.  Tiene una lista de MapPoints, una lista de KeyFrames, y métodos para la administración del mapa.
 *
 * \subsection System System
 * \subsection KeyFrameDatabase KeyFrameDatabase
 * \subsection
 * \subsection
 * \subsection
 * \subsection
 * \subsection
 * \subsection
 * \subsection
 * \subsection
 * \subsection
 * \subsection
 * \subsection
 * \subsection
 * \subsection
 * \subsection
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

#ifndef SYSTEM_H
#define SYSTEM_H

#include <string>
#include <thread>
#include <opencv2/core.hpp>
//#include <opencv2/videoio.hpp>
#include <mutex>


#include "ORBVocabulary.h"



namespace ORB_SLAM2{

class KeyFrameDatabase;
class MapDrawer;

/** Visor 3D del sistema, única instancia de Viewer. */
class Viewer;

/** Visor de la imagen procesada, única instancia de FrameDrawer. */
class FrameDrawer;

/** Mapa global de puntos 3D.*/
class Map;

/** Única instancia de Tracking, raíz del sistema que invoca al resto.*/
class Tracking;

/** Única instancia de mapa local.*/
class LocalMapping;

/** Única instancia d*/
class LoopClosing;

class Serializer;

/**
 * System se instancia una única vez en main, en la variable SLAM.
 * Sus métodos representan la API del sistema.
 * Luego de la inicialización, el bucle principal de main invoca un método de System para cada cuadro.
 * main controla el origen de imágenes, y lo envía a Track* para su proceso.  Track* es uno de:
 * - TrackMonocular
 * - TrackStereo
 * - TrackRGBD
 *
 * En ese mismo bucle la gui invoca comandos de System, como ActivateLocalizationMode() y DeactivateLocalizationMode().
 * Otros comandos disponibles: Reset, Shutdown, Save*.
 *
 * Las propiedades mp del sistema son punteros a los objetos principales, todos de instancia única, del tipo:
 * - ORBVocabulary
 * - KeyFrameDatabase
 * - Map
 * - FramDrawer
 * - MapDrawer
 *
 * Puntero a los objetos de los hilos, de tipo:
 * - Tracking (está en el mismo hilo que System)
 * - LocalMapping y a su hilo
 * - LoopClosing y a su hilo
 * - Viewer y a su hilo
 *
 * Las propiedades de estado son booleanas:
 * - mbReset
 * - mbActivateLocalizationMode
 * - mbDeactivateLocalizationMode
 *
 * Otras propiedades misceláneas:
 * - mMutexMode
 * - mMutexReset
 * - mSensor: MONOCULAR, STEREO o RGBD
 */
class System
{
public:
	// Para que se vea desde main y viewer
	//mutex mutexVideoEntrada;


	// Input sensor
	/** Tipo de sensor, modo de operación: mono, estéreo o mapa de profundidad.*/
    enum eSensor{
        MONOCULAR=0//,
        //STEREO=1,
        //RGBD=2
    };

public:

    /**
     * Constructor único.
     *
     * @param strVocFile Nombre del archivo (binario) con el vocabulario BoW, para abrir.
     * @param strSettingsFile Nombre del archivo de configuración, para abrir.
     * @param sensor Tipo de sensor.  MONOCULAR para mono slam.
     * @param bUseViewer Señal para activar el visor.
     *
     */
    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true);


    /**
     * Procesa una imagen de la cámara.
     *
	 * Envoltorio del método principal, invocado repetidamente por el bucle principal, una vez por cada imagen del sensor.
	 *
	 * Chequea variables de estado que reflejan comandos de la gui.
	 *
	 * Finalmente invoca el método principal: Tracking::GrabImageMonocular
	 *
     * @param im Imagen de cámara a procesar.
     * @param timestamp Marca temporal, sólo para registro, orb-slam2 no la utiliza.
     *
     */
    // Proccess the given monocular frame
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

    /** Pasa a modo de sólo localización, sin mapeo (sólo tracking).*/
    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode();

    /** Pasa a modo de localización y mapeo.*/
    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();

    /**
     * Comando para reiniciar el sistema, que entre otras cosas limpia el mapa.
     *
     * Marca el flag mbReset = true, para que lo procese System::TrackMonocular, que está en el while principal.
     * Éste siplemente resetea el flag e invoca Tracking::Reset, donde realmente comienza el reseteo.
     *
     * El sistema recién reseteado se pone en modo de inicialización, pero la carga de un mapa lo puede pasar directamente a relocalización.
     *
     * Invocado sólo desde:
     * - Tracking::Track cada vez que falla la inicialización.
     * - Viewer::Run invocado por el usuario, desde otro hilo.
     */
    // Reset the system (clear map)
    void Reset();


    /**
     * Solicita el cierre del sistema.
     * Solicita terminar a todos los hilos, y espera hasta que terminen.
     * Los datos permanecen en memoria.  Esta función se debe invocar antes de guardar la trayectoria.
     */
    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();

    // Save camera trajectory in the TUM RGB-D dataset format.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    //void SaveTrajectoryTUM(const string &filename);

    // Save keyframe poses in the TUM RGB-D dataset format.
    // Use this function in the monocular case.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    // Save camera trajectory in the KITTI dataset format.
    // Call first Shutdown()
    // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    //void SaveTrajectoryKITTI(const string &filename);

    /** Devuelve el mapa.  @returns Mapa del mundo.*/
    Map* GetMap();


    // TODO: Save/Load functions
    // LoadMap(const string &filename);

    /** Informa el estado.*/
    //	enum eTrackingState GetStatus();
    int GetStatus();

    /** Establece el estado.*/
    void SetStatus(int status);

    /** Devuelve un puntero al objeto de tracking.*/
    Tracking* GetTracker() {return mpTracker;}


    /* Agregados */

    /** Video de entrada.*/
    //cv::VideoCapture* video;

    /** Imagen de entrada.*/
    cv::Mat imagenEntrada;

    /**
     * Flags para guardar archivos.
     *
     * 1- Guardar archivo en formato mínimo: opera sobre KeyFrame.
     */
    unsigned int guardadoFlags = 0;

    // Hice públicos los singletons, para que se puedan obtener vía System

    /** Visor, maneja el visor del mapa y el de cámara.   Usa Pangolin.  Lo hice público para que main pueda interactuar con él a través de System.*/
    // The viewer draws the map and the current camera pose. It uses Pangolin.
    Viewer* mpViewer;

    // ORB vocabulary used for place recognition and feature matching.
    /** Vocabulario ORB BOW.*/
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    /** Base de datos de keyframes, para búsqueda de keyframes por BOW.*/
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    /** Mapa del mundo, con la lista de puntos 3D y de keyframes.*/
    Map* mpMap;

    /** Objecto de tracking.*/
    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

    /** Objeto de mapeo.*/
    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

    /** Objeto de cierre de bucles.*/
    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;

    /** Visor de cámara, que muestra la imagen con los puntos detectados sobre ella.*/
    FrameDrawer* mpFrameDrawer;

    /** Visor del mapa que muestra los puntos 3D y la pose de la cámara.   Usa Pangolin.*/
    MapDrawer* mpMapDrawer;

    /** Serializador singleton.*/
    Serializer *serializer;

private:

    // Input sensor
	/** Tipo de sensor, modo de operación: mono, estéreo o mapa de profundidad.*/
    eSensor mSensor;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    /** Hilo de mapeo.*/
    std::thread* mptLocalMapping;
    /** Hilo de cierre de bucle.*/
    std::thread* mptLoopClosing;
    /** Hilo de visor.*/
    std::thread* mptViewer;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;

    // Change mode flags
    std::mutex mMutexMode;
    /** Señal solicitud de modo only tracking, sin mapeo.*/
    bool mbActivateLocalizationMode;
    /** Señal solicitud de modo tracking&mapping.*/
    bool mbDeactivateLocalizationMode;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
