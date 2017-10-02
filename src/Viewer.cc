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

#include "Viewer.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"
//#include "Video.h"
#include <pangolin/pangolin.h>

//#include <opencv2/core.hpp>	// Cargado en Video.h
#include <opencv2/imgcodecs.hpp>

#include <mutex>

namespace ORB_SLAM2{

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(false), mbStopRequested(false)
{
	cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];

    // Explicación de las funciones de teclado
    cout << endl << "Teclas que actúan sobre la ventana FrameDrawer:" << endl
    		<< "t: Cambia el tamaño de las ventanas de imágenes.  Hay 3 tamaños: 100%, 50% y 25%." << endl
    		<< "e: Entrada.  Alterna entre mostrar y ocultar la imagen de entrada a color.  Puede mostrarse distorsionada o antidistorsionada (undistort) con 'u'" << endl
    		<< "u: Undistort, muestra la imagen de entrada antidistorsionada." << endl
    		<< endl
    		<< "Si se procesa un archivo de video:" << endl
    		<< "espacio: Pausa el video." << endl
    		<< "r: Reversa, para invertir la secuencia de imágenes del video.  Alterna entre reversa y normal." << endl
    		<< "a: Automático.  Cuando pierde el tracking, invierte la dirección del video hasta relocalizarse." << endl
    		<< endl
    		<< "Otros:" << endl
    		<< "p: Pose, muestra la matriz de pose, de 4x4, compuesta de la submatriz rotación de 3x3, el vector traslación, y la última fila 0,0,0,1" << endl
    		<< "c: Comprimir.  Alterna entre modo comprimido y normal, que se usa al guardar un mapa." << endl
    		<< "w: abre la webcam, usando la calibración webcam.yaml.  Cicla entre webcams si hay más de una." << endl
    		<< "" << endl
    		<< "" << endl
    		<< "" << endl
    		<< endl;
}

void Viewer::Run(){
    mbFinished = false;

    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1024,768);
	cout << "Pangolin creado." << endl;

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Panel y botonera
    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Seguir la camara",true,true);
    pangolin::Var<bool> menuPajaro("menu.Pajaro", false, true);
    pangolin::Var<bool> menuShowPoints("menu.Puntos del mapa",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Grafo",true,true);
    pangolin::Var<bool> menuShowRgb("menu.Puntos a color",false,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Tracking, sin mapeo",false,true);
    pangolin::Var<bool> menuCargarMapa("menu.Cargar mapa", false, false);
    pangolin::Var<bool> menuGuardarMapa("menu.Guardar mapa", false, false);
    pangolin::Var<bool> menuAbrirVideo("menu.Abrir video", false, false);
    pangolin::Var<bool> menuGrabar("menu.Grabar video", false, false);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);
    pangolin::Var<bool> menuSalir("menu.Salir",false,false);


    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
		pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
		pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
    );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
		.SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
		.SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();




	cout << "Ventana para frame." << endl;
    cv::namedWindow("ORB-SLAM2: Current Frame");
    {
    	unique_lock<mutex> lock(mutexIniTrackbar);
    	cv::createTrackbar("tiempo", "ORB-SLAM2: Current Frame", &tiempo, duracion? duracion:1);	//Duración 1 para inicializar.  Luego se corrige con setDuracion
    	iniTrackbar = true;
    }

    // Línea agregada para un trackbar paramétrico.
    cv::createTrackbar("Parámetro", "ORB-SLAM2: Current Frame", &mpSystem->mpLocalMapper->param, 1000);

    cv::setMouseCallback("ORB-SLAM2: Current Frame", FrameDrawer::onMouse, mpFrameDrawer);


    // Variables controladas por el usuario

    // Pangolin y orb-slam2
    bool bFollow = true;
    bool bLocalizationMode = false;


    // Registra la posición anterior, para saber si el usuario lo movió.  Se compara con la variable tiempo, que refleja la posición del trackbar.
    int trackbarPosicionAnterior = 0;

    // Factor de escala de las imágenes
    float factorEscalaImagenParaMostrar = 1.0;
    mpFrameDrawer->factorEscalaImagenParaMostrar = 1.0;


    // Inicialización para capturar imágenes de pangolin
    pangolin::Image<unsigned char> buffer;
    pangolin::VideoPixelFormat fmt = pangolin::VideoFormatFromString("RGBA32");
    pangolin::Viewport v = d_cam.v;
    buffer.Alloc(v.w, v.h, v.w * fmt.bpp/8 );
    cv::Mat imgBuffer = cv::Mat(v.h, v.w, CV_8UC4, buffer.ptr);

    // Grabación de video
    cv::VideoWriter grabador;
    bool grabando = false;
    cv::Size tamano;
    float factorEscalaCuadro, factorEscalaMapa;


    // Bucle principal del visor
    while(1){

    	// Análisis de teclas
        char tecla = cv::waitKey(mT);	// Retardo de bucle en ms, 1000/fps
        switch(tecla){
        // Tamaño de las ventanas de imágenes.  T cicla por los divisores 1, 2 y 4.
        case 't':
        	if(factorEscalaImagenParaMostrar == 1.0)
        		factorEscalaImagenParaMostrar = 0.5;
        	else if(factorEscalaImagenParaMostrar == 0.5)
        		factorEscalaImagenParaMostrar = 0.25;
        	else
        		factorEscalaImagenParaMostrar = 1.0;

        	// Informa al FrameDrawer
        	mpFrameDrawer->factorEscalaImagenParaMostrar = factorEscalaImagenParaMostrar;
        	break;

        // Reversa, alterna el sentido del tiempo en archivos de video.  Inocuo en otras entradas.
        case 'r':
        	//cout << "Reversa." << endl;
        	tiempoReversa = !tiempoReversa;
        	sentidoModoAutomatico = tiempoReversa;	// Usado en el modo automático solamente.
        	break;

        // Pausa y resumen.
        case ' ':
        	videoPausado = !videoPausado;
        	break;

        // Alternar imagen antidistorsionada
        case 'u':
        	mostrarEntradaAntidistorsionada = !mostrarEntradaAntidistorsionada;
        	mostrarEntrada = true;
        	break;

            // Mostrar u ocultar imagen de entrada
		case 'e':
			mostrarEntrada = !mostrarEntrada;
			break;

        // Modo automático, que invierte el video cuando se pierde, y lo vuelve a invertir cuando se relocaliza.
        case 'a':
        	modoAutomatico = !modoAutomatico;
        	break;

		// Inicial: debug, contextual, efímero, envía el video al frame 1960, buen lugar de inicialización para un video determinado
		case 'i':
			trackbarPosicionAnterior = 1960;
			cv::setTrackbarPos("tiempo", "ORB-SLAM2: Current Frame", trackbarPosicionAnterior);
			tiempoAlterado = true;
			break;

		// Archivo en versión comprimida (eliminando keypoints inútiles).  Alterna el flag
		case 'c':
			mpSystem->guardadoFlags ^= 1;
			cout << "Flag de guardado (impar para guardado comprimido): " << mpSystem->guardadoFlags << endl;
			break;


		// Muestra en consola la pose del frame actual
		case 'p':
			cout << "\nmCurrentFrame:\n" << mpTracker->mCurrentFrame.mTcw << endl;
			break;

		// Abre webcam, envía flag a main.
		case 'w':
			abrirCamara = true;
			break;

        }



        // Pangolin, visor del mapa
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);
        // Corrige la pose de la cámara de Pangolin, sólo si se está siguiendo la cámara.  Si no, queda como estaba.
        if(menuFollowCamera){
        	if(!bFollow){
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
                bFollow = true;
        	}
        	if(menuPajaro){
        		// Rototraslación del pájaro respecto de la cámara
        		float matriz[] = {	1, 0, 0, 0,
									0, 0, 1, 0,
									0,-1, 0,-0.25,	// Desplazamiento para compensar el desplazamiento establecido para el visor de Pangolin en la inicialización
									0, 0, 0, 1
        		};
        		cv::Mat Tcp = cv::Mat(4,4, CV_32F, matriz);
        		pangolin::OpenGlMatrix Twp;
                mpMapDrawer->GetCurrentOpenGLCameraMatrixModified(Tcp, Twp);
                s_cam.Follow(Twp);
        	} else
        		s_cam.Follow(Twc);
        } else if(bFollow)
        	bFollow = false;

        if(menuLocalizationMode && !bLocalizationMode){
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }else if(!menuLocalizationMode && bLocalizationMode){
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }



        // Mostrar mapa con Pangolin =========================================================
        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        mpMapDrawer->DrawCurrentCamera(Twc);
        if(menuShowKeyFrames || menuShowGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints(menuShowRgb);

        // Captura imagen de pangolin en el Mat imagen
        glReadBuffer(GL_BACK);	// Color de relleno
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(v.l, v.b, v.w, v.h, GL_RGBA, GL_UNSIGNED_BYTE, buffer.ptr );	// v.w es el ancho de la ventana con el panel.  buffer toma la imagen sin el panel, más angosta, y se rellena con una barra negra al a derecha.
        cv::Mat imagenMapa;
        cv::cvtColor(imgBuffer, imagenMapa,  cv::COLOR_RGBA2BGR);
        imagenMapa = imagenMapa.colRange(0, imagenMapa.cols - 175);	// Quita el espacio de la barra de botones, que aparece negro a la derecha
        cv::flip(imagenMapa, imagenMapa, 0);

        pangolin::FinishFrame();



        // imshow ============================================================================

        // Mostrar cuadro con imshow
        {
            cv::Mat imagenFrame = mpFrameDrawer->DrawFrame(
            		1/factorEscalaImagenParaMostrar,
					mpSystem->mpLocalMapper->AcceptKeyFrames()? -1 :  mpSystem->mpLocalMapper->KeyframesInQueue()
			);
            cv::Mat imagenEntrada;	// Imagen de la cámara, puede ser antidistorsionada
            cv::Mat imagenParaMostrar;	// Imagen efímera para mostrar con imshow, con el tamaño elegido por el usuario

            // Mostrar Frame
            cv::resize(imagenFrame, imagenParaMostrar, cv::Size(), factorEscalaImagenParaMostrar, factorEscalaImagenParaMostrar);
			cv::imshow("ORB-SLAM2: Current Frame",imagenParaMostrar);

			// Mostrar imagen de entrada (si lo eligió el usuario, o si está grabando, porque la grabación la necesita.
			if(mostrarEntrada || grabando){
				imagenEntrada = mpSystem->imagenEntrada;
				if(mostrarEntradaAntidistorsionada){
					int ancho = imagenEntrada.cols;
					int alto = imagenEntrada.rows;
					cv::Mat K = mpFrameDrawer->K;	// Matriz de calibración tomada del cuadro actual.

					if(mpTracker->camaraModo)
						// Modo 0, fisheye sin distorsión
						cv::fisheye::undistortImage(imagenEntrada, imagenParaMostrar, K, cv::Mat::zeros(4,1,CV_32F), K);
					else{
						// Modo 1, cámara normal
						cv::Mat distCoef = mpFrameDrawer->distCoef;
						cv::Mat nuevaK = cv::getOptimalNewCameraMatrix(K, distCoef, cv::Size(ancho, alto), 1.0);
						cv::undistort(imagenEntrada, imagenParaMostrar, K, distCoef, nuevaK);
					}
					imagenEntrada = imagenParaMostrar;
					cv::resize(imagenEntrada, imagenParaMostrar, cv::Size(), factorEscalaImagenParaMostrar, factorEscalaImagenParaMostrar);
				}else{
					cv::resize(imagenEntrada, imagenParaMostrar, cv::Size(), factorEscalaImagenParaMostrar, factorEscalaImagenParaMostrar);
				}
				cv::imshow("entrada", imagenParaMostrar);
			}

			// Grabar video con estas imágenes
	        if(menuGrabar && !grabando){
	        	// Inicia la grabación
	        	grabando = true;
	        	factorEscalaMapa = 720.0/imagenMapa.rows;
	        	factorEscalaCuadro = 720.0/(imagenFrame.rows + imagenEntrada.rows);
				tamano = cv::Size(imagenFrame.cols*factorEscalaCuadro + imagenMapa.cols*factorEscalaMapa , 720);

				cout << tamano << ", factorEscalaCuadro: " << factorEscalaCuadro << ", factorEscalaMapa: " << factorEscalaMapa << endl;
	        	grabador.open("video.avi", cv::VideoWriter::fourcc('a','v','c','1')	, 30.0, tamano, true);
	        	//grabador.open("video.avi", cv::VideoWriter::fourcc('M','J','P','G')	, 30.0, tamano, true);
	        	//grabador.open("video.mpg", cv::VideoWriter::fourcc('M','P','G','4') , 30.0, tamano, true);
				//grabador.open("video.wmv", cv::VideoWriter::fourcc('W','M','V','2') , 30.0, tamano, true);
	        } else if(!menuGrabar && grabando){
	        	// Termina la grabación
	        	grabando = false;
	        	grabador.release();
	        }
			if(grabando){
				cv::Mat compaginacion;
				cv::vconcat(imagenEntrada, imagenFrame, compaginacion);
				cv::resize(compaginacion, compaginacion, cv::Size(), factorEscalaCuadro, factorEscalaCuadro);
				cv::resize(imagenMapa, imagenMapa, cv::Size(), factorEscalaMapa, factorEscalaMapa);
				cv::hconcat(compaginacion, imagenMapa, compaginacion);
				grabador << compaginacion;
			}
        }


        // Control de tiempo sobre archivos de video
        // duración cero para entradas que no son archivos de video, y que por ende no usa el trackbar.
        // Tampoco se procesa si el usuario movió el trackbar de tiempo, y el cambio de tiempo está en proceso.
        if(duracion){
        	// Es archivo de video

        	// Controlar la barra de tiempo
        	if(!tiempoAlterado){
				if(tiempo == trackbarPosicionAnterior){
					// El usuario no cambió el trackbar de tiempo.  Hay que reflejar la posición actual en el trackbar.
					trackbarPosicionAnterior = mpFrameDrawer->timestamp;
					cv::setTrackbarPos("tiempo", "ORB-SLAM2: Current Frame", trackbarPosicionAnterior);	// indirectamente se asigna también a la variable tiempo
				}else{
					// El usuario cambió el trackbar de tiempo
					trackbarPosicionAnterior = tiempo;
					tiempoAlterado = true;
				}
            }

        	// El modo automático controla el sentido de avance del video si se pierde
        	if(modoAutomatico){
            	if(mpTracker->mState==Tracking::OK)
            		tiempoReversa = sentidoModoAutomatico;
            	else if(mpTracker->mState==Tracking::LOST)
            		tiempoReversa = !sentidoModoAutomatico;
        	}
        }


        // Acción de botones de Pangolin

        if(menuGuardarMapa){
        	menuGuardarMapa = false;

        	// Pasar a tracking para pausar el mapa
        	mpSystem->ActivateLocalizationMode();
        	menuLocalizationMode = true;
        	videoPausado = true;

        	// Señal para que main guarde el mapa
        	guardarMapa = true;
        }


        if(menuCargarMapa){
        	menuCargarMapa = false;

        	// Pasar a tracking
        	mpSystem->ActivateLocalizationMode();
        	menuLocalizationMode = true;
        	videoPausado = true;

        	// Señal para que main cargue el mapa
        	cargarMapa = true;
        }

        if(menuAbrirVideo){
        	abrirVideo = true;
        	menuAbrirVideo = false;
        }

        if(menuReset){
        	// Reestablece los botones
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;

            // Solicita resetear el sistema, retorna inmediatamente.
            mpSystem->Reset();
            menuReset = false;
        }

        if(menuSalir){
        	// Termina, cierra OS1
        	exit(0);
        }


        // Realiza una pausa cuando otro hilo puso a Viewer en Stop
        if(Stop()){
            while(isStopped()) usleep(3000);	// Espera hasta que otro hilo retira el Stop
            if(CheckFinish()) break;	// Si el Stop fue el paso previo al Finish, termina saliendo del bucle.
        }
    }

    // Salió del bucle sólo para terminar.
    SetFinish();
}

void Viewer::RequestFinish(){
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish(){
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish(){
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished(){
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop(){
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped(){
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop(){
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested){
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;
}

void Viewer::Release(){
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

void Viewer::setDuracion(int T){
	unique_lock<mutex> lock(mutexIniTrackbar);
	duracion = T;
	if(iniTrackbar) cv::setTrackbarMax("tiempo", "ORB-SLAM2: Current Frame", T<1? 1:T);
}

}	// namespace ORB_SLAM2
