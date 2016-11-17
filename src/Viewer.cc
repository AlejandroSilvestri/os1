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
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM2{

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath, cv::VideoCapture* video_):
	video(video_),
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
    cout << endl << "Teclas:" << endl
    		<< "t: Cambia el tamaño de las ventanas de imágenes.  Hay 3 tamaños: 100%, 50% y 25%." << endl
    		<< "u: Undistort, muestra la imagen de entrada antidistorsionada." << endl
    		<< endl
    		<< "Si se procesa un archivo de video:" << endl
    		<< "espacio: Pausa el video." << endl
    		<< "r: Reversa, invierte la secuencia de imágenes del video." << endl
    		<< "" << endl
    		<< "" << endl
    		<< "" << endl
    		<< endl;
}

void Viewer::Run(){
	//cout << "Viewer Run inciando." << endl;

    mbFinished = false;

    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1024,768);
	cout << "Pangolin creado." << endl;

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Seguir la camara",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Puntos del mapa",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Grafo",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Tracking, sin mapeo",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);
    pangolin::Var<bool> menuGuardarMapa("menu.Guardar mapa",false,false);
    pangolin::Var<bool> menuCargarMapa("menu.Cargar mapa",false,false);
    //pangolin::Var<bool> menuSalir("menu.Salir",false,false);


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

    // Línea agregada para un trackbar paramétrico.
    mpTracker->param = 100;
    cv::createTrackbar("Parámetro", "ORB-SLAM2: Current Frame", &mpTracker->param, 1023);

    // Video de entrada con línea de tiempo
    cv::namedWindow("entrada");
    if(duracion) cv::createTrackbar("tiempo", "entrada", &tiempo, video->get(CV_CAP_PROP_FRAME_COUNT));



    bool bFollow = true;
    bool bLocalizationMode = false;

    // Registra la posición anterior, para saber si el usuario lo movió.  Se compara con la variable tiempo, que refleja la posición del trackbar.
    int trackbarPosicionAnterior = 0;

    // Factor de escala de las imágenes
    float factorEscalaImagenParaMostrar = 1.0;

    // Bucle principal del visor
    while(1){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        if(menuLocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }

        //Mostrar mapa con Pangolin
        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        mpMapDrawer->DrawCurrentCamera(Twc);
        if(menuShowKeyFrames || menuShowGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints();

        pangolin::FinishFrame();

        cv::Mat im = mpFrameDrawer->DrawFrame();
        cv::Mat imagenParaMostrar;

        // Mostrar cuadro con imshow
        cv::resize(im, imagenParaMostrar, cv::Size(), factorEscalaImagenParaMostrar, factorEscalaImagenParaMostrar);
        cv::imshow("ORB-SLAM2: Current Frame",imagenParaMostrar);

        // Mostrar imagen de entrada
        if(mostrarEntradaAntidistorsionada){
        	cv::Mat imagenEntrada = mpSystem->imagenEntrada;
        	int ancho = imagenEntrada.cols;
        	int alto = imagenEntrada.rows;
        	cv::Mat K = mpFrameDrawer->K;	// Matriz de calibración tomada del cuadro actual.
        	cv::Mat distCoef = mpFrameDrawer->distCoef;
        	cv::Mat nuevaK = cv::getOptimalNewCameraMatrix(K, distCoef, cv::Size(ancho, alto), 1.0);

        	cv::undistort(imagenEntrada, imagenParaMostrar, K, distCoef, nuevaK);
        	cv::resize(imagenParaMostrar, imagenParaMostrar, cv::Size(), factorEscalaImagenParaMostrar, factorEscalaImagenParaMostrar);
        }else{
            cv::resize(mpSystem->imagenEntrada, imagenParaMostrar, cv::Size(), factorEscalaImagenParaMostrar, factorEscalaImagenParaMostrar);
        }
        cv::imshow("entrada", imagenParaMostrar);


        // duración cero para entradas que no son archivos de video, y que por ende no usa el trackbar.
        // Tampoco se procesa si el usuario movió el trackbar de tiempo, y el cambio de tiempo está en proceso.
        if(duracion){
        	// Es archivo de video

        	// Controlar la barra de tiempo
        	if(!tiempoAlterado){
				if(tiempo == trackbarPosicionAnterior){
					// El usuario no cambió el trackbar de tiempo.  Hay que reflejar la posición actual en el trackbar.
					trackbarPosicionAnterior = video->get(CV_CAP_PROP_POS_FRAMES);
					cv::setTrackbarPos("tiempo", "entrada", trackbarPosicionAnterior);	// indirectamente se asigna también a la variable tiempo
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

        // Retardo de bucle en ms, 1e3/fps
        char tecla = cv::waitKey(mT);

        switch(tecla){
        // Tamaño de las ventanas de imágenes.  T cicla por los divisores 1, 2 y 4.
        case 't':
        	if(factorEscalaImagenParaMostrar == 1.0)
        		factorEscalaImagenParaMostrar = 0.5;
        	else if(factorEscalaImagenParaMostrar == 0.5)
        		factorEscalaImagenParaMostrar = 0.25;
        	else
        		factorEscalaImagenParaMostrar = 1.0;

        	//cout << "t " << factorEscalaImagenParaMostrar << endl;
        	break;

        // Reversa, alterna el sentido del tiempo en archivos de video.  Inocuo en otras entradas.
        case 'r':
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
        	break;

        // Modo automático, que invierte el video cuando se pierde, y lo vuelve a invertir cuando se relocaliza.
        case 'a':
        	modoAutomatico = !modoAutomatico;

        	break;

        }

        if(menuGuardarMapa){
        	menuGuardarMapa = false;
        	Map* mapa = mpMapDrawer->mpMap;

        	// Pasar a tracking para pausar el mapa
        	mpSystem->ActivateLocalizationMode();
        	menuLocalizationMode = true;
        	videoPausado = true;

        	LocalMapping* mapeador = mpTracker->mpLocalMapper;
        	while(!mapeador->isStopped()){
        		mapeador->RequestStop();	// Por las dudas
        		usleep(1000);
        	}

        	char archivo[] = "mapa.bin";
        	mapa->save(archivo);
        	cout << "Mapa guardado." << endl;
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


        if(menuReset)
        {
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

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
    }

    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

}
