/*
 * Video.h
 *
 *  Created on: 27 jun. 2017
 *      Author: alejandro
 */

#ifndef INCLUDE_VIDEO_H_
#define INCLUDE_VIDEO_H_

#include <string>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <condition_variable> // std::condition_variable  		#include <mutex> std::mutex, std::unique_lock


namespace ORB_SLAM2{

/**
 * Singleton que se ejecuta en su propio thread.
 * Encargado de procesar la entrada de video, en un hilo aparte del de Tracking, para paralelizar.
 *
 * Uso:
 *
 * 1- Crear un objeto, un hilo, y en él ejecutar el método Run
 * 2- Abrir orígenes de video con Video::abrirVideo y Video::abrirCamara
 * 3- Alternar entre diferentes flujos con Video::setFlujo
 * 4- Leer imagen con Video::getImagen
 *
 * Al cambiar flujos de video puede ser necesario cambiar la calibración de cámara.  La clase Video no se ocupa de esos cambios.
 */
class Video{
public:
	/**
	 * Constructor
	 */
	Video();
	void Run();

	/**
	 * Devuelve la imagen disponible ya procesada.
	 * Para los flujos en tiempo real (VIDEO_RT y CAM) la imagen disponible se va reemplazando en tiempo real, las que no se leyeron se pierden.
	 * Para los flujos en tiempo no real (VIDEO) la imagen disponible no se reemplaza hasta que se lee.
	 *
	 * @param delta argumento opcional que indica cuantos cuadros avanzar.  -1 para reproducción en reversa.  Usado sólo para flujos VIDEO, es decir, para archivos de video en tiempo no real.  Ignorado con otros flujos.
	 */
	cv::Mat getImagen(int delta = 1);

	/**
	 * Flujos de video:
	 *
	 * NEGRO: imagen negra.
	 * CAM: cámara web, flujo push en tiempo real.  Video::getImage devuelve la última imagen disponible ya procesada.
	 * VIDEO: video de archivo, flujo pull.  Video::getImage devuelve la imagen siguiente.
	 * VIDEO_RT: video de archivo reproducido en tiempo real.  Se reproduce el video en tiempo real, Video::getImage devuelve la última imagen disponible ya procesada.
	 *
	 */
	enum flujos {NEGRO, CAM, VIDEO, VIDEO_RT};

	/**
	 * flujo indica el tipo de flujo.  Es público para facilitar su lectura.  No se debería escribir.
	 */
	flujos flujo = NEGRO;

	/**
	 * Abre el archivo de video y establece el flujo VIDEO.  Se puede cambiar a VIDEO_RT con Video::setFlujo.
	 * Inicializa el objeto cv::VideoCapture video.
	 *
	 * @param ruta del archivo de video
	 * @returns true si hay error, y no se pudo abrir el origen.
	 */
	bool abrirVideo(std::string ruta);

	/**
	 * Abre la cámara en vivo, y establece el flujo en CAM.
	 *
	 * Si no logra abrir la cámara, no hace nada y retorna true indicando error.
	 *
	 * @param cam número de cámara según el sistema operativo.  0 cuando hay una única cámara.
	 * Es -1 por defecto, indicando que busque una cámara para abrir.  Si ya abrió una cámara, busca otra.
	 * @returns true si hay error, y no se pudo abrir el origen.
	 */
	bool abrirCamara(int cam = -1);

	/**
	 * Cambia el tipo de flujo, pero manteniendo la ruta.
	 * Se usa con archivos de video, para cambiar entre tiempo real y no tiempo real.
	 * Se niega a cambiar de flujo si el flujo argumento no se abrió antes con setOrigen.
	 *
	 * @param tipoDeFlujo VIDEO o VIDEO_RT
	 * @returns true hay error, y no se cambió el flujo.  Ocurre cuando todavía no se abrió el origen para ese flujo con setOrigen.
	 */
	bool setFlujo(flujos tipoDeFlujo);


	/**
	 * Posiciona el flujo de video de archivo en la posición indicada.
	 *
	 * @param pos Posición en la línea de tiempo, número de cuadro dentro del archivo.  Si supera la longitud del video, se posiciona en el último cuadro.
	 */
	void setCuadroPos(int pos);

	/**
	 * Propiedad pública de sólo lectura, para indicar la cantidad de cuadros en el video de archivo.
	 * Su valor es 0 si el flujo no es de archivo.
	 * abrirVideo actualiza su valor.
	 */
	int cantidadCuadros = 0;

	/**
	 * Posición del cuadro actual en el video de archivo.
	 * Sólo usado con flujos VIDEO y VIDEO_RT.
	 * Indefinido para otros flujos.
	 */
	int posCuadro = 0;

	/**
	 * Entrada de video desde cámara.
	 * Se inicializa cuando se abre la cámara.
	 * No se debe acceder si el flujo no es CAM.
	 */
	cv::VideoCapture camara,

	/**
	 * Entrada de video desde archivo.
	 * Se inicializa cuando se abre el archivo.
	 * No se debe acceder si el flujo no es VIDEO o VIDEO_RT
	 */
	video;

	/**
	 * Flag que indica si hay una nueva imagen procesada.
	 * true si hay una nueva imagen y todavía no fue leída con getImagen.
	 */
	bool imagenDisponible = false;

	/**
	 * Índice de la cámara abierta.
	 * -1 si no se abrió ninguna.
	 */
	int camaraIndice = -1;



private:

	/**
	 * Última imagen procesada, lista para devolver Video::getImagen
	 */
	cv::Mat imagen,

	/**
	 * Imagen negra para cuando no hay flujo de entrada (flujo ==  NEGRO)
	 * Se genera en el constructor.
	 */
	imNegra;

	/**
	 * mutex para pausar el thread hasta que se consuma la imagen en cola de salida.
	 * Se usa para flujo VIDEO, no tiempo real.
	 */
	std::mutex mutexImagen, mutexEspera;
	std::condition_variable conVarEspera;
};

}	//namespace ORB_SLAM2

#endif /* INCLUDE_VIDEO_H_ */
