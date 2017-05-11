/*
 * PuntoLejano.h
 *
 *  Created on: 10/2/2017
 *      Author: alejandro
 */

#ifndef PUNTOLEJANO_H_
#define PUNTOLEJANO_H_

//#include "KeyFrame.h"
class KeyFrame;

#include <opencv2/opencv.hpp>
#include <stdio.h>
using namespace std;
using namespace cv;
namespace ORB_SLAM2{


/**
 * Conjunto que apunta a una determinada observación en un determinado keyframe.
 *
 * Se podría lograr lo mismo con makepair.
 */
typedef pair<KeyFrame*, int> Observacion;
//struct Observacion{
	/** Keyframe con la observación.*/
//	KeyFrame *pKF;

	/** Índice de la observación en los vectores alineados.*/
//	int idx;
//};


/**
 * Punto lejano candidato.
 *
 * Un punto lejano registra un keyframey una determinada observación,  de referencia.
 *
 * También registra otras observaciones.
 */
class PuntoLejano{

	PuntoLejano();

public:
	/**
	 * Observación de referencia.
	 *
	 * El punto lejano se define en la dirección de esta observación.
	 *
	 * Otras observaciones sólo alejan el vértice del cono dominio.
	 */
	Observacion observacionReferencia;

	/**
	 * Observaciones del punto lejano.
	 */
	vector<Observacion> observaciones;

	/** Word Id de BoW de este punto lejano.*/
	unsigned int bow;

	/** Peso del bow.*/
	double bowPeso;

	/**
	 * Vértice del cono del dominio del punto lejano candidato.
	 * Mat float de 3x1
	 */
	Mat vertice;
};

} //namespace ORB_SLAM


#endif /* PUNTOLEJANO_H_ */
