/*
 * PuntoLejanoDB.h
 *
 *  Created on: 10/2/2017
 *      Author: alejandro
 */

#ifndef PUNTOSLEJANOSDB_H_
#define PUNTOSLEJANOSDB_H_

#include "PuntoLejano.h"
#include "ORBVocabulary.h"
#include <stdio.h>
using namespace std;
namespace ORB_SLAM2{
class PuntoLejano;
/**
 * Singleton, base de datos de puntos lejanos.
 *
 * Esta "base de datos" registra los puntos lejanos candidatos y confirmados.
 *
 * Incluye una base de datos invertida para acceso rápido por BoW.
 */
class PuntosLejanosDB{
public:

	/**
	 * Constructor.
	 */
	PuntosLejanosDB(const ORBVocabulary &voc);

	/**
	 * Agrega un punto lejano candidato a la base de datos.
	 */
	void agregar(PuntoLejano &candidato);

	/**
	 * Quita el punto lejano candidato de la base de datos.
	 */
	void quitar(PuntoLejano &candidato);

// Propiedades
public:

	/**
	 * Vocabulario BoW.
	 */
	const ORBVocabulary *vocabularioBow;

	/**
	 * Base de datos invertida, para acceder a los puntos lejanos a partir de BoW.
	 */
	vector<vector<PuntoLejano*>> BowAPuntosLejanos;

};

} //namespace ORB_SLAM


#endif /* PUNTOSLEJANOSDB_H_ */
