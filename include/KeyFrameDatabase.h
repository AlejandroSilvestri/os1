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

#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

//#include "KeyFrame.h"
//#include "Frame.h"
#include "ORBVocabulary.h"

#include<mutex>


namespace ORB_SLAM2
{

class KeyFrame;
class Frame;


/**
 * Lista invertida de KeyFrames, accesible por BoW.
 * Para cada palabra del vocabulario, hay una lista de keyframes que la contienen.
 */
class KeyFrameDatabase
{
public:
	/**
	 * Constructor que dimensiona mvInvertedFile del mismo tamaño del vocabulario.
	 * Se invoca sólo desde el constructor de System.
	 */
    KeyFrameDatabase(const ORBVocabulary &voc);


    /**
     * Agrega un KeyFrame a la base de datos.
     * Lo agrega a las listas de keyframes de cada BoW que tenga registrado.
     * @param pKF KeyFrame
     *
     * Invocado sólo desde LoopClosing::DetectLoop, un método que se invoca durante la creación de cada keyframe,
     * en el hilo de LoopClosing.
     */
    void add(KeyFrame* pKF);

    /**
     * Borra un KeyFrame de la base de datos.
     * Lo borra de cada lista de keyframes de cada BoW que tenga registrado.
     * @param pKF KeyFrame
     */
    void erase(KeyFrame* pKF);

    /**
     * Libera el vocabulario y asigna memoria para volver a cargarlo, invocado sólo desde Tracking::Reset.
     */
    void clear();

	/**
	 * A partir de un KeyFrame intenta detectar un bucle.
	 *
     * @param pKF KeyFrame
     * @param minScore Cantidad mínima de BoW coincidentes a encontrar para considerar que un KeyFrame es candidato para cerrar el bucle.
	 * @returns Los KeyFrames candidatos a cerrar el bucle, que tendrán que pasar la prueba de la pose.
	 *
	 * Los candidatos devueltos son los que tienen una cantidad mínima de BoWs coincidentes con los del KeyFrame argumento.
	 *
	 * Invocado sólo desde LoopClosing::DetectLoop, una única vez por cada keyframe, siempre con keyframes nuevos o recientes.
	 */
    // Loop Detection
	std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame* pKF, float minScore);

	/**
	 * Intenta relocalizar a partir de un cuadro.
	 * @param F Cuadro actual cuya pose se pretende localizar.
	 * @returns Vector de KeyFrames candidatos a relocalización, en los que se encontraron suficiente coincidencias BoW.
	 *
	 * Busca todos los keyframes que comparten algún BoW con el frame actual argumento.
	 *
	 * Identifica el que más BoWs tiene en común, y arma un conjunto con los keyframes con al menos 80% de ese máximo.
	 *
	 * Para cada keyframe calcula un puntaje de similaridad al comparar su conjunto de BoWs con el del cuadro actual.
	 * El puntaje es la suma de los pesos de los BoW coincidentes.
	 *
	 * Luego acumula en cada keyframe el puntaje de todos los keyframes covisibles.
	 *
	 * Finalmente devuelve los keyframes con al menos 75% del mejor puntaje.
	 *
	 * Estos candidatos luego deben pasar una prueba de pose.
	 */
	// Relocalization
	std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);

	/**
	 * Agregado, para postergar la carga del vocabulario desde archivo.
	 * mvInvertedFile ajusta su tamaño durante la construcción.
	 * Este método permite construir el singleton antes de contar con el vocabulario.
	 * Se invoca desde el constructor de System.
	 */
	void resizeInvertedFile(size_t);

protected:

	/**
	 * Vocabulario BoW.
	 */
	// Associated vocabulary
	const ORBVocabulary* mpVoc;

	/**
	* Cada elemento del vector corresponde a una palabra del vocabulario BoW.
	* Cada elemento consiste de una lista de keyframes que contienen esa palabra BoW.
	* De este modo, a partir de la palabra Bow (la palabra es el índice en este vector), se obtiene una lista de KeyFrames que
	* observan puntos singulares cuyos descriptores corresponden a ella.
	* Se utiliza para cierre de bucles y relocalización.
	*/
	// Inverted file
	std::vector<std::list<KeyFrame*> > mvInvertedFile;

	// Mutex
	std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif
