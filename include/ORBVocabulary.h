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


#ifndef ORBVOCABULARY_H
#define ORBVOCABULARY_H

#include "../Thirdparty/DBoW2/DBoW2/FORB.h"
#include "../Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

namespace ORB_SLAM2
{
/** Vocabulario que mapea descriptores ORB con Bow.
 * OBRVocabulary es un tipo basado en plantilla que define un tipo de descriptores y una clase de funciones DBOW2 para manipularlos.
 * OBRVocabulary usa Mat como descriptor (DBoW2::FORB::TDescriptor es Mat), y la clase DBoW2::FORB como implementación específica para ORB
 * de las funciones generales de manipulación de descriptores que requiere DBOW2.
 *
 * main.cc crea la única instancia de este objeto, carga su vocabulario desde un archivo,
 * y luego lo pasa como referencia a los constructores principales, iniciando la cascada.
 * En lo sucesivo, cada objeto que necesita un vocabulario lo recibe por referencia al construirse.
 *
 * Frame, KeyFrame, LoopClosing y Tracking lo denominan mpORBvocabulary.
 * KeyFrameDatabase lo denomina mpVoc.
 * Frame y KeyFrame utilizan solamente su método transform, que obtiene los BoW correspondientes a un conjunto de descriptores.
 * LoopClosing utiliza solamente su método score, que compara dos BoW.
 * KeyFrameDatabase utiliza score y size, este último simplemente informa la cantidad de palabras en el vocabulario.
 * Tracking se limita a pasarlo a los objetos que construye.  Actúa como pasamanos.
 *
 *
 *
 *
 */

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> ORBVocabulary;

} //namespace ORB_SLAM

#endif // ORBVOCABULARY_H
