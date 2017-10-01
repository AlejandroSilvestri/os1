/*
 * Serializer.h
 *
 *  Created on: 6/1/2017
 *      Author: alejandro
 *
 * Crea clases envoltorio para acceder a las propiedades protegidas.
 */

#ifndef SERIALIZER_H_
#define SERIALIZER_H_

#include <boost/serialization/access.hpp>
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"

namespace ORB_SLAM2{

class System;
class Serializer;


/**
 * Brinda acceso a las propiedades protegidas de Map.
 * MapAccess deriva de Map, para ser usado en casteo.
 */
class MapAccess: Map{
	friend class boost::serialization::access;	// no estoy seguro que se necesite.
	friend class Serializer;
};



/**
 * Clase encargada de la serialización (guarda y carga) del mapa.
 * Singleton, clase estática, no se instacia.  Tiene métodos estáticos y variables estáticas, que se inicializan con init(...)
 *
 * No guarda estado, sólo registra punteros al sistema y al mapa.
 */

class Serializer{
public:
	/**
	 * Inicializador del singleton.
	 * Carga las variables estáticas.
	 */
	static void init(System*);


	/**
	 * Serializa el mapa.
	 * Punto inicial de la serialización del mapa.
	 * Puesto que el mapa es un singleton, y que ya existe al momento de cargar un mapa,
	 * no se serializa el objeto mapa sino solamente sus propiedades esenciales en este orden:
	 * - mnMaxKFid
	 * - mspMapPoints
	 * - mspKeyFrames
	 * - mvpKeyFrameOrigins
	 *
	 * Los puntos del mapa no serializan punteros a keyframes, lo que garantiza que primero se guardan todos los puntos,
	 * luego todos los keyframes.  Finalmente mvpKeyFrameOrigins sólo guarda punteros a los keyframes ya serializados.
	 *
	 * mapLoad y mapSave invocan dos versiones de esta plantilla.
	 * mapSave prepara los datos antes de serializar, mapLoad serializa primero y luego recontruye.
	 */
	template<class Archivo> static void serialize(Archivo& ar, const unsigned int version);

	/**
	 * Guarda el mapa desde el archivo binario.
	 *
	 * @param archivo es el nombre del archivo binario a abrir.
	 *
	 * Depura el mapa, eliminando elementos remanentes que perduran por error.
	 * Abre el archivo y guarda el mapa en binario.
	 */
	static void mapSave(std::string archivo);

	/**
	 * Carga el mapa desde el archivo binario.
	 *
	 * @param archivo es el nombre del archivo binario a abrir.
	 *
	 * Abre el archivo binario y carga su contenido.
	 *
	 * Omite propiedades efímeras y reconstruíbles.
	 *
	 * Es necesario asegurar que otros hilos no modifiquen el mapa ni sus elementos (keyframes y mappoints) durante la carga,
	 * pues pueden corromper los datos en memoria, con alta probabilidad de abortar la aplicación por Seg Fault.
	 * Los otros hilos deben detenerse antes de invocar save.
	 */
	static void mapLoad(std::string archivo);



	/**
	 * Serializa un vector o set de MapPoints o KeyFrames.
	 * Realiza la conversión de MapPoint a la clase especializada MapPointSerializer, o de KeyFrame a KeyFrameSerializer
	 */
	template<class Archivo> static void serializeContainer(Archivo& ar, vector<MapPoint*>& v);
	template<class Archivo> static void serializeContainer(Archivo& ar, vector<KeyFrame*>& v);
	template<class Archivo> static void serializeContainer(Archivo& ar, set<MapPoint*>& s);
	template<class Archivo> static void serializeContainer(Archivo& ar, set<KeyFrame*>& s);
	template<class Archivo> static void serializeSetKF(Archivo& ar, set<KeyFrame*>& s);




	/**
	 * La serialización requiere un keyframe cualquiera para la construcción de MapPoints, con datos efímeros que enseguida se pisan.
	 * Aquí se registra un puntero a un keyframe que será usado para tal propósito.
	 * Es estático para poder referenciarse desde la inicialización de MapPointSerializer.
	 * No se puede escribir en el contructor de Serializer, por lo tanto se escribe en MapLoad.
	 */
	static KeyFrame* KFMuleto;

	/**
	 * Puntero a System, de donde se tomarán otros objetos, como por ejemplo el mapa.
	 */
	static System* sistema;

	/**
	 * Frame muleto.
	 * Para construir el keyframe muleto que requiere el constructor de MapPoint durante la serialización,
	 * y para construir los keyframes durante la serialización.
	 */
	static Frame* frame;

	/**
	 * Puntero al mapa a serializar.
	 * El mapa es un singleton, no se contruye durante la carga de un mapa, sólo se le actualizan sus propiedades.
	 */
	static MapAccess *mapa;

	/**
	 * Depura los conjuntos de keyframes y mappoints que constituyen el mapa.
	 *
	 * Realiza una chequeo de todos los keyframes en MapPoints.mObservations y mappoints en KeyFrames.mvpMapPoints,
	 * constatando que:
	 *
	 * - no sean malos (isBad)
	 * - estén en el mapa (en Map::mspMapPoints y Map::mspKeyFrames)
	 *
	 * Si no cumplen estos requisitos, los elimina.
	 *
	 * Con esto se evita la serialización de elementos retirados del mapa, que por error siguen apuntados desde algún lugar.
	 *
	 */
	static void depurar();
};


/**
 * Envoltorio que agrega capacidades de serialización a MapPoint.
 * Sólo agrega métodos, no propiedades.
 * Esta clase se usa exclusivamente en casting.
 */
class MapPointSerializer: public MapPoint{
public:
	//static KeyFrame* KFMuleto;

	/**
	 * Constructor por defecto de MapPoint para el serializador.
	 *
	 * Se encarga de inicializar las variables const de MapPoint, para que el compilador no chille.
	 */
	MapPointSerializer();
	friend class boost::serialization::access;
	friend class Serializer;

	/**
	 * Serializador de MapPoint
	 * Se invoca al serializar Map::mspMapPoints y KeyFrame::mvpMapPoints, cuyos mapPoints nunca tienen mbBad true.
	 * La serialización de MapPoint evita punteros para asegurar el guardado consecutivo de todos los puntos antes de proceder con los KeyFrames.
	 * Esto evita problemas no identificados con boost::serialization, que cuelgan la aplicación al guardar.
	 *
	 * Versionado:
	 * La versión 1 reconstruye mRefKF, y ya no guarda mnFirstKFid.
	 */
	template<class Archivo> void serialize(Archivo&, const unsigned int);
	// Fin del agregado para serialización


};

/**
 * Envoltorio que agrega capacidades de serialización a KeyFrame
 */
class KeyFrameSerializer: public KeyFrame{
public:

	/**
     * Constructor por defecto para KeyFrame
     * Se ocupa de inicializar los atributos const, para que el compilador no chille.
     * Entre ellos inicializa los atributos no serializables (todos punteros a singletons).
     * Luego serialize se encarga de cambiarle los valores, aunque sean const.
     */
	KeyFrameSerializer();

    /**
     * Reconstruye las observaciones de los puntos observados por el keyframe.
     *
     * Usado exclusivamente en la serialización, para reconstruir datos redundantes.
     *
     * Los keyframes registran los puntos obervados, y éstos registran los keyframes que los observan.
     * Sólo los primeros se serializan, los segundos se reconstruyen con este método.
     *
     * Invocado sólo por Serializer::mapLoad
     */
	void buildObservations();

	friend class boost::serialization::access;
	friend class Serializer;


	/**
	 * Serializador para KeyFrame.
	 * Invocado al serializar Map::mspKeyFrames.
	 * No guarda mpKeyFrameDB, que se debe asignar de otro modo.
	 */
	template<class Archivo> void serialize(Archivo&, const unsigned int);
	// Fin del agregado para serialización

	/**
	 * Functor de comparación para ordenar un set de KeyFrame*, por su mnId.
	 * Se usa en el mapa de keyframes, para que se serialicen ordenadamente.
	 */
	struct lessPointer{
		bool operator()(const KeyFrame* k1, const KeyFrame* k2) const{
			return k1->mnId < k2->mnId;
		}
	};

};



}//namespace ORB_SLAM2

#endif /* SERIALIZER_H_ */
