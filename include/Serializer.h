/*
 * Serializer.h
 *
 *  Created on: 6/1/2017
 *      Author: alejandro
 */

#ifndef SERIALIZER_H_
#define SERIALIZER_H_

namespace ORB_SLAM2{

class Map;

/**
 * Clase encargada de la serialización (guarda y carga) del mapa.
 *
 * Singleton, basta con instanciarla una sola vez.
 *
 * No guarda estado, sólo registra punteros al sistema y al mapa.
 */

class Serializer{
public:
	/**
	 * Clase encargada de la serialización (guarda y carga) del mapa.
	 *
	 * Ordena los keyframes por orden de mnId.
	 * Constructor único y por defecto.
	 * Sólo inicializa la propiedad mapa.
	 */
	Serializer(Map*);

	/**
	 * Serializa el mapa.
	 *
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
	template<class Archivo> void serialize(Archivo& ar, const unsigned int version);

	/**
	 * Guarda el mapa desde el archivo binario.
	 *
	 * @param archivo es el nombre del archivo binario a abrir.
	 *
	 * Depura el mapa, eliminando elementos remanentes que perduran por error.
	 * Abre el archivo y guarda el mapa en binario.
	 */
	void mapSave(std::string archivo);

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
	void mapLoad(std::string archivo);


protected:

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
	void depurar();

	/**
	 * Puntero al mapa a serializar.
	 * El mapa es un singleton, no se contruye durante la carga de un mapa, sólo se le actualizan sus propiedades.
	 */
	Map *mapa;

};


}//namespace ORB_SLAM2

#endif /* SERIALIZER_H_ */
