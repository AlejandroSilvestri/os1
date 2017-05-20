/*
 * Serializer.cpp
 *
 * Este archivo intenta reunir todos los elementos necesarios para la serialización del mapa de orb-slam2.
 * No todo el código está aquí, es necesario modificar las declaraciones de las clases KeyFrame, Map y MapPoint
 * agregando declaraciones para constructor por defecto, método serialize y clases friend.
 * En la MapPoint.h se debe agregar dentro de la declaración de la clase:
 *
  	MapPoint();
	friend class boost::serialization::access;
	friend class Serializer;
	template<class Archivo> void serialize(Archivo&, const unsigned int);
 *
 * En KeyFrame.h se debe agregar dentro de la declaración de la clase:
 *
  	KeyFrame();
    friend class boost::serialization::access;
	friend class Serializer;
	template<class Archivo> void serialize(Archivo&, const unsigned int);
 *
 * En Map.h se debe agregar dentro de la declaración de la clase:
 *
	friend class Serializer;
 *
 * Map no requiere constructor por defecto.
 *
 * Finalmente, en main.cc o en cualquier otro lugar se debe crear una única instancia de Serializer, pasando el sistema de orb-slam2 como argumento.
 *
 *
 * Partes de este archivo
 *
 * 1- includes
 * 2- defines: macros para la expansión de boost::serialization
 * 3- Serialize de OpenCV: KeyPoint y Map
 * 4- MapPoint: constructor por defecto y definición de MapPoint::serialize
 * 5- KeyFrame: constructor por defecto y definición de KeyFrame::serialize
 * 6- Serializer: definición de la clase.
 *
 *  Created on: 6/1/2017
 *      Author: alejandro
 */

#include <typeinfo>

// boost::serialization
#include <boost/serialization/serialization.hpp>
#include <boost/archive/tmpdir.hpp>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/assume_abstract.hpp>
#include <boost/serialization/version.hpp>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include <typeinfo>

// Clases de orb-slam2
#include "Serializer.h"
#include "Map.h"	// Incluye MapPoint y KeyFrame, éste incluye a KeyFrameDatabase
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "MapPoint.h"
#include "System.h"
#include "Tracking.h"


// Defines
#define TIPOS_ARCHIVOS(FORMATO) \
	typedef boost::archive::FORMATO##_iarchive ArchivoEntrada;\
	typedef boost::archive::FORMATO##_oarchive ArchivoSalida;

// Aquí se define el tipo de archivo: binary, text, xml...
TIPOS_ARCHIVOS(binary)

// Versiones de serialización al guardar
BOOST_CLASS_VERSION(ORB_SLAM2::MapPoint, 1)
BOOST_CLASS_VERSION(ORB_SLAM2::KeyFrame, 1)
BOOST_CLASS_VERSION(ORB_SLAM2::Serializer, 1)


/**
 * La bifurcación consiste en distinguir si serializa carga o guarda.  Se activa definiendo BIFURCACION, pero requiere c++11 para typeinfo
 * En un método serialize(Archivo...) la macro GUARDANDO(Archivo) devuelve true si se está guardando, false si se está cargando.
 *
 * Comentar la siguiente línea para desactivar la capacidad de bifurcación
 */
#define CARGANDO(ARCHIVO) ( typeid(ARCHIVO) == typeid(ArchivoEntrada) )

/** Instanciación explícita*/
#define INST_EXP(CLASE)\
template void CLASE::serialize<ArchivoEntrada>(ArchivoEntrada&,const unsigned int);\
template void CLASE::serialize<ArchivoSalida>(ArchivoSalida&,const unsigned int);




/**
 * OpencCV KeyPoint y Mat
 * Copiado de https://github.com/MathewDenny/ORB_SLAM2 MapPoint.h
 */

// Serialización no intrusiva fuera de la clase: Mat, KeyPoint
BOOST_SERIALIZATION_SPLIT_FREE(::cv::Mat)
namespace boost{namespace serialization{


// KeyPoint
template<class Archive> void serialize(Archive & ar, cv::KeyPoint& kf, const unsigned int version){
	string marca = " KP ";
	ar & marca;
	ar & kf.pt.x;
	ar & kf.pt.y;
	ar & kf.angle;	// Usado en matcher
	ar & kf.octave;	// Usado en LocalMapping y otros
}

// Mat: save
template<class Archive> void save(Archive & ar, const ::cv::Mat& m, const unsigned int version){
	size_t elem_size = m.elemSize();
	int elem_type = m.type();

	ar << m.cols;
	ar << m.rows;
	ar << elem_size;
	ar << elem_type;

	size_t data_size = m.cols * m.rows * elem_size;
	ar << boost::serialization::make_array(m.ptr(), data_size);
}

// Mat: load
template<class Archive> void load(Archive & ar, ::cv::Mat& m, const unsigned int version){
	int cols, rows, elem_type;
	size_t elem_size;

	ar >> cols;
	ar >> rows;
	ar >> elem_size;
	ar >> elem_type;

	m.create(rows, cols, elem_type);

	size_t data_size = m.cols * m.rows * elem_size;
	ar >> boost::serialization::make_array(m.ptr(), data_size);
}


}}

extern ORB_SLAM2::System *Sistema;


// Definiciones de los métodos serialize de las clases MapPoint y KeyFrame
namespace ORB_SLAM2{

//extern System *Sistema;

// MapPoint: usando map ========================================================

/**
 * Constructor por defecto de MapPoint para el serializador.
 * Se encarga de inicializar las variables const, para que el compilador no chille.
 */
MapPoint::MapPoint():
mnFirstKFid(0), nObs(0), mnTrackReferenceForFrame(0),
mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(NULL), mnVisible(1), mnFound(1), mbBad(false),
mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(Sistema->mpMap)//Map::mpMap)
{}

/**
 * Serializador de MapPoint
 * Se invoca al serializar Map::mspMapPoints y KeyFrame::mvpMapPoints, cuyos mapPoints nunca tienen mbBad true.
 * La serialización de MapPoint evita punteros para asegurar el guardado consecutivo de todos los puntos antes de proceder con los KeyFrames.
 * Esto evita problemas no identificados con boost::serialization, que cuelgan la aplicación al guardar.
 *
 * Versionado:
 * La versión 1 reconstruye mRefKF, y ya no guarda mnFirstKFid.
 */
template<class Archivo> void MapPoint::serialize(Archivo& ar, const unsigned int version){
	// Propiedades
	ar & mnId;
	if(version == 0) ar & mnFirstKFid;
	ar & mnVisible;
	ar & mnFound;
	//ar & rgb;	// ¿Serializa cv::Vec3b?

	// Mat
	ar & mWorldPos;
	ar & mDescriptor;	// Reconstruíble, pero con mucho trabajo
}
INST_EXP(MapPoint)


// KeyFrame ========================================================
/**
 * Constructor por defecto para KeyFrame
 * Se ocupa de inicializar los atributos const, para que el compilador no chille.
 * Entre ellos inicializa los atributos no serializables (todos punteros a singletons).
 * Luego serialize se encarga de cambiarle los valores, aunque sean const.
 */
KeyFrame::KeyFrame():
	// Públicas
    mnFrameId(0),  mTimeStamp(0.0), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(Frame::mfGridElementWidthInv),
    mfGridElementHeightInv(Frame::mfGridElementHeightInv),

    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),

    fx(Frame::fx), fy(Frame::fy), cx(Frame::cx), cy(Frame::cy), invfx(Frame::invfx), invfy(Frame::invfy),
    mbf(0.0), mb(0.0), mThDepth(0.0),	// Valores no usados en monocular, que pasan por varios constructores.
    N(0), mnScaleLevels(Sistema->mpTracker->mCurrentFrame.mnScaleLevels),
    mfScaleFactor(Sistema->mpTracker->mCurrentFrame.mfScaleFactor),
    mfLogScaleFactor(Sistema->mpTracker->mCurrentFrame.mfLogScaleFactor),
    mvScaleFactors(Sistema->mpTracker->mCurrentFrame.mvScaleFactors),
    mvLevelSigma2(Sistema->mpTracker->mCurrentFrame.mvLevelSigma2),
    mvInvLevelSigma2(Sistema->mpTracker->mCurrentFrame.mvInvLevelSigma2),
    mnMinX(Frame::mnMinX), mnMinY(Frame::mnMinY), mnMaxX(Frame::mnMaxX), mnMaxY(Frame::mnMaxY),
    mK(Sistema->mpTracker->mCurrentFrame.mK),
    // Protegidas:
    mpKeyFrameDB(Sistema->mpKeyFrameDatabase),
    mpORBvocabulary(Sistema->mpVocabulary),
    mbFirstConnection(false),
	mpParent(NULL),
	mbBad(false),
	mpMap(Sistema->mpMap)
{}

void KeyFrame::buildObservations(){
	// Recorre los puntos
	size_t largo = mvpMapPoints.size();
	for(size_t i=0; i<largo; i++){
		MapPoint *pMP = mvpMapPoints[i];
		if (pMP)
			pMP->AddObservation(this, i);
	}
}

/**
 * Serializador para KeyFrame.
 * Invocado al serializar Map::mspKeyFrames.
 * No guarda mpKeyFrameDB, que se debe asignar de otro modo.
 *
 */
template<class Archive> void KeyFrame::serialize(Archive& ar, const unsigned int version){
	// Preparación para guardar
	if(!CARGANDO(ar)){
		// Recalcula mbNotErase para evitar valores true efímeros
		mbNotErase = !mspLoopEdges.empty();
	}
	ar & mnId;
	ar & mbNotErase;

	// Mat
	ar & Tcw;	//ar & const_cast<cv::Mat &> (Tcw);
	ar & const_cast<cv::Mat &> (mK);	// Mismo valor en todos los keyframes

	// Contenedores
	if(Sistema->guardadoFlags && 1 && !CARGANDO(ar)){
		// Guardar archivo minimizado: eliminar keypoints no asociados a mappoints

		// Cuenta la cantidad de puntos en mvpMapPoints
		size_t cantidad = count_if(mvpMapPoints.begin(), mvpMapPoints.end(), [](MapPoint *MP){return MP;});

		// Define los vectores alternativos para guardar.  Estos vectores contendrán una cantidad mínima de datos para guardar un archivo menor.  No afecta al sistema.
		cv::Mat mDescriptors2;
		vector<cv::KeyPoint> mvKeysUn2(cantidad);
		vector<MapPoint*> mvpMapPoints2(cantidad);

		// Copia los valores a preservar
		int i, j=0;
		for(i=0; i<N; i++){
			if(mvpMapPoints[i]){
				mDescriptors2.push_back(mDescriptors.row(i));
				mvKeysUn2[j] = mvKeysUn[i];
				mvpMapPoints2[j] = mvpMapPoints[i];
				j++;
			}
		}

		ar & const_cast<cv::Mat &> (mDescriptors2);
		ar & const_cast<std::vector<cv::KeyPoint> &> (mvKeysUn2);
		ar & mvpMapPoints2;

	} else {
		ar & const_cast<cv::Mat &> (mDescriptors);
		ar & const_cast<std::vector<cv::KeyPoint> &> (mvKeysUn);
		ar & mvpMapPoints;
	}

	if(mbNotErase){
		cout << "KF con ejes de bucle " << mnId << endl;
		ar & mspLoopEdges;
	}

	// Punteros

	if(version == 0){
		// mpParent, al guardar lo pasa a NULL si no está en el mapa.  Al cargar se regenerará con UpdateConnections.
		if(mpParent)	// NULL si está cargando
			if(mpParent->isBad() || !mpMap->isInMap(mpParent)){
				mpParent = NULL;	// Evita que se serialice un KF fuera de mapa.
				cout << "Padre anulado por no estar en el mapa, se serializa como NULL" << endl;
			}
		ar & mpParent;
		mpParent = NULL;	// Anulo la carga para generar el spanning tree
	}


	// Sólo load
	if(CARGANDO(ar)){
		// Reconstrucciones
		//int n = const_cast<int &> (N);
		const_cast<int &> (N) = mvKeysUn.size();

		mbNotErase = !mspLoopEdges.empty();

		ComputeBoW();	// Sólo actúa al cargar, porque si el keyframe ya tiene los datos no hace nada.
		SetPose(Tcw);
		// UpdateConnections sólo se puede invocar luego de cargados todos los keyframes, para generar mConnectedKeyFrameWeights, mvpOrderedConnectedKeyFrames, mvOrderedWeights y msChildrens


		// Reconstruir la grilla

		// Dimensiona los vectores por exceso
		std::vector<std::size_t> grid[FRAME_GRID_COLS][FRAME_GRID_ROWS];
		int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
		for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
			for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
				grid[i][j].reserve(nReserve);

		for(int i=0;i<N;i++){
			const cv::KeyPoint &kp = mvKeysUn[i];
			int posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
			int posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

			//Keypoint's coordinates are undistorted, which could cause to go out of the image
			if(!(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS))
				grid[posX][posY].push_back(i);
		}

		// Copia al vector final.  No sé si esta parte agrega valor.
		mGrid.resize(mnGridCols);
		for(int i=0; i<mnGridCols;i++){
			mGrid[i].resize(mnGridRows);
			for(int j=0; j<mnGridRows; j++)
				mGrid[i][j] = grid[i][j];
		}
	}
}
INST_EXP(KeyFrame)







/**
 * Clase encargada de la serialización (guarda y carga) del mapa.
 *
 * Ordena los keyframes por orden de mnId.
 */

Serializer::Serializer(Map *mapa_):mapa(mapa_){}

/**
 * Functor de comparación para ordenar un set de KeyFrame*, por su mnId.
 *
 * Se usa en el mapa de keyframes, para que se serialicen ordenadamente.
 */
template<class T> struct lessId{
	bool operator()(const T* t1, const T* t2) const{
		return t1->mnId < t2->mnId;
	}
};



template<class Archivo> void Serializer::serialize(Archivo& ar, const unsigned int version){

	if(version){
		// Encabezado, a partir de la versión 1
		string marca = "Mapa de OS1 - ORB-SLAM2.  Versión Map " + to_string(version) + ".";
		ar & marca;

		int flags = 0;
		ar & flags;
	} else {
		// Única propiedad, sólo en versión 0, porque es reconstruíble
		ar & mapa->mnMaxKFid;
	}

	// Puntos del mapa
	ar & mapa->mspMapPoints;
	cout << "MapPoints serializados: " << mapa->mspMapPoints.size() << endl;



	// KeyFrames del mapa
	if(CARGANDO(ar)){
		ar & mapa->mspKeyFrames;
	} else {
		// Guardando: primero ordena el set de KF por mnId
		std::set<KeyFrame*, lessId<KeyFrame>> KFOrdenados(mapa->mspKeyFrames.begin(), mapa->mspKeyFrames.end());
		ar & KFOrdenados;
	}
	cout << "KeyFrames serializados: " << mapa->mspKeyFrames.size() << endl;
	cout << "Último keyframe: " << mapa->mnMaxKFid << endl;


	if(version == 0)
		ar & mapa->mvpKeyFrameOrigins;
}


//void Serializer::mapSave(char* archivo){
void Serializer::mapSave(std::string archivo){
	// Elimina keyframes y mappoint malos de KeyFrame::mvpMapPoints y MapPoint::mObservations
	depurar();

	// Previene la modificación del mapa por otros hilos
	unique_lock<mutex> lock(mapa->mMutexMap);

	// Abre el archivo
	std::ofstream os(archivo);
	ArchivoSalida ar(os, ::boost::archive::no_header);

	// Guarda mappoints y keyframes
	serialize<ArchivoSalida>(ar, 0);
}

/**
 * Carga el mapa desde el archivo binario.
 *
 * @param archivo es el nombre del archivo binario a abrir.
 *
 * Abre el archivo binario y carga su contenido.
 *
 * Luego reconstruye propiedades y grafos necesarios.
 *
 * Omite propiedades efímeras, limitándose a inicializarlas en el constructor por defecto.
 *
 * Es necesario asegurar que otros hilos no modifiquen el mapa ni sus elementos (keyframes y mappoints) durante la carga,
 * pues pueden corromper los datos en memoria, con alta probabilidad de abortar la aplicación por Seg Fault.
 */
//void Serializer::mapLoad(char* archivo){
void Serializer::mapLoad(std::string archivo){
	// A este punto se llega con el mapa limpio y los sistemas suspendidos para no interferir.

	{
		// Por las dudas, se previene la modificación del mapa mientras se carga.  Se libera para la reconstrucción, por las dudas.
		unique_lock<mutex> lock(mapa->mMutexMap);

		// Abre el archivo
		std::ifstream is(archivo);
		ArchivoEntrada ar(is, ::boost::archive::no_header);

		// Carga mappoints y keyframes en Map
		serialize<ArchivoEntrada>(ar, 0);
	}



	// Reconstrucción, ya cargados todos los keyframes y mappoints.

	long unsigned int maxId = 0;

	/*
	 * Recorre los keyframes del mapa:
	 * - Reconstruye la base de datos agregándolos
	 * - UpdateConnections para reconstruir los grafos
	 * - MapPoint::AddObservation sobre cada punto, para reconstruir mObservations y mObs
	 */
	cout << "Reconstruyendo DB, grafo de conexiones y observaciones de puntos 3D..." << endl;
	KeyFrameDatabase* kfdb = Sistema->mpKeyFrameDatabase;//Map::mpKeyFrameDatabase;
	for(KeyFrame *pKF : mapa->mspKeyFrames){
		// Busca el máximo id, para al final determinar nNextId
		maxId = max(maxId, pKF->mnId);

		// Agrega el keyframe a la base de datos de keyframes
		kfdb->add(pKF);

		// UpdateConnections para reconstruir el grafo de covisibilidad.  Conviene ejecutarlo en orden de mnId.
		pKF->UpdateConnections();
		if(pKF->mConnectedKeyFrameWeights.empty() && pKF->mnId){
			// Este keyframe está aislado del mundo, se elimina
			cout << "Eliminando KF aislado del mundo " << pKF->mnId << endl;
			pKF->SetBadFlag();
		}

		// Reconstruye las observaciones de todos los puntos
		size_t largo = pKF->mvpMapPoints.size();
		for(size_t i=0; i<largo; i++){
			MapPoint *pMP = pKF->mvpMapPoints[i];
			if (pMP)
				pMP->AddObservation(pKF, i);
		}

		/*
		 * Verificar que todos los ejes de bucle estén en el mapa.
		 */
		for(auto pKFLoop: pKF->mspLoopEdges)
			if(mapa->isInMap(pKFLoop)){
				cout << "Eliminado de grafo de bucle KF " << pKF->mnId << endl;
				pKF->mspLoopEdges.erase(pKFLoop);
			}
	}

	mapa->mnMaxKFid = maxId;	// Último KF del mapa
	KeyFrame::nNextId = maxId + 1;	// Próximo KF


	/**
 	 * Genera un spanning tree asignando mpParent a cada KeyFrame
 	 * Asigna un padre a cada KF (excepto al mnId 0).
 	 * El padre no puede ser huérfano.
 	 * Itera, y termina cuando ya no asigna ningún padre, o cuando ya asignó a todos los KF.
	 */

	cout << "Reconstruyendo árbol de expansión (spanning tree) de keyframes..." << endl;

	// Set de keyframes ordenados por mnId.
	std::set<KeyFrame*, lessId<KeyFrame>> KFOrdenados(mapa->mspKeyFrames.begin(), mapa->mspKeyFrames.end());

	// Este vector debería estar vacío, y se le agrega el único elemento, que es el primer keyframe.
	mapa->mvpKeyFrameOrigins.push_back(*KFOrdenados.begin());

	//KFOrdenados.begin()->mpParent;

	// Cantidad de padres asignados en cada iteración, y total.
	int nPadres = -1, nPadresTotal = 0;
	while(nPadres){
		nPadres = 0;
		for(auto pKF: KFOrdenados)
			if(!pKF->mpParent && pKF->mnId)	// Para todos los KF excepto mnId 0, que no tiene padre
				for(auto pConnectedKF : pKF->mvpOrderedConnectedKeyFrames)
					if(pConnectedKF->mpParent || pConnectedKF->mnId == 0){	// Candidato a padre encontrado: no es huérfano o es el original
						nPadres++;
						pKF->ChangeParent(pConnectedKF);
						break;
					}
		nPadresTotal += nPadres;
		cout << "Enramados " << nPadres << endl;
	}

	cout << "KFs: " << KFOrdenados.size() << ", arbolados: " << nPadresTotal << endl;

	/*
	 * Recorre los MapPoints del mapa:
	 * - Determina el id máximo, para luego establecer MapPoint::nNextId
	 * - Reconstruye mpRefKF de cada MapPoint, tomando la primer observación, que debería ser el KF de menor mnId
	 * - Reconstruye propiedades con UpdateNormalAndDepth (usa mpRefKF)
	 */
	cout << "Reconstruyendo keyframes de referencia de cada punto 3D..." << endl;
	for(MapPoint *pMP : mapa->mspMapPoints){
		// Busca el máximo id, para al final determinar nNextId
		maxId = max(maxId, pMP->mnId);


		/*
		// Reconstruye mpRefKF a partir de mnFirstKFid.  setRefKF escribe la propiedad protegida.  Requiere mObservations
		long unsigned int id = pMP->mnFirstKFid;
		std::set<KeyFrame*>::iterator it = std::find_if(mapa->mspKeyFrames.begin(), mapa->mspKeyFrames.end(), [id](KeyFrame *KF){return KF->mnId == id;});
		if(it != mapa->mspKeyFrames.end())
			pMP->mpRefKF = *it;
		else
			pMP->mpRefKF = (*pMP->mObservations.begin()).first;
		 */

		// Reconstruye mpRefKF.  Requiere mObservations
		if(pMP->mObservations.empty()){
			cout << "MP sin observaciones" << pMP->mnId << ".  Se marca como malo." << endl;
			pMP->SetBadFlag();
			continue;
		}
		// Asume que la primera observación es la de menor mnId.
		pMP->mpRefKF = (*pMP->mObservations.begin()).first;

		if(!pMP->mpRefKF)	// No debería ser true nunca
			cout << "MP sin mpRefKF" << pMP->mnId << ", ¿está en el mapa? " << mapa->isInMap(pMP) <<endl;

		/* Reconstruye:
		 * - mNormalVector
		 * - mfMinDistance
		 * - mfMaxDistance
		 *
		 * mediante UpdateNormalAndDepth, que requiere haber reconstruído antes mpRefKF.
		 */
		pMP->UpdateNormalAndDepth();
	}
	MapPoint::nNextId = maxId + 1;

	cout << "Reconstrucción terminada." << endl;

}

/**
 * Depura los conjuntos de keyframes y mappoints que constituyen el mapa->
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
void Serializer::depurar(){
	cout << "\nDepurando..." << endl;
	//Primero se deben eliminar los puntos del keyframe, luego los keyframes de los puntos.

	// Anula puntos malos en el vector KeyFrame::mvpMapPoints
	for(auto &pKF: mapa->mspKeyFrames){	// Keyframes del mapa, en Map::mspKeyFrames

		// Anula puntos malos, y advierte si no está en el mapa.
		// Usualmente no encuentra nada.
		auto pMPs = pKF->GetMapPointMatches();	// Vector KeyFrame::mvpMapPoints
		for(int i=pMPs.size(); --i>=0;){
			auto pMP = pMPs[i];

			if(!pMP) continue;	// Saltear si es NULL

			if(pMP->isBad()){	// Si el mappoint es malo...
				cout << "Mal MP:" << pMP->mnId << ". ";
				pKF->EraseMapPointMatch(i);	// Lo reemplaza por NULL
				cout << "KF " << pKF->mnId << endl;
			} else if(!mapa->isInMap(pMP)){	// Agregar al mapa
				mapa->AddMapPoint(pMP);
				cout << "MP agregado al mapa:" << pMP->mnId << ". ";
			}
		}

		// Anula de los ejes de bucle los KF malos o que no estén en el mapa
		for(auto pKFLoop: pKF->mspLoopEdges)
			if(pKFLoop->isBad() || !mapa->isInMap(pKFLoop)){
				pKF->mspLoopEdges.erase(pKFLoop);
				cout << "Eliminado de grafo de bucle KF " << pKF->mnId << endl;
			}

		// Revisa
	}
}

}	// namespace ORB_SLAM2
