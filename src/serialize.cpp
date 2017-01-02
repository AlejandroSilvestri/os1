/**
 * @file:serialize.cpp
 *
 * Concentra la definición de los métodos serialize de orb-slam2, para facilitar su adaptación a los muchos branches.
 * La declaración se realiza en el archivo hpp de cada clase, de este modo, en public:
 *
 * friend class boost::serialization::access;
 * template<class Archivo> void serialize(Archivo&, const unsigned int);
 *
 * y un constructor por defecto, con la debida inicialización de atributos no serializables.
 *
 * Sus hpp deben agregar includes:
 *
 *
 * Las implementaciones pueden distinguir la dirección del proceso (guardado o carga) con la macro GUARDANDO(ar).
 *
 *
 * Las clases a serializar son:
 * - Map
 * - MapPoint
 * - KeyFrame
 * - KeyFrameDatabase
 * - Mat
 * - KeyPoint
 *
 * Map inicia la carga sobre el objeto existente.  KeyFrameDatabase se reconstruye, no requiere serialización.
 * Se serializan KeyFrame y MapPoint con método serialize, y Mat y KeyPoint con función serialize.
 *
 *
 * Este archivo se divide en las siguietes partes:
 *
 * - includes: boost::serialization, y clases de orb-slam2
 * - defines: macros y typenames para facilitar el uso de boost::serialize
 * - funciones serialize: para objetos de opencv, que no se pueden modificar
 * - métodos serialize: para los objetos de orb-slam2
 *
 * El código de cada serialize fue adaptado de https://github.com/MathewDenny/ORB_SLAM2
 *
 *  Created on: 8/11/2016
 *      Author: alejandro
 *
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

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

// Clases de orb-slam2
#include "Map.h"	// Incluye MapPoint y KeyFrame, éste incluye a KeyFrameDatabase
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "MapPoint.h"
#include "System.h"
#include "Tracking.h"

extern ORB_SLAM2::System* Sistema;

// Defines
#define TIPOS_ARCHIVOS(FORMATO) \
	typedef boost::archive::FORMATO##_iarchive ArchivoEntrada;\
	typedef boost::archive::FORMATO##_oarchive ArchivoSalida;
// Aquí se define el tipo de archivo: binary, text, xml...
TIPOS_ARCHIVOS(binary)

/**
 * La bifurcación consiste en distinguir si serializa carga o guarda.  Se activa definiendo BIFURCACION, pero requiere c++11 para typeinfo
 * En un método serialize(Archivo...) la macro GUARDANDO(Archivo) devuelve true si se está guardando, false si se está cargando.
 *
 * Comentar la siguiente línea para desactivar la capacidad de bifurcación
 */
#define BIFURCACION
#ifdef BIFURCACION
	#include <typeinfo>
	#define CARGANDO(ARCHIVO) ( typeid(ARCHIVO) == typeid(ArchivoEntrada) )
#endif

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


// Definiciones de los métodos serialize de las clases Map, MapPoint y KeyFrame
namespace ORB_SLAM2{


// Map: usando set ========================================================
template<class Archivo> void Map::serialize(Archivo& ar, const unsigned int version){
	cout << "MapPoints: " << mspMapPoints.size() << endl
		 << "KeyFrames: " << mspKeyFrames.size() << endl;
    cout << "Último keyframe: " << mnMaxKFid << endl;

    // Propiedades
    ar & mnMaxKFid;

    // Contenedores
	ar & mspMapPoints; cout << "MapPoints serializados." << endl;
    ar & mspKeyFrames; cout << "Keyframes serializados." << endl;
    ar & mvpKeyFrameOrigins;
}
INST_EXP(Map)



void Map::save(char* archivo){
	// Elimina keyframes y mappoint malos de KeyFrame::mvpMapPoints y MapPoint::mObservations
	depurar();

	// Previene la modificación del mapa por otros hilos
	unique_lock<mutex> lock(mMutexMap);

	// Abre el archivo
	std::ofstream os(archivo);
	ArchivoSalida ar(os, boost::archive::no_header);

	// Guarda mappoints y keyframes
	serialize<ArchivoSalida>(ar, 0);
}

void Map::load(char* archivo){
	// A este punto se llega con el mapa limpio y los sistemas suspendidos para no interferir.

	{
		// Por las dudas, se previene la modificación del mapa mientras se carga.  Se libera para la reconstrucción, por las dudas.
		unique_lock<mutex> lock(mMutexMap);

		// Abre el archivo
		std::ifstream is(archivo);
		ArchivoEntrada ar(is, boost::archive::no_header);

		// Carga mappoints y keyframes en Map
		serialize<ArchivoEntrada>(ar, 0);
	}



	// Reconstrucción, ya cargados todos los keyframes y mappoints.
	KeyFrame::nNextId = mnMaxKFid + 1;	// mnMaxKFid es el id del último keyframe agregado al mapa

	/*
	 * Recorre los keyframes del mapa:
	 * - Reconstruye la base de datos agregándolos
	 * - UpdateConnections para reconstruir los grafos
	 * - MapPoint::AddObservation sobre cada punto, para reconstruir mObservations y mObs
	 */
	KeyFrameDatabase* kfdb = Sistema->mpKeyFrameDatabase;//Map::mpKeyFrameDatabase;
	for(KeyFrame *pKF : mspKeyFrames){
		// Agrega el keyframe a la base de datos de keyframes
		kfdb->add(pKF);

		// UpdateConnections para reconstruir el grafo de covisibilidad.  Conviene ejecutarlo en orden de mnId.
		pKF->UpdateConnections();

		// Reconstruye mObservations y mObs en MapPoint
		pKF->buildObservations();

	}

	/*
	 * Recorre los MapPoints del mapa:
	 * - Determina el id máximo, para luego establecer MapPoint::nNextId
	 * - Reconstruye mpRefKF
	 * - Reconstruye propiedades con UpdateNormalAndDepth (usa mpRefKF)
	 */
	long unsigned int maxId = 0;
	for(MapPoint *pMP : mspMapPoints){
		// Busca el máximo id, para al final determinar nNextId
		maxId = max(maxId, pMP->mnId);

		// Reconstruye mpRefKF a partir de mnFirstKFid.  setRefKF escribe la propiedad protegida.  Requiere mObservations
		long unsigned int id = pMP->mnFirstKFid;
		std::set<KeyFrame*>::iterator it = std::find_if(mspKeyFrames.begin(), mspKeyFrames.end(), [id](KeyFrame *KF){return KF->mnId == id;});
		pMP->setRefKF((it != mspKeyFrames.end())? *it : NULL);

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


}


bool Map::enMapa(KeyFrame *pKF){
	return mspKeyFrames.count(pKF);
}
bool Map::enMapa(MapPoint *pMP){
	return mspMapPoints.count(pMP);
}

void Map::depurar(){
	cout << "\nDepurando..." << endl;
	//Primero se deben eliminar los puntos del keyframe, luego los keyframes de los puntos.

	// Anula puntos malos en el vector KeyFrame::mvpMapPoints
	for(auto &pKF: mspKeyFrames){	// Keyframes del mapa, en Map::mspKeyFrames
		auto pMPs = pKF->GetMapPointMatches();	// Vector KeyFrame::mvpMapPoints
		for(auto pMP: pMPs){	// Puntos macheados en el keyframe.
			bool borrar = false;

			if(!pMP) continue;	// Saltear si es NULL
			if(pMP->isBad()){	// Si el mappoint es malo...
				cout << "Mal MP:" << pMP->mnId << ". ";
				borrar = true;
			}
			if(!enMapa(pMP)){
				cout << "MP no en mapa:" << pMP->mnId << ". ";
				//borrar = true;
			}
			if(borrar){
				// Para borrar el punto del vector en el keyframe, el keyframe debe poder encontrarse en las observaciones del punto.
				//pKF->EraseMapPointMatch(pMP);	// lo borra (lo pone NULL en el vector mvpMapPoints)

				// Buscar el punto en el vector
				int idx=pMPs.size();
				for(; --idx>=0;){
					auto pMP2 = pMPs[idx];
					if(pMP2 && pMP2 == pMP){
						// encontrado, borrar
						pKF->EraseMapPointMatch(idx);
						//break;// En vez de parar cuando lo enuentra, continúa por si aparece varias veces.
					}
				}
				pMPs = pKF->GetMapPointMatches();
				if(std::find(pMPs.begin(), pMPs.end(), pMP) != pMPs.end()){
					cout << "¡No se borró! ";
				}

				cout << "KF " << pKF->mnId << endl;
			}
		}
	}

	// Elimina keyframes malos en mObservations
	for(auto &pMP: mspMapPoints){	// Set de puntos del mapa
		auto obs = pMP->GetObservations();	// map, sin null, devuelve el mapa mObservations, mapa de keyframes que observan al punto
		for(auto it = obs.begin(); it != obs.end();){	// Keyframes que observan el punto.  Bucle para borrar selectivamente.
			KeyFrame* pKF = it->first;
			it++;
			bool borrar = false;

			if(pKF->isBad()){	// it->first es el keyframe, y si es malo...
				cout << "Mal KF:" << pKF->mnId << ". ";
				borrar = true;
			}
			if(!enMapa(pKF)){
				cout << "KF no en mapa:" << pKF->mnId << ". ";
				//borrar = true;
			}
			if(pKF->flagNotErase()){
				cout << "Not Erase:" << pKF->mnId << ". ";
				borrar = false;

			}
			if(borrar){
				pMP->EraseObservation(pKF);	// se borra del mapa de observaciones
				if(pMP->GetIndexInKeyFrame(pKF)>=0) cout << "¡No se borró! ";
				cout << "MP " << pMP->mnId << endl;
			}
		}
	}
}

std::string Map::analisis(bool profundo){
	std::string reporte;
	int nulos = 0, malos = 0;

	// Reportar keyframes y mappoints isBad
	reporte = "\n\nMap";

	// std::set<MapPoint*> mspMapPoints
	for(auto &pMP: mspMapPoints)
		if(!pMP)
			nulos++;
		else if(pMP->isBad())
			malos++;

	reporte += "\nmspMapPoints Total:" + to_string(mspMapPoints.size()) + ", Null:" + to_string(nulos) + ", Bad:" + to_string(malos);

	// std::set<KeyFrame*> mspKeyFrames
	for(auto &pKF: mspKeyFrames)
		if(!pKF)
			nulos++;
		else if(pKF->isBad())
			malos++;

	reporte += "\nmspKeyFrames Total:" + to_string(mspKeyFrames.size()) + ", Null:" + to_string(nulos) + ", Bad:" + to_string(malos);

	if(profundo){
		// El análisis profundo consiste en generar el análisis de cada keyframe y cada mappoint

		// std::set<MapPoint*> mspMapPoints
		for(auto &pMP: mspMapPoints)
			reporte += pMP->analisis();

		// std::set<KeyFrame*> mspKeyFrames
		for(auto &pKF: mspKeyFrames)
			reporte += pKF->analisis();
	}

	return reporte;
}

// MapPoint: usando map ========================================================

/**
 * Constructor por defecto de MapPoint para el serializador.
 * Se encarga de inicializar las variables const, para que el compilador no chille.
 */
MapPoint::MapPoint():
nObs(0), mnTrackReferenceForFrame(0),
mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
mnCorrectedReference(0), mnBAGlobalForKF(0),mnVisible(1), mnFound(1), mbBad(false),
mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(Sistema->mpMap)//Map::mpMap)
{}

/**
 * Serializador de MapPoint
 * Se invoca al serializar Map::mspMapPoints y KeyFrame::mvpMapPoints, cuyos mapPoints nunca tienen mbBad true.
 * La serialización de MapPoint evita punteros para asegurar el guardado consecutivo de todos los puntos antes de proceder con los KeyFrames.
 * Esto evita problemas no identificados con boost::serialization, que cuelgan la aplicación al guardar.
 */
template<class Archivo> void MapPoint::serialize(Archivo& ar, const unsigned int version){
	// Propiedades
	ar & mnId;
	ar & mnFirstKFid;
	ar & mnVisible;
	ar & mnFound;
	//ar & rgb;	// ¿Serializa cv::Vec3b?

	// Mat
	ar & mWorldPos;
	ar & mDescriptor;	// Reconstruíble, pero con mucho trabajo
}
INST_EXP(MapPoint)


void MapPoint::setRefKF(KeyFrame* pKF){
	if(pKF)
		mpRefKF = pKF;
	else
		mpRefKF = (*mObservations.begin()).first;

}

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
    mbFirstConnection(true),
    mpMap(Sistema->mpMap)
{}

bool KeyFrame::flagNotErase(){
	return mbNotErase;
}

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

	//cout << "KeyFrame ";
	ar & mnId;
	//cout << mnId << endl;

	ar & mbNotErase;


	// Mat
	ar & Tcw;	//ar & const_cast<cv::Mat &> (Tcw);
	ar & const_cast<cv::Mat &> (mK);	// Mismo valor en todos los keyframes
	ar & const_cast<cv::Mat &> (mDescriptors);

	// Contenedores
	ar & const_cast<std::vector<cv::KeyPoint> &> (mvKeysUn);
	ar & mvpMapPoints;
	if(mbNotErase)
		ar & mspLoopEdges;


	// Punteros
	ar & mpParent;


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

}

