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
	string marca;
	marca = " Mat " + to_string(m.rows) + ", " + to_string(m.cols) + ". ";
	ar & marca;

	size_t elem_size = m.elemSize();
	int elem_type = m.type();

	ar << m.cols;
	ar << m.rows;
	ar << elem_size;
	ar << elem_type;

	const size_t data_size = m.cols * m.rows * elem_size;
	ar & boost::serialization::make_array(m.ptr(), data_size);
	marca = " fin Mat.";
	ar & marca;
}

// Mat: load
template<class Archive> void load(Archive & ar, ::cv::Mat& m, const unsigned int version){
	int cols, rows, elem_type;
	size_t elem_size;

	string marca = " Mat ";
	ar & marca;
	ar >> cols;
	ar >> rows;
	ar >> elem_size;
	ar >> elem_type;

	m.create(rows, cols, elem_type);
	size_t data_size = m.cols * m.rows * elem_size;

	ar & boost::serialization::make_array(m.ptr(), data_size);
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
	ar & mspMapPoints; cout << "MapPoints guarados." << endl;
    ar & mspKeyFrames; cout << "Keyframes guarados." << endl;
    ar & mvpKeyFrameOrigins;
}
INST_EXP(Map)



void Map::save(char* archivo){
	depurar();

	// Abre el archivo
	std::ofstream os(archivo);
	ArchivoSalida ar(os, boost::archive::no_header);

	// Guarda mappoints y keyframes
	//ar & *this;
	serialize<ArchivoSalida>(ar, 0);
}

void Map::load(char* archivo){
	// A este punto se llega con el mapa limpio y los sistemas suspendidos para no interferir.

	// Abre el archivo
	std::ifstream is(archivo);
	ArchivoEntrada ar(is, boost::archive::no_header);

	// Carga mappoints y keyframes en Map
	//ar & *this;
	serialize<ArchivoEntrada>(ar, 0);


	// En este punto terminó la carga, se procede a la reconstrucción de lo no serializado.

	/*
	 * Reconstruye KeyFrameDatabase luego que se hayan cargado todos los keyframes.
	 * Recorre la lista de keyframes del mapa para reconstruir su lista invertida mvInvertedFile con el método add.
	 * No puede asignar el puntero mpKeyFrameDB del keyframe porque es protegido.  Esto se hace en el constructor de keyframe.
	 *
	 * kfdb Única instancia de KeyFrameDatabase, cuyo mvInvertedFile se reconstruirá
	 * mapa Única instancia del mapa de cuyo mspKeyFrames se recorrerán los keyframes
	 */
	KeyFrameDatabase* kfdb = Sistema->mpKeyFrameDatabase;//Map::mpKeyFrameDatabase;


	// Recorre todos los keyframes cargados
	for(KeyFrame* kf : mspKeyFrames){
		// Agrega el keyframe a la base de datos de keyframes
		kfdb->add(kf);

		// UpdateConnections para reconstruir el grafo de covisibilidad.  Conviene ejecutarlo en orden de mnId.
		kf->UpdateConnections();
		// kf->UpdateBestCovisibles();
	}
	//KeyFrame::nNextId = maxId + 1;
	KeyFrame::nNextId = mnMaxKFid + 1;	// mnMaxKFid es el id del último keyframe agregado al mapa

	//for(MapPoint* mp : mspMapPoints){
		// Reconstruye normal, profundidad y alguna otra cosa.  Requiere cargados los keyframes.
		//mp->UpdateNormalAndDepth();
	//}

	// Next id para MapPoint
	long unsigned int maxId = 0;
	for(auto mp : mspMapPoints) maxId = max(maxId, mp->mnId);
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
 */
template<class Archivo> void MapPoint::serialize(Archivo& ar, const unsigned int version){
	bool enMapa = mpMap->enMapa(this);

	/*
	std::string marca = " MapPoint " + std::to_string(mnId) + (enMapa? ". " : ": No en mapa! ") + (mbBad?": Malo. ":"");
	ar & marca; cout << marca << endl;

	if(!enMapa) mbBad = true;


	if(mbBad){
		ar & mbBad;
		return;
	}


	// Preparación para guardar
	if(!CARGANDO(ar)){
		// Depuración de mObservations.  Sólo deja los keyframes que están en el mapa.
		for(auto it = mObservations.begin(); it != mObservations.end();){
			KeyFrame* pKF = it->first;
			it++;
			if(pKF && !mpMap->enMapa(pKF)){	// Si no es nulo ni está en el mapa
				cerr << "Keyframe fuera de mapa:" << pKF->mnId << " en MP:" << mnId << endl;
				//EraseObservation(pKF);
			}
		}

	}

	ar & mbBad;
	if(mbBad)
		return;
	*/

	// Propiedades
	ar & mnId;
	ar & mnFirstKFid;
	ar & nObs;
	ar & mnVisible;
	ar & mnFound;
	//ar & rgb;	// ¿Serializa cv::Vec3b?

	// Mat
	ar & mWorldPos;
	ar & mDescriptor;

	// Punteros
	//ar & mpRefKF;	// Se puede reconstruir como mpRefKF=mObservations.begin()->first;

	// Contenedores
	//ar & mObservations;

	// Reconstruíbles con mp->UpdateNormalAndDepth();
	ar & const_cast<cv::Mat &> (mNormalVector); // Reconstruíble con mp->UpdateNormalAndDepth();
	ar & const_cast<float &> (mfMinDistance); // Reconstruíble con mp->UpdateNormalAndDepth();
	ar & const_cast<float &> (mfMaxDistance); // Reconstruíble con mp->UpdateNormalAndDepth();

	//if(CARGANDO(ar))
	//	mpRefKF=mObservations.begin()->first;
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
    mbFirstConnection(true),
    mpMap(Sistema->mpMap)
{}

bool KeyFrame::flagNotErase(){
	return mbNotErase;
}


/**
 * Serializador para KeyFrame.
 * Invocado al serializar Map::mspKeyFrames.
 * No guarda mpKeyFrameDB, que se debe asignar de otro modo.
 *
 */
template<class Archive> void KeyFrame::serialize(Archive& ar, const unsigned int version){
	bool enMapa = mpMap->enMapa(this);

	/*std::string marca = " KeyFrame " + std::to_string(mnId) + (enMapa? ". " : ": No en mapa! ") + (mbBad?": Malo. ":"");
	ar & marca; cout << marca << endl;

	if(!enMapa) mbBad = true;

	// Propiedades
	ar & mbBad;	//ar & const_cast<bool &> (mbBad);
	if(mbBad)	// Es malo o no está en map, no continúa serializando.  No debería suceder si se depura antes.
		return;
	*/

	// Preparación para guardar
	if(!CARGANDO(ar)){
		// Recalcula mbNotErase para evitar valores true efímeros
		mbNotErase = !mspLoopEdges.empty();

		/*// Depuración de mvpMapPoints.  Sólo deja los que están en el mapa.
		for(auto &pMP: mvpMapPoints)
			if(pMP && !mpMap->enMapa(pMP)){	// Si no es nulo ni está en el mapa
				cerr << "Punto fuera de mapa:" << pMP->mnId << " en KF:" << mnId << endl;
				//pMP = NULL;
			}
		*/
	}

	ar & mnId;
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


		/*
		 * ,
		 * se debe invocar UpdateConnections luego de cargar todos los keyframes y mappoints.
		 * No se puede invocar durante la carga porque colgaría el programa al intentar acceder a objetos que no están terminados.
		 *
		 * UpdateBestCovisibles() genera mvpOrderedConnectedKeyFrames y mvOrderedWeights
		 */
	}
}
INST_EXP(KeyFrame)

}

