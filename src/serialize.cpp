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


// boost::serialization
#include <boost/serialization/serialization.hpp>
#include <boost/archive/tmpdir.hpp>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/assume_abstract.hpp>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

// Clases de orb-slam2
#include "Map.h"	// Incluye MapPoint y KeyFrame, éste incluye a KeyFrameDatabase
//#include "KeyFrame.h"
//#include "KeyFrameDatabase.h"
//#include "MapPoint.h"


// Defines
#define TIPOS_ARCHIVOS(FORMATO) \
	typedef boost::archive::FORMATO##_iarchive ArchivoEntrada;\
	typedef boost::archive::FORMATO##_oarchive ArchivoSalida;
// Aquí se define el tipo de archivo: binary, text, xml...
TIPOS_ARCHIVOS(binary)

// La bifurcación consiste en distinguir si serializa carga o guarda.  Se activa definiendo BIFURCACION, pero requiere c++11 para typeinfo
// En un método serialize(Archivo...) la macro GUARDANDO(Archivo) devuelve true si se está guardando, false si se está cargando.
// Comentar la siguiente línea para desactivar la capacidad de bifurcación
#define BIFURCACION
#ifdef BIFURCACION
	#include <typeinfo>
	#define GUARDANDO(ARCHIVO) (typeinfo(ARCHIVO) == typeinfo(ArchivoSalida))
#endif

// Instancias explícitas
#define INST_EXP(CLASE)\
template void CLASE::serialize<ArchivoEntrada>(ArchivoEntrada&,const unsigned int);\
template void CLASE::serialize<ArchivoSalida>(ArchivoSalida&,const unsigned int);








// Serialización no intrusiva fuera de la clase: Mat, KeyPoint
BOOST_SERIALIZATION_SPLIT_FREE(::cv::Mat)
namespace boost{namespace serialization{
/**
 * OpencCV KeyPoint y Mat
 * Copiado de https://github.com/MathewDenny/ORB_SLAM2 MapPoint.h
 */
// KeyPoint
template<class Archive> void serialize(Archive & ar, cv::KeyPoint& kf, const unsigned int version){
  ar & kf.angle;
  ar & kf.class_id;
  ar & kf.octave;
  ar & kf.response;
  ar & kf.response;
  ar & kf.pt.x;
  ar & kf.pt.y;
}

// Mat: save
template<class Archive> void save(Archive & ar, const ::cv::Mat& m, const unsigned int version){
  size_t elem_size = m.elemSize();
  size_t elem_type = m.type();

  ar & m.cols;
  ar & m.rows;
  ar & elem_size;
  ar & elem_type;

  const size_t data_size = m.cols * m.rows * elem_size;
  ar & boost::serialization::make_array(m.ptr(), data_size);
}

// Mat: load
template<class Archive> void load(Archive & ar, ::cv::Mat& m, const unsigned int version){
  int cols, rows;
  size_t elem_size, elem_type;

  ar & cols;
  ar & rows;
  ar & elem_size;
  ar & elem_type;

  m.create(rows, cols, elem_type);
  size_t data_size = m.cols * m.rows * elem_size;

  ar & boost::serialization::make_array(m.ptr(), data_size);
}


}}


// Definiciones de los métodos serialize de las clases Map, MapPoint y KeyFrame
namespace ORB_SLAM2{

// Map: usando set ========================================================
template<class Archivo> void Map::serialize(Archivo& ar, const unsigned int version){
	ar & mspMapPoints;
    ar & mspKeyFrames;
    ar & mvpKeyFrameOrigins;
    ar & const_cast<long unsigned int &> (mnMaxKFid);
    //ar & mvpReferenceMapPoints;	// Es un vector efímero, no hace falta guardarlo, se genera durante el tracking normal.
}
INST_EXP(Map)

void Map::save(char* archivo){
	// Abre el archivo
	std::ofstream os(archivo);
	ArchivoSalida ar(os, boost::archive::no_header);

	// Guarda mappoints y keyframes
	serialize<ArchivoSalida>(ar, 0);
}

void Map::load(char* archivo){
	// Abre el archivo
	std::ifstream is(archivo);
	ArchivoEntrada ar(is, boost::archive::no_header);

	// Guarda mappoints y keyframes
	serialize<ArchivoEntrada>(ar, 0);

	/*
	 * Reconstruye KeyFrameDatabase luego que se hayan cargado todos los keyframes.
	 * Recorre la lista de keyframes del mapa para reconstruir su lista invertida mvInvertedFile con el método add.
	 * No puede asignar el puntero mpKeyFrameDB del keyframe porque es protegido.  Esto se hace en el constructor de keyframe.
	 *
	 * kfdb Única instancia de KeyFrameDatabase, cuyo mvInvertedFile se reconstruirá
	 * mapa Única instancia del mapa de cuyo mspKeyFrames se recorrerán los keyframes
	 */
	KeyFrameDatabase* kfdb = Map::mpKeyFrameDatabase;
	for(KeyFrame* kf : mspKeyFrames)
		kfdb->add(kf);
}

// Inicializa punteros a singleton
Map* Map::mpMap = NULL;
KeyFrameDatabase* Map::mpKeyFrameDatabase = NULL;
ORBVocabulary* Map::mpVocabulary = NULL;




// MapPoint: usando map ========================================================

/**
 * Constructor por defecto de MapPoint para el serializador.
 * Se encarga de inicializar las variables const, para que el compilador no chille.
 */
MapPoint::MapPoint():
nObs(0), mnTrackReferenceForFrame(0),
mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
mnCorrectedReference(0), mnBAGlobalForKF(0),mnVisible(1), mnFound(1), mbBad(false),
mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(Map::mpMap)
{}

/**
 * Serializador de MapPoint
 */
template<class Archivo> void MapPoint::serialize(Archivo& ar, const unsigned int version){

	//if(mbBad) return;	// Evita guardar puntos inútiles

	ar & const_cast<long unsigned int &> (mnId );
	ar & nNextId;
	ar & const_cast<long int &> (mnFirstKFid);
	ar & const_cast<long int &> (mnFirstFrame);
	ar & const_cast<int &> (nObs);
	ar & const_cast<float &> (mTrackProjX);
	ar & const_cast<float &> (mTrackProjY);
	ar & const_cast<float &> (mTrackProjXR);
	ar & const_cast<bool &> (mbTrackInView);
	ar & const_cast<int &> (mnTrackScaleLevel);
	ar & const_cast<float &> (mTrackViewCos);
	ar & const_cast<long unsigned int &> (mnTrackReferenceForFrame);
	ar & const_cast<long unsigned int &> (mnLastFrameSeen);
	ar & const_cast<long unsigned int &> (mnBALocalForKF);
	ar & const_cast<long unsigned int &> (mnFuseCandidateForKF);
	ar & const_cast<long unsigned int &> (mnLoopPointForKF);
	ar & const_cast<long unsigned int &> (mnCorrectedByKF);
	ar & const_cast<long unsigned int &> (mnCorrectedReference);

	ar & const_cast<cv::Mat &> (mPosGBA);
	ar & const_cast<long unsigned int &> (mnBAGlobalForKF);
	ar & const_cast<cv::Mat &> (mWorldPos);

	ar & mObservations;

	ar & const_cast<cv::Mat &> (mNormalVector);
	ar & const_cast<cv::Mat &> (mDescriptor);
	ar & mpRefKF;

	ar & const_cast<int &> (mnVisible);
	ar & const_cast<int &> (mnFound);
	ar & const_cast<bool &> (mbBad);
	ar & const_cast<float &> (mfMinDistance);
	ar & const_cast<float &> (mfMaxDistance);
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
    mnFrameId(0),  mTimeStamp(0.0), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(0.0), mfGridElementHeightInv(0.0),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
    fx(0.0), fy(0.0), cx(0.0), cy(0.0), invfx(0.0), invfy(0.0),
    mbf(0.0), mb(0.0), mThDepth(0.0), N(0), mnScaleLevels(0), mfScaleFactor(0),
    mfLogScaleFactor(0.0),
    mnMinX(0), mnMinY(0), mnMaxX(0), mnMaxY(0),
    mpKeyFrameDB(Map::mpKeyFrameDatabase), mpORBvocabulary(Map::mpVocabulary), mpMap(Map::mpMap)
{}

/**
 * Serializador para KeyFrame
 * No guarda mpKeyFrameDB, que se debe asignar de otro modo.
 */
template<class Archive> void KeyFrame::serialize(Archive& ar, const unsigned int version){

	//if(mbToBeErased || mbBad) return;

	ar & nNextId;
	ar & const_cast<long unsigned int &> (mnId);
	ar & const_cast<long unsigned int &> (mnFrameId);
	ar & const_cast<double &> (mTimeStamp);
	ar & const_cast<int &> (mnGridCols);
	ar & const_cast<int &> (mnGridRows);
	ar & const_cast<float &>  (mfGridElementWidthInv);
	ar & const_cast<float &>  (mfGridElementHeightInv);
	ar & const_cast<long unsigned int &> (mnTrackReferenceForFrame);
	ar & const_cast<long unsigned int &> (mnFuseTargetForKF);
	ar & const_cast<long unsigned int &> (mnBALocalForKF);
	ar & const_cast<long unsigned int &> (mnBAFixedForKF);
	ar & const_cast<long unsigned int &> (mnLoopQuery);
	ar & const_cast<int &> (mnLoopWords);
	ar & const_cast<float &> (mLoopScore);
	ar & const_cast<long unsigned int &> (mnRelocQuery);
	ar & const_cast<int &> (mnRelocWords);
	ar & const_cast<float &> (mRelocScore);
	ar & const_cast<cv::Mat &> (mTcwGBA);
	ar & const_cast<cv::Mat &> (mTcwBefGBA);
	ar & const_cast<long unsigned int &> (mnBAGlobalForKF);
	ar & const_cast<float &> (fx);
	ar & const_cast<float &> (fy);
	ar & const_cast<float &> (cx);
	ar & const_cast<float &> (cy);
	ar & const_cast<float &> (invfx);
	ar & const_cast<float &> (invfy);
	ar & const_cast<float &> (mbf);
	ar & const_cast<float &> (mb);
	ar & const_cast<float &> (mThDepth);
	ar & const_cast<int &> (N);
	ar & const_cast<std::vector<cv::KeyPoint> &> (mvKeys);
	ar & const_cast<std::vector<cv::KeyPoint> &> (mvKeysUn);
	ar & const_cast<std::vector<float> &> (mvuRight);
	ar & const_cast<std::vector<float> &> (mvDepth);
	ar & const_cast<cv::Mat &> (mDescriptors);
	ar & const_cast<cv::Mat &> (mTcp);
	ar & const_cast<int &> (mnScaleLevels);
	ar & const_cast<float &> (mfScaleFactor);
	ar & const_cast<float &> (mfLogScaleFactor);
	ar & const_cast<std::vector<float> &> (mvScaleFactors);
	ar & const_cast<std::vector<float> &> (mvLevelSigma2);
	ar & const_cast<std::vector<float> &> (mvInvLevelSigma2);

	ar & const_cast<int &> (mnMinX);
	ar & const_cast<int &> (mnMinY);
	ar & const_cast<int &> (mnMaxX);
	ar & const_cast<int &> (mnMaxY);
	ar & const_cast<cv::Mat &> (mK);
	ar & const_cast<cv::Mat &> (Tcw);
	ar & const_cast<cv::Mat &> (Twc);
	ar & const_cast<cv::Mat &> (Ow);
	ar & mvpMapPoints;
	ar & mConnectedKeyFrameWeights;
	ar & mvpOrderedConnectedKeyFrames;
	ar & const_cast<std::vector<int> &>(mvOrderedWeights);

	ar & const_cast<bool &> (mbFirstConnection);
	ar & mpParent;
	ar & mspChildrens;
	ar & mspLoopEdges;

	ar & const_cast<bool &> (mbNotErase);
	ar & const_cast<bool &> (mbToBeErased);
	ar & const_cast<bool &> (mbBad);
	ar & const_cast<float &> (mHalfBaseline);

	//ar & const_cast<cv::Mat &> (Cw);
}
INST_EXP(KeyFrame)

}

