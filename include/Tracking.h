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


#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"
#include "Frame.h"

#include <mutex>

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;
class Frame;

/** Objeto único que se ejecuta en su propio Trhead, y se inicia con Run().
 * Este objeto empaqueta el código que realiza el tracking y se ejecuta en su thread único y exclusivo.
 * .Run() cuelga el listener GrabImage(), que recibe mensajes con imágenes desde el nodo cámara vía ros.
 *
 * Por su naturaleza asincrónica, GrabImage implementa un autómata finito
 * que conduce el hilo por los distintos estados de inicialización, y de relocalización cuando se pierde el tracking.
 * Su variable de estado es mState.
 *
 * En cada ciclo (cada vez que recibe una imagen) crea una instancia de Frame, donde guarda los resultados de la
 * detección de puntos, extracción de descriptores, macheo, vínculo con los puntos 3D, etc.
 *
 * En el thread de Tracking se realiza la triangulación inicial, el SfM, y se generan los KeyFrames.
 * También se dispara la relocalización cuando se la necesita.
 *
 * Construye el mapa local en mvpLocalKeyFrames, un vector de KeyFrames.
 *
 * Convención de prefijos de miembros:
 * Las propiedades adoptan los siguientes prefijos:
 * - m: miembro
 * - mp: miembro puntero
 * - mvp: miembro vector de punteros
 * - mb: miembro booleano
 * - mn: ? se usa en algunos miembros int.
 *
 * La mayoría de los métodos son de uso interno, y muchos de ellos se invocan por única vez,
 * en el sentido que su código no necesita estar en un método, pero está así para mejorar la legibilidad.
 *
 * Sólo otros tres objetos utilizan métodos de tracking:
 * - FramePublisher
 * - LocalMapping
 * - LoopClosing
 *
 * FramPublisher consulta los estados de Tracking mState y mLastState, utiliza su enum de estados,
 * y en Update() toma datos del mCurrentFrame, mInitialFrame y mvIniMatches.
 *
 * LocalMapping::SetTracker y LoopClosing::SetTracker guardan el tracking en mpTracking, y no lo utilizan.
 *
 * La mayoría de los métodos del objeto son partes de un único algoritmo, dividido para mejor comprensión.
 * Estos métodos son invocados solo desde otro método del objeto, y no desde afuera.
 *
 * La siguiente secuencia de métodos corresponden a un único algoritmo que podría estar en una única función:
 *
 * - Tracking::GrabImageMonocular: recibe la imagen y la convierte a grises
 *  - Tracking::Track: ejecuta el autómata finito:
 *    - Tracking::MonocularInitialization: busca maches para triangular con PnP
 *      - Tracking::CreateInitialMapMonocular: crea el mapa inicial con los puntos triangulados
 *    - Tracking::TrackWithMotionModel: tracking normal, busca macheos según el modelo de movimiento
 *      - Invoca Optimizer::PoseOptimization para obtener la primera estimación de pose del cuadro actual
 *    - Tracking::UpdateLastFrame: trivial
 *    - Tracking::TrackLocalMap: busca puntos no encontrados con el modelo de movimiento
 *      - Tracking::UpdateLocalMap: genera el mapa local: puntos y keyframes
 *        - Tracking::UpdateLocalKeyFrames: genera el mapa local de keyframes
 *        - Tracking::UpdateLocalPoints: genera el mapa local con los puntos observados por los keyframes del mapa local
 *      - Tracking::SearchLocalPoints: actualiza la "visibilidad" (el conteo de vistas) de los puntos, y filtra puntos que ya están en el mapa
 *        - ORBmatcher::SearchByProjection busca puntos que se deberían observar
 *      - Optimizer::PoseOptimization refina la pose
 *    - Tracking::NeedNewKeyFrame: evalúa si hace falta un nuevo keyframe
 *    - Tracking::CreateNewKeyFrame: crea y registra un nuevo keyframe a partir del frame actual
 *    - Tracking::TrackReferenceKeyFrame: tracking alternativo, basado en BoW, para cuando falla el tracking normal
 *    - Tracking::CheckReplacedInLastFrame: actualiza puntos en lastKeyFrame que pudieron haber cambiado por BA, culling u otros motivos
 *    - Tracking::Relocalization: se invoca si el sistema perdió tracking
 *
 */
class Tracking
{  

public:
	/**
	 * Constructor de la única instancia de este objeto se invoca continuamente en el thread principal.
	*/
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, unsigned int cantidadCuadros = 0);

    /**
     * Recibe una imagen para procesar, desde System, en el mismo hilo.
     * Convierte la imagen a monocromática, sabiendo de antemano por el archivo de configuración si es RGB o BGR.
     * Genera mCurrentFrame.  En modo normal usa mpORBextractorLeft (Right se usa solamente en estéreo).
     * En el modo de inicialización (NOT_INITIALIZED o NO_IMAGES_YET) usa mpIniORBextractor, que es igual al anterior pero con el doble de puntos singulares requeridos.
     * A continuación invoca Track(), la máquina de estados.
     * @param im Nueva imagen a procesar.
     * @param timestamp Marca temporal puramente para registro.  ORB-SLAM2 no la utiliza, sólo la registra en el frame.
     * @returns La pose de la cámara.
     *
     * GrabImageMonocular se invoca exclusivamente desde System::TrackMonocular, que a su vez es invocada exclusivamente desde el bucle principal en main.
     *
     * El algoritmo disparado desde TrackMonocular es extenso y se dividió en varios métodos de Tracking solamente para simplificar la lectura,
     * pues cada uno de estos métodos son invocados desde un único punto.
     *
     * La siguiente secuencia de métodos corresponden a un único algoritmo que podría estar en una única función:
     *
     * - GrabImageMonocular
     * - Track, continuación de la anterior
     * - 	MonocularInitialization
     * 			CreateInitialMapMonocular
     * - 	TrackWithMotionModel
     * - 		UpdateLastFrame
     * 		TrackLocalMap
     * 			SearchLocalPoints
     * 			UpdateLocalMap
     * 				UpdateLocalMapPoints
     * 				UpdateLocalKeyFrames
     * 		NeedNewKeyFrame
     * 		CreateNewKeyFrame
     * 		TrackReferenceKeyFrame
     * 		CheckReplacedInLastFrame
     * 		Relocalization
     */
    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    //cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    //cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    /** Registra el objeto de LocalMapping. @param pLocalMapper Objeto de mapeo local.*/
    void SetLocalMapper(LocalMapping* pLocalMapper);

    /** Registra el objeto de LoopClosing. @param pLoopClosing Objeto de cierre de bucle.*/
    void SetLoopClosing(LoopClosing* pLoopClosing);

    /** Registra el objeto de visualización. @param pViewer Visualizador.*/
    void SetViewer(Viewer* pViewer);

    /**
     * Comando para cambiar la calibración.
     * No usado en esta implementación, pero permitiría cambiar de cámara, o usar una cámara sobre el mapa relevado por otra.
     * Sin embargo las profundidades de visualización de los puntos 3D todavía están muy vinculados a la distancia focal, y todavía no son compatibles con distancias diferentes.
     * Carga nuevos parámetros de calibración del archivo de configuración.
     *
     * @param strSettingPath Nombre del archivo de configuración.
     *
     */
    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string &strSettingPath);

    /**
     * Control de usuario para alternar entre tracking y mapping o solo tracking.
     */
    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);


public:

	/**
	 * Estado de tracking.
	 *
	 * La variable de estado mState del objeto Tracking tiene uno de estos valores.
	 */
    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    /** Estado del autómata finito del tracking.  mState adopta los valores del enum eTrackingState.*/
    eTrackingState mState;

    /** Estado anterior al actual.  Último estado que fue procesado por completo. */
    eTrackingState mLastProcessedState;

    /** Tipo de sensor: monocular, estéreo o rgbd.  Siempre System::MONOCULAR en esta versión.*/
    // Input sensor
    int mSensor;

    /** Cuadro actual.*/
    // Current Frame
    Frame mCurrentFrame;

    /**
     * Imagen del cuadro actual, recién convertida a grises.
     * GrabImageMonocular recibe la imagen como Mat, y si es a color la convierte a grises; la guarda en mImGray.
     */
    cv::Mat mImGray;

    /** Agregado, imagen de entrada para visualización.  Se registra en GrabImage*/
    //cv::Mat imagenEntrada;


    /** Variables de inicialización.  Luego de la inicialización, estos valores están en el Frame.*/
    // Initialization Variables (Monocular)

    /** Vector con los matches anteriores, para inicialización.*/
    std::vector<int> mvIniLastMatches;

    /** Vector con los matches de inicialización.*/
    std::vector<int> mvIniMatches;

    /** Vector con los matches anteriores, para inicialización.*/
    std::vector<cv::Point2f> mvbPrevMatched;

    /** Vector con los puntos 3D iniciales.*/
    std::vector<cv::Point3f> mvIniP3D;

    /**
     * Primer cuadro para la inicialización.
     * La triangulación de puntos para el mapa inicial se realiza entre dos cuadros:
     * - mCurrentFrame
     * - mInitialFrame
     *
     * Una vez que el sistema se inicializó, mInitialFrame no se vuelve a usar.
     */
    Frame mInitialFrame;

    ///@{
    /** Listas para registrar la trayectoria de la cámara, para guardarla al terminar la aplicación.*/
    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;
    ///@}

    /**
     * Flag indicador de modo Tracking (true) o tracking y mapping (false).
     */
    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    /**
     * Reinicia el sistema.
     * Olvida el mapa y demás y pasa al estado NOT_IMAGE_YET.
     *
     * Este método:
     * - pausa Viewer
     * - resetea LocalMapping
     * - resetea LoopClosing
     * - resetea KeyFrameDB
     * - resetea Map
     * - resetea Tracking eliminando propiedades
     *
     * Finalmente, reanuda Viewer.
     *
     * Al terminar el método, el sistema se encuentra reseteado.
     *
     * El reseteo del los otros hilos hace esperar hasta que el hilo termina de resetearlo.
     * Podría paralelizare, pero no es un proceso que requiera mayor velocidad.
     *
     * Invocado indirectamente por el usuario vía System::Reset.
     *
     * Invocado sólo desde System::TrackMonocular y Tracking::CreateInitialMapMonocular.
     *
     */
    void Reset();

    /** Agregado: parámetro del trackbar de la ventana que muestra el cuadro actual.*/
    int param = 100;

    /** Público para que lo pueda ver Viewer al serializar mapas.*/
    LocalMapping* mpLocalMapper;

    /**
     * Modo de cámara.
     * 0 normal, viene con orb-slam, la común de opencv.
     * 1 fisheye sin distorsión, sólo aplica la deformación de la matriz intrínseca según el modelo de fisheye (equidistance projection).
     * Su valor proviene del archivo de configuración, y se pasa a Frame.
     */
    int camaraModo;


protected:

    /**
     * Es la máquina de estados del tracking.
     * Este método implementa el autómata finito cuya variable de estado es mState,
     * y sus valores se definen en el enum eTrackingState.
     * La variable informa sobre los diversos estados para la inicialización primero,
     * y luego sobre los estados de operación.
     * ORB-SLAM opera principalmente en el estado WORKING.
     *
     *
     * Autómata:
     * -SYSTEM_NOT_READY: El sistema no está listo, algunos componentes se están inicializando.
     * -NO_IMAGES_YET: Estado inicial.  GrabImage se invoca la primera vez con una imagen, y pasa al estado siguiente.  Es el único momento en que todavía no existe mLastFrame.
     * -NOT_INITIALIZED: Recibió la primera imagen y existe un mLastFrame, e invoca repetidamente la inicialización MonocularInitialization(), hasta que logra inicializar y pasa a OK.
     * -OK: El tracking operando normalmente.  En orb-slam se llamaba WORKING.
     * -LOST: Perdido, si ocurre en los primeros 5 frames, reinicia con Reset(), y si no sigue intentando relocalizar.
     *
     * mState tiene el estado actual, mLastProcessedState tiene el estado de la ejecución anterior.
     * Si el sistema no está inicializado, se invoca MonocularInitialization.
     *
     * Inicialización: NOT_INITIALIZED
     * El estado NOT_INITIALIZED también indica que está intentando inicializar.  Usa variables propias para el autómata de inicialización.
     * Primero busca un cuadro de referencia, el primero con suficientes features para la inicialización (100 o más)
     * Luego se queda macheando con cuadros posteriores.
     * Mientras consiga suficientes macheos, intenta la inicialización invocando mpInitializer->Initialize, que retorna con éxito o fallo.
     * Si tiene éxito pasa al estado siguiente, OK.  Es la única vía para salir de la inicialización.
     * Si falla continúa macheando cuadros posteriores.
     * Si continúa fallando, la cantidad de macheos irá decreciendo y si baja del mínimo busca un nuevo cuadro de referencia.
     *
     * En modo normal (estado OK con tracking y mapeo) se invocan TrackWithMotionModel, TrackLocalMap
     * y luego se decide si es momento de agregar un nuevo KeyFrame.
     *
     * Track Se invoca únicamente desde GrabImageMonocular, podría ser parte de ese método.
     */
    // Main tracking function. It is independent of the input sensor.
    void Track();

    /**
     * Incovado por el autómata Track() para intentar inicializar.
     * En la primer llamada (!mpInitializer) designa el frame actual como inicial.
     * En la segunda llamada (mpInitializer) utiliza el frame actual como el segundo, machea e intenta PnP.
     * Requiere 100 macheos.  Cambié la constante 100 por minMatches, que toma su valor de la propiedad param, controlada por trackbar.
     *
     */
    // Map initialization for monocular
    void MonocularInitialization();

    /**
     * Crea el mapa inicial.
     * Crea dos KeyFrames (inicial y actual) a partir de los cuadros inicial y actual.
     * Computa Bag of Words en ambos keyframes.
     * Agrega los keyframes al mapa.
     * Recorre los puntos macheados, los computa como puntos del mapa para agregarlos.
     * Esto significa que les asocia los keyframes que los observan, sus normales y profundidades, y sus descriptores distintivos.
     * Luego realiza un primer bundle adjustment.
     *
     * Invocado sólo desde MonocularInitialization.
     */
    void CreateInitialMapMonocular();

    /**
     * Revisa y actualiza mLastFrame por las dudas que el mapa local haya cambiado.
     * Recorre todos los puntos del mapa registrados en el cuadro (los puntos vistos desde ese cuadro y macheados),
     * y si alguno fue reemplazado (se sabe con punto->GetReplaced) se lo actualiza.
     *
     * Invocado sólo desde Track.
     */
    void CheckReplacedInLastFrame();

    /**
     * Una manera de hacer tracking, alternativa a TrackWithMotionModel.
     * Intenta obtener la pose de la cámara respecto del keyframe de referencia, a partir de los puntos observados.
     * El macheo se realiza por BoW.
     * Se usa si falla TrackWithMotionModel, o si no hay "motion model", por ejemplo porque recién se inicializa o se realizó una relocalización.
     * Requiere 15 macheos como mínimo, si no, falla.
     * Invocado sólo desde Track.
     */
    bool TrackReferenceKeyFrame();

    /**
     * Calcula la pose de mLastFrame respecto del keyframe de referencia, y la registra en mLastFrame con .SetPose.
     * Esta función se invoca exclusivamente desde TracWithMotionModel.
     * Se empaquetó en una función aparte porque, para casos diferentes de monocular, hay mucho más trabajo.
     */
    void UpdateLastFrame();

    /**
     * SfM.  Función principal, que determina la performance del sistema.
     * Rastrea los puntos de la imagen en el mapa, macheándolos con ORBmatcher::SearchByProjection,
     * que extiende el macheo consecutivo desde lastFrame a currentFrame, asociando los puntos 3D a los puntos singulares del cuadro actual.
     * Si no logra 20 macheos, falla, retorna false.
     * Con 20 o más puntos macheados calcula la nueva pose con Optimizer::PoseOptimization() (donde ocurre el verdadero SfM).
     * y quita los outliers del frame.  Si no quedan al menos 10 puntos, retorna false indicando la falla.
     *
     * @returns true si logró optimizar la pose, false si no.
     *
     * Esta función se invoca exclusivamente desde Tracking::Track, según el autómata finito, siempre en el estado OK.
     */
    bool TrackWithMotionModel();

    /**
     * Dispara una relocalización.
     *
     * Machea keyframes por BoW, obteniendo un conjunto de puntos 3D compatible con los BoW del cuadro actual.
     *
     * Luego se intenta resolver la pose con PnPsolver.
     *
     * Invocado sólo desde el autómata finito en Tracking::Track.
     */
    bool Relocalization();

    /**
     * Actualiza los puntos y keyframes en el mapa local.
     * Invoca los métodos Tracking::UpdateLocalKeyFrames y Tracking::UpdateLocalPoints.
     * Estos métodos sólo se invocan desde aquí, podrían ser parte de UpdateLocalMap.
     *
     * Invocada sólo desde Tracking::TrackLocalMap.
     */
    void UpdateLocalMap();

    /**
     * Rehace la lista de puntos del mapa local.
     *
     * La lista de puntos es Tracking::mvpLocalMapPoints.
     *
     * Recorre la lista de keyframes Tracking::mvpLocalKeyFrames para recorrer sus puntos obtenidos con KeyFrame::GetMapPointMatches().
     *
     * Marca los puntos agregados para no volverlos a agregar, asegurando que no habrá repeticiones.
     *
     * Invocado sólo desde Tracking::UpdateLocalMap.
     */
    void UpdateLocalPoints();

    /**
     * Reconstruye el mapa local de keyframes.
     *
     * Reconstruye Tracking::mvpLocalKeyFrames, la lista de keyframes que observan los puntos rastreados, y sus vecinos.
     *
     * De este modo genera el mapa local.
     *
     * Invocado sólo desde Tracking::UpdateLocalMap.
     */
    void UpdateLocalKeyFrames();


    /**
     * Actualiza el mapa y realiza el tracking.
     * Se invoca cuando el sistema está ubicado: tiene pose de cámara y lleva el rastro de varios puntos en el cuadro.
     * Actualiza el mapa local a partir de la pose estimada:
     * busca puntos del mapa que se deberían observar desde esa pose, usando Tracking::SearchLocalPoints().
     * Optimiza la pose a partir de los puntos observados, obteniendo la lista de inliers.
     * Incrementa la cantidad de veces que estos puntos fueron observados, para estadística.
     * Para retornar con éxito exige 30 puntos inliers, o 50 (más exigente) si hubo una relocalización reciente.
     */
    bool TrackLocalMap();

    /**
     * Invocado por TrackLocalMap.
     *
     * 1. Recorre los puntos 3D asociados al cuadro actual, incrementa su visibilidad y marca que fueron vistos por última vez desde este cuadro.
     * 2. Cuenta los puntos que deberían haber sido vistos y no lo fueron.
     * 3. Si hay puntos no vistos, realiza un ORBmatcher::SearchByProjection
     */
    void SearchLocalPoints();


    /**
     * Informa si es momento de insertar un nuevo KeyFrame en el mapa.
     * Este método implementa la inteligencia para elegir los frames ideales y mantener pequeño el mapa.
     * Para insertar un nuevo KeyFrame se tienen que dar alguna de estas condiciones:
     * - Que hayan pasado al menos mMinFrames desde el último KeyFrame, y que el frame actual tenga menos del 90% de los puntos del KeyFrame de referencia y al menos 15 inliers.
     * - Que hayan pasado mMaxFrames desde el último KeyFrame.
     *
     * No se considera insertar KeyFrame en estas situaciones:
     * - Hay un cierre de bucle en curso
     * - Desde la última relocalización no pasaron todavía una cantidad mínima de frames mMaxFrames.
     *
     * Este método se invoca solamente desde Track().
     *
     */
    bool NeedNewKeyFrame();

    /**
     * Crea un keyframe a partir del frame actual.
     * Crea un KeyFrame usando mCurrentFrame, y lo inserta en el mapa con mpLocalMapper.
     * Lo nombra LasKeyFrame.InsertKeyFrame, y registra su mnid en mnLastKeyFrameId.
     *
     * Este método se invoca desde un único punto del código, en Track(), luego de preguntar NeedNewKeyFrame().
     */
    void CreateNewKeyFrame();

    /**
     * Flag de odometría visual.
     * Es true cuando hay pocos macheos con puntos del mapa.
     * El sistema entra en modo de odometría visual para adivinar en qué lugar del mapa está, mientras intenta relocalizar.
     * La diferencia con el tracking normal es que sólo intenta Tracking::TrackWithMotionModel, y no intenta Tracking::TrackReferenceKeyFrame.
     * Además, al mismo tiempo que realiza VO, intenta relocalizar.
     *
     * TrackWithMotionModel pasa a odometría visual cuando no hace mapping (onlyTracking) y visualiza menos de 10 puntos del mapa.
     */
    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LoopClosing* mpLoopClosing;

    //ORB

    /**
     * Extractores de cámara.
     * Right se usa solamente en estéreo; monocular usa solamente Left.
     * Ini es una versión más exigente, que durante la inicialización intenta recuperar el doble de features y macheos que lo normal.
     */
    ORBextractor* mpORBextractorLeft;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    /**
     * Inicializador, responsable de la triangulación de los primeros puntos del mapa.
     * Sólo se usa en modo monocular.
     * El inicializador es modular, por eso en lugar de estar incorporado en Tracking se proporciona en un objeto Initializer, a ese efecto.
     */
    // Initalization (only for monocular)
    Initializer* mpInitializer;

    //Local Map
    KeyFrame* mpReferenceKF;

    /** Lista del keyframes locales, parte del mapa local. */
    std::vector<KeyFrame*> mvpLocalKeyFrames;

    /** Mapa local, lista de puntos 3D.*/
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    /**Sistema.
     * Única instancia.
     */
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    /** Mapa general.*/
    //Map
    Map* mpMap;

    //Calibration matrix

    /** Matriz intrínseca, de cámara, de calibración K.  Mat de 3x3 CV_32F (float).*/
    cv::Mat mK;

    /**
     * Coeficientes de distorsión del modelo pihole estándar de OpenCV.
     * Mat de 8x1 CV_32F (float).
     *
     * Contiene los coeficientes de distorsión radial y tangencial, para ser usados exclusivamente por cv::undistortPoints
     * en Frame::UndistortKeyPoints y Frame::ComputeImageBounds, ambos ejecutados solamente durante la construcción del Frame.
     *
     * Su contenido se toma de la configuración, y podría cambiarse vía Tracking::ChangeCalibration, que no se usa en esta implementación.
     */
    cv::Mat mDistCoef;

    /**
     * Leído del archivo de configuración, parte de los parámetros de calibración, pasado al constructor de Frame.
     * No usado en monocular.  Ya se eliminó el código que lo usaba.  Se deja para hasta que se cambien los constructores de Frame y KeyFrame.
     */
    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;



private:
    /**
     * Sólo para estéreo, el código actual no lo usa, sólo se pasa entre constructores a Frame y a KeyFrame.
     */
    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    //float mThDepth;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    /**
     * Rototraslación del modelo de movimiento.
     * Es una matriz de 4x4 como Tcw, surge de multiplicar Tcw del cuadro actual con Twc del anterior.
     * El resultado es la rototraslación relativa del último cuadro respecto del anterior.
     */
    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
