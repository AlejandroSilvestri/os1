#ifndef KEYFRAMETRIANGULACION_H
#define KEYFRAMETRIANGULACION_H

#define D(var) std::cout << #var << ":" << var << endl;

#include "KeyFrame.h"	// Incluye MapPoint.h, que incluye opencv

using namespace cv;
namespace ORB_SLAM2{

/**
 * Clase para triangulación de rayos.
 *
 * La triangulación de puntos se hace para crear nuevos puntos 3D, a partir de sus vistas desde dos keyframes.
 *
 * Esta clase reúne los métodos que se aplica a cada keyframe, para simplificar el código y evitar la repitencia.
 *
 * Además guarda cálculos intermedios que se utilizan en pasos siguientes.
 *
 * Contiene también código para la triangulación, que requiere dos objetos de éstos.
 *
 * Esta clase se construye tomando los datos del keyframe paredro (pasado como argumento),
 * para evitar las consecuencias de que otros hilos los cambien durante la operación.
 *
 * Algunos métodos buscan datos inmutables en el keyframe.
 *
 * La secuencia de uso es la siguiente:
 *
 * 1- KeyFrameTriangulacion::KeyFrameTriangulacion: Construcción de este objeto, tomando datos de pose y calibración para asegurar su inmutabilidad
 * 2- KeyFrameTriangulacion::rayo inicializa parámetros de cálculos intermedios, necesarios en los métodos que siguen.
 * 3- KeyFrameTriangulacion::computarA proporciona filas para el armado de la matriz A que se descompondrá por SVD
 * 4- KeyFrameTriangulacion::coorddenadaZ calcula y recuerda la distancia del punto sobre el eje z.  Necesario para calcular el error de reproyección.
 * 4- KeyFrameTriangulacion::validarErrorReproyección decide si el error es aceptable.
 * 5- KeyFrameTriangulacion::errorReproyección calcula el error, invocado por el anterior.
 *
 *
 */
class KeyFrameTriangulacion{
public:
    // Propiedades efímeras, usadas durante la triangulación, y su mutex

	/**
	 * Punto singular a triangular.
	 */
    KeyFrame &kf;

    /**
     * Parámetros intrínsecos del keyframe.
     */
    float fx, fy, cx, cy, invfx, invfy,

    /**
     * Coordenada z del punto 3D triangulado, en el sistema de referencia del keyframe.
     *
     * Calculado en KeyFrameTriangulacion::coordenadaZ, y reusado en KeyFrameTriangulacion::errorReproyeccion.
     */
    z;

    /**
     * Pose del keyframe, copiado en el constructor.
     *
     * Mat 4x4 float, rototraslación 3D en coordenadas homogéneas, en el sistema de referencia del keyframe.
     */
    cv::Mat Tcw,

    /**
     * Posición del keyframe en el mundo, copiada en el constructor.
     */
    Ow,

    /**
     * Rotación del keyframe, calculada en el constructor.
     */
    Rcw,

    /**
     * Rotación del keyframe respecto del mundo, traspuesta de Rcw, calculada en el constructor.
     */
    Rwc,

    /**
     * Posición del keyframe, calculado en el constructor.
     */
    tcw,

    /**
     * Punto singular normalizado según parámetros intrínsecos, y en coordenadas homogéneas.
     */
    xn;
    cv::KeyPoint kp;

    /**
     * Constructor único, que copia datos de pose del keyframe argumento.
     */
    KeyFrameTriangulacion(KeyFrame *pKF);


    /**
     * Calcula el rayo de proyección de un punto singular, y otros datos de interés para la triangulación, que se guardan en propiedades efímeras.
     *
     * Lo presenta como versor en el sistema de referencia del mundo,
     * convenientemente normalizado para hacer un producto escalar con otro y obtener el coseno del ángulo.
     *
     * @param indice Índice del vector de puntos singulares KeyFrame::mvKeysUn
     * @returns Mat 3x1 float con el versor del rayo de proyección del punto singular indicado por el índice.
     *
     *
     */
    cv::Mat rayo(int indice);



    /**
     * Devuele el punto triangulado, en coordenadas homogéneas.
     *
     * Construye la matriz A de triangulación de punto y la descompone por SVD.
     */
    Mat triangular(KeyFrameTriangulacion &kft);


    /**
     * @returns El error de reproyección al cuadrado.
     *
     * Utiliza parámetros efímeros de triangulación.
     */
    float errorReproyeccion(cv::Mat x3Dt);


    /**
     * @returns true si el error es aceptable.
     */
	inline bool validarErrorReproyeccion(cv::Mat x3Dt){
		return errorReproyeccion(x3Dt) <= 5.991 * kf.mvLevelSigma2[kp.octave];
	}


	/**
     * Calcula la distancia del punto 3D al centro de la cámara.
     */
    inline float distancia(cv::Mat punto3D){
    	return norm(punto3D-Ow);
    }


    /**
     * Coordenada z del punto, en el sistema de referencia de la cámara.
     *
     * Registra z para su posterior uso en KeyFrameTriangulacion::errorReproyeccion.
     */
    inline float coordenadaZ(cv::Mat x3Dt){
    	return z = Rcw.row(2).dot(x3Dt)+tcw.at<float>(2);
    }
};

}// ORB_SLAM2

#endif // KEYFRAMETRIANGULACION_H
