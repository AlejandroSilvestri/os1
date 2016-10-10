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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
//#include <opencv/cv.h>
#include <opencv2/core/core.hpp>	// Cambiado por prolijidad, sólo se usa para definir Mat, Point, Point2i y KeyPoint


namespace ORB_SLAM2
{
/**
 * Pendiente de documentación.
 * Objeto usado exclusivamente en DistributeOctTree.
 */
class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    /**
     * Divide el nodo en 4: n1, n2, n3 y n4.
     */
    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;

    /**
     * Vértice del rectángulo límite.
     * U: upper, B: bottom
     * L: left,  R: right
     *
     */
    cv::Point2i UL, UR, BL, BR;

    /**
     *
     */
    std::list<ExtractorNode>::iterator lit;

    /**
     * Señal que indica que el nodo tiene exactamente un punto asignado.
     * Se inicializa como false, diversos métodos verifican y lo ponen en true cuando tiene un único punto.
     */
    bool bNoMore;
};

/**
 * Empaqueta todos los métodos de detección de puntos singulares y extracción de descriptores.
 * ORBextractor procesa imágenes con el operador ():
 * - detectando puntos singulares
 * - computando sus orientaciones
 * - extrayendo sus descriptores
 *
 * El objeto administra las pirámides y la grilla de celdas en la que se divide la imagen para homogeneizar la distribución de puntos singulares.
 * Tracking crea las únicas dos instancias de este objeto, de larga vida, mpORBextractorLeft y mpIniORBextractor,
 * el primero como parte inicial del proceso de tracking en estado OK, y el segundo para inicializar.
 *
 * La descripción de ORBextractor::ComputeKeyPointsOctTree contiene una buena descripción de los puntos singulares.
 */
class ORBextractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    /**
     * Constructor que carga los valores de configuración recibidos como argumentos, computa pirámide y precalcula factores.
     */
    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~ORBextractor(){}

    /**
     * ORBextractor(...) procesa imágenes con el operador ():
	 * - detectando puntos singulares
	 * - computando sus orientaciones
	 * - extrayendo sus descriptores
	 *
	 * @param image Imagen a procesar.
	 * @param mask Máscara.  No implementada.
	 * @param keypoints Puntos singulares detectados como resultado de la operación.
	 * @param descriptors Descriptores extraídos como resultado de la operación.
     */
    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    void operator()( cv::InputArray image, cv::InputArray mask,
      std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors);

    /**
     * Devuelve el atributo protegido nlevels, la cantidad de niveles en la pirámide, establecida por el constructor y de sólo lectura.
     */
    int inline GetLevels(){
        return nlevels;}

    /**
     * Devuelve el atributo protegido scaleFactor, el factor de escala entre niveles de la pirámide, establecido por el constructor y de sólo lectura.
     */
    float inline GetScaleFactor(){
        return scaleFactor;}

    /**
     * Devuelve el atributo protegido mvScaleFactor.
     */
    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    /**
     * Devuelve el atributo protegido mvInvScaleFactor.
     */
    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    /**
     * Devuelve el atributo protegido mvLevelSigma2.
     */
    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    /**
     * Devuelve el atributo protegido mvInvLevelSigma2.
     */
    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    /**
     * Imágenes de la pirámide.
     * Producida por ComputePyramid, es un vector de ´nlevels´ imágenes.
     */
    std::vector<cv::Mat> mvImagePyramid;

protected:

    /**
     * Genera las imágenes de la pirámide y las guarda en el vector mvImagePyramid.
     * @param image Imagen a procesar, obtenida de la cámara.
     * Invocado sólo desde el constructor.
     */
    void ComputePyramid(cv::Mat image);

    /**
     * Detecta puntos singulares, y los dispersa con ORBextractor::DistributeOctTree.
     *
     * @param allKeypoints Vector de vectores de puntos singulares a detectar, resultado del proceso.  Un vector de puntos singulares para cada nivel de la pirámide.
     *
     * Los KeyPoints obtenidos almacenan los siguientes datos:
     * - pt, coordenadas del punto singular sobre la imagen.
     * - angle, orientación obtenida con IC_Angle.
     * - octave, nivel de la pirámide.
     * - size, tamaño según el factor de escala del nivel de la pirámide.
     */
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);

    /**
     * Distribuye puntos singulares con un octTree.
     * Recibe una cantidad de puntos singulaes mucho mayor a la deseada, este método elimina la mayoría de manera que los puntos sobrevivientes se encuentren dispersos en la imagen de manera homogénea.
     * OcTree es un árbol cuyos nodos tienen 8 hijos exactamente, que de manera abstracta representan 8 vértices de un cubo.
     * https://es.wikipedia.org/wiki/Octree
     *
     * @param vToDistributeKeys Puntos singulares a distribuir.
     * @param minX Borde, umbral, franja de la imagen que no se analiza.
     * @param maxX Borde, umbral, franja de la imagen que no se analiza.
     * @param minY Borde, umbral, franja de la imagen que no se analiza.
     * @param maxY Borde, umbral, franja de la imagen que no se analiza.
     * @param nFeatures Cantidad deseada de puntos singulaes.
     * @param level Nivel de la pirámide a procesar.
     * @returns Puntos singulares distribuídos.
     */
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    /**
     * Antiguo método para detectar puntos singulares, reemplazado por ComputeKeyPointsOctTree.
     * Método no usado.
     */
    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);

    /**
     * Coordenadas BRIEF.
     * Cada renglón contiene un par de coordenadas x,y cuyas intensidades se comparan para obtener un bit del descriptor binario.
     * Hay 256 renglones para los 256 bits del BRIEF.  Todas las coordenadas entran en un parche circular de 31 píxeles de diámetro.
     * Las coordenadas son relativas al punto singular a considerar, y a su orientación.
     * Esto significa que estas coordenadas patrón se rototrasladan para obtener las coordenadas reales para extraer un descriptor.
     * Se definen primero en
     * ´static int bit_pattern_31_[256*4]´
     */
    std::vector<cv::Point> pattern;

    /**
     * Cantidad de puntos singulares deseados en una imagen.
     * Establecido en el archivo de configuración de ORB-SLAM2.
     */
    int nfeatures;

    /**
     * Factor de escala entre niveles de la pirámide.
     * Establecido en el archivo de configuración de ORB-SLAM2.
     */
    double scaleFactor;

    /**
     * Cantidad de niveles de la pirámide.
     * Establecido en el archivo de configuración de ORB-SLAM2.
     */
    int nlevels;

    /**
     * Umbral FAST inicial, usado para la detección de puntos singulares en todas las imágenes.
     * Establecido en el archivo de configuración de ORB-SLAM2.
     */
    int iniThFAST;

    /**
     * Umbral FAST mínimo, para maximizar la cantidad de puntos singulares detectados en una celda de la grilla, cuando el umbral inicial arroja pocos resultados.
     * Establecido en el archivo de configuración de ORB-SLAM2.
     */
    int minThFAST;

    /**
     * Cantidad de puntos singulares deseados por cada nivel de la pirámide.
     * Todos sus valores se calculan a partir de nfeatures.
     * Vector de longitud nlevels.
     */
    std::vector<int> mnFeaturesPerLevel;

    /**
     * Borde del parche circular de diámetro 31.
     * umax es un vector de ´HALF_PATCH_SIZE + 1´ elementos, con el límite de cada línea en un cuarto de circunferencia.
     * Tiene 16 elementos para una circunferencia de 31 píxeles de diámetro, el parche dentro del cual se extrae el descriptor.
     * Se calcula en el constructor.
     */
    std::vector<int> umax;

    /**
     * Factor de escala abosluto de cada nivel de la pirámide, calculado a partir de scaleFactor.
     * Vector de longitud nlevels.
     */
    std::vector<float> mvScaleFactor;

    /**
     * Inversa precalculada de mvScaleFactor.
     * Vector de longitud nlevels.
     */
    std::vector<float> mvInvScaleFactor;

    /**
     * Cuadrado de mvScaleFactor.
     */
    std::vector<float> mvLevelSigma2;

    /**
     * Inversa precalculada de mvLevelSigma2.
     */
    std::vector<float> mvInvLevelSigma2;
};

} //namespace ORB_SLAM

#endif

