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
 * Cada instancia es un nodo de un árbol QuadTree, que divide un rectángulo de una imagen en cuatro rectángulos similares.
 * ORBextractor utiliza una versión simplificada de Octree (en rigor es un Quadtree),
 * creando un árbol cuyos nodos representan un rectángulo de una imagen y sus cuatro hijos representan un cuarto de esa imagen.
 * ExtractorNode es la clase de esos nodos, que registran las coordenadas de los vértices del rectángulo asociado,
 * y la lista de puntos singulares en él.
 *
 * El método de búsqueda de zonas consiste en seguir dividiendo los rectángulos hasta que contengan un o ningún punto singular.
 * El árbol termina cuando se alcanza una cantidad máxima de nodos esperados, o cuando ningún nodo tiene más de un puntos singular.
 *
 * Objeto usado exclusivamente en ORBextactor::DistributeOctTree.
 */
class ExtractorNode
{
public:
	/**
	 * Constructor único que sólo inicialia bNoMore = false.
	 * El resto de las propiedades se debe inicializar luego de la construcción.
	 */
    ExtractorNode():bNoMore(false){}

    /**
     * Divide el nodo en 4: n1, n2, n3 y n4.
     * Notar que este método no construye los cuatro nodos derivados, éstos son construidos por el método que invoca.
     */
    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    /**
     * Vector de puntos singulares de este nodo.
     * vKeys.empty() responde si el nodo tiene o no puntos singulares.
     */
    std::vector<cv::KeyPoint> vKeys;

    ///@{
    //@{
    /**
     * Vértice del rectángulo límite:
     * U: upper, B: bottom,
     * L: left,  R: right
     *
     * El procedimiento que invoca la construcción de un objeto ExtractorNode debe luego inicializar esta propiedad.
     */
    cv::Point2i UL, UR, BL, BR;
    ///@}
    //@}

    /**
     * Iterador que apunta a la posición del propio nodo en una lista.
     * El procedimiento que invoca la construcción de un objeto ExtractorNode debe luego inicializar esta propiedad.
     */
    std::list<ExtractorNode>::iterator lit;

    /**
     * Señal que indica que el nodo tiene exactamente un punto asignado.
     * Se inicializa como false, diversos métodos verifican y lo ponen en true cuando tiene un único punto.
     * Diversos métodos evalúan el nodo para determinar si tienen un único punto singular (bNoMore == true) y no requiere más análisis,
     * ningún punto singular (vKeys.empty() == true) lo que hace al nodo eliminable,
     * o en otro caso contiene más de un punto singular con lo que se puede seguir procesando.
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
 * Con la excepción de mvImagePyramid, todas las propiedades son protegidas, se establecen durante la construcción y no cambian.
 *
 *
 * ORBextractor::ComputeKeyPointsOctTree contiene una buena descripción de los puntos singulares.
 *
 *
 *
 *
 * La clase usa las siguientes funciones globales exclusivas:
 *
 * IC_Angle(const Mat& image, Point2f pt,  const vector<int> & u_max)
 * Determina el ángulo del parche circular con centro en pt, y diámetro PATH_SIZE = 31.
 * Es el ángulo del vector que une pt con el baricentro del parche.
 * u_max es un vector con los extremos línea por línea de la circunferencia pixelada.
 * Devuelve el ángulo en radianes.
 *
 *
 *
 * computeOrientation(const Mat& image, vector<KeyPoint>& keypoints, const vector<int>& umax)
 * Determina el ángulo de cada keypoints con IC_Angle.
 * Guarda el resultado en keypoints[].angle .
 *
 *
 *
 * computeOrbDescriptor(const KeyPoint& kpt, const Mat& img, const Point* pattern, uchar* desc)
 * Extrae el descriptor ORB del punto singular.
 * @param kpt Punto singular.  Su ángulo expresa la orientación.
 * @param img Imagen sobre la que se extraerá el descriptor.
 * @param pattern Siempre el mismo, patrón de coordenadas para la evaluación BRIEF.
 * @param desc Descriptor resultado, de 256 bits (32 bytes, 4 int).
 * Invocado sólo desde computeDescriptors.
 *
 *
 *
 * static int bit_pattern_31_[256*4]
 * Coordenadas BRIEF.
 * Cada renglón contiene un par de coordenadas x,y cuyas intensidades se comparan para obtener un bit del descriptor binario.
 * Hay 256 renglones para los 256 bits del BRIEF.
 * Las coordenadas son relativas al punto singular a considerar, y a su orientación.
 * Esto significa que estas coordenadas patrón se rototrasladan para obtener las coordenadas reales para extraer un descriptor.
 * Los valores son enteros y varían entre -13 y 13.  Quizás estén todos en un círculo de diámetro 31 denominado parche.
 *
 *
 * const float factorPI = (float)(CV_PI/180.f)
 * Factor conversor de grados a radianes.
 * Al multiplicar un ángulo en grados, se obtiene el ángulo en radianes.
 *
 *
 *
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
	 *
	 * Invocado sólo desde Frame::ExtractORB
     */
    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    void operator()( cv::InputArray image, cv::InputArray mask,
      std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors);

    /**
     * Devuelve el atributo protegido nlevels, la cantidad de niveles en la pirámide, establecida por el constructor y de sólo lectura.
     *
     * Invocado sólo desde el constructor de Frame.
     */
    int inline GetLevels(){
        return nlevels;}

    /**
     * Devuelve el atributo protegido scaleFactor, el factor de escala entre niveles de la pirámide, establecido por el constructor y de sólo lectura.
     *
     * Invocado sólo desde el constructor de Frame.
     */
    float inline GetScaleFactor(){
        return scaleFactor;}

    /**
     * Devuelve el atributo protegido mvScaleFactor.
     *
     * Invocado sólo desde el constructor de Frame.
     */
    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    /**
     * Devuelve el atributo protegido mvInvScaleFactor.
     *
     * Invocado sólo desde el constructor de Frame.
     */
    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    /**
     * Devuelve el atributo protegido mvLevelSigma2.
     *
     * Invocado sólo desde el constructor de Frame.
     */
    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    /**
     * Devuelve el atributo protegido mvInvLevelSigma2.
     *
     * Invocado sólo desde el constructor de Frame.
     */
    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    /**
     * Imágenes de la pirámide, a las que se aplica FAST.
     * Producida por ComputePyramid, es un vector de ´nlevels´ imágenes.
     */
    std::vector<cv::Mat> mvImagePyramid;

protected:

    /**
     * Genera las imágenes de la pirámide y las guarda en el vector mvImagePyramid.
     * @param image Imagen a procesar, obtenida de la cámara.
     *
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
     *
     * Invocado sólo desde ORBextractor::operator()
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
     *
     * Invocado sólo desde ORBextractor::ComputeKeyPointsOctTree .
     */
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

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
     * El vector se escribe solamente en el constructor.
     * Vector de longitud nlevels.
     */
    std::vector<float> mvScaleFactor;

    /**
     * Inversa precalculada de mvScaleFactor.
     * El vector se escribe solamente en el constructor.
     * Vector de longitud nlevels.
     */
    std::vector<float> mvInvScaleFactor;

    /**
     * Cuadrado de mvScaleFactor.
     * El vector se escribe solamente en el constructor.
     */
    std::vector<float> mvLevelSigma2;

    /**
     * Inversa precalculada de mvLevelSigma2.
     * El vector se escribe solamente en el constructor.
     */
    std::vector<float> mvInvLevelSigma2;
};

} //namespace ORB_SLAM

#endif

