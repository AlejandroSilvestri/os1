# os1
OrbSlam2 comentado y modificado para pruebas de monocular slam.

Se basa en el proyecto original ORB-SLAM2 de Raúl Mur Artal et al de la Universidad de Zaragoza.
https://github.com/raulmur/ORB_SLAM2
http://webdiis.unizar.es/~raulmur/orbslam/

Esta versión ejecuta orb-slam2 directamente sobre la webcam en tiempo real.
El proyecto está documentado con estilo javadoc para ser generado con doxygen, y alojado en GitHub:
https://alejandrosilvestri.github.io/os1/doc/html/index.html

La página principal de la documentación está en castellano y se encuentra en main.cc.
La documentación es un trabajo en proceso.

Este proyecto es trabajo de Alejandro Silvestri para de la materia de Robótica y visión artificial de ingeniería informática, y como parte del proyecto RAC del laboratorio de mecatrónica de la Universidad Austral, Pilar, Buenos Aires, Argentina.




#Agregados y modificaciones
- Implementa vocabulario BoW binario, reemplazando al de texto, que carga 100 veces más rápido.
- Permite la visualización de los puntos 3D a color, con el color extraído de la imagen.
- Guarda y carga el mapa en un archivo binario.  Bugs pueden colgar la aplicación durante la carga o guardado.
- Implementa parámetros de distorsión fisheye, para cámaras de gran angular.
- Indicador de salud del tracking: más verde, más saludable; más negro, más cerca de perderse.
- Visualiza la imagen de entrada a color y antidistorsionada.
- Admite entrada de webcam y de video.
- Permite controlar el video de entrada: pausa, reversa, saltar a otro cuadro.


#Algunas pocas indicaciones
Este proyecto no cuenta con cmake, se compila con G++ desde Eclipse en Ubuntu.
Librerías que se deben incluir en el proyecto:
[[https://github.com/AlejandroSilvestri/os1/blob/master/doc/Librerias.png]]

Webcam.yaml tiene la calibración de la cámara.

#Uso

Líneas de comando:

    1) $ os1 conf.yaml video.mp4
    2) $ os1

1) La primera versión carga la configuración del archivo conf.yaml y procesa el archivo de video video,mp4.

2) La segunda versión, sin argumentos, carga la configuración directamente del archivo webcam.yaml y procesa la entrada de la cámara 0.  Para abrir otra cámara hay que modificar el código.

La carpeta de trabajo debe ser la del ejecutable os1.


#Notas
El branch Video es el más actualizado.
Se ejecuta en Linux.  Utiliza zenity como cuadro de diálogo para abrir y guardar archivos.  Una migración a otro sistema operativo requiere al menos reemplazar zenity por otro medio.
