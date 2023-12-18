# duckietown_sistemo_2023
Movimiento autónomo de un duckiebot usando una grilla de arucos.

## Prerequisitos
- ROS noetic
- Python 3
- OpenCV 2
- [Módulos ROS de Duckietown Chile](https://github.com/Duckietown-Chile/Software)

## Guía de inicio rápido
1. Instalar dependencias.
2. [Calibrar la cámara](http://wiki.ros.org/camera_calibration) y actualizar cameraMatrix y distCoeffs en testarucodet.py acordemente.
3. Instalar una grilla de 4x4 arucos de 20mm cada uno con una separación de 110mm entre sí y con los IDs en la forma:

| 12 | 13 | 14 | 15 |
| 8  | 9  | 10 | 11 |
| 4  | 5  | 6  | 7  |
| 0  | 1  | 2  | 3  |

4. Iniciar ROS en el duckiebot y colocar al robot en la grilla.
5. Desde un computador conectado al ROS del duckiebot, correr testarucodet.py, movement.py y entregar los comandos de la forma x y.


## Módulos
### testarucodet.py
Toma la imagen de la camara y publica la imágen con sus detecciones respectivas en el tópico /img_with_detections y publica la posición del robot en milímetros junto con su orientación con respecto al orignen en el formato "x y theta" como un string en el tópico /coords.
### cmd.py
Lee continuamente comandos desde el standard input y los publica en /cmd
### movement.py
Recibe las coordenadas del robot en /img_with_detections junto con comandos en /cmd de la forma "x y" y hace que el robot se mueva a las coordenadas dadas.
