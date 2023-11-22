# duckietown_sistemo_2023
## arucodet.py
resumen: tomar imagen de la camara y publica la imágen con sus detecciones respectivas en el tópico img_with_detections y publica las posiciones en pixeles junto con sus IDs en el tópico detections.
Retorna las coordenadas de las cuatro esquinas de los N arucos en sentido horario partiendo por la superior izquierda detectados como un arreglo de Nx5x2, en el que cada detección está de la forma [[x1,y1],...,[x4,y4],[ID,0]].
## posedet.py
Publica la posición del aruco c/r al robot como un punto (x, y, theta). Depende de arucodet.
