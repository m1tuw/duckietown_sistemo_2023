# resumen: tomar imagen de la camara y publicarla en un nodo camera_image_raw
# pendiente: ver que hacer con la información q se publica (por ejemplo: todos los returns de la función)
# posible solución: publicar en varios nodos diferentes o publicar todo solo en uno, preguntar (mas que nada por el tipo del mensaje)

#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32,Float32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist, Point # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Image # importar mensajes de ROS tipo Image
import cv2
from cv_bridge import CvBridge # importar convertidor de formato de imagenes
import numpy as np # importar libreria numpy

class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		
		# Suscribrirse a la camara
		# (poner aquí nodos a los que me suscribo)
		self.Sub_Cam = rospy.Subscriber("/duckiebot/camera_node/image/raw", Image, self.procesar_img)
		
	        # Publicar imagen(es)
	        # (poner aquí tópicos donde publico)
	        
	        # publicar imagen
		self.pub_img = rospy.Publisher("img_with_detections", Image, queue_size = 1)
		
		# publiar return de la función (cambiar mas adelante. nombre del topico: detections)
		#self.pub_mask = rospy.Publisher("mascara", Image, queue_size = 1)

	def procesar_img(self, msg):
		#Transformar Mensaje a Imagen
		bridge = CvBridge()
		image = bridge.imgmsg_to_cv2(msg, "bgr8")
	
		#Espacio de color

			#cv2.COLOR_RGB2HSV
			#cv2.COLOR_RGB2GRAY
			#cv2.COLOR_RGB2BGR
			#cv2.COLOR_BGR2HSV
			#cv2.COLOR_BGR2GRAY
			#cv2.COLOR_BGR2RGB

		image_out = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 

		# ==================================
				
		dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
		par = cv2.aruco.DetectorParameters()


		#cap = cv2.VideoCapture(0)
		

		# Our operations on the frame come here
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		(corners, imID, rejected) = cv2.aruco.detectMarkers(gray, dic, parameters = par)
		if type(imID) != type(None):
			for i in range(len(imID)):
			    cv2.putText(image, imID[i].astype(str)[0], corners[i][0][0].astype(int), cv2.FONT_HERSHEY_PLAIN, 5, (255,255, 255))


		
		# ==================================
		
		# Publicar imagen final
		# publicamos la imagen final
		msg = bridge.cv2_to_imgmsg(image, "bgr8")
		self.pub_img.publish(msg)


def main():
	rospy.init_node('test3') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()

