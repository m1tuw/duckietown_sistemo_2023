#!/usr/bin/env python
# resumen: tomar imagen de la camara y publica la imágen con sus detecciones respectivas en el tópico img_with_detections, y la lista de puntos en el formato ((x_1,y_1),...,(x_4,y_4),(ID,0)) (coordenadas en pixeles) en el tópico detections.
# pendiente: ver que hacer con la información q se publica (por ejemplo: todos los returns de la función)
# posible solución: publicar en varios tópicos diferentes o publicar todo solo en uno, preguntar (mas que nada por el tipo del mensaje)



import rospy #importar ros para python
from std_msgs.msg import String, Int32,Float32,Float32MultiArray,Int32MultiArray,MultiArrayDimension # importar mensajes de ROS tipo String y tipo Int32
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
		# (poner aquí tópicos los que me suscribo)
		self.Sub_Cam = rospy.Subscriber("/duckiebot/camera_node/image/raw", Image, self.procesar_img)

			# Publicar imagen(es)
			# (poner aquí tópicos donde publico)

			# publicar imagen
		self.pub_img = rospy.Publisher("img_with_detections", Image, queue_size = 1)

		# publicar return de la función (cambiar mas adelante. nombre del topico: detections)
		# return: sea N el numero de detecciones. entonces: se retornan Corners, Im ID
		# Corners: vector<pair<int,int>> de N puntos
		# ImID: vector<int> de N enteros que describen el ID de los arucos detectados.
		# formato: msg[0][i] = corners[i].x
		# msg[1][i] = corners[i].y
		# msg[2][i] = Id[i]

		#self.pub_detections = rospy.Publisher("detections", Float32MultiArray, queue_size = 1)
		
		self.pub_corners = rospy.Publisher("corners", String, queue_size = 10)
		self.pub_ids = rospy.Publisher("ids", String, queue_size = 10)
		


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
		par = cv2.aruco.DetectorParameters_create()

		#cap = cv2.VideoCapture(0)


		# Our operations on the frame come here
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		(corners, imID, rejected) = cv2.aruco.detectMarkers(gray, dic, parameters = par)

		if type(imID) == type(None):
			msg = bridge.cv2_to_imgmsg(image, "bgr8")
			self.pub_img.publish(msg)
			return

		for i in range(len(imID)):
			cv2.putText(image, imID[i].astype(str)[0], (corners[i][0][0][0], corners[i][0][0][1]), cv2.FONT_HERSHEY_PLAIN, 5, (255,255,255))

		# llenar matriz de detecciones
		N = len(imID)
		detections = np.zeros((N, 5, 2))

		# detections[j][i] = coordenadas x,y primera esquina
		'''
		print(corners)
		for i in range(N):
			for j in range(4):
				print(corners[i][0][j])
				detections[i][j] = corners[i][0][j]
			detections[i][4][0] = float(imID[i])
		'''
		# ==================================

		# Publicar imagen final
		# publicamos la imagen final
		msg = bridge.cv2_to_imgmsg(image, "bgr8")
		
		corners_ros = String()
		ids_ros = String()
		
		for i in range(N):
			ids_ros.data += str(imID[i][0])+" "
			
		corners_ros.data += str(N)+" "
		
		for i in range(N):
			c = ""
			for k in range(4):
				c += str(corners[i][0][k][0]) + " "
				c += str(corners[i][0][k][1]) + " "
			corners_ros.data += c
		
		self.pub_corners.publish(corners_ros)
		self.pub_ids.publish(ids_ros)
		self.pub_img.publish(msg)


def main():
	rospy.init_node('aruco_det_node') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
