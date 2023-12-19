#!/usr/bin/env python
# resumen: tomar imagen de la camara y publica la imágen con sus detecciones respectivas en el tópico img_with_detections, y la lista de puntos en el formato ((x_1,y_1),...,(x_4,y_4),(ID,0)) (coordenadas en pixeles) en el tópico detections.
# pendiente: ver que hacer con la información q se publica (por ejemplo: todos los returns de la función)
# posible solución: publicar en varios tópicos diferentes o publicar todo solo en uno, preguntar (mas que nada por el tipo del mensaje)



import rospy #importar ros para python
import math
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
		
		self.pub_coords = rospy.Publisher("coords", String, queue_size = 10)

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
		
		# BORRAR=============
		
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
			
		msg = corners_ros
		
		# rebuild corners
		cstr = msg.data.split()	
		N = int(cstr[0])
		N = min(N, 1)
		
		corners = [[[]]*N]
		for i in range(N):
			for j in range(4):
				A = np.zeros(2)
				x = float(cstr[1 + N*i + 2*j])
				y = float(cstr[1 + N*i + 2*j + 1])
				A[0] = x
				A[1] = y
				corners[i][0].append(A)
		
		# hardcode matriz (pinhole)
		cameraMatrix = [[301.360315, 0.000000, 151.013685], [0.000000, 301.949166, 115.746894], [0.000000, 0.000000, 1.000000]]
		distCoeffs = [0.097030, -0.226118, -0.001924, -0.003349]
			
		# fisheye
		#cameraMatrix = [[109.244443, 0.360267, 158.945421], [0.000000, 110.007361, 117.624169], [0.000000, 0.000000, 1.000000]]
		#distCoeffs = [-0.041032, -0.024147, 0.145470, -0.088792]
		
		h = 10.0 # half square length
		
		objectPoints = [[-h, h, 0],[h, h, 0],[h, -h, 0],[-h, -h, 0]]
		objectPoints = np.array(objectPoints)
		
		corners = np.array(corners)
		
		cameraMatrix = np.array(cameraMatrix)
		distCoeffs = np.array(distCoeffs)
		
		#retval, rvec, tvec = cv2.solvePnP(objectPoints, corners[0][0], cameraMatrix, distCoeffs, False, cv2.SOLVEPNP_IPPE_SQUARE)
		retval, rvec, tvec = cv2.solvePnP(objectPoints, corners[0][0], cameraMatrix, distCoeffs, False)
		
		# para debugear
		#rvecdeb = rvec * 180/3.1415926535897932384626433832795028841971
		
		id = int(ids_ros.data.split()[0])
		#print(id)
		
		coords = {}
		for i in range(16):
			coords[i] = [110*(i%4), 110*(i//4), 0]
		
		#print(rvec)
		
		rot = np.zeros([2, 2])
		fix = np.zeros([2, 2])
		
		fix[0][0] = 0
		fix[1][0] = 1
		fix[0][1] = -rot[1][0]
		fix[1][1] = rot[0][0]
		
		p1 = np.zeros(2)
		p2 = np.zeros(2)
		
		p1[0] = corners[0][0][0][0]
		p1[1] = corners[0][0][0][1]
		p2[0] = corners[0][0][1][0]
		p2[1] = corners[0][0][1][1]	
		coords
		disp = np.zeros(2) # short for displacement
		disp[0] = p2[0] - p1[0]
		disp[1] = p2[1] - p1[1]
		
		theta = -np.arctan2(disp[1], disp[0]) # por alguna razon se resta en vez de sumar xd
		#print(theta*180/np.pi) 		
		
		rot[0][0] = np.cos(-theta)
		rot[1][0] = np.sin(-theta)
		rot[0][1] = -rot[1][0]
		rot[1][1] = rot[0][0]
		
		currentPos = np.zeros(2)
		
		for i in range(2):
			currentPos[i] = coords[id][i] + np.matmul(rot,[tvec[0], tvec[1]])[i]
		
		# ===================
		
		# agregar flechita
		cx = 200
		cy = 100
		start_point = (cx, cy)
		end_point = (int(cx + 50*math.cos(theta)), int(cy + 50*math.sin(theta)))
		color = (0, 0, 255)
		thickness = 2
		image = cv2.arrowedLine(image, start_point, end_point, color, thickness, tipLength = 0.5)
		for i in range(4): 
			p = corners[0][0][i]
			image = cv2.circle(image, (int(p[0]), int(p[1])), 10, (0, 80*i, 0), 5)
	
		# Publicar imagen final
		# publicamos la imagen final
		msg = bridge.cv2_to_imgmsg(image, "bgr8")
			
		self.pub_img.publish(msg)
		
		coord_msg = String()
		
		for i in range(2):
			coord_msg.data += str(currentPos[i])+" "
			
		coord_msg.data += str(theta)
		
		self.pub_coords.publish(coord_msg)
		



def main():
	rospy.init_node('aruco_det_node') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
