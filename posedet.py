import rospy #importar ros para python
from std_msgs.msg import String, Int32,Float32,Float32MultiArray # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist, Point # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Image # importar mensajes de ROS tipo Image
import cv2 # abstenerse de usar import cv2 as cv hay metodos que tienen cv2 como parte de su nombre y no se cambian D:
from cv_bridge import CvBridge # importar convertidor de formato de imagenes
import numpy as np # importar libreria numpy

class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args

		# (poner aquí tópicos a los que me suscribo)
		# suscripción: matriz con detecciones.
		self.sub_detections = rospy.Subscriber("detections", Float32MultiArray, self.detect_pose)

	        # Publicar imagen(es)
	        # (poner aquí tópicos donde publico)

	        # publicar tupla con posición y rotación en radianes (point 3d)
	        # se publica un punto (x, y, theta) con theta el ángulo de rotación y (x,y) la posición respecto al origen
		self.pub_pose = rospy.Publisher("img_with_detections", Point, queue_size = 1)


	def detect_pose(self, msg):
		detections = msg
		N = len(detections[0])
		# rearmar vector corners
		corners = []
		imID = []

		for i in range(N):
			for k in range(4):
				y = detections[k+1][i]
				corners.append(detections[i][j])
			imID.append(detections[i][5][0])



def main():
	rospy.init_node('test3') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
