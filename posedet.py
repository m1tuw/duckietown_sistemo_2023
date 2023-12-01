import rospy #importar ros para python
from std_msgs.msg import String, Int32,Float32,Float32MultiArray,Int32MultiArray # importar mensajes de ROS tipo String y tipo Int32
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
		#self.sub_detections = rospy.Subscriber("detections", Float32MultiArray, self.detect_pose)
		self.sub_corners = rospy.Subscriber("corners", String, self.detect_pose)
		self.sub_ids = rospy.Subscriber("ids", String, self.detect_ids)
		
		self.ids = []

	        # Publicar imagen(es)
	        # (poner aquí tópicos donde publico)

	        # publicar tupla con posición y rotación en radianes (point 3d)
	        # se publica un punto (x, y, theta) con theta el ángulo de rotación y (x,y) la posición respecto al origen
		self.pub_pose = rospy.Publisher("pose", Point, queue_size = 1)
		
		
	def detect_ids(self, msg):
		self.ids = msg


	def detect_pose(self, msg):
		#detections = msg
		#N = len(detections[0])
		# rearmar vector corners
		
		# pose[i][0] = posicion en X
		# pose[i][1] = posicion en Y
		# pose[i][2] = rotación en radianes
		
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
		
		# hardcode matriz
		cameraMatrix = [[109.244443, 0.360267, 158.945421], [0.000000, 110.007361, 117.624169], [0.000000, 0.000000, 1.000000]]
		distCoeffs = [-0.041032, -0.024147, 0.145470, -0.088792]
		
		h = 10.0 # half square length
		
		objectPoints = [[-h, h, 0],[h, h, 0],[h, -h, 0],[-h, -h, 0]]
		objectPoints = np.array(objectPoints)
		
		corners = np.array(corners)
		
		cameraMatrix = np.array(cameraMatrix)
		distCoeffs = np.array(distCoeffs)
		
		retval, rvec, tvec = cv2.solvePnP(objectPoints, corners[0][0], cameraMatrix, distCoeffs, False, cv2.SOLVEPNP_IPPE_SQUARE)
		
		#rvec *= 180/3.1415926535897932384626433832795028841971
		
		'''
		print("rotation: ")
		print(rvec)
		print("translation: ")
		print(tvec)
		'''
		
		#rmat = cv2.Rodrigues(rvec)
		
		#print("rotation matrix: ")
		#print(rmat)
		
		id = int(self.ids.data.split()[0])
		print(id)
		
		coords = {}
		for i in range(11):
			coords[i] = [0, 80*i, 0]
			
		currentPos = np.zeros(3)
		
		for i in range(3):
			currentPos[i] = coords[id][i] + tvec[i]

		pose = Point()
		pose.x = currentPos[0]
		pose.y = currentPos[1]
		pose.z = rvec[0][0]
		
		print(currentPos)
			
		pub_pose.publish(pose)
		
		
		

def main():
	rospy.init_node('test3') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
