#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 #importa mensajes de ROS tipo String y Int32
from sensor_msgs.msg import Joy # impor mensaje tipo Joy
from geometry_msgs.msg import Point, Twist # importar mensajes de ROS tipo geometry / Twist
from duckietown_msgs.msg import Twist2DStamped
import numpy as np

class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args

		# (poner aquí tópicos a los que me suscribo)
		# suscripción: coordenadas.
		#self.sub_cmd = rospy.Subscriber("cmd", String, self.cmd, tcp_nodelay=True)
		self.sub_pose = rospy.Subscriber("coords", String, self.move	)
		self.cmdx = None
		self.cmdy = None
		
		# publicar movimiento
		self.twist = Twist2DStamped()
		self.publi = rospy.Publisher("/duckiebot/wheels_driver_node/car_cmd", Twist2DStamped, queue_size = 10)

	        
	# recibe el comando de las coordenadas
	'''
	def cmd(self, msg):
		print("test1")
	
		command = msg.data.split()
		print(command)
		self.cmdx = float(command[0])
		self.cmdy = float(command[1])
		print(msg, "b")
		'''
	# recibe las coordenadas actuales y se mueve
	def move(self, msg):
		#print("test2")
	
		coords = msg.data.split()
		x = float(coords[0])
		y = float(coords[1])
		theta = float(coords[2])
		eps = 10
		teps = 0.5
		
		#print(self.cmdx, self.cmdy)
		
		if self.cmdx == None or (x-self.cmdx)**2+(y-self.cmdy)**2<eps:
			command = input()
			if command == "xd":
				self.twist.v = 0
				self.twist.omega = 0
				self.publi.publish(self.twist)
				return
			cmd = command.split()
			self.cmdx = float(cmd[0])
			self.cmdy = float(cmd[1])
		

		thetacmd = np.arctan2(y-self.cmdy, x-self.cmdx)
		print("Theta:", theta)
		print("Thetacmd:", thetacmd)
		print("x:", x, "y:", y)
		print("query:", self.cmdx, self.cmdy)	
		dtheta = theta-thetacmd
		speed = 1
		tspeed = 0.1
		
		if abs(dtheta) > teps:
			self.twist.v = 0
			if dtheta > 0:
				self.twist.omega = tspeed
			else:
				self.twist.omega = -tspeed
		else: # rotacion ok
			print("eps1 ok")
			self.twist.omega = 0
		
			dist2 = (y-self.cmdy)**2 + (x-self.cmdx)**2
			
			if dist2 > eps:
				self.twist.v = speed
			else:
				print("eps2 ok")
				self.twist.v = 0
			
		self.publi.publish(self.twist)
		

def main():
	rospy.init_node('movement') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
