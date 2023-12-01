#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 #importa mensajes de ROS tipo String y Int32
from sensor_msgs.msg import Joy # impor mensaje tipo Joy
from geometry_msgs.msg import Twist, Point # importar mensajes de ROS tipo geometry / Twist

class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args

		# (poner aquí tópicos a los que me suscribo)
		# suscripción: matriz con detecciones.
		self.sub_pose = rospy.Subscriber("pose", Point, self.move)
		
		# publicar movimiento
		self.twist = Twist()
		self.publi = rospy.Publisher("/duckiebot/possible_cmd", Twist, queue_size = 10)

	        # Publicar imagen(es)
	        # (poner aquí tópicos donde publico)
		
		
	def move(self, msg):
		currentPos = [msg.x, msg.y, msg.z]
		y = currentPos[1]
		
		print(currentPos)
		
		y0, y1 = 10, 200
		speed = 1
		
		if y < y1:
			self.twist.v = speed
		else:
			self.twist.v = 0
		
		self.publi.publish(self.twist)
		

def main():
	rospy.init_node('test3') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
