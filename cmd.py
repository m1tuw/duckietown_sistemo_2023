#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 #importa mensajes de ROS tipo String y Int32
from sensor_msgs.msg import Joy # impor mensaje tipo Joy
from geometry_msgs.msg import Point, Twist # importar mensajes de ROS tipo geometry / Twist
from duckietown_msgs.msg import Twist2DStamped

class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		
		# publicar movimiento
		self.publi = rospy.Publisher("/cmd", String, queue_size = 1)
	
	def start(self):
		while True:
			msg = String()
			msg.data = input()
			self.publi.publish(msg)
			rospy.sleep(1)



def main():
	rospy.init_node('test3') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba
	obj.start()
	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
