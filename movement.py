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

		# (poner aquí tópicos a los que me suscribo)
		# suscripción: coordenadas.
		self.sub_pose = rospy.Subscriber("coords", String, self.move)
		self.sub_cmd = rospy.Subscriber("cmd", String, self.cmd)
		self.cmdx = None
		self.cmdy = None
		
		# publicar movimiento
		self.twist = Twist2DStamped()
		self.publi = rospy.Publisher("/duckiebot/possible_cmd", Twist2DStamped, queue_size = 10)



	        
	# recibe el comando de las coordenadas
	def cmd(self, msg):
		command = msg.data.split()
		self.cmdx = command[0]
		self.cmdy = command[1]
		print(msg, "b")
		
		
	# recibe las coordenadas actuales y se mueve
	def move(self, msg):
		coords = msg.data.split()
		x = coords[0]
		y = coords[1]
		theta = coords[2]
		eps = 5
		teps = 0.1
		#print("a", self.cmdx)
		
		if self.cmdx == None:
			return
		

		thetacmd = np.arctan2(y-cmdy, x-cmdx)
		print("Theta:", theta)
		print("Thetacmd:", thetacmd)
		dtheta = theta-thetacoords
		speed = 0.3
		tspeed = 0.2
		
		if abs(dtheta) < teps:
			if dtheta > 0:
				self.twist.theta = tspeed
			else:
				self.twist.theta = -tspeed
		else:
			self.twist.theta = 0
		
			dist2 = (y-cmdy)**2 + (x-cmdx)**2
			
			if dist2 > eps:
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
