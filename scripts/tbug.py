#!/usr/bin/env python
import rospy
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import cos, sin, pi, sqrt, atan2
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2
import numpy as np


class bug:
	def __init__(self, px, py):
		self.robot_pos = [0, 0]
		self.robot_vel = [0, 0]
		self.robot_ori = 0
		self.curve_pos = [px, py]

		self.lidar_raw = []
		self.l_max = 0
		self.l_min = 0
		self.vel_msg = Twist()


		self.controlador = control()

		rospy.init_node("Tbug", anonymous=True)
		rospy.Subscriber('/odom', Odometry, self.callback_robot_odom)
		rospy.Subscriber('/base_scan', LaserScan, self.sensor_callback)
		self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)


	def callback_robot_odom(self,data):
		self.robot_pos[0] = data.pose.pose.position.x
		self.robot_pos[1] = data.pose.pose.position.y

		x_q = data.pose.pose.orientation.x
		y_q = data.pose.pose.orientation.y
		z_q = data.pose.pose.orientation.z
		w_q = data.pose.pose.orientation.w
		euler = euler_from_quaternion([x_q, y_q, z_q, w_q])

		self.robot_ori = euler[2]

	def sensor_callback(self, data):
		self.lidar_raw  = data.ranges
		self.l_max = data.range_max
		self.l_min = data.range_min



	def distancy(self):
		d = sqrt((self.curve_pos[0]-self.robot_pos[0])**2 + (self.curve_pos[1]-self.robot_pos[1])**2)
		return d

	def dist_obst(self, px, py):
		d = sqrt((px-self.robot_pos[0])**2 + (py-self.robot_pos[1])**2)
		return d


	def find_endpoints(self):
		end_ids = []
		cont = 0
		for i in range(1, len(self.lidar_raw)-1):
			if(self.l_min < self.lidar_raw[i] < self.l_max):
				if(self.lidar_raw[i+1] >= self.l_max):
					end_ids.append(i)
				elif(self.lidar_raw[i-1] >= self.l_max):
					end_ids.append(i)

		return end_ids



	def min_obst_dist(self,end_ids):
		# frente - 500 , 581
		# lado - 740, 851
		# dxo = self.lidar_raw[end_ids]

		d = self.distancy()
		d_min = 100
		alfa = 0
		obst_pos = []
		for i in end_ids:
			dxo = self.lidar_raw[i]
			# doq = sqrt((self.curve_pos[0]-obst_pos[0])**2 + (self.curve_pos[1]-obst_pos[1])**2)
			doq = sqrt(d**2 + dxo**2)
			ds = dxo+doq
			if(d < d_min):
				d_min = ds
				alfa = i

		print(self.lidar_raw[alfa])
		obst_pos = [-self.lidar_raw[alfa]*cos((alfa - 180) * 180.0 / pi), self.lidar_raw[alfa]*sin((alfa -180) * 180.0 / pi)]
		print(obst_pos)

		return d_min, alfa-180, obst_pos


	def follow_target(self, px, py):
		self.vel_msg.linear.x, self.vel_msg.angular.z = self.controlador.control_([px,py],self.robot_pos, self.robot_ori)

		self.pub_cmd_vel.publish(self.vel_msg)

	def go_obst(self, alfa):
		self.vel_msg.linear.x = 1.0
		self.vel_msg.angular.z = self.k*(alfa - self.robot_ori)

		self.pub_cmd_vel.publish(self.vel_msg)


	def contourn_obst(self, s, alfa, obst_detec):
		Ux = atan(math.abs(s - obst_detec))
		Uy = sqrt(1 - Ux**2)
		self.vel_msg.linear.x, self.vel_msg.angular.z = self.controlador.feedback_linearization(Ux,Uy,self.robot_ori)

		self.pub_cmd_vel.publish(self.vel_msg)






class control:
	def __init__(self):
		self.d = 0.2
		self.k = 1


	def control_(self,pos_curve, pos_robot, theta):
	    """
	    Entradas:
	    pos_curve: Vetor contendo a posicao x,y de referencia da curva
	    pos_robot: Posicao do robo no mundo

	    Retorno:
	    Ux: Velocidade no eixo x do sistema de coordenadas inercial
	    Uy: Velocidade no eixo y do sistema de coordenadas inercial
	    """

	    Ux = self.k * (pos_curve[0] - pos_robot[0])
	    Uy = self.k * (pos_curve[1] - pos_robot[1])

	    return self.feedback_linearization(Ux,Uy,theta)


	def feedback_linearization(self,Ux, Uy, theta_n):
	    """
	    Entradas: 
	    Ux: Velocidade no eixo x do sistema de coordenadas inercial
	    Uy: Velocidade no eixo y do sistema de coordenadas inercial
	    theta_n: orientacao do robo

	    Retorno: 
	    Vx: Velocidade no eixo x do robo 
	    w: Velocidade angular do robo
	    """

	    vx = cos(theta_n) * Ux + sin(theta_n) * Uy
	    w = -(sin(theta_n) * Ux)/ self.d  + (cos(theta_n) * Uy) / self.d 

	    return vx, w





def follow():
	rospy.sleep(0.2)

	px, py, = raw_input('Insira o valor do raio em x e y da curva (valores separados por espaco): ').split()
	px, py = [float(i) for i in [px, py]]


	Tbug = bug(px,py)
	delta = 0.3 # min dist of the target to stop algorithm
	obst_detec = 1  # min dist of the obstacle to contour
	px_ = px
	py_ = py

	rate = rospy.Rate(20)

	while not rospy.is_shutdown():
		if (Tbug.lidar_raw):
			if (Tbug.distancy() < delta):
				print("Alvo alcancado!\n")
				break


			d = Tbug.distancy()

			end_ids = Tbug.find_endpoints()

			dq,alfa,obst_pos = Tbug.min_obst_dist(end_ids)

			# if(end_ids):
				
			# 	Tbug.go_obst(alfa)

			

			print("Menor distancia robo objeto: %f \n Menor distancia objeto destino: %f \n  Angulo para o alvo: %f" % (d, dq, alfa))


			px_ = obst_pos[0]
			py_ = obst_pos[1]

			while (Tbug.dist_obst(px_,py_) > delta):
				Tbug.follow_target(px_, py_)



		rate.sleep()



if __name__ == '__main__':
    try:
        follow()
    except rospy.ROSInterruptException:
        pass