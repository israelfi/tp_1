#!/usr/bin/env python
import rospy
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import cos, sin, pi, sqrt, atan2, atan, floor, ceil
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
		self.lidar_x = [0] * 360
		self.lidar_y = [0] * 360
		self.l_max = 0
		self.l_min = 0
		self.vel_msg = Twist()

		self.controlador = control()

		rospy.init_node("Tbug", anonymous=True)
		rospy.Subscriber('/base_pose_ground_truth', Odometry, self.callback_robot_odom)
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

		alfa = 0
		for x in self.lidar_raw:
			sx = (cos(self.robot_ori)*(x*cos(np.deg2rad(alfa-180))) - sin(self.robot_ori)*(x*sin(np.deg2rad(alfa-180)))) +self.robot_pos[0]
			sy = (sin(self.robot_ori)*(x*cos(np.deg2rad(alfa-180))) + cos(self.robot_ori)*(x*sin(np.deg2rad(alfa-180)))) +self.robot_pos[1]
			self.lidar_x[alfa]= sx
			self.lidar_y[alfa] = sy
			alfa = alfa+1


	def distancy(self):
		d = sqrt((self.curve_pos[0]-self.robot_pos[0])**2 + (self.curve_pos[1]-self.robot_pos[1])**2)
		return d

	def find_endpoints(self):
		# a = min(2,self.l_max)
		# ind = 0.1*a

		# dist = np.array(self.lidar_raw)

		# for i in range(len(dist)):
		# 	if(dist[i] >= self.l_max):
		# 		dist[i] = 0

		# np.append(dist,dist[0])
		# dist2 = np.append(dist[1:len(dist)], 0)
		# duum = np.abs(dist2 - dist)
		# duum = duum[0:len(duum)-1]
		# idx = []
		# for i in range(len(duum)):
		# 	if(duum[i] > ind):
		# 		idx.append(i)
		# print idx
		# return idx
		end_ids = []
		cont = 0
		for i in range(1, len(self.lidar_raw)-1):
			if(self.l_min < self.lidar_raw[i] < self.l_max):
				if(self.lidar_raw[i+1] >= self.l_max):
					end_ids.append(i)
				elif(self.lidar_raw[i-1] >= self.l_max):
					end_ids.append(i)


		return end_ids


	def best_obstacle(self,endp):
		d_f_n = 1000000
		for i in range(len(endp)):
			dummy_f = np.array([self.lidar_x[endp[i]], self.lidar_y[endp[i]]]) - np.array([self.curve_pos[0],self.curve_pos[1]])
			dummy_f = np.sqrt(np.sum(dummy_f**2))
			dummy_d = np.array([self.lidar_x[endp[i]], self.lidar_y[endp[i]]]) - np.array([self.robot_pos[0],self.robot_pos[1]])
			dummy_d = np.sqrt(np.sum(dummy_d**2))
			# print "\nendp:", endp[i], "dummy_f:", dummy_f, "dummy_d:", dummy_d, "\n"
			if(d_f_n > dummy_f + dummy_d):
				d_f_n = dummy_f + dummy_d
				go_end_p = np.array([self.lidar_x[endp[i]], self.lidar_y[endp[i]]])

		return d_f_n, go_end_p

	def min_dist(self):
		s = self.lidar_raw[0]
		cont = 0
		alfa = 0
		for i in self.lidar_raw:
			if (i<s):
				s = i
				alfa = cont
			cont = cont + 1

		return s, alfa

	def find_curve(self, px, py, t):
		x = self.robot_pos[0] + t*(px - self.robot_pos[0])
		y = self.robot_pos[1] + t*(py - self.robot_pos[1])

		return x,y


	def follow_target(self, px, py):
		self.vel_msg.linear.x, self.vel_msg.angular.z = self.controlador.control_([px,py],self.robot_pos, self.robot_ori)

		self.pub_cmd_vel.publish(self.vel_msg)

	def contourn_obst(self, s, alfa, obst_detec):
		K = 5.0
		Ux = -(2.0/pi)*atan(K*np.abs(s - obst_detec))
		Uy = sqrt(1 - Ux**2)
		self.vel_msg.linear.x, self.vel_msg.angular.z = self.controlador.feedback_linearization(Ux,Uy,self.robot_ori)

		self.pub_cmd_vel.publish(self.vel_msg)






class control:
	def __init__(self):
		self.d = 0.2
		self.k = 1


	def control_(self,pos_curve, pos_robot, theta):

		Ux = self.k * (pos_curve[0] - pos_robot[0])
		Uy = self.k * (pos_curve[1] - pos_robot[1])

		return self.feedback_linearization(Ux,Uy,theta)


	def feedback_linearization(self,Ux, Uy, theta_n):

		vx = cos(theta_n) * Ux + sin(theta_n) * Uy
		w = -(sin(theta_n) * Ux)/ self.d  + (cos(theta_n) * Uy) / self.d

		return vx, w





def follow():
	rospy.sleep(0.2)

	px, py, = raw_input('Insira o valor do destino em x e y (valores separados por espaco, considere o tamanho do mapa 25x25): ').split()
	px, py = [float(i) for i in [px, py]]

	# px , py = [7.0, 20.0]

	Tbug = bug(px,py)
	delta = 0.5 # min dist of the target to stop algorithm
	obst_detec = 0.8  # min dist of the obstacle to contour
	px_ = px
	py_ = py

	d_f = 1000000


	rate = rospy.Rate(20)

	stage = 0
	t_init = rospy.get_time()

	while not rospy.is_shutdown():

		if(Tbug.lidar_raw):

			# Dist. ate o alvo
			dq = Tbug.distancy()

			theta_s = atan2((py - Tbug.robot_pos[1]), (px - Tbug.robot_pos[0]))
			theta_s = np.mod(theta_s, 2*pi)
			theta_q = np.rad2deg(theta_s)
						
			# theta_q_idx = ((theta_q + (Tbug.robot_ori + pi)) * 180 / pi) + 180 - 360 + 90
			ori_rad = Tbug.robot_ori
			ori_rad = np.mod(ori_rad, 2*pi)
			ori_deg = np.rad2deg(ori_rad)
			
			theta_q_idx = theta_q - ori_deg + 180
			# print '\ntheta:', theta_q, '| robot:', ori_deg
			if theta_q_idx > 360:
				theta_q_idx -= 360

			# print 'theta_idx:', theta_q_idx

			# Leituras do laser
			do = Tbug.lidar_raw
			dpo = [Tbug.lidar_x, Tbug.lidar_y]  ## dpo[coord][angle] -> dpo[0][10] = pos x referente a leitura de feixe 10 graus

			# Dist. e angulo do obstaculo mais prox.
			do_min, idx_do_min = Tbug.min_dist()

			

			# Follow Target	
			# print(do[int(floor(theta_q_idx))], Tbug.l_max, dq, do_min, obst_detec)		
			if( (do[int(floor(theta_q_idx))] >= min([Tbug.l_max, dq])) and (do[int(ceil(theta_q_idx))+1] >= min([Tbug.l_max, dq])) and (do_min > obst_detec)):
			# if( (do[int(floor(theta_q)+1)] >= min([Tbug.l_max, dq])) and (do[int(ceil(theta_q))+1] >= min([Tbug.l_max, dq])) and (do_min > obst_detec)):
				print("Going to the Taget\n")
				t = rospy.get_time() - t_init
				rx, ry = Tbug.find_curve(px,py,t)
				Tbug.follow_target(rx, ry)

				if (Tbug.distancy() < delta):
					print("Target Founded!\n")
					px, py, = raw_input('Insira o valor do proximo destino em x e y (valores separados por espaco, considere o tamanho do mapa 30x30): ').split()
					d_f = 1000000
					px, py = [float(i) for i in [px, py]]
					Tbug.curve_pos = [px, py]

			

			# Leaving from obstacles
			else:
				endp = Tbug.find_endpoints()
				if(endp):
					endp = Tbug.find_endpoints()
					## print(endp)
					# Dist. e coordenadas do ponto de descontinuidade que minimza a dist ate o alvo
					d_f_n, go_end_p = Tbug.best_obstacle(endp)
					print 'Endpoint:', go_end_p

					print "d_f_n:", d_f_n, "| d_f:", d_f, "\n"	
					if ((do_min > obst_detec) and not (d_f_n > d_f)):
						print("Going for the best obstacle avoidance.")
						d_f = d_f_n
						go_end = go_end_p
						dummy_d2 = go_end - np.array([Tbug.robot_pos[0],Tbug.robot_pos[1]])
						dummy_angle = atan2(dummy_d2[1], dummy_d2[0])
						dummy_d2 = sqrt(np.sum(dummy_d2**2))
						do_min, idx_do_min = Tbug.min_dist()
						
						t = rospy.get_time() - t_init
						rx, ry = Tbug.find_curve(go_end[0],go_end[1],t)
						Tbug.follow_target(rx, ry)

						# while(do_min > obst_detec and not (d_f_n > d_f)):
						# 	print "\nd_f_n:", d_f_n, "| d_f:", d_f
						# 	print("Going for the best obstacle avoidance")
						# 	d_f_n, go_end_p = Tbug.best_obstacle(endp)
						# 	d_f = d_f_n
						# 	go_end = go_end_p
						# 	dummy_d2 = go_end - np.array([Tbug.robot_pos[0],Tbug.robot_pos[1]])
						# 	dummy_angle = atan2(dummy_d2[1], dummy_d2[0])
						# 	dummy_d2 = sqrt(np.sum(dummy_d2**2))
						# 	do_min, idx_do_min = Tbug.min_dist()
						# 	t = rospy.get_time() - t_init
						# 	rx, ry = Tbug.find_curve(go_end[0],go_end[1],t)
						# 	# rx, ry = go_end[0], go_end[1]
						# 	print(rx, ry)
						# 	Tbug.follow_target(rx, ry)

					else:
						d_re = d_f
						d_angle = dummy_angle

						while(True):
							print("Following the Wall")
							d, alfa = Tbug.min_dist()
							Tbug.contourn_obst(d,alfa,obst_detec)

							dq = Tbug.distancy()

							if(dq < delta or d_re < d_f):
								break

							do = Tbug.lidar_raw
							dpo = [Tbug.lidar_x, Tbug.lidar_y]  ## dpo[coord][angle] -> dpo[0][10] = pos x referente a leitura de feixe 10 graus

							do_min, idx_do_min = Tbug.min_dist()

							if(do_min > 1.4*obst_detec):
								break

							d_goal = sqrt(np.array(dpo[0][idx_do_min] - px)**2 + np.array(dpo[1][idx_do_min] - py)**2)
							d_re = do[idx_do_min] + d_goal


				else:
					print("Do not have solution! \n")
					break


		rate.sleep()



if __name__ == '__main__':
	try:
		follow()
	except rospy.ROSInterruptException:
		pass