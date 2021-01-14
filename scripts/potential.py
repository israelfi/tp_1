#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import cos, sin, pi, sqrt, atan
import numpy as np

d = 0.2

def callback_pose(data):
    """
    Rotina callback para a obtencao da pose do robo
    """
    global x_n, y_n, theta_n

    x_n = data.pose.pose.position.x  # posicao 'x' do robo no mundo 
    y_n = data.pose.pose.position.y  # posicao 'y' do robo no mundo 

    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])

    theta_n = euler[2]
            
    return


def callback_laser(data):
    global laser, l_range_max, l_range_min
    laser = data.ranges
    l_range_max = data.range_max
    l_range_min = data.range_min



def min_dist(laser):
    global x_n,y_n,theta_n
    d_min = laser[0]
    alfa = 0
    for i in range(len(laser)):
        if(laser[i] < d_min):
            d_min = laser[i]
            alfa = i - 180

    sx = (cos(theta_n)*(d_min*cos(np.deg2rad(alfa))) + sin(theta_n)*(d_min*sin(np.deg2rad(alfa)))) + x_n
    sy = (-sin(theta_n)*(d_min*cos(np.deg2rad(alfa))) + cos(theta_n)*(d_min*sin(np.deg2rad(alfa)))) + y_n

    # sx = d_min*cos(np.deg2rad(alfa))
    # sy = d_min*sin(np.deg2rad(alfa))
    obs_pos = [sx, sy]

    # print("Pos = [%f, %f]" % (sx,sy))

    return d_min, alfa, obs_pos


def feedback_linearization(Ux, Uy):

    global theta_n, d

    vx = cos(theta_n) * Ux + sin(theta_n) * Uy
    w = -(sin(theta_n) * Ux)/ d + (cos(theta_n) * Uy) / d

    # print "Lin: ", round(vx, 2), round(w, 2)

    return vx, w



def pot_att(x,y,px,py):
    D = sqrt((px-x)**2 + (py-y)**2)
    K = 0.1
    D_safe = 100

    if(D > D_safe):
        Ux = - D_safe*K*(x - px)/D
        Uy = - D_safe*K*(y - py)/D
        U_a = [Ux, Uy]
    else:
        Ux = - K*(x - px)
        Uy = - K*(y - py)
        U_a = [Ux, Uy]

    return U_a

def pot_rep(x, y, D, alfa, pos):
    global theta_n
    K = 4.0
    D_safe = 5.0

    print("OBST Dist = %f ; Pos_obs = [%f, %f] ; Pos_r = [%f, %f]" % (D,pos[0],pos[1], x, y))

    if( D > D_safe):
        Ux = 0
        Uy = 0
        U_r = [Ux, Uy]
    else:
        Ux = K*((1.0/D) - 1.0/D_safe) * (x - pos[0])/D
        Uy = K*((1.0/D) - 1.0/D_safe) * (y - pos[1])/D
        U_r = [Ux, Uy]

    return U_r


def controller():
    """
    Rotina primaria
    """
    global t, x_n, y_n, theta_n
    global rx, ry, cx, cy
    global freq
    global laser, l_range_max, l_range_min

    x_n = 0.0
    y_n = 0.0
    theta_n = 0.0
    laser = []

    rospy.init_node("vector_field", anonymous=True)

    # Topico onde sera publicada a velocidade do robo
    pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    
    # Topico contendo a pose do robo
    rospy.Subscriber('/base_pose_ground_truth', Odometry, callback_pose)

    # Topico contendo a leitura do lase
    rospy.Subscriber('/base_scan', LaserScan, callback_laser)

    vel_msg = Twist()

    freq = 20
    
    rate = rospy.Rate(freq)

    rospy.sleep(0.2)

    px, py, = raw_input('Insira o valor do destino em x e y (valores separados por espaco, considere o tamanho do mapa 30x30): ').split()
    px, py = [float(i) for i in [px, py]]


    while not rospy.is_shutdown():

        if(laser):
            D, alfa, obs_pos = min_dist(laser)
            if(D<l_range_max and D>l_range_min):
                U_r = pot_rep(x_n, y_n, D, alfa, obs_pos)
            else:
                U_r = [0, 0]

            U_a = pot_att(x_n, y_n, px, py)


            Ux = U_a[0] + U_r[0]
            Uy = U_a[1] + U_r[1]
            vel_msg.linear.x, vel_msg.angular.z = feedback_linearization(Ux, Uy)

            pub_cmd_vel.publish(vel_msg)

            print("Attractive = [%f, %f]\n Repulsive = [%f, %f]\n Complete = [%f, %f]\n\n\n" % (U_a[0], U_a[1], U_r[0], U_r[1], Ux, Uy))

            if( (sqrt((px-x_n)**2 + (py-y_n)**2)) < 0.2 ):
                print("Alvo alcancado!\n")
                px, py, = raw_input('Insira o valor do proximo destino em x e y (valores separados por espaco, considere o tamanho do mapa 30x30): ').split()
                px, py = [float(i) for i in [px, py]]



        rate.sleep()


if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
