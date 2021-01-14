#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import cos, sin, pi

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
    laser = data.ranges
    i = 1
    while i < 269:
        # print i, laser [i+1], len(laser)
        if (laser[i] < laser [i-1]) and (laser[i] < laser[i+1]):
            print "Posicao", i, "-", laser[i], ": sou um minimo local"
        i += 1


# Sinais de controle
def control(pos, vel):
    """
    Entradas:
    pos: Vetor contendo a posicao planar do robo
    vel: Vetor contendo a velocidade de referencia a ser seguida

    Retorno:
    Ux: Velocidade no eixo x do sistema de coordenadas inercial
    Uy: Velocidade no eixo y do sistema de coordenadas inercial
    """

    global x_n, y_n
    k = 1

    Ux = k * (pos[0] - x_n) + vel[0]
    Uy = k * (pos[1] - y_n) + vel[1]

    # print "Control: ", round(Ux, 2), round(Uy, 2)

    return Ux, Uy


def feedback_linearization(Ux, Uy):
    """
    Entradas: 
    Ux: Velocidade no eixo x do sistema de coordenadas inercial
    Uy: Velocidade no eixo y do sistema de coordenadas inercial

    Retorno: 
    Vx: Velocidade no eixo x do robo 
    w: Velocidade angular do robo
    """

    global theta_n, d

    vx = cos(theta_n) * Ux + sin(theta_n) * Uy
    w = -(sin(theta_n) * Ux)/ d + (cos(theta_n) * Uy) / d

    # print "Lin: ", round(vx, 2), round(w, 2)

    return vx, w


def controller():
    """
    Rotina primaria
    """
    global t, x_n, y_n, theta_n
    global rx, ry, cx, cy
    global freq

    x_n = 0.0
    y_n = 0.0
    theta_n = 0.0

    rospy.init_node("vector_field", anonymous=True)

    # Topico onde sera publicada a velocidade do robo
    pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    
    # Topico contendo a pose do robo
    rospy.Subscriber('/odom', Odometry, callback_pose)

    # Topico contendo a leitura do lase
    rospy.Subscriber('/base_scan', LaserScan, callback_laser)

    vel_msg = Twist()

    freq = 20
    
    rate = rospy.Rate(freq)

    rospy.sleep(0.1)

    t_init = rospy.get_time()

    t = 0

    while not rospy.is_shutdown():
        # t = rospy.get_time() - t_init

        # p_curve, v_curve = parametric_curve()
        # Ux, Uy = control(p_curve, v_curve)
        # vel_msg.linear.x, vel_msg.angular.z = feedback_linearization(Ux, Uy)

        # pub_cmd_vel.publish(vel_msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
