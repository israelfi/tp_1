#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import cos, sin, pi

d = 0.2
samples = 270

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
    global laser, intensities
    
    laser = data.ranges
    intensities = data.intensities

    return


def potential():
    global laser, intensities
    global x_n, y_n, theta_n

    alfa = 5.0
    i = 1
    angulo_grau = 0
    grad_obs = [0, 0] # Gradiente (x, y)
    
    # Posicao do Obstaculo
    pos_obs = [0, 0]

    while i < 269:
        if (laser[i] < laser [i-1]) and (laser[i] < laser[i+1]) and (intensities[i] == 1.0):
            angulo_grau = i - samples/2
            angulo_rad = angulo_grau * pi / 180.

            grad_obs[0] += - cos(angulo_rad + theta_n) / (laser[i] ** 2)
            grad_obs[1] += - sin(angulo_rad + theta_n) / (laser[i] ** 2)

            pos_obs[0] = [x_n + laser[i]*cos(angulo_rad + theta_n)]
            pos_obs[1] = [y_n + laser[i]*sin(angulo_rad + theta_n)]

        i += 1

    grad_obs[0] = alfa * grad_obs[0]
    grad_obs[1] = alfa * grad_obs[1]

    # print grad_obs

    return grad_obs

    


# Sinais de controle
def control(pos):
    """
    Entradas:
    pos: Vetor contendo a posicao planar do robo
    vel: Vetor contendo a velocidade de referencia a ser seguida

    Retorno:
    Ux: Velocidade no eixo x do sistema de coordenadas inercial
    Uy: Velocidade no eixo y do sistema de coordenadas inercial
    """

    global x_n, y_n
    global x_goal, y_goal

    k = 1.

    Ux = k * (x_goal + pos[0] - x_n) 
    Uy = k * (y_goal + pos[1] - y_n)

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
    global x_goal, y_goal
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

    x_goal, y_goal, = raw_input('Insira as coordenadas de destino): ').split()
    x_goal, y_goal = [float(i) for i in [x_goal, y_goal]]

    t = 0

    while not rospy.is_shutdown():
        t = rospy.get_time() - t_init

        pos_potential = potential()
        Ux, Uy = control(pos_potential)
        vel_msg.linear.x, vel_msg.angular.z = feedback_linearization(Ux, Uy)

        pub_cmd_vel.publish(vel_msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
