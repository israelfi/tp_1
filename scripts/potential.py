#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import cos, sin, pi, sqrt, atan, atan2
import numpy as np



d = 0.2

##    Rotina callback para a obtencao da pose do robo
def callback_pose(data):
    global x_n, y_n, theta_n

    x_n = data.pose.pose.position.x  # posicao 'x' do robo no mundo 
    y_n = data.pose.pose.position.y  # posicao 'y' do robo no mundo 

    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])

    theta_n = euler[2]  # orientacao do robo no mundo 
            
    return


##    Rotina callback para a obtencao dos dados do Lidar
def callback_laser(data):
    global laser, l_range_max, l_range_min, lp, pc2_msg
    laser = data.ranges                     # Distancias detectadas
    l_range_max = data.range_max          # range max do lidar
    l_range_min = data.range_min          # range min do lidar



## Calcula a menor distancia detectada pelo lidar (obstaculo mais proximo)
## retornar a menor distancia, o angulo de deteccao e a posicao x,y do obstaculo
def min_dist(laser):
    global x_n,y_n,theta_n
    d_min = laser[0]
    alfa = 0
    for i in range(len(laser)):
        if(laser[i] < d_min):
            d_min = laser[i]
            alfa = i

    # sx = d_min*cos(np.deg2rad(alfa))
    # sy = d_min*sin(np.deg2rad(alfa))
    sx = (cos(theta_n)*(d_min*cos(np.deg2rad(alfa - 180))) + sin(theta_n)*(d_min*sin(np.deg2rad(alfa - 180)))) + x_n
    sy = (-sin(theta_n)*(d_min*cos(np.deg2rad(alfa - 180))) + cos(theta_n)*(d_min*sin(np.deg2rad(alfa - 180)))) + y_n    

    obs_pos = [sx, sy]
    return d_min, alfa, obs_pos



## Feedback Linearization - calcula lei de controle v e omega para o robo
## Entra com Vx e Vy no mundo e determinar v e omega
def feedback_linearization(Ux, Uy):

    global theta_n, d

    vx = cos(theta_n) * Ux + sin(theta_n) * Uy
    w = -(sin(theta_n) * Ux)/ d + (cos(theta_n) * Uy) / d

    return vx, w


## Calcula o gradiente da func potencial de atracao para o alvo
## Determinar as componentes Vx e Vy
def pot_att(x,y,px,py):
    D = sqrt((px-x)**2 + (py-y)**2)
    K = 0.2
    D_safe = 10.0

    if(D > D_safe):
        Ux = - D_safe*K*(x - px)/D
        Uy = - D_safe*K*(y - py)/D
        U_a = [Ux, Uy]
    else:
        Ux = - K*(x - px)
        Uy = - K*(y - py)
        U_a = [Ux, Uy]

    return U_a


## Calcula o gradiente da func potencial de repulsao do obstaculo detectado
## Determinar as componentes Vx e Vy
def pot_rep(x, y, D, alfa, pos):
    global theta_n
    K = 2.0
    D_safe = 4.0

    print("Obst Dist = %f ; Pos_obst = [%f, %f] ; Pos_robot = [%f, %f]" % (D,pos[0],pos[1], x, y))

    if( D > D_safe):
        Ux = 0
        Uy = 0
        U_r = [Ux, Uy]
    else:
        # Ux = - K*(D - D_safe)*cos(alfa*pi/180 + theta_n)
        # Uy = - K*(D - D_safe)*sin(alfa*pi/180 + theta_n)
        grad_x = - cos(alfa*pi/180 + theta_n)
        grad_y = - sin(alfa*pi/180 + theta_n)
        Ux = K * (1.0/D_safe - 1.0/D) * (1.0/D**2) * grad_x 
        Uy = K * (1.0/D_safe - 1.0/D) * (1.0/D**2) * grad_y

        U_r = [Ux, Uy]

    return U_r



## Codigo de Controle
## determina os valores Vx e Vy com base nas func potenciais (atracao e repulsao)
## utiliza o feedback linearization p determinar v e omega para o robo se mover ate o alvo, desviando de obstaculos
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

    # Topico para o ponto mais proximo
    pub_obst = rospy.Publisher("/closet", PointStamped, queue_size=1)
    
    # Topico contendo a pose do robo
    rospy.Subscriber('/base_pose_ground_truth', Odometry, callback_pose)
    # rospy.Subscriber('/odom', Odometry, callback_pose)

    # Topico contendo a leitura do lase
    rospy.Subscriber('/base_scan', LaserScan, callback_laser)

    vel_msg = Twist()
    pos_obst = PointStamped()

    freq = 20
    
    rate = rospy.Rate(freq)

    rospy.sleep(0.2)

    px, py, = raw_input('Insira o valor do destino em x e y (valores separados por espaco, considere o tamanho do mapa 30x30): ').split()
    px, py = [float(i) for i in [px, py]]


    while not rospy.is_shutdown():

        if(laser):
            D, alfa, obs_pos = min_dist(laser)
            pos_obst.header.frame_id = 'base_link'
            pos_obst.point.x = obs_pos[0]
            pos_obst.point.y = obs_pos[1]
            pub_obst.publish(pos_obst)

            U_r = pot_rep(x_n, y_n, D, alfa, obs_pos)
            # if (U_r[0] > 0 or U_r[1] > 0):
            #     U_a = [0, 0]
            # else:
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



### Main code - chama o codigo de controle
if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass

