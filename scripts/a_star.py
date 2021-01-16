#!/usr/bin/env python
import rospy
import rospkg
import os
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseArray, Twist
from tf.transformations import euler_from_quaternion
import matplotlib.image as img
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import math


## A class for the Nodes in the Grid
class Node():
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


## Compute a list of nodes of a path from the given start to the given target
def Astar(maze, start, target):
    

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, target)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Remove current from open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)

    return open_list




class control:
    def __init__(self):
        self.d = 0.2
        self.k = 1


    def pot_att(self,x,y,px,py):
        D = math.sqrt((px-x)**2 + (py-y)**2)
        K = 2
        D_safe = 10

        if(D > D_safe):
            Ux = - D_safe*K*(x - px)/D
            Uy = - D_safe*K*(y - py)/D
            U_a = [Ux, Uy]
        else:
            Ux = - K*(x - px)
            Uy = - K*(y - py)
            U_a = [Ux, Uy]

        return U_a

    def feedback_linearization(self,Ux, Uy, theta_n):

        vx = math.cos(theta_n) * Ux + math.sin(theta_n) * Uy
        w = -(math.sin(theta_n) * Ux)/ self.d  + (math.cos(theta_n) * Uy) / self.d 

        return vx, w





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




### MAIN CODE
def planning():
   
    ## Create a grid from image mape
    rospack = rospkg.RosPack()
    path = rospack.get_path('tp_1')
    image_path = path + '/worlds/map2.bmp'
    image = img.imread(image_path)
    image.setflags(write=1)

    # white = np.ones((100, 100), dtype=np.float)

    r_color = [255, 0, 0]
    g_color = [0, 255, 0]
    b_color = [0, 0, 255]

    M = np.zeros((len(image),len(image)))

    for i in range(len(image)):
        for j in range(len(image)):
            if(image[i,j,0] == 255 and image[i,j,1] == 255 and image[i,j,2] == 255):
                # white[i,j]= 1
                M[i,j] = 0
            else:
                # white[i,j] = 0
                M[i,j] = 1

    # plt.imshow(white, cmap='gray',vmin=0,vmax=1)
    # plt.show()



    ## ROS STUFFS
    rospy.init_node("a_star", anonymous=True)

    # Topico onde sera publicada a velocidade do robo
    pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    # Topico contendo a pose do robo
    rospy.Subscriber('/base_pose_ground_truth', Odometry, callback_pose)

    vel_msg = Twist()

    rate = rospy.Rate(20)

    ## Other Variables
    controlador = control()
    global x_n, y_n, theta_n

    x_n = 0.0
    y_n = 0.0
    theta_n = 0.0


    flag_first = True

    # Main Loop
    while not rospy.is_shutdown():
        flag_first = True
        image = img.imread(image_path)
        image.setflags(write=1)
        px, py, = raw_input('Insira o valor do destino em x e y (valores separados por espaco, considere o tamanho do mapa -50x50): ').split()
        px, py = [float(i) for i in [px, py]]

        resolution_map = 1     # cada 1 pixel, equivale a 1 m
        x_desloc = 50
        y_desloc = 50

        x_start = 0
        y_start = 0
        x_target = px
        y_target = py
        start = (int(round((y_start*resolution_map)+y_desloc)), int(round((x_start*resolution_map)+x_desloc)))
        target = (int(round((-y_target*resolution_map)+y_desloc)), int(round((x_target*resolution_map)+x_desloc)))

        print(target)


        if(M[target[0],target[1]] == 1):
            print("This point can not be reached")
        else:
            path = Astar(M, start, target)
            vec_path = np.zeros((len(path),2))
            for i in range(len(path)):
                s = list(path[i])
                image[s[0],s[1]] = g_color
                vec_path[i,:] = list(path[i])
                vec_path[i,0] = vec_path[i,0] - x_desloc
                vec_path[i,1] = vec_path[i,1] - y_desloc

            t_x = []
            t_y = []
            t_x = vec_path[:,1]
            t_y = -vec_path[:,0]


            if (flag_first):
                flag_first = False
                image[start[0],start[1]] = r_color
                image[target[0],target[1]] = b_color
                pic = Image.fromarray(image, 'RGB')
                pic.show()

            
            # Controle
            for i in range(len(t_x)):
                t_init = rospy.get_time()
                D = 1000
                while(D > 0.2):
                    D = math.sqrt((t_y[i]-y_n)**2+(t_x[i]-x_n)**2)
                    t = rospy.get_time() - t_init
                    # dx,dy = controlador.find_curve(x_n, y_n, t_x, t_y, t)
                    print("Robot Pos = [%f, %f]\n Target Pos = [%f, %f]\n Distancy = %f\n\n" % (x_n,y_n,t_x[i],t_y[i],D))
                    U_a = []
                    U_a = controlador.pot_att(x_n,y_n,t_x[i],t_y[i])
                    vel_msg.linear.x, vel_msg.angular.z = controlador.feedback_linearization(U_a[0],U_a[1],theta_n)
                    pub_cmd_vel.publish(vel_msg)

            



        rate.sleep()



### Main code - chama o codigo de controle
if __name__ == '__main__':
    try:
        planning()
    except rospy.ROSInterruptException:
        pass
