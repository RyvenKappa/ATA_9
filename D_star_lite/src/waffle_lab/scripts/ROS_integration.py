#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np
import math
import time
import matplotlib.pyplot as plt
from sensor_msgs.msg import PointCloud2

########################################################################################################################
x = 0.0 #Posición del robot
y = 0.0
x_pixel_r = 0.0
y_pixel_r = 0.0
theta = 0.0

#Angulo de distancia
angulo_x=0.0
angulo_y=0.0


def callback_laser_original(data):
    global yaw
    global currentx
    global currenty
    global obstacles
    global scale
    global neighbours
    global obst_neighbours

    obst_neighbours = []
    # range_min = data.range_min
    angle_min = data.angle_min
    angle_max = data.angle_max
    angle_increment = data.angle_increment
    ranges = np.array(data.ranges)
    print(ranges)
    print(f"{angle_min},{angle_max},{angle_increment}")

    #Actualizar el grafo de obstaculos

    """ if (min(ranges) <= scale):
        obstacles = [(round(currentx + ranges[i] * 100 * math.cos(i * (math.pi / 180) + yaw)), round(currenty +
                                                                                                     ranges[
                                                                                                         i] * 100 * math.sin(
            i * (math.pi / 180) + yaw))) for i in range(len(ranges)) if ranges[i] <= scale]
        neighbours = np.array(findNeighbours(currenty, currentx))
        obstacles = np.array(obstacles)
        for obstacle in obstacles:
            min_dist = float("inf")
            obst_neighbour = 0
            for neighbour in neighbours:
                dist = (neighbour[0] - obstacle[0]) ** 2 + \
                       (neighbour[1] - obstacle[1]) ** 2
                if (dist < min_dist):
                    min_dist = dist
                    obst_neighbour = neighbour
            obst_neighbours.append((obst_neighbour))
        obst_neighbours = np.array(obst_neighbours)
        obst_neighbours = np.unique(obst_neighbours, axis=0)
        print(obst_neighbours) """

def callback_angle(data):
    global yaw
    global currentx
    global currenty
    pose = data.pose.pose.position
    currentx = round(pose.x*100)
    currenty = round(pose.y*100)
    rot_q = data.pose.pose.orientation
    roll, pitch, yaw = euler_from_quaternion(
        [rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def newOdom(msg):
    global x
    global y
    global theta
    global x_pixel_r
    global y_pixel_r

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    x_pixel_r = int((x)*555/10)
    y_pixel_r = int((x)*505/10)

    quat = msg.pose.pose.orientation
    (_, _, theta) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    


########################################################################################################################
# Class structures
class Node:
    def __init__(self):
        self.is_obstacle = False
        self.parent = None
        self.neighbours = {}
        self.on_path = False
        self.g_cost = 999999  # should be infinite
        self.rhs = 999999  # should be infinite

class Graph:
    subDepth=None
    bg = None
    # graph constructor that creates an empty dictionary
    # nodes = {(x,y):Node} where x,y are coordinates of node
    # open_list = {(x,y): key}
    def __init__(self):
        self.nodes = {}
        self.open_list = {}
        self.obstacle_space = set()
        self.current_path = []
        self.open_length = 0

    def callback_laser(self,data):
        global yaw
        global currentx
        global currenty
        global obstacles
        global scale
        global neighbours
        global obst_neighbours

        obst_neighbours = []
        # range_min = data.range_min
        angle_min = data.angle_min
        angle_max = data.angle_max
        angle_increment = data.angle_increment
        ranges = np.array(data.ranges)
        #print(ranges)
        #print(f"{angle_min},{angle_max},{angle_increment}")
        angulo_actual = 0
        x_relativa = 0
        y_relativa = 0
        count = 0
        for i in ranges:
            if i != float("inf"):
                angulo_actual = count*angle_increment
                x_relativa = math.cos((angulo_actual+theta))*i #Sacando la posición relativa de los puntos respecto al eje principal del mapa
                y_relativa = math.sin((angulo_actual+theta))*i
                #Ahora hace falta meter esto en la lista negra exceptuando que sea un objetivo por el mapa indicado
                #Por ahora vamos a asumir que todo es obstaculo
                x_pixel = int((x + x_relativa)*555/10)
                y_pixel = int((y + y_relativa)*505/10)
                self.obstacle_space.add((x_pixel,y_pixel)) #Lo añadimos a obstaculosç
                Graph.bg[505-y_pixel,555+x_pixel] = (0,0,0)
            #Incrementamos el objeto
            count = count + 1
        



        


    # loop through image and create node object for each pixel
    def create_nodes(self):
        for x in range(-555, 555):
            for y in range(-505, 505):
                y = -y
                if (x,y) not in self.obstacle_space:
                    self.nodes[(x, y)] = Node()

    # for given pixel and find it's neighbours
    def calculate_neighbours(self, curr_node):
        x = curr_node[0]
        y = curr_node[1]
        dig = 1
        strght = 1
        if (x-1,y+1) not in self.obstacle_space and x-1 >= -555 and y+1 < 505:
            if (x-1,y+1) not in self.open_list and self.nodes[(x-1,y+1)].rhs == 999999:
                self.nodes[(x,y)].neighbours[(x-1,y+1)] = dig
        if (x,y+1) not in self.obstacle_space and y+1 < 505:
            if (x,y+1) not in self.open_list and self.nodes[(x,y+1)].rhs == 999999:
                self.nodes[(x,y)].neighbours[(x,y+1)] = strght
        if (x+1,y+1) not in self.obstacle_space and x+1 < 555 and y+1 < 505:
            if (x+1,y+1) not in self.open_list and self.nodes[(x+1,y+1)].rhs == 999999:
                self.nodes[(x,y)].neighbours[(x+1,y+1)] = dig
        if (x-1,y-1) not in self.obstacle_space and x-1 >= -555 and y-1 >= -505:
            if (x-1, y-1) not in self.open_list and self.nodes[(x-1,y-1)].rhs == 999999:
                self.nodes[(x, y)].neighbours[(x-1, y-1)] = dig
        if (x,y-1) not in self.obstacle_space and y-1 >= -505:
            if (x, y-1) not in self.open_list and self.nodes[(x,y-1)].rhs == 999999:
                self.nodes[(x, y)].neighbours[(x, y-1)] = strght
        if (x+1,y-1) not in self.obstacle_space and x+1 < 555 and y-1 >= -505:
            if (x+1,y-1) not in self.open_list and self.nodes[(x+1,y-1)].rhs == 999999:
                self.nodes[(x, y)].neighbours[(x+1, y-1)] = dig
        if (x-1,y) not in self.obstacle_space and x-1 >= -555:
            if (x-1,y) not in self.open_list and self.nodes[(x-1,y)].rhs == 999999:
                self.nodes[(x, y)].neighbours[(x-1,y)] = strght
        if (x+1,y) not in self.obstacle_space and x+1 < 555:
            if (x+1,y) not in self.open_list and self.nodes[(x+1,y)].rhs == 999999:
                self.nodes[(x,y)].neighbours[(x+1,y)] = strght
    
    # Check if node is consistent
    def node_is_consistent(self,node):
        if self.nodes[node].g_cost == self.nodes[node].rhs:
            return True
        else:
            return False
    # Get heuristic distance
    def h(self,node,start):
        return np.sqrt((node[0] - start[0])**2 + (node[1] - start[1])**2)

    # Get key for sorting the open_list:
    def get_key(self,node, start):
        key = min(self.nodes[node].g_cost, self.nodes[node].rhs) + 0.9 * self.h(node,start)
        return key
    # Get smallest element from the open_list:
    def get_smallest(self,open_list):
        smallest = 9999999
        smallest_node = (-9999,-9999)
        for key, value in open_list.items():
            if open_list[key] < smallest:
                smallest = value
                smallest_node = key
        return smallest_node
    # D* Lite algorithm to find the shortest path
    def d_star_lite_algo(self, rob_x, rob_y, goal_x, goal_y,bg):
        bg[505 - rob_y, rob_x + 555] = (250,11,156)
        bg[505 - goal_y, goal_x + 555] = (250, 11, 156)
        ## Color declarations
        # define color for node in open list
        green = (60, 179, 113)
        # define color for the current node
        red = (0, 0, 250)
        # get coordinates for the start node
        start_node = (rob_x, rob_y)
        # get coordinates for the goal node
        goal_node = (goal_x, goal_y)
        # make cost of start node zero
        self.nodes[goal_node].rhs = 0
        self.open_list[goal_node] = 0  # key needs to be written here
        self.open_length += 1
        curr_node = goal_node
        while not curr_node == start_node and not len(self.open_list) == 0:
            #print("Curr",curr_node)
            bg[505 - curr_node[1], curr_node[0] + 555] = red
            # make g_cost = rhs
            self.nodes[curr_node].g_cost = self.nodes[curr_node].rhs
            # remove curr_node from the open list
            del self.open_list[curr_node]
            self.open_length -= 1
            # get successors of the curr_node
            self.calculate_neighbours(curr_node)
            for n in self.nodes[curr_node].neighbours:
                bg[505 - n[1], n[0] + 555] = green
                self.nodes[n].parent = curr_node
                # rhs of successor = g of parent + path cost
                self.nodes[n].rhs = self.nodes[curr_node].g_cost + self.nodes[curr_node].neighbours[n]
                if not self.node_is_consistent(n):
                    self.open_list[n] = self.get_key(n, start_node)
                    self.open_length += 1
            curr_node = self.get_smallest(self.open_list)
        current_path = []
        while not self.nodes[curr_node].parent == None:
            bg[505 - curr_node[1], curr_node[0] + 555] = (250, 0, 0)
            current_path.append(curr_node)
            curr_node = self.nodes[curr_node].parent
        current_path.append(curr_node)
        cv2.imshow("D star lite output", bg)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        print("Path length: ",len(current_path))
        return current_path

    # new neighbours
    def new_calculate_neighbours(self, curr_node, open_list, current_path):
        x = curr_node[0]
        y = curr_node[1]
        dig = 1.41
        strght = 1
        if (x-1,y+1) not in self.obstacle_space and x-1 >= -555 and y+1 < 505:
            if (x-1,y+1) not in open_list and not self.nodes[(x,y)].parent == (x-1,y+1) and (x-1,y+1) not in current_path:
                self.nodes[(x,y)].neighbours[(x-1,y+1)] = dig
        if (x,y+1) not in self.obstacle_space and y+1 < 505:
            if (x,y+1) not in open_list and not self.nodes[(x,y)].parent == (x,y+1) and (x,y+1) not in current_path:
                self.nodes[(x,y)].neighbours[(x,y+1)] = strght
        if (x+1,y+1) not in self.obstacle_space and x+1 < 555 and y+1 < 505:
            if (x+1,y+1) not in open_list and not self.nodes[(x,y)].parent == (x+1,y+1) and (x+1,y+1) not in current_path:
                self.nodes[(x,y)].neighbours[(x+1,y+1)] = dig
        if (x-1,y-1) not in self.obstacle_space and x-1 >= -555 and y-1 >= -505:
            if (x-1, y-1) not in open_list and not self.nodes[(x,y)].parent == (x-1,y-1) and (x-1,y-1) not in current_path:
                self.nodes[(x, y)].neighbours[(x-1, y-1)] = dig
        if (x,y-1) not in self.obstacle_space and y-1 >= -505:
            if (x, y-1) not in open_list and not self.nodes[(x,y)].parent == (x,y-1) and (x,y-1) not in current_path:
                self.nodes[(x, y)].neighbours[(x, y-1)] = strght
        if (x+1,y-1) not in self.obstacle_space and x+1 < 555 and y-1 >= -505:
            if (x+1,y-1) not in open_list and not self.nodes[(x,y)].parent == (x+1,y-1) and (x+1,y-1) not in current_path:
                self.nodes[(x, y)].neighbours[(x+1, y-1)] = dig
        if (x-1,y) not in self.obstacle_space and x-1 >= -555:
            if (x-1,y) not in open_list and not self.nodes[(x,y)].parent == (x-1,y) and (x-1,y) not in current_path:
                self.nodes[(x, y)].neighbours[(x-1,y)] = strght
        if (x+1,y) not in self.obstacle_space and x+1 < 555:
            if (x+1,y) not in open_list and not self.nodes[(x,y)].parent == (x+1,y) and (x+1,y) not in current_path:
                self.nodes[(x,y)].neighbours[(x+1,y)] = strght
    # get new key for replanning
    def get_new_key(self, node, start):
        key = min(self.nodes[node].g_cost, self.nodes[node].rhs) + self.h(node, start)
        return key

    # replanning the path from current start point
    def replan(self,rob_x, rob_y, goal_x, goal_y,current_path,bg):
        # get coordinates for the start node
        start_node = (rob_x, rob_y)
        # get coordinates for the goal node
        goal_node = (goal_x, goal_y)
        new_open_list = {}
        visited = []
        # make cost of start node zero
        self.nodes[goal_node].rhs = 0
        new_open_list[goal_node] = 0  # key needs to be written here
        curr_node = goal_node
        parent = self.nodes[goal_node].parent
        while not parent == None:
            visited.append(parent)
            parent = self.nodes[parent].parent
        while not curr_node == start_node and not len(new_open_list) == 0:
            visited.append(curr_node)
            # make g_cost = rhs
            self.nodes[curr_node].g_cost = self.nodes[curr_node].rhs
            # first make all neighbour rhs = infinitys
            # Now recalculate for new path
            self.nodes[curr_node].neighbours = {}
            self.new_calculate_neighbours(curr_node, new_open_list, visited)
            for n in self.nodes[curr_node].neighbours:
                if n in visited:
                    continue
                self.nodes[n].parent = curr_node
                # rhs of successor = g of parent + path cost
                self.nodes[n].rhs = self.nodes[curr_node].g_cost + self.nodes[curr_node].neighbours[n]
                if not self.node_is_consistent(n):
                    new_open_list[n] = self.get_new_key(n, start_node)
            # remove curr_node from the open list
            del new_open_list[curr_node]
            curr_node = self.get_smallest(new_open_list)
        new_path = []
        count = 0
        print(f"Nodo actual: {curr_node}")
        while not self.nodes[curr_node].parent == None:
            #while not curr_node == goal_node:
            Graph.bg[505 - curr_node[1], curr_node[0] + 555] = (250, 0, 0)
            new_path.append(curr_node)
            curr_node = self.nodes[curr_node].parent
            count += 1
        new_path.append(curr_node)
        return new_path

    # Travel across the received path
    def traverse(self,bg,current_path,rob_x,rob_y):
        global x_pixel_r
        global y_pixel_r
        #self.obstacle_space.add((18, 0))
        #bg[505 - 18, 0 + 555] = (0, 0, 0)
        Graph.bg = bg
        rospy.init_node('speed_controller')
        sub = rospy.Subscriber('/odom', Odometry, newOdom)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        subDept = rospy.Subscriber("/scan",LaserScan,self.callback_laser)
        rate = rospy.Rate(4)
        new_path = []
        for i in current_path:
            goal = Point()
            goal.x = i[0] / 100.0
            print(goal.x)
            goal.y = i[1] / 100.0
            print(goal.y)
            new_path.append(goal)
        print(new_path)

        vel = Twist()
        
        while not rospy.is_shutdown():
            i = 0
            for g in new_path:
                p = current_path[i]
                i += 1
                Graph.bg[505 - p[1], p[0] + 555] = (255, 255, 255)
                print("P:",p)
                #Esta función hace llamada recursiva para recalcular ruta
                if self.nodes[p].parent in self.obstacle_space:
                    print("Obstacle found at ", self.nodes[p].parent)
                    parent = self.nodes[p].parent
                    # get next available parent on path
                    while parent in self.obstacle_space:
                        print("Obstacle found at ", parent)
                        parent = self.nodes[parent].parent
                    new_path = self.replan(p[0], p[1], rob_x, rob_y, current_path, Graph.bg)
                    self.traverse(Graph.bg, new_path,150, 0)
                cv2.imshow("Output", Graph.bg)
                cv2.waitKey(1)
                del_x = g.x - x
                del_y = g.y - y
                while np.sqrt((del_x) ** 2 + (del_y) ** 2) > 0.03:
                    del_x = g.x - x
                    del_y = g.y - y

                    angle_to_goal = math.atan2(del_y, del_x)

                    if (angle_to_goal - theta) > 0.2:
                        vel.linear.x = 0.0
                        vel.angular.z = 0.1
                    elif (theta - angle_to_goal) > 0.2:
                        vel.linear.x = 0.0
                        vel.angular.z = -0.1
                    else:
                        vel.linear.x = 0.05
                        vel.angular.z = 0.0

                    pub.publish(vel)
                    rate.sleep()

                vel.linear.x = 0.0
                vel.angular.z = 0.0
                pub.publish(vel)
                rospy.sleep(0.01)
            cv2.destroyAllWindows()
            while not rospy.is_shutdown():
                vel.linear.x = 0.0
                vel.angular.z = 0.0
                pub.publish(vel)
                rate.sleep()

        return
########################################################################################################################
def initiate():
    # scale the window size
    height = 1010
    width = 1110
    # create a background image
    bg = np.zeros((height,width,3),dtype=np.uint8)
    # Calculate Mikowski space
    r = 25
    graph = Graph()
    """
        Define obstaculos manualmente
    """
    # iterate for each pixel and find out if it is an obstacle
    # if it is in the obstacle store it in the obstacle set
    #for x in range(-555, 555): #Relacionado con la resolución
    #    for y in range(-505, 505): #Relacionado con la resolución
    #        y = -y
    #        if is_on_obstacle(x, y, r):
    #            graph.obstacle_space.add((x, y))


    graph.create_nodes()
    for node in graph.nodes:
        x = node[0]
        y = node[1]
        # y = -y
        bg[505 - y, x + 555] = (192, 192, 192) #color gris para nodos no visitados
    x_r = 0 #Antes era -300, siendo el inicio de robot malo
    y_r = 0
    x_g = 200
    y_g = 0
    #global x
    #global y
    x = x_r
    y = y_r
    print("Goal",x_g,y_g)
    current_path = graph.d_star_lite_algo(x_r,y_r,x_g,y_g,bg)
    graph.traverse(bg,current_path,150,0)
if __name__ == '__main__':
	try:
		initiate()
	except rospy.ROSInterruptException:
		pass
