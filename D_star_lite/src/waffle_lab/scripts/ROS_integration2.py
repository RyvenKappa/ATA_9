#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np
import math
import time
import matplotlib.pyplot as plt
#D*Lite Diego Aceituno Seoane
from d_star_lite import initDStarLite, moveAndRescan
from utils import stateNameToCoords
from grid import GridWorld

########################################################################################################################
x = 0.0 #Posición del robot
y = 0.0
theta = 0.0

#Angulo de distancia
angulo_x=0.0
angulo_y=0.0

def newOdom(msg):
    global x
    global y
    global theta
    global x_pixel_r
    global y_pixel_r

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    quat = msg.pose.pose.orientation
    (_, _, theta) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

def algorithm_to_real(position:str)->tuple:
    #Sacamos los pixeles
    x = int(position[1:position.rfind("y")])
    y = int(position[position.rfind("y")+1:])
    """
    Estas posiciones se refieren desde la esquina izquierda arriba, siendo ese nodo 0,0
    Vamos a transformar estas posiciones para que respeten el centro
    """
    x -=50 #Cambiar según tamaño del grid 100  es 50
    y -=50 #Cambiar según tamaño del grid 100  es 50
    #Los convertimos a coordenadas reales
    x = x*20/100 #Cambiar según tamaño del grid 100  es 50
    y = y*20/100 #Cambiar según tamaño del grid 100  es 50
    return (x,-y)

def real_to_algortihm(position:tuple)->str:
    #Del tipo (x,y)
    x = position[0]
    y = position[1]
    return(f"x{x+50}y{50-y}") #Cambiar según tamaño del grid 100  es 50

########################################################################################################################
# Class structures

class MoveManager:
    subDepth=None
    def __init__(self) -> None:
        MoveManager.graph=GridWorld(100,100) #Cambiar según tamaño del grid
    def callback_laser(self,data):
        global yaw
        global currentx
        global currenty
        global obstacles
        global scale
        global neighbours
        global obst_neighbours

        obst_neighbours = []
        angle_increment = data.angle_increment
        ranges = np.array(data.ranges)
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
                x_pixel = round((x + x_relativa)*50/10) #Cambiar según tamaño del grid 100  es 50
                y_pixel = round((y + y_relativa)*50/10) #Cambiar según tamaño del grid 100  es 50
                position:str = real_to_algortihm((x_pixel,y_pixel))
                row = int(position[position.rfind("y")+1:])
                column = int(position[1:position.rfind("y")])
                #print(f"{column}  {row}")
                #Anadimos el obstaculo con un reborde extra de 2
                for i in range(row-1,row+1):#y
                    for j in range(column-1,column+1):#x
                        if (MoveManager.graph.cells[i][j] == 0):
                            MoveManager.graph.cells[i][j] = -1
            #Incrementamos el objeto
            count = count + 1

    # Travel across the received path
    def traverse(self,start,goal,altura,ancho):
        #Comunicación con ROS
        rospy.init_node('speed_controller')
        sub = rospy.Subscriber('/odom', Odometry, newOdom)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        subDept = rospy.Subscriber("/scan",LaserScan,self.callback_laser)
        rate = rospy.Rate(4)
        vel = Twist()

        MoveManager.graph = GridWorld(ancho,altura)
        #Automatizar las coordenadas
        s_start = real_to_algortihm(start)
        s_goal = real_to_algortihm(goal)
        MoveManager.graph.setStart(s_start)
        MoveManager.graph.setGoal(s_goal)
        k_m = 0
        s_last = s_start
        queue = []
        move = False
        MoveManager.graph, queue, k_m = initDStarLite(MoveManager.graph, queue, s_start, s_goal, k_m)
        s_current = s_start
        done = False
        iteration_time_start = 0
        while not done:
            iteration_time_start = time.time()
            s_new,k_m = moveAndRescan(MoveManager.graph,queue,s_current,5,k_m)#3 es El rango
            print(time.time()-iteration_time_start)
            print(f"{s_new} nueva posición")
            #Nos movemos a s_new, que es del tipo "xnumeroynumero"
            #Necesitamos una transformación
            plt.plot(MoveManager.graph.cells)
            if str(s_new)=="goal":
                s_new=s_goal
                break
            else:
                next = algorithm_to_real(s_new)
            last = algorithm_to_real(s_current)
            del_x = next[0] - x
            del_y = next[1] - y
            angle_to_goal = math.atan2(del_y, del_x)
            while np.sqrt((del_x) ** 2 + (del_y) ** 2) >0.03:
                del_x = next[0] - x
                del_y = next[1] - y
                angle_to_goal = math.atan2(del_y, del_x)
                turn = math.atan2(math.sin(angle_to_goal-theta),math.cos(angle_to_goal-theta))
                if abs(angle_to_goal-theta ) < 0.2:
                    vel.linear.x = 0.1 #hacia delante
                    vel.angular.z = 0.0 #Girar
                else:
                    vel.angular.z = 0.2 * (turn/abs(turn))
                pub.publish(vel)
                rate.sleep()
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            pub.publish(vel)
            rospy.sleep(0.01)
            if str(s_new) == 'goal':
                print('Goal Reached!')
                done = True
            else:
                s_current = s_new
        return
    

########################################################################################################################
def initiate():
    global x
    global y
    # scale the window size
    height = 100
    width = 100
    inicio =(0,0)
    goal = (-10,13) # Cerveza
    manager = MoveManager()
    manager.traverse(inicio,goal,height,width)

    inicio = (-10,13)
    goal = (-22,-3) # Coca Cola1
    manager.traverse(inicio,goal,height,width)

    inicio = (-22,-3)
    goal = (-10,-19) # CocaCola 2
    manager.traverse(inicio,goal,height,width)
"""
    inicio = (-10,-19)
    goal = (19,7) # Coca Cola 3
    manager.traverse(inicio,goal,height,width)  

    inicio = (19,7)
    goal = (36,-6) # Coca Cola 4
    manager.traverse(inicio,goal,height,width) 
"""


    
if __name__ == '__main__':
	try:
		initiate()
	except rospy.ROSInterruptException:
		pass
