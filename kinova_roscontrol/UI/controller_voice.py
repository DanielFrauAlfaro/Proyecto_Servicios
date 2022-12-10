#! /usr/bin/python3
# -*- coding: utf-8 -*-

'''
KinovaCommException: Could not initialize Kinova API
RLException: Invalid <param> tag: Cannot load command parameter [robot_description]: no such command [['/opt/ros/noetic/share/xacro/xacro.py', '/home/daniel/Desktop/Proyecto_Servicios/kinova_roscontrol/src/kinova-ros/kinova_description/urdf/m1n6s300_standalone.xacro']]. 
Param xml is <param name="robot_description" command="$(find xacro)/xacro.py '$(find kinova_description)/urdf/$(arg kinova_robotType)_standalone.xacro'"/>
The traceback for the exception was written to the log file
'''

import copy
import time
from pynput import keyboard as kb

# ROS
import rospy
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
import moveit_commander
import numpy as np

# Transformar de ángulos de Euler a cuaternios
def get_quaternion_from_euler(roll, pitch, yaw):
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]




############################### IMPORTANTE ################################################################
# AHORA EL PROGRAMA ESTÁ PARA LA SIMULACIÓN, NO SABEMOS SI FUNCIONARÁ ASI CON MOVEIT                      #
# DIRECTAMENTE. AUN ASÍ SE PUEDE PLANIFICAR CON MOVEIT, PERO HABRÍA QUE TENER LAS POSICIONES              #
# DEL ROBOT EN CADA MOMENTO --> LAS COSAS QUE ESTAN CON "REAL TODO REAL" SON LAS QUE DEPENDEN DEL REAL    #
###########################################################################################################




# Clase para el control del robot
class Scullion():

    def __init__(self,parent=None):
        # Inicializar el nodo
        rospy.init_node("scullion")
        
        # Suscriptor al nodo del control por voz
        rospy.Subscriber("/voice_ui", String, self.__cb)
        
        # Suscriptor al nodo de la cámara
        rospy.Subscriber("/camera", String, self.__cb)

        #Publica la disponibilidad de los ingredientes
        self.pub = rospy.Publisher("/ingredients",String,queue_size=10)
        
        # Grupos de movimiento
        self.arm = moveit_commander.MoveGroupCommander("arm_kinova")
        self.gripper = moveit_commander.MoveGroupCommander("gripper_kinova")
        
        # Lista con las posiciones articulares del robot
        self.q = [] 
        
        ################# REAL TODO REAL ############
        # OBTENER LAS POSICIONES ACTUALES DEL ROBOT: con un callback del topic presumiblemente
        #############################################

        # Lista de ingredientes y sus posiciones
        self.__ingredients = []
        
        self.salt = Point()
        self.pepper = Point()
        self.sugar = Point()
        
        self.salt.x = -0.43
        self.salt.y = 0.14
        self.__ingredients.append(("sal", True))
        
        self.sugar.x = -0.35
        self.sugar.y = 0.35
        self.__ingredients.append(("azúcar", True))
        
        self.pepper.x = -0.17 
        self.pepper.y = 0.44
        self.__ingredients.append(("pimienta", True))
        
        # Lista de comandos
        self.__cmd = []
        
        
        # Se mueve el robot a la posición inicial
        print(" ------------ Moving to initial position ---------")
        self.Move_to_initial_position()
        self.Open()
        
    
    
    # Bucle de control
    def control_loop(self):
        
        # Bucle de control
        while not rospy.is_shutdown():
            
            # Comprueba que hay un comando a realizar
            if len(self.__cmd) > 0:
                
                # Saca el comando de la lista de comandos
                command = self.__cmd.pop(0)
                command = command.lower().split()
                
                # Realiza el pick and place si tiene que dar un ingrediente, luego pone la tupla a False (no hay ingrediente en la zona de almacén)
                if len(command) == 1:

                    #Mensaje para comunicar a la interfaz el ingrediente que se coge
                    mensaje = String()

                    if command[0] == "sal" and self.__ingredients[0][1]:
                        mensaje.data = "sal"
                        self.pub.publish(mensaje)
                        self.grab(self.salt.x, self.salt.y, 0.06, self.salt.x, -self.salt.y, True)
                        tupla = ("sal",False)
                        self.__ingredients[0] = tupla
                        
                        
                    elif command[0] == "azúcar" and self.__ingredients[1][1]:
                        mensaje.data = "azúcar"
                        self.pub.publish(mensaje)
                        self.grab(self.sugar.x, self.sugar.y, 0.06, self.sugar.x, -self.sugar.y, True)
                        tupla = ("azúcar",False)
                        self.__ingredients[1] = tupla
                        
                                            
                    elif command[0] == "pimienta" and self.__ingredients[2][1]:
                        tupla = ("pimienta",False)
                        mensaje.data = "pimienta"
                        self.pub.publish(mensaje)
                        self.grab(self.pepper.x, self.pepper.y, 0.06, self.pepper.x, -self.pepper.y, True)
                        self.__ingredients[2] = tupla
                        
                elif len(command) > 1:

                    X,Y = map(float,command)
                    # Realiza el pick and place si tiene que devolver un ingrediente, luego pone la tupla a True (hay ingrediente en la zona de almacén)
                    if command[2] == "0":
                        mensaje.data = "sal"
                        self.pub.publish(mensaje)
                        self.grab(X,Y, 0.06, self.salt.x, self.salt.y, False)
                        tupla = ("sal",True)
                        self.ingredients[0] = tupla
                    
                    elif command[2] == "1":
                        mensaje.data = "pimienta"
                        self.pub.publish(mensaje)
                        self.grab(X,Y, 0.06, self.pepper.x, self.pepper.y, False)
                        tupla = ("pimienta",True)
                        self.ingredients[1] = tupla
                        
                    elif command[2] == "2":
                        mensaje.data = "azúcar"
                        self.pub.publish(mensaje)
                        self.grab(X,Y, 0.06, self.sugar.x, self.sugar.y, False)
                        tupla = ("azúcar",True)
                        self.ingredients[2] = tupla
     
     
     
    # Callback de la interfaz por voz y cámara: recoge los comandos y los almacena
    def __cb(self, data):
        self.__cmd.append(data.data)
        
        
    # Función donde se llama a todos los pasos para coger el objeto 
    def grab(self, x_move, y_move, z_move, x_place, y_place, interm):
        self.move(x_move, y_move, z_move + 0.2)
        
        time.sleep(1)
        self.move(x_move, y_move, z_move)

        time.sleep(1)
        self.Grab()

        time.sleep(1)
        self.place_on_target(x_place, y_place, interm)

        time.sleep(1)
        self.Open()

        time.sleep(1)
        self.Move_to_initial_position()


    # Función para mover el robot a una posición deseada
    def move(self, x,y,z):
        waypoints = []
        arm_current_pose = self.arm.get_current_pose()
        self.arm.clear_pose_targets()
        self.arm.set_goal_tolerance(0.01)

        waypoint1 = Pose()
        waypoint1.position.x = x
        waypoint1.position.y = y
        waypoint1.position.z = z

        waypoint1.orientation = arm_current_pose.pose.orientation
        waypoints.append(copy.deepcopy(waypoint1))
        
        (plan, fraction) = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0)  # waypoints to follow  # eef_step
        self.arm.execute(plan, wait=True)  
     

    # Función que coloca el objeto en el lugar correspondiente
    def place_on_target(self, x, y, interm):

        waypoints = []

        arm_current_pose = Pose()
        arm_current_pose = self.arm.get_current_pose()
        self.arm.clear_pose_targets()
        self.arm.set_goal_tolerance(0.01)

        waypoint1 = Pose()
        waypoint1.position.x = arm_current_pose.pose.position.x
        waypoint1.position.y = arm_current_pose.pose.position.y
        waypoint1.position.z = 0.3
        waypoint1.orientation = arm_current_pose.pose.orientation  
        waypoints.append(copy.deepcopy(waypoint1))

        waypoint3 = Pose()
        
        if interm == True:
            waypoint3.position.x = 0
            waypoint3.position.y = 0.3
            waypoint3.position.z = 0.3
            waypoint3.orientation = arm_current_pose.pose.orientation  
            waypoints.append(copy.deepcopy(waypoint3))
        
        else:
            waypoint3.position.x = 0.35
            waypoint3.position.y = 0.1
            waypoint3.position.z = 0.3
            waypoint3.orientation = arm_current_pose.pose.orientation  
            waypoints.append(copy.deepcopy(waypoint3))
            
        waypoint2 = Pose()
        waypoint2.position.x = x
        waypoint2.position.y = y
        waypoint2.position.z = 0.3
    
        waypoint2.orientation = arm_current_pose.pose.orientation  
        waypoints.append(copy.deepcopy(waypoint2))

        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = 0.06

        target_pose.orientation = arm_current_pose.pose.orientation  
        waypoints.append(copy.deepcopy(target_pose))

        (plan, fraction) = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0)  # waypoints to follow  # eef_step
        self.arm.execute(plan, wait=True)


# Funciones para abrir y cerrar la pinza (J1: 0.3, J2: 1.3)
    def Grab(self):
        self.gripper.set_goal_tolerance(0.1)
        self.gripper.set_named_target("close")
        self.gripper.go()


    def Open(self):
        self.gripper.set_goal_tolerance(0.1)
        self.gripper.set_named_target("open")
        self.gripper.go()


    # Función para ir a la posición inicial
    def Move_to_initial_position(self):
        self.arm.set_goal_tolerance(0.01)
        self.arm.set_named_target("ready")
        self.arm.go()


# Main
if __name__ == '__main__':
    # Se define el objeto
    scullion = Scullion()
    
    scullion.control_loop()