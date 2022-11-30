#! /usr/bin/python3
# -*- coding: utf-8 -*-

'''
KinovaCommException: Could not initialize Kinova API

RLException: Invalid <param> tag: Cannot load command parameter [robot_description]: no such command [['/opt/ros/noetic/share/xacro/xacro.py', '/home/daniel/Desktop/Proyecto_Servicios/kinova_roscontrol/src/kinova-ros/kinova_description/urdf/m1n6s300_standalone.xacro']]. 

Param xml is <param name="robot_description" command="$(find xacro)/xacro.py '$(find kinova_description)/urdf/$(arg kinova_robotType)_standalone.xacro'"/>
The traceback for the exception was written to the log file
'''

import copy
import os
import time
from pynput import keyboard as kb

# ROS
import rospy
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
import moveit_commander
import numpy as np
from pynput import keyboard as kb

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
        rospy.Subscriber("/voice_ui", String, self.list)
        rospy.Subscriber("/ready", String, self.rec)
        rospy.Subscriber("/teclas", String, self.teclas)
        
        # TODO: suscribirse al nodo de la cámara
        print("TODO: susciptor al nodo de la cámara")
        
        # Grupos de movimiento
        self.arm = moveit_commander.MoveGroupCommander("arm_kinova")
        self.gripper = moveit_commander.MoveGroupCommander("gripper_kinova")
        
        # Lista con las posiciones articulares del robot
        self.q = [] 
        
        # Se mueve el robot a la posición inicial
        ################# REAL TODO REAL ############
        # OBTENER LAS POSICIONES ACTUALES DEL ROBOT: con un callback del topic presumiblemente
        #############################################
        
        # Se mueve el robot a la posición inicial
        print(" ------------ Moving to initial position ---------")
        self.Move_to_initial_position()
        
        # Lista de ingredientes y sus posiciones
        self.ingredients = []
        
        self.red = Point()
        self.green = Point()
        self.blue = Point()
        
        self.red.x = 0.35
        self.red.y = 0.35
        self.ingredients.append(("red", True))
        
        self.blue.x = -0.35
        self.blue.y = 0.35
        self.ingredients.append(("blue", True))
        
        self.green.x = 0.0
        self.green.y = 0.5
        self.ingredients.append(("green", True))
        
        self.cmd = []
    
    
    # Bucle de control
    def control_loop(self):
        
        # Bucle de control
        while not rospy.is_shutdown():
            if len(self.cmd) > 0:               # Si hay un comando por realizar
                    
                command = self.cmd.pop(0)       # Se extrae el comando y se selecciona la acción
                command = command.lower().split()
                print(command)
                
                if len(command) == 1:
                    # Se coge el objeto y se pasa a la zona de manipulación, luego se pone a False la tupla (no está en la zona de alamcenaje)
                    if command[0]  == "rojo" and self.ingredients[0][1]:
                        self.grab(self.red.x, self.red.y, 0.06, 0, -0.5)
                        tupla = ("red",False)
                        self.ingredients[0] = tupla
                    
                    if command[0]  == "verde" and self.ingredients[1][1]:
                        self.grab(self.green.x, self.green.y, 0.06, 0, -0.5)
                        tupla = ("green",False)
                        self.ingredients[1] = tupla
                    
                    if command[0]  == "azul" and self.ingredients[2][1]:
                        self.grab(self.blue.x, self.blue.y, 0.06, 0, -0.5)
                        tupla = ("blue",False)
                        self.ingredients[2] = tupla
                
                else:
                    # Se coge el objeto y se guarda en la zona de almacenaje, luego se pone a True la tupla (está en la zona de almacenaje)
                    if command[2] == "0":
                        self.grab(float(command[0]), float(command[1]), 0.06, self.red.x, self.red.y)
                        tupla = ("red",True)
                        self.ingredients[0] = tupla
                    
                    if command[2] == "1" :
                        self.grab(float(command[0]), float(command[1]), 0.06, self.green.x, self.green.y)
                        tupla = ("green",True)
                        self.ingredients[1] = tupla
                        
                    if command[2] == "2":
                        self.grab(float(command[0]), float(command[1]), 0.06, self.blue.x, self.blue.y)
                        tupla = ("blue",True)
                        self.ingredients[2] = tupla
    
    
    # Callbacks de la cámara y de las teclas (simula interfaz por voz)
    def rec(self, data):
        self.cmd.append(data.data)
        print("data.data")
    def teclas(self, data):
        self.cmd.append(data.data)
          
    # Callback del interfaz por voz (AHORA ESTÁ DEL TECLADO Y EN BUCLE INFINITO)
    def list(self, data):
        if data.data == "rojo":
            self.grab(self.red.x, self.red.y, 0.06, 0, -0.5)
        
        if data.data == "verde":
            self.grab(self.green.x, self.green.y, 0.06, 0, -0.5)
            
        if data.data == "azul":
            self.grab(self.blue.x, self.blue.y, 0.06, 0, -0.5)
        
        
    # Función donde se llama a todos los pasos para coger el objeto 
    def grab(self, x_move, y_move, z_move, x_place, y_place):
        self.Open()
        
        time.sleep(1)
        self.move(x_move, y_move, z_move + 0.2)
        
        time.sleep(1)
        self.move(x_move, y_move, z_move)

        time.sleep(1)
        self.Grab()

        time.sleep(1)
        self.place_on_target(x_place, y_place)

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
    def place_on_target(self, x, y):

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
        waypoint3.position.x = 0.3
        waypoint3.position.y = 0
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


# Se define el objeto
scullion = Scullion()
scullion.control_loop()

# Se define el callback de la presión de la tecla
def callback(tecla):
    global scullion
    
    s = String()
    
    print("Se ha pulsado la tecla ")
    
    if(str(tecla) == "'r'"):
        print("R")
        scullion.cmd.append("rojo")
            
    elif(str(tecla) == "'g'"):
        print("G")
        scullion.cmd.append("verde")
           
    elif(str(tecla) == "'b'"):
        print("B")
        scullion.cmd.append("azul")

        
    else:
        scullion.Move_to_initial_position()


# Main
if __name__ == '__main__':
    print("------- Start --------")
    
    scullion.control_loop()
    
    