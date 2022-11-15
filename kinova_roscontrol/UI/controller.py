#! /usr/bin/python3
# -*- coding: utf-8 -*-

'''
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.Qt import *
from controller_window import Ui_Form
'''

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
from geometry_msgs.msg import Pose, Position
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




############################### IMPORTANTE #####################################
# AHORA EL PROGRAMA ESTÁ PARA LA SIMULACIÓN, NO SABEMOS SI FUNCIONARÁ ASI CON MOVEIT
# DIRECTAMENTE. AUN ASÍ SE PUEDE PLANIFICAR CON MOVEIT, PERO HABRÍA QUE TENER LAS POSICIONES
# DEL ROBOT EN CADA MOMENTO --> LAS COSAS QUE ESTAN CON "REAL TODO REAL" SON LAS QUE DEPENDEN DEL REAL







# Clase para el control del robot
class Scullion():

    def __init__(self,parent=None):
        # Suscriptor al nodo del control por voz
        rospy.Subscriber("/voice_ui", String, self.list)
        
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
        self.Move_to_initial_position()
        
        # Lista de ingredientes y sus posiciones
        self.ingredients = []
        
        self.red = Position()
        self.green = Position()
        self.blue = Position()
        
        self.red.x = 0.35
        self.red.y = 0.35
        self.ingredients.append(("red", True))
        
        self.blue.x = -0.35
        self.blue.y = 0.35
        self.ingredients.append(("blue", True))
        
        self.green.x = 0.0
        self.green.y = 0.5
        self.ingredients.append(("green", True))
    
    
    #  TODO: Bucle de control
    def control_loop(self):
        print("------- TODO: Bucle de control --------")
        
        ################## TODO #################
        '''
            BUCLE DE CONTROL COMO UNA MÁQUINA DE ESTADOS
              - Estado inicial: reposo
              - Comando por voz: coge el objeto y lo deja
              - Aparece objeto zona de recogida: se detecta cual es y lo deja en su lugar
        '''
        #########################################
        
        
    # Callback del interfaz por voz (AHORA ESTÁ DEL TECLADO)
    def list(self, data):
        if data.data == "rojo":
            self.grab(self.red.x, self.red.y, 0.06, 0, -0.5)
        
        if data.data == "verde":
            self.grab(self.green.x, self.green.y, 0.06, 0, -0.5)
            
        if data.data == "azul":
            self.grab(self.blue.x, self.blue.y, 0.06, 0, -0.5)
        
        
    # Función donde se llama a todos los pasos para coger el objeto 
    def grab(self, x_move, y_move, z_move, x_place, y_place):
        time.sleep(2)
        self.Open()

        time.sleep(2)
        self.move(x_move, y_move, z_move)

        time.sleep(2)
        self.Grab()

        time.sleep(2)
        self.place_on_target(x_place, y_place)

        time.sleep(2)
        self.Open()

        time.sleep(2)
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
        self.gripper.set_goal_tolerance(0.05)
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


# Se define el callback de la presión de la tecla
def callback(tecla):
    global scullion
    
    print("Se ha pulsado la tecla ")
    
    if(str(tecla) == "'r'"):
        print("R")
        scullion.grab_red()
            
    elif(str(tecla) == "'g'"):
        print("G")
        scullion.grab_green()
           
    elif(str(tecla) == "'b"):
        print("B")
        scullion.grab_blue()


# Main
if __name__ == '__main__':
    while rospy.is_not_shutdown():
        a = 1
    