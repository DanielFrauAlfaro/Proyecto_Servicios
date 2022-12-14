#! /usr/bin/python3
# -*- coding: utf-8 -*-


import time

# ROS
import rospy
import actionlib
import std_msgs.msg
import geometry_msgs.msg
import kinova_msgs.msg
from geometry_msgs.msg import Point
from std_msgs.msg import String


# Clase para el control del robot
class Scullion():

    def __init__(self,parent=None):
        # Inicializar el nodo
        rospy.init_node("scullion")
        
        # Suscriptor al nodo del control por voz
        rospy.Subscriber("/voice_ui", String, self.__cb)
        
        # Suscriptor al nodo de la cámara
        rospy.Subscriber("/ready", String, self.__cb)

        #Publica la disponibilidad de los ingredientes
        self.pub = rospy.Publisher("/ingredients",String,queue_size=10)
        

        # Lista de ingredientes y sus posiciones
        self.__ingredients = []
        
        self.pimientat = Point()
        
        
        self.pimientat.x = 0.35
        self.pimientat.y = 0.0
        self.__ingredients.append(("pimienta", True))
        
        
        # Lista de comandos
        self.__cmd = []
        
        # Código del nombre del Kinova Mico
        self.prefix = "m1n6s300_"

        self.orient = [0.794731, -0.6059, 0.03549, 0.00087]
        
        time.sleep(15)
        
        # Se mueve el robot a la posición inicial
        print(" ------------ Moving to initial position ---------")
        self.Move_to_initial_position()
        self.gripper(0.0)
        
    
    
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

                    if command[0] == "pimienta":
                        mensaje.data = "pimienta"
                        self.pub.publish(mensaje)
                        self.grab(self.pimientat.x, self.pimientat.y, 0.05, 0, -0.35, True)
                        tupla = ("pimienta",False)
                        self.__ingredients[0] = tupla
                        
                    # Realiza el pick and place si tiene que devolver un ingrediente, luego pone la tupla a True (hay ingrediente en la zona de almacén)
                    if command[0] == "0":
                        mensaje.data = "pimienta"
                        self.pub.publish(mensaje)
                        self.grab(0, -0.35, 0.05, self.pimientat.x, self.pimientat.y, False)
                        tupla = ("pimienta",True)
                        self.__ingredients[0] = tupla
     
     
     
    # Callback de la interfaz por voz y cámara: recoge los comandos y los almacena
    def __cb(self, data):
        self.__cmd.append(data.data)
        
        
    # Función donde se llama a todos los pasos para coger el objeto 
    def grab(self, x_move, y_move, z_move, x_place, y_place, interm):
        self.move(x_move, y_move, z_move + 0.25)
        
        time.sleep(1)
        self.move(x_move, y_move, z_move)

        time.sleep(1)
        self.gripper(4080)

        time.sleep(1)
        self.place_on_target(x_move, y_move, x_place, y_place, interm)

        time.sleep(1)
        self.gripper(0.0)
        
        time.sleep(1)
        self.move(x_place, y_place, z_move + 0.25)

        time.sleep(1)
        self.Move_to_initial_position()


    # Función para mover el robot a una posición deseada
    def move(self, x,y,z):
        action_address = '/' + self.prefix + 'driver/pose_action/tool_pose'
        client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
        client.wait_for_server()

        goal = kinova_msgs.msg.ArmPoseGoal()
        goal.pose.header = std_msgs.msg.Header(frame_id=(self.prefix + 'link_base'))
        goal.pose.pose.position = geometry_msgs.msg.Point(
            x=x, y=y, z=z)
        goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
            x=self.orient[0], y=self.orient[1], z=self.orient[2], w=self.orient[3])

        client.send_goal(goal)

        if client.wait_for_result(rospy.Duration(10.0)):
            return client.get_result()
        else:
            client.cancel_all_goals()
            print('        the cartesian action timed-out') 
     

    # Función que coloca el objeto en el lugar correspondiente
    def place_on_target(self, x_, y_, x, y, interm):
        self.move(x_,y_,0.3)
        
        self.move(x,y,0.3)
         
        self.move(x,y,0.05)


# Funciones para abrir y cerrar la pinza (J1: 0.3, J2: 1.3)
    def gripper(self, g):
        action_address = '/' + self.prefix + 'driver/fingers_action/finger_positions'

        client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.SetFingersPositionAction)
        client.wait_for_server()

        goal = kinova_msgs.msg.SetFingersPositionGoal()
        goal.fingers.finger1 = g
        goal.fingers.finger2 = g
        goal.fingers.finger3 = g
        
        
        client.send_goal(goal)
        if client.wait_for_result(rospy.Duration(5.0)):
            return client.get_result()
        else:
            client.cancel_all_goals()
            rospy.logwarn('        the gripper action timed-out')




    # Función para ir a la posición inicial
    def Move_to_initial_position(self):
        self.move(0.35, 0, 0.3)
        self.move(0.2, 0, 0.3)


# Main
if __name__ == '__main__':
    # Se define el objeto
    scullion = Scullion()
    
    scullion.control_loop()