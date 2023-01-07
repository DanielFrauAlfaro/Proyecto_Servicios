#! /usr/bin/python3
# -*- coding: utf-8 -*-


import copy
import time

# ROS
import rospy
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
import moveit_commander


# Clase para el control del robot
class Scullion():

    def __init__(self,parent=None):
        # Inicializar el nodo
        rospy.init_node("scullion")
        
        
        # Suscriptor al nodo de la cámara
        rospy.Subscriber("/ready", String, self.__cb)
        rospy.Subscriber("/store", String, self.__cb)

        #Publica la disponibilidad de los ingredientes
        self.pub = rospy.Publisher("/ingredients",String,queue_size=10)
        
        time.sleep(15)
        
        # Grupos de movimiento
        self.arm = moveit_commander.MoveGroupCommander("arm_kinova")
        self.gripper = moveit_commander.MoveGroupCommander("gripper_kinova")
        

        
        self.salt = Point()
        self.pepper = Point()
        self.sugar = Point()
        
        self.salt.x = -0.43
        self.salt.y = 0.14
        
        self.sugar.x = -0.35
        self.sugar.y = 0.35
        
        self.pepper.x = -0.17 
        self.pepper.y = 0.44
        
        # Lista de comandos
        self.__cmd = []

        # Lista de puntos de paso
        self.waypoints = []
        
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

                #Mensaje para comunicar a la interfaz el ingrediente que se coge
                mensaje = String()
                
                # Realiza el pick and place si tiene que dar un ingrediente, luego pone la tupla a False (no hay ingrediente en la zona de almacén)
                if len(command) == 4:
                    X = float(command[0])
                    Y = float(command[1])

                    if command[3] == "sal":
                        mensaje.data = "sal"
                        self.pub.publish(mensaje)
                        self.grab(X, Y, 0.06, 0.5, 0, True)

                        self.salt.x = X
                        self.salt.y = Y

                        
                        
                    elif command[3] == "pimienta":
                        mensaje.data = "pimienta"
                        self.pub.publish(mensaje)
                        self.grab(X, Y, 0.06,0.3, 0.3, True)

                        self.pepper.x = X
                        self.pepper.y = Y

                        
                                            
                    elif command[3] == "azúcar" :
                        mensaje.data = "azúcar"
                        self.pub.publish(mensaje)
                        self.grab(X, Y, 0.06, 0.3, -0.15, True)

                        self.sugar.x = X
                        self.sugar.y = Y

                        
                elif len(command) > 1:

                    X = float(command[0])
                    Y = float(command[1])
                    # Realiza el pick and place si tiene que devolver un ingrediente, luego pone la tupla a True (hay ingrediente en la zona de almacén)
                    if command[2] == "0":
                        mensaje.data = "sal"
                        self.pub.publish(mensaje)
                        self.grab(X,Y, 0.06, self.salt.x, self.salt.y, False)
                        
                    
                    elif command[2] == "1":
                        mensaje.data = "pimienta"
                        self.pub.publish(mensaje)
                        self.grab(X,Y, 0.06, self.pepper.x, self.pepper.y, False)
                        
                        
                    elif command[2] == "2":
                        mensaje.data = "azúcar"
                        self.pub.publish(mensaje)
                        self.grab(X,Y, 0.06, self.sugar.x, self.sugar.y, False)
                        
     
     
     
    # Callback de la interfaz por voz y cámara: recoge los comandos y los almacena
    def __cb(self, data):
        self.__cmd.append(data.data)
        
        
    # Función donde se llama a todos los pasos para coger el objeto 
    def grab(self, x_move, y_move, z_move, x_place, y_place, interm):
        # Resetea la lista de puntos de paso
        self.waypoints = []

        # Abre la pinza
        self.Open()

        # ----------------- Añade puntos ---------------

        # En el caso de que se vaya a la zona de recogida hace un movimiento adicional
        if not interm:
            self.move(0.2, 0.2, z_move + 0.2)
            
            self.move(0.2, -0.2, z_move + 0.2)
            
        
        self.move(x_move, y_move, z_move + 0.2)
        
        self.move(x_move, y_move, z_move)

        # Ejecuta los movimientos
        (plan, fraction) = self.arm.compute_cartesian_path(self.waypoints, 0.01, 0.0)  # waypoints to follow  # eef_step
        self.arm.execute(plan, wait=True)

        # Espera y cierra la pinza
        time.sleep(1)
        self.Grab()

        # Lo lleva al objetivo
        time.sleep(1)
        self.place_on_target(x_place, y_place, interm)

        # Deja el objeto
        time.sleep(1)
        self.Open()

        # Posición inicial
        time.sleep(1)
        self.Move_to_initial_position()


    # Función para mover el robot a una posición deseada
    def move(self, x,y,z):
        arm_current_pose = self.arm.get_current_pose()
        self.arm.clear_pose_targets()
        self.arm.set_goal_tolerance(0.01)

        waypoint1 = Pose()
        waypoint1.position.x = x
        waypoint1.position.y = y
        waypoint1.position.z = z

        waypoint1.orientation = arm_current_pose.pose.orientation
        self.waypoints.append(copy.deepcopy(waypoint1))
        
          
     

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
        waypoint4 = Pose()
        
        if interm == True:
            waypoint3.position.x = 0
            waypoint3.position.y = 0.3
            waypoint3.position.z = 0.3
            waypoint3.orientation = arm_current_pose.pose.orientation  
            waypoints.append(copy.deepcopy(waypoint3))
        
        else:
            waypoint4.position.x = 0.3
            waypoint4.position.y = -0.3
            waypoint4.position.z = 0.3
            waypoint4.orientation = arm_current_pose.pose.orientation  
            waypoints.append(copy.deepcopy(waypoint4))
            
            waypoint3.position.x = 0.2
            waypoint3.position.y = 0.35
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