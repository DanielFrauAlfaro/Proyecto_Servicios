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
        rospy.Subscriber("/ready", String, self.rec)
        rospy.Subscriber("/store", String, self.rec)
        time.sleep(10)
        
        # Grupos de movimiento
        self.arm = moveit_commander.MoveGroupCommander("arm_kinova")
        self.gripper = moveit_commander.MoveGroupCommander("gripper_kinova")
        
        # Se mueve el robot a la posición inicial
        print(" ------------ Moving to initial position ---------")
        self.Move_to_initial_position()
        self.Open()

        # Lista de ingredientes y sus posiciones
        self.ingredients = []
        
        self.salt = Point()
        self.pepper = Point()
        self.sugar = Point()
        
        self.salt.x = -0.43
        self.salt.y = 0.14
        self.ingredients.append(("sal", True))
        
        self.sugar.x = -0.35
        self.sugar.y = 0.35
        self.ingredients.append(("azúcar", True))
        
        self.pepper.x = -0.17 
        self.pepper.y = 0.44
        self.ingredients.append(("pimienta", True))
        
        self.cmd = []
    
    
    # Bucle de control
    def control_loop(self):
        
        # Bucle de control
        while not rospy.is_shutdown():
            if len(self.cmd) > 0:               # Si hay un comando por realizar
                    
                command = self.cmd.pop(0)       # Se extrae el comando y se selecciona la acción
                command = command.lower().split()
                
                if len(command) == 4:
                    X = float(command[0])
                    Y = float(command[1])

                    if command[3] == "sal":
                        self.grab(X, Y, 0.06, 0.5, 0, True)
                        self.salt.x = X
                        self.salt.y = Y

                        
                    elif command[3] == "azúcar":
                        self.grab(X, Y, 0.06,0.35, 0.35, True)
                        self.sugar.x = X
                        self.sugar.y = Y
                        
                                            
                    elif command[3] == "pimienta":
                        self.grab(X, Y, 0.06, 0.35, -0.15, True)
                        self.pepper.x = X
                        self.pepper.y = Y

                
                elif len(command) == 3:
                    
                    X = float(command[0])
                    Y =  float(command[1])
                    # Se coge el objeto y se guarda en la zona de almacenaje, luego se pone a True la tupla (está en la zona de almacenaje)
                    if command[2] == "0":
                        self.grab(X,Y, 0.06, self.salt.x, self.salt.y, False)
                    
                    elif command[2] == "1" :
                        self.grab(X,Y, 0.06, self.pepper.x, self.pepper.y, False)
                        
                    elif command[2] == "2":
                        self.grab(X,Y, 0.06, self.sugar.x, self.sugar.y, False)
                        
    
    
    # Callbacks de la cámara y de las teclas (simula interfaz por voz)
    def rec(self, data):
        self.cmd.append(data.data)

        
        
    # Función donde se llama a todos los pasos para coger el objeto 
    def grab(self, x_move, y_move, z_move, x_place, y_place, interm):
        self.Open()
        
        time.sleep(0.5)
        self.move(x_move, y_move, z_move + 0.2)
        
        time.sleep(0.5)
        self.move(x_move, y_move, z_move)

        time.sleep(0.5)
        self.Grab()

        time.sleep(0.5)
        self.place_on_target(x_place, y_place, interm)

        time.sleep(0.5)
        self.Open()

        time.sleep(0.5)
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
        self.gripper.set_goal_tolerance(0.001)
        self.gripper.set_named_target("close")
        self.gripper.go()

    def Open(self):
        self.gripper.set_goal_tolerance(0.001)
        self.gripper.set_named_target("open")
        self.gripper.go()

    # Función para ir a la posición inicial
    def Move_to_initial_position(self):
        self.arm.set_goal_tolerance(0.001)
        self.arm.set_named_target("ready")
        self.arm.go()




# Main
if __name__ == '__main__':
    print("------- Start --------")
    # Se define el objeto
    scullion = Scullion()
    scullion.control_loop()
    
    