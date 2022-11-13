#! /usr/bin/python3
# -*- coding: utf-8 -*-

'''
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.Qt import *
from controller_window import Ui_Form
'''
import copy
import os
import time
from pynput import keyboard as kb

# ROS
import rospy
from geometry_msgs.msg import Pose
import moveit_commander
import numpy as np

# Speech recognition
import speech_recognition as sr


def get_quaternion_from_euler(roll, pitch, yaw):
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

class Scullion():

    def __init__(self,parent=None):
        # super(GUI, self).__init__(parent)
        # self.ui = Ui_Form()
        # self.ui.setupUi(self)

        self.arm = moveit_commander.MoveGroupCommander("arm_kinova")
        self.gripper = moveit_commander.MoveGroupCommander("gripper_kinova")
        self.r = sr.Recognizer()
        
        self.Move_to_initial_position()
                
    def select(self, tecla):
        if(str(tecla) == 'r'):
            self.grab_red()
            
        elif(str(tecla) == 'g'):
            self.grab_green()
            
        elif(str(tecla) == 'b'):
            self.grab_blue()
        

    def grab_red(self):
        time.sleep(2)
        self.Open()
        time.sleep(2)
        self.test2()
        time.sleep(2)
        self.Grab()
        time.sleep(2)
        self.Place_on_red()
        time.sleep(2)
        self.Open()
        time.sleep(2)
        self.Move_to_initial_position()
        
    def grab_green(self):
        time.sleep(2)
        self.Open()
        time.sleep(2)
        self.test()
        time.sleep(2)
        self.Grab()
        time.sleep(2)
        self.Place_on_red()
        time.sleep(2)
        self.Open()
        time.sleep(2)
        self.Move_to_initial_position()
        
    def grab_blue(self):
        time.sleep(2)
        self.Open()
        time.sleep(2)
        self.test3()
        time.sleep(2)
        self.Grab()
        time.sleep(2)
        self.Place_on_red()
        time.sleep(2)
        self.Open()
        time.sleep(2)
        self.Move_to_initial_position()
       
    def test(self):
        waypoints = []
        arm_current_pose = self.arm.get_current_pose()
        self.arm.clear_pose_targets()
        self.arm.set_goal_tolerance(0.01)

        waypoint1 = Pose()
        waypoint1.position.x = 0
        waypoint1.position.y = 0.5
        waypoint1.position.z = 0.06

        waypoint1.orientation = arm_current_pose.pose.orientation
        waypoints.append(copy.deepcopy(waypoint1))
        
        (plan, fraction) = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0)  # waypoints to follow  # eef_step
        self.arm.execute(plan, wait=True)  
          
    def test2(self):
        waypoints = []
        arm_current_pose = self.arm.get_current_pose()
        self.arm.clear_pose_targets()
        self.arm.set_goal_tolerance(0.01)

        waypoint1 = Pose()
        waypoint1.position.x = 0.35
        waypoint1.position.y = 0.35
        waypoint1.position.z = 0.06

        waypoint1.orientation = arm_current_pose.pose.orientation
        waypoints.append(copy.deepcopy(waypoint1))
        
        (plan, fraction) = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0)  # waypoints to follow  # eef_step
        self.arm.execute(plan, wait=True)
        
    def test3(self):
        waypoints = []
        arm_current_pose = self.arm.get_current_pose()
        self.arm.clear_pose_targets()
        self.arm.set_goal_tolerance(0.01)

        waypoint1 = Pose()
        waypoint1.position.x = -0.35
        waypoint1.position.y = 0.35
        waypoint1.position.z = 0.06

        waypoint1.orientation = arm_current_pose.pose.orientation
        waypoints.append(copy.deepcopy(waypoint1))
        
        (plan, fraction) = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0)  # waypoints to follow  # eef_step
        self.arm.execute(plan, wait=True)
        
    def Place_on_red(self):

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
        waypoint2.position.x = 0
        waypoint2.position.y = -0.5
        waypoint2.position.z = 0.3
    
        waypoint2.orientation = arm_current_pose.pose.orientation  
        waypoints.append(copy.deepcopy(waypoint2))

        target_pose = Pose()
        target_pose.position.x = 0
        target_pose.position.y = -0.5
        target_pose.position.z = 0.06

        target_pose.orientation = arm_current_pose.pose.orientation  
        waypoints.append(copy.deepcopy(target_pose))

        (plan, fraction) = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0)  # waypoints to follow  # eef_step
        self.arm.execute(plan, wait=True)

    def Grab(self):
        self.gripper.set_goal_tolerance(0.05)
        self.gripper.set_named_target("close")
        self.gripper.go()

    def Open(self):
        self.gripper.set_goal_tolerance(0.1)
        self.gripper.set_named_target("open")
        self.gripper.go()

    def Pick_up_the_block(self):
        global block_pose
        waypoints = []
        arm_current_pose = Pose()
        arm_current_pose = self.arm.get_current_pose()
        self.arm.clear_pose_targets()
        self.arm.set_goal_tolerance(0.01)

        waypoint1 = Pose()
        waypoint1.position.x = -0.3
        waypoint1.position.y = -0.3
        waypoint1.position.z = arm_current_pose.pose.position.z
        waypoint1.orientation = arm_current_pose.pose.orientation  

        waypoints.append(copy.deepcopy(waypoint1))

        target_pose = Pose()
        target_pose.position.x = -0.3
        target_pose.position.y = -0.3
        target_pose.position.z = 0.06
        target_pose.orientation = arm_current_pose.pose.orientation  

        waypoints.append(copy.deepcopy(target_pose))

        (plan, fraction) = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0)  # waypoints to follow  # eef_step
        self.arm.execute(plan, wait=True)

    def Move_to_initial_position(self):
        self.arm.set_goal_tolerance(0.01)
        self.arm.set_named_target("ready")
        self.arm.go()

def block_pose_callback(data):
    global block_pose
    block_pose = data

scullion = Scullion()

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

rospy.init_node('controller')
rospy.Subscriber("/block_pose", Pose, block_pose_callback)

if __name__ == '__main__':
    # app = QApplication(sys.argv)
    print("start")
    kb.Listener(callback).run()
    print("end")
    # sys.exit(app.exec_())
    