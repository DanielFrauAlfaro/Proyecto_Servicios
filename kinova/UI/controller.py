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
# ROS
import rospy
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory
import moveit_commander
import numpy as np

# block_pose = Pose()

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

        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.gripper = moveit_commander.MoveGroupCommander("gripper")
        
        self.pub = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=10)
        
       
    # --------------------------- BUCLE DE CONTROL -------------------------
    def show(self):
        
        self.Move_to_initial_position()

        self.test()
        
        self.Move_to_initial_position()
    # ----------------------------------------------------------------------
    
    def test(self):
        waypoints = []
        arm_current_pose = self.arm.get_current_pose()
        self.arm.clear_pose_targets()
        self.arm.set_goal_tolerance(0.01)

        waypoint1 = Pose()
        waypoint1.position.x = 0
        waypoint1.position.y = 0.5
        waypoint1.position.z = 0.05
        
        x,y,z,w = get_quaternion_from_euler(0, 0, 0)
        
        waypoint1.orientation.x = x
        waypoint1.orientation.y = y
        waypoint1.orientation.z = z
        waypoint1.orientation.w = w
        waypoint1.orientation = arm_current_pose.pose.orientation
        waypoints.append(copy.deepcopy(waypoint1))
        
        (plan, fraction) = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0)  # waypoints to follow  # eef_step
        # self.arm.execute(plan, wait=True)
        print(plan.joint_trajectory)
    
    def test2(self):
        waypoints = []
        arm_current_pose = self.arm.get_current_pose()
        self.arm.clear_pose_targets()
        self.arm.set_goal_tolerance(0.01)
        #-----------------------------
        waypoint2 = Pose()
        waypoint2.position.x = 0
        waypoint2.position.y = 0.5
        waypoint2.position.z = 0.15
        
        x,y,z,w = get_quaternion_from_euler(3.14, 0, 0)
        
        waypoint2.orientation.x = x
        waypoint2.orientation.y = y
        waypoint2.orientation.z = z
        waypoint2.orientation.w = w
        waypoint2.orientation = arm_current_pose.pose.orientation
        waypoints.append(copy.deepcopy(waypoint2))
        # -----------------------------
        waypoint_aux = Pose()
        waypoint_aux.position.x = 0.5
        waypoint_aux.position.y = 0
        waypoint_aux.position.z = 0.15
        
        x,y,z,w = get_quaternion_from_euler(3.14, 0, 0)
        
        waypoint_aux.orientation.x = x
        waypoint_aux.orientation.y = y
        waypoint_aux.orientation.z = z
        waypoint_aux.orientation.w = w
        waypoint_aux.orientation = arm_current_pose.pose.orientation
        waypoints.append(copy.deepcopy(waypoint_aux))
        # ----------------------------------
        waypoint3 = Pose()
        waypoint3.position.x = 0
        waypoint3.position.y = -0.5
        waypoint3.position.z = 0.15
        
        x,y,z,w = get_quaternion_from_euler(3.14, 0, 0)
        
        waypoint3.orientation.x = x
        waypoint3.orientation.y = y
        waypoint3.orientation.z = z
        waypoint3.orientation.w = w
        waypoint3.orientation = arm_current_pose.pose.orientation
        waypoints.append(copy.deepcopy(waypoint3))
        
        # ----------------------------------------
        
        waypoint4 = Pose()
        waypoint4.position.x = 0
        waypoint4.position.y = -0.5
        waypoint4.position.z = 0.05
        
        x,y,z,w = get_quaternion_from_euler(3.14, 0, 0)
        
        waypoint4.orientation.x = x
        waypoint4.orientation.y = y
        waypoint4.orientation.z = z
        waypoint4.orientation.w = w
        waypoint4.orientation = arm_current_pose.pose.orientation
        waypoints.append(copy.deepcopy(waypoint4))
        
        (plan, fraction) = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0)  # waypoints to follow  # eef_step
        self.arm.execute(plan, wait=True)
        print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
        
    def Place_on_red(self):

        waypoints = []

        arm_current_pose = Pose()
        arm_current_pose = self.arm.get_current_pose()
        self.arm.clear_pose_targets()
        self.arm.set_goal_tolerance(0.01)

        waypoint1 = Pose()
        waypoint1.position.x = arm_current_pose.pose.position.x
        waypoint1.position.y = arm_current_pose.pose.position.y
        waypoint1.position.z = 0.5
        waypoint1.orientation = arm_current_pose.pose.orientation  
        waypoints.append(copy.deepcopy(waypoint1))

        waypoint2 = Pose()
        waypoint2.position.x = 0
        waypoint2.position.y = -0.2
        waypoint2.position.z = 0.5
        x,y,z,w = get_quaternion_from_euler(0, 1.57, 1.57)
        
        print("----------------------")
        print(x)
        print(y)
        print(z)
        print(w)
        print("----------------------")
    
        print(arm_current_pose.pose.orientation)
        waypoint2.orientation.x = x
        waypoint2.orientation.y = y
        waypoint2.orientation.z = z
        waypoint2.orientation.w = w 
        #waypoint2.orientation = arm_current_pose.pose.orientation  
        waypoints.append(copy.deepcopy(waypoint2))

        target_pose = Pose()
        target_pose.position.x = 0
        target_pose.position.y = -0.34
        target_pose.position.z = 0.5
        target_pose.orientation.x = x
        target_pose.orientation.y = y
        target_pose.orientation.z = z
        target_pose.orientation.w = w 
        #target_pose.orientation = arm_current_pose.pose.orientation  
        waypoints.append(copy.deepcopy(target_pose))

        print("target pose")
        print(target_pose)
        (plan, fraction) = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0)  # waypoints to follow  # eef_step
        self.arm.execute(plan, wait=True)

    def Grab(self):
        print(str(self.gripper.get_current_state()))
        self.gripper.set_goal_tolerance(0.01)
        self.gripper.set_named_target("close")
        self.gripper.go()

    def Open(self):
        print(str(self.gripper.get_current_state()))
        self.gripper.set_goal_tolerance(0.01)
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

        print("target pose")
        print(target_pose)
        (plan, fraction) = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0)  # waypoints to follow  # eef_step
        self.arm.execute(plan, wait=True)

    def Move_to_initial_position(self):
        self.arm.set_goal_tolerance(0.01)
        self.arm.set_named_target("ready")
        self.arm.go()

def block_pose_callback(data):
    global block_pose
    block_pose = data

rospy.init_node('controller')
rospy.Subscriber("/block_pose", Pose, block_pose_callback)

if __name__ == '__main__':
    # app = QApplication(sys.argv)
    scullion = Scullion()
    scullion.show()
    # sys.exit(app.exec_())