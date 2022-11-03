#! /usr/bin/python3
# -*- coding: utf-8 -*-

import sys, copy
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.Qt import *
from controller_window import Ui_Form
# ROS
import rospy
from geometry_msgs.msg import Pose
import moveit_commander

# block_pose = Pose()

class Scullion():

    def __init__(self,parent=None):
        # super(GUI, self).__init__(parent)
        # self.ui = Ui_Form()
        # self.ui.setupUi(self)

        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.gripper = moveit_commander.MoveGroupCommander("gripper")
        self.Move_to_initial_position()
        
    def show(self):
        while(True):
            self.Place_on_red()
            self.Grab()
            self.Move_to_initial_position()
        
    def Place_on_red(self):

        waypoints = []

        arm_current_pose = Pose()
        arm_current_pose = self.arm.get_current_pose()
        self.arm.clear_pose_targets()
        self.arm.set_goal_tolerance(0.01)

        waypoint1 = Pose()
        waypoint1.position.x = arm_current_pose.pose.position.x
        waypoint1.position.y = arm_current_pose.pose.position.y
        waypoint1.position.z = 0.2
        waypoint1.orientation = arm_current_pose.pose.orientation  
        waypoints.append(copy.deepcopy(waypoint1))

        waypoint2 = Pose()
        waypoint2.position.x = 0.3
        waypoint2.position.y = -0.3
        waypoint2.position.z = 0.2
        waypoint2.orientation = arm_current_pose.pose.orientation  
        waypoints.append(copy.deepcopy(waypoint2))

        target_pose = Pose()
        target_pose.position.x = 0.3
        target_pose.position.y = -0.3
        target_pose.position.z = 0.1
        target_pose.orientation = arm_current_pose.pose.orientation  
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
        waypoint1.position.x = 0.5
        waypoint1.position.y = 0
        waypoint1.position.z = arm_current_pose.pose.position.z
        waypoint1.orientation = arm_current_pose.pose.orientation  

        waypoints.append(copy.deepcopy(waypoint1))

        target_pose = Pose()
        target_pose.position.x = 0.5
        target_pose.position.y = 0
        target_pose.position.z = 0.04
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