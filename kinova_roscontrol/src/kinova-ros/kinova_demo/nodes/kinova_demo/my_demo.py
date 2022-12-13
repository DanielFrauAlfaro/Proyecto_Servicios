#! /usr/bin/env python3
"""A helper program to test gripper goals for the JACO and MICO arms."""

import roslib; roslib.load_manifest('kinova_demo')
import rospy

import sys

import actionlib
import kinova_msgs.msg
import std_msgs.msg
import geometry_msgs.msg

import argparse


""" Global variable """
prefix = 'm1n6s300_'  # m1n6s300


def cartesian_pose_client(position, orientation):
    """Send a cartesian goal to the action server."""
    action_address = '/' + prefix + 'driver/pose_action/tool_pose'
    client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.ArmPoseGoal()
    goal.pose.header = std_msgs.msg.Header(frame_id=(prefix + 'link_base'))
    goal.pose.pose.position = geometry_msgs.msg.Point(
        x=position[0], y=position[1], z=position[2])
    goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

    # print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

    client.send_goal(goal)

    if client.wait_for_result(rospy.Duration(10.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        print('        the cartesian action timed-out')
        return None

def gripper_client(finger_positions):
    """Send a gripper goal to the action server."""
    action_address = '/' + prefix + 'driver/fingers_action/finger_positions'

    client = actionlib.SimpleActionClient(action_address,
                                          kinova_msgs.msg.SetFingersPositionAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.SetFingersPositionGoal()
    goal.fingers.finger1 = float(finger_positions[0])
    goal.fingers.finger2 = float(finger_positions[1])
    
    # The MICO arm has only two fingers, but the same action definition is used
    if len(finger_positions) < 3:
        goal.fingers.finger3 = 0.0
    else:
        goal.fingers.finger3 = float(finger_positions[2])
    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(5.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        rospy.logwarn('        the gripper action timed-out')
        return None


# ---------- Arm Joint --------------
def joint_angle_client(angle_set):
    """Send a joint angle goal to the action server."""
    action_address = '/' + prefix + 'driver/joints_action/joint_angles'
    client = actionlib.SimpleActionClient(action_address,
                                          kinova_msgs.msg.ArmJointAnglesAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.ArmJointAnglesGoal()

    goal.angles.joint1 = angle_set[0]
    goal.angles.joint2 = angle_set[1]
    goal.angles.joint3 = angle_set[2]
    goal.angles.joint4 = angle_set[3]
    goal.angles.joint5 = angle_set[4]
    goal.angles.joint6 = angle_set[5]
    goal.angles.joint7 = angle_set[6]

    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(20.0)):
        return client.get_result()
    else:
        print('        the joint angle action timed-out')
        client.cancel_all_goals()
        return None
    

def setcurrentJointCommand(feedback):
    global currentJointCommand

    currentJointCommand_str_list = str(feedback).split("\n")
    for index in range(0,len(currentJointCommand_str_list)):
        temp_str=currentJointCommand_str_list[index].split(": ")
        currentJointCommand[index] = float(temp_str[1])
        
def getcurrentJointCommand(prefix_):
    # wait to get current position
    topic_address = '/' + prefix_ + 'driver/out/joint_command'
    rospy.Subscriber(topic_address, kinova_msgs.msg.JointAngles, setcurrentJointCommand)
    rospy.wait_for_message(topic_address, kinova_msgs.msg.JointAngles)
    print('position listener obtained message for joint position. ')


# Main
if __name__ == '__main__':
    print("Starting demo")
    joint_angle_client([1.57, 3.5135, 4.7716, 0.8852, -2.3084, 1.1282])

    cartesian_pose_client([0.5, 0, 0.26], [0.79, -0.6059, 0.0359, 0.00087])
    
    
    '''
    
Posición de agarre (superior)  (JOINTS):    (Position: [x = 0.5, y = 0.0, z = 0.26])

    <joint name="j2n6s300_joint_1" value="0.1048575812317214"/>
    <joint name="j2n6s300_joint_2" value="2.5416042919908124"/>
    <joint name="j2n6s300_joint_3" value="4.5650001372527305"/>
    <joint name="j2n6s300_joint_4" value="0.38050183567458795"/>
    <joint name="j2n6s300_joint_5" value="-1.2330303362193176"/>
    <joint name="j2n6s300_joint_6" value="0.702691615105369"/> 

                                (CARTESIAN ORIENTATION): [X = 0.794731, Y = -0.6059, Z = 0.03549, W = 0.00087]



Posición de agarre (inferior)  (JOINTS):    (Position: [x = 0.5, y = 0.0, z = 0.06])

    <joint name="j2n6s300_joint_1" value="0.10755548301292706/>
    <joint name="j2n6s300_joint_2" value="2.188503829630644"/>
    <joint name="j2n6s300_joint_3" value="4.729273331377606"/>
    <joint name="j2n6s300_joint_4" value="0.24368134950591092"/>
    <joint name="j2n6s300_joint_5" value="-0.6329153886244301"/>
    <joint name="j2n6s300_joint_6" value="0.4743015064614271"/> 

                                (CARTESIAN ORIENTATION): [X = 0.794731, Y = -0.6059, Z = 0.03549, W = 0.00087]
    '''