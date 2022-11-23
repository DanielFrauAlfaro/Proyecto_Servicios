#!/usr/bin/env python3
import rospy, os, sys, math, time

from std_msgs.msg import Header, String, Int32, Float64
from geometry_msgs.msg import Twist, Vector3, Pose
from sensor_msgs.msg import Joy
from math import pi

import numpy as np
import cv2
import cv2.aruco as aruco

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge() 

arucoMarkerLength = 0.057 
font = cv2.FONT_HERSHEY_SIMPLEX

class AR():

    def __init__(self, videoPort):
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50) # Rojo: DICT_4X4_50   Verde: DICT_6X6_250
        self.dictionary2 = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        
        self.blocks = []

    def find_ARMarker(self, frame):
        self.frame = frame
        if len(self.frame.shape) == 3:
            self.Height, self.Width, self.channels = self.frame.shape[:3]
        else:
            self.Height, self.Width = self.frame.shape[:2]
            self.channels = 1
        self.halfHeight = int(self.Height / 2)
        self.halfWidth = int(self.Width / 2)
        self.corners, self.ids, self.rejectedImgPoints = aruco.detectMarkers(self.frame, self.dictionary)
        aruco.drawDetectedMarkers(self.frame, self.corners, self.ids, (0,255,0))
        
        self.corners2, self.ids2, self.rejectedImgPoints2 = aruco.detectMarkers(self.frame, self.dictionary2)
        aruco.drawDetectedMarkers(self.frame, self.corners2, self.ids2, (0,255,0))
        

    def show(self):
        cv2.imshow("result", self.frame)

    def get_exist_Marker(self):
        return len(self.corners)

    def is_exist_marker(self, i):
        num = self.get_exist_Marker()
        if i >= num:
            return False
        else:
            return True


####################################
    def get_exist_Marker2(self):
        return len(self.corners2)

    def is_exist_marker2(self, i):
        num = self.get_exist_Marker2()
        if i >= num:
            return False
        else:
            return True
#################################

    def release(self):
        self.cap.release()

    def get_ARMarker_points(self, i):
        if self.is_exist_marker(i):
            return self.corners[i]

#############################################
    def get_ARMarker_points2(self, i):
        if self.is_exist_marker2(i):
            return self.corners2[i]
##############################################



    def get_average_point_marker(self, i):
        if self.is_exist_marker(i):
            points = self.get_ARMarker_points(i)
            print(points)
            points_reshape = np.reshape(np.array(points), (4, -1))
            G = np.mean(points_reshape, axis = 0)
            cv2.circle(self.frame, (int(G[0]), int(G[1])), 10, (255, 255, 255), 5)
            print(int(G[0]))
            print(int(G[1]))
            print("---------")
        if self.is_exist_marker2(i):
            points = self.get_ARMarker_points2(i)
            print(points)
            points_reshape = np.reshape(np.array(points), (4, -1))
            G = np.mean(points_reshape, axis = 0)
            cv2.circle(self.frame, (int(G[0]), int(G[1])), 10, (255, 255, 255), 5)
            print(int(G[0]))
            print(int(G[1]))
            print("---------")
            print("")
        
            return G[0], G[1]

myCap = AR(0)

pub_block_pose = rospy.Publisher('/block_pose', Pose, queue_size=10)
block_pose = Pose()

def callback_color_img(data):
    cv_color_image = bridge.imgmsg_to_cv2(data, "bgr8")
    myCap.find_ARMarker(cv_color_image)
    myCap.get_average_point_marker(0)
    
    myCap.show()
    if cv2.waitKey(1) > 0:
        myCap.release()
        cv2.destroyAllWindows()

def commander():

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        try:
            rate.sleep()
        except rospy.ROSException:
            print("restart simulation")

if __name__ == '__main__':
    rospy.init_node('commander', anonymous=True)
    rospy.Subscriber("/robot_camera/image_raw", Image, callback_color_img)
    commander()
    rospy.spin()