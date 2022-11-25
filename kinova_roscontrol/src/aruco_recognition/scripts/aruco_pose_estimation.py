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

    def __init__(self, videoPort, cameraMatrix, distortionCoefficients):
        self.cameraMatrix = cameraMatrix
        self.distortionCoefficients = distortionCoefficients
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

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

    def release(self):
        self.cap.release()

    def get_ARMarker_points(self, i):
        if self.is_exist_marker(i):
            return self.corners[i]

    def get_average_point_marker(self, i):
        if self.is_exist_marker(i):
            points = self.get_ARMarker_points(i)
            points_reshape = np.reshape(np.array(points), (4, -1))
            G = np.mean(points_reshape, axis = 0)
            cv2.circle(self.frame, (int(G[0]), int(G[1])), 10, (255, 255, 255), 5)
            return G[0], G[1]

    def get_ARMarker_pose(self, i):
        if self.is_exist_marker(i):
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(self.corners[i], arucoMarkerLength, self.cameraMatrix, self.distortionCoefficients)
            self.frame = aruco.drawAxis(self.frame, self.cameraMatrix, self.distortionCoefficients, rvec, tvec, 0.1)
            return rvec, tvec

    def get_degrees(self, i):
        if self.is_exist_marker(i):
            rvec, tvec, = self.get_ARMarker_pose(i)

            Xtemp = tvec[0][0][0]
            Ytemp = tvec[0][0][1]*math.cos(-pi/4) - tvec[0][0][2]*math.sin(-pi/4)
            Ztemp = tvec[0][0][1]*math.sin(-pi/4) + tvec[0][0][2]*math.cos(-pi/4)

            Xtemp2 = Xtemp - 0.4
            Ytemp2 = Ytemp - 0.5
            Ztemp2 = Ztemp - 0.4

            Xtarget = -Xtemp2
            Ytarget = -Ztemp2
            Ztarget = -Ytemp2

            print(f"(X, Y, Z) : {Xtarget}, {Ytarget}, {Ztarget}")

            (roll_angle, pitch_angle, yaw_angle) =  rvec[0][0][0]*180/pi, rvec[0][0][1]*180/pi, rvec[0][0][2]*180/pi
            if pitch_angle < 0:
                roll_angle, pitch_angle, yaw_angle = -roll_angle, -pitch_angle, -yaw_angle
            return Xtarget, Ytarget, Ztarget, roll_angle, pitch_angle, yaw_angle

camera_matrix = np.matrix([[381.36246688113556, 0.0, 320.5], [0.0, 381.36246688113556, 240.5], [0.0, 0.0, 1.0]])
distortion = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
myCap = AR(0, camera_matrix, distortion)

pub_block_pose = rospy.Publisher('/block_pose', Pose, queue_size=10)
block_pose = Pose()

def callback_color_img(data):
    cv_color_image = bridge.imgmsg_to_cv2(data, "bgr8")
    myCap.find_ARMarker(cv_color_image)
    myCap.get_average_point_marker(0)
    try:
        Xtarget, Ytarget, Ztarget, roll_angle, pitch_angle, yaw_angle = myCap.get_degrees(0)
        block_pose.position.x = Xtarget
        block_pose.position.y = Ytarget
        block_pose.position.z = Ztarget
        pub_block_pose.publish(block_pose)
    except:
        print("No marker detected!")
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