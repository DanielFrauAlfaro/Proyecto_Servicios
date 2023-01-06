#!/usr/bin/env python
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv_bridge
import numpy as np

class ObjectDetector():
	# class for detecting object: if in crop zone and for a miminum time without moving
	# means it has been put there for pick up
	def __init__(self):
		# initialization with publishers, subscribers and class constants
		rospy.init_node("object_detector")
		self.img_out_pub = rospy.Publisher("/object_detection/output",Image,queue_size=1)
		self.img_test_pub = rospy.Publisher("/object_detection/test",Image,queue_size=1)
		self.obj_pub = rospy.Publisher("/object_detection/detection",String,queue_size=1)
		self.bridge = cv_bridge.CvBridge()
		self.image_sub = rospy.Subscriber("/robot_camera/image_raw",Image,self.image_callback)
		self.rate = rospy.Rate(10)
		self.frame = np.array([[]],dtype="uint8")
		self.first_frame = rospy.wait_for_message("/robot_camera/image_raw",Image)
		self.first_frame = self.bridge.imgmsg_to_cv2(self.first_frame,"bgr8")
		self.r,self.c,_ = self.first_frame.shape
		self.first_frame = self.first_frame[int(round(self.r*2.0/3)):self.r,0:self.c] # 33% below screen
		self.object_first_frame = True
		self.prev_x = 0
		self.prev_y = 0
		self.margin = 0.2 # 20%
		self.t_appearance = 0
		self.publish_once = True
		
	def image_callback(self,data):
		# callback to retrieve image from camera topic
		try:
			frame = self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
			self.frame = frame
		# error in bridge
		except cv_bridge.CvBridgeError:
			print("Error cv bridge")
	
	def process_image(self):
		# main loop processing per frame
		# copy frame recieved
		frame = self.frame
		# crop frame to same as first frame
		frame = frame[int(round(self.r*2.0/3)):self.r,0:self.c] # 33% below screen
		# subtract background
		diff = cv2.subtract(self.first_frame,frame)
		# process noise
		gray = cv2.cvtColor(diff,cv2.COLOR_BGR2GRAY)
		_,thresh = cv2.threshold(gray,20,255,cv2.THRESH_BINARY)
		blur = cv2.medianBlur(thresh,5)
		erode = cv2.erode(blur,(3,3),iterations=2)
		# show for reference
		self.img_test_pub.publish(self.bridge.cv2_to_imgmsg(erode,"passthrough"))
		# get contours and filter
		contours,_ = cv2.findContours(erode,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2:]
		if len(contours) > 0:	# if there is at least 1 contour
			# find biggest contour
			c = max(contours,key=cv2.contourArea)
			# get bounding rect
			x,y,w,h = cv2.boundingRect(c)
			# filter min size (area)
			if cv2.contourArea(c) >= 1000:
				# first detection of object
				if self.object_first_frame:
					print("object first frame")
					self.prev_x = x
					self.prev_y = y
					self.object_first_frame = False
					self.t_appearance = rospy.Time.now() # set first appearance time
				else:
					# object in same position
					if (x >= self.prev_x-self.prev_x*self.margin and x <= self.prev_x+self.prev_x*self.margin) and \
					   (y >= self.prev_y-self.prev_y*self.margin and y <= self.prev_y+self.prev_y*self.margin):
							print("object same position")
							# object stayed put -> verify time
							print("resta",rospy.Time.now()-self.t_appearance)
							print("Duration(5)",rospy.Duration(5))
							if rospy.Time.now()-self.t_appearance >= rospy.Duration(5): # 5 seconds
								print("same object for over 5 seconds now")
								if self.publish_once:
									print("### PUBLISHED PICK ###")
									self.obj_pub.publish("pick")
									self.publish_once = False
									# NOTE: object must leave the scene to publish to pick another object
					else:
						print("object moved")
						# object moved -> track position
						self.prev_x = x
						self.prev_y = y
						self.t_appearance = rospy.Time.now()
				cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
		else:
			print("no object in scene")
			# object left scene -> restart for new object
			self.object_first_frame = True
			self.publish_once = True
		# show for tracking object reference
		self.img_out_pub.publish(self.bridge.cv2_to_imgmsg(frame,"bgr8"))

	def main(self):
		# main execution of image processing
		print("running main...")
		#self.initial_frame = rospy.wait_for_message("/video_source/raw",Image)
		while not rospy.is_shutdown():
			# while ROS node active
			try:
				self.process_image()
			except Exception as e:
				print("wait for image source")
				print(e)
			self.rate.sleep()

if __name__ == "__main__":
	# object detector initialization
	try:
		od = ObjectDetector()
		od.main()
	# interrupt node execution
	except (rospy.ROSInterruptException,rospy.ROSException("Topic interrupted")):
		pass
