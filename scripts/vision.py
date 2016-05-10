#!/usr/bin/env python
import math
import rospy
import cv2
import sys
from robot_miniproj.msg import *
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from numpy import median,array
import numpy as np


sub = None


def callback(image_message):
	global sub
	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(image_message,"bgr8")
	# Convert BGR to HSV
	hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
	# define range of blue color in HSV
	# lower mask (0-10)
	lower_red = np.array([0,50,50])
	upper_red = np.array([10,255,255])
	# Threshold the HSV image to get only blue colors
	mask = cv2.inRange(hsv, lower_red, upper_red)

	# Bitwise-AND mask and original image
	res = cv2.bitwise_and(cv_image,cv_image, mask= mask)
	print "here!!!!!!!!!!"
	cv2.imshow('frame',cv_image)
	cv2.imshow('mask',mask)
	cv2.imshow('res',res)
	#cv2.destroyAllWindows()
	

def listener():
	global sub
	rospy.sleep(30)
	print "camera node is now active"
	sub = rospy.Subscriber("/komodo_1/Asus_Camera/rgb/image_raw", Image, callback)

if __name__ == '__main__':
	print "****************************************"
	try:
		first_frame = None
		got_first_frame = False
		rospy.init_node('vision', anonymous=True)
 		listener()
 		rospy.spin()

 	except rospy.ROSInterruptException:
 		first_frame = None
 		got_first_frame = False