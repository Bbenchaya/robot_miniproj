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
	bridge = CvBridge()
	
	try:
		cv_image = bridge.imgmsg_to_cv2(image_message, "bgr8")
	except CvBridgeError as e:
		print(e)

	hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
	
	lower_red = np.array([0,50,50])
	upper_red = np.array([10,255,255])

	mask = cv2.inRange(hsv, lower_red, upper_red)

	res = cv2.bitwise_and(cv_image,cv_image, mask= mask)

	cv2.imshow("Mask", mask)
	cv2.imshow("Result", res)
	cv2.waitKey(3)
	rospy.sleep(10)


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