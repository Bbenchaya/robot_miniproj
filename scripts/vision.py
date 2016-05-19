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
advance = rospy.Publisher('advance', Distance, queue_size=10)

def callback(image_message):
	global advance
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

	#cv2.imshow("Mask", mask)
	#cv2.imshow("Result", res)
	#cv2.waitKey(3)
	#rospy.sleep(10)
	dist = Distance()
	found = 0
	#print type(mask)
	#print image_message.data[image_message.width+1]
	height, width = mask.shape

	for i in range(height):
		for j in range(width * 4/9, width * 5/9):
			#if np.array_equal(np.array(mask[i,j]),np.array([255,255,255]))==0:
			if mask[i,j] != 0: 
				dist.distance = 1
				advance.publish(dist)
				found = 1
				rospy.signal_shutdown("Mover node finished its job.")

	if found == 0:
		dist.distance =- 1
		advance.publish(dist)


def listener(data):
	global sub

	sub = rospy.Subscriber("/komodo_1/Asus_Camera/rgb/image_raw", Image, callback)
	rospy.wait_for_message('/komodo_1/Asus_Camera/rgb/image_raw', Image)
	

if __name__ == '__main__':
	print "****************************************"
	try:
		rospy.sleep(10)
		print "camera node is now active"	
		first_frame = None
		got_first_frame = False
		rospy.init_node('vision', anonymous=True)
 		sub_advance = rospy.Subscriber('vision', Distance, listener)
 		rospy.sleep(1)
 		rospy.spin()

 	except rospy.ROSInterruptException:
 		first_frame = None
 		got_first_frame = False