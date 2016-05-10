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

global first_frame
global got_first_frame

sub = None
first_frame = []
got_first_frame = False
first_move = True
start_index = 721
end_index = -1
move = rospy.Publisher('advance', Distance, queue_size=10)


def med(lst):
	tol = 5
	med_ =  median(array(lst))
	new_lst = [i for i in lst if  abs(i - med_) < tol]
	return new_lst

def callback(data):
	global sub
	global first_frame
	global got_first_frame
	global first_move
	global start_index
	global end_index
	global move


	if not first_move:
		rospy.signal_shutdown("Vision node ended its job.")
		rospy.sleep(100)

 	if not got_first_frame:
 		first_frame = data.ranges
 		got_first_frame = True
 	else:
 		noise = 3
 		num_of_changed_cells = 0

 		start_index = 721
		end_index = -1

		lst = []
 		for i,r in enumerate(data.ranges):
 			if abs(first_frame[i] - r) > 2.5 and r < 0.95 * data.range_max:
 				lst.append(i)
 				# print "Changed cell: " + str(i) + " with range " + str(r) + " as opposed to " + str(first_frame[i])
 				num_of_changed_cells += 1
 				# if i < start_index:
 				# 	start_index = i
 				# if i > end_index:
 				# 	end_index = i

 		lst = med(lst)
 		if len(lst) > 1:
	 		start_index = lst[0]
	 		end_index = lst[-1]
	 		print num_of_changed_cells

	 		if num_of_changed_cells > noise: # Found significant change
		 		dist = Distance()
		 		mid_index = (start_index + end_index)/2

		 		dist.distance = data.ranges[mid_index]
		 		dist.radians = (mid_index/4 - 90) * math.pi / 180.0

				first_move = False
	 			print "MOVING!!!"
				move.publish(dist)

	sub.unregister()
	rospy.sleep(1)
	sub = rospy.Subscriber("/komodo_1/scan", LaserScan, callback) 

def listener():
	global sub
	rospy.sleep(30)
	print "Depth node is now active"
	sub = rospy.Subscriber("/komodo_1/scan", LaserScan, callback)

if __name__ == '__main__':
	try:
		first_frame = None
		got_first_frame = False
		rospy.init_node('vision', anonymous=True)
 		listener()
 		rospy.spin()

 	except rospy.ROSInterruptException:
 		first_frame = None
 		got_first_frame = False