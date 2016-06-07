#!/usr/bin/env python
import math
import rospy
from robot_miniproj.msg import *
from geometry_msgs.msg import Twist, PoseWithCovariance, Quaternion, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

current_angle = 0.0
current_x = 0.0
current_y = 0.0
previous_remaining_dist = 9
remaining_dist = -1.0
center_offset = 5
ask = rospy.Publisher('vision', Distance, queue_size=10)
pub = rospy.Publisher('/komodo_1/diff_driver/command', Twist, queue_size=10)
arm = rospy.Publisher('arm', Distance, queue_size=10)

def fix_degree(data):
	global pub
	global previous_remaining_dist
	global remaining_dist
	cells_to_fix = 0
	print "FIXING DEGREE"
	twist = Twist()
	pub.publish(Twist())
	# print "remaining_dist: " + str(remaining_dist) + " previous_remaining_dist: " + str(previous_remaining_dist)
	for i in range(30):
		if (data.ranges[360 - i]) <= previous_remaining_dist + 0.5:
			# print "Fixing to the right"
			twist.angular.z = -0.1
			cells_to_fix = i
			break
		elif (data.ranges[360 + i]) <= previous_remaining_dist + 0.5:
			# print "Fixing to the left"
			twist.angular.z = 0.1
			cells_to_fix = i
			break
		#print "to the right i see: " + str(data.ranges[360 - i]) + " and left: " + str(data.ranges[360 + i])

	#print "Fixing - moving by" + str(twist.angular.z)
	while (cells_to_fix > 0):
		print "Fixing loop: " + str(cells_to_fix) + " cells to fix"
		pub.publish(twist)
		rospy.sleep(0.2)
		cells_to_fix -= 1

	pub.publish(Twist())

def update_remaining_dist(data):
	global remaining_dist
	global previous_remaining_dist
	
	remaining_dist = min(data.ranges[i] for i in range(360 - center_offset, 360 + center_offset))
	# print "update: remaining_dist: " + str(remaining_dist) + " previous_remaining_dist: " + str(previous_remaining_dist)
	if (remaining_dist > previous_remaining_dist + 1):
		fix_degree(data)
	previous_remaining_dist = remaining_dist

def update_angle(data):
	global current_angle
	current_angle = math.asin(data.pose.pose.orientation.z) * 2

def drive(data):
	global current_angle
	global pub

	print "MOVING, Distance: " + str(data.distance)

	laser = rospy.Subscriber('/komodo_1/scan', LaserScan, update_remaining_dist)
	rospy.wait_for_message('/komodo_1/scan', LaserScan)

	while remaining_dist > 0.625:
		print "Remaining distance: " + str(remaining_dist)
		twist = Twist()
	 	twist.linear.x = 0.05
	 	pub.publish(twist)
		rospy.sleep(0.1)
		rospy.wait_for_message('/komodo_1/scan', LaserScan)
	print

	pub.publish(Twist())

def check_distance(data):
	print "check_distance", data.distance
	if data.distance == -1:
		return

	drive(data)

	arm.publish(Distance())
	rospy.signal_shutdown("Mover node finished its job.")

def init():
	rospy.init_node('mover', anonymous=True)

def search():
	global pub
	global ask
	global should_drive
	should_drive = rospy.Subscriber('advance', Distance, check_distance)

	rospy.sleep(20)
	while(1):
		ask.publish(Distance())
		print "Waiting on advance"
		rospy.wait_for_message('advance', Distance)

		# Vision node returned -1, continue search
		print "Resuming spin loop"
		spin = Twist()
		spin.angular.z = 0.1
		pub.publish(spin)
		rospy.sleep(0.1)
		pub.publish(Twist())
		#rospy.sleep(1)


if __name__ == '__main__':
	try:
		init()
		search()
		
	except rospy.ROSInterruptException:
		pub = rospy.Publisher('/komodo_1/diff_driver/command', Twist, queue_size=10)
		pub.publish(Twist())
