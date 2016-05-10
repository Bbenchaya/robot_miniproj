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
pub = rospy.Publisher('/komodo_1/diff_driver/command', Twist, queue_size=10)

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
		rospy.sleep(0.1)
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

def update_pos(data):
	global current_y
	global current_x

	current_x = data.pose.pose.position.x
	current_y = data.pose.pose.position.y


def frange(x, y, jump):
	while x < y:
		yield x
		x+=jump

def drive(data):
	global current_angle
	global pub

	print "MOVING, Distance: " + str(data.distance) + " Degrees: " + str(data.radians)

	for i in range(int(27)):
		twist = Twist()
		twist.linear.x = 0.1

		pub.publish(twist)
		rospy.sleep(0.1)

	pub.publish(Twist())

	direction = 1
	if (data.radians < 0):
		direction = -1

	odom_angle = rospy.Subscriber('/komodo_1/diff_driver/odometry', Odometry, update_angle)

	rospy.wait_for_message('/komodo_1/diff_driver/odometry', Odometry)

	while direction * current_angle < direction * (data.radians - (direction * 0.002)):
		print "Current angle: " + str(current_angle)
		twist = Twist()
		twist.angular.z = direction * 0.05
		
		pub.publish(twist)
		rospy.sleep(0.1)
		rospy.wait_for_message('/komodo_1/diff_driver/odometry', Odometry)
	print
	pub.publish(Twist())

	odom_angle.unregister();

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

def wait_for_distance():
	rospy.sleep(30)
	rospy.init_node('mover', anonymous=True)
	rospy.Subscriber('advance', Distance, drive)
	rospy.spin()

if __name__ == '__main__':
	try:
		wait_for_distance()
		
	except rospy.ROSInterruptException:
		pub = rospy.Publisher('/komodo_1/diff_driver/command', Twist, queue_size=10)
		pub.publish(Twist())
