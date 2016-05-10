#!/usr/bin/env python
import math
import rospy
from robot_miniproj.msg import *
from geometry_msgs.msg import Twist, PoseWithCovariance, Quaternion, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
#from std_msgs.msg import String

current_angle = 0.0
current_x = 0.0
current_y = 0.0
previous_remaining_dist = 1000
remaining_dist = -1.0
center_offset = 5
pub = rospy.Publisher('/komodo_1/diff_driver/command', Twist, queue_size=10)

def fix_degree(data):
	global pub
	global previous_remaining_dist
	global remaining_dist
	
	twist = Twist()
	pub.publish(Twist())
	while remaining_dist > previous_remaining_dist + 1:
		print "remaining_dist: " + str(remaining_dist) + " previous_remaining_dist: " + str(previous_remaining_dist)
		for i in range(15):
			if (data.ranges[360 - i]) <= previous_remaining_dist:
				print "Fixing to the right"
				twist.angular.z = -0.2
				break
			elif (data.ranges[360 + i]) <= previous_remaining_dist:
				print "Fixing to the left"
				twist.angular.z = 0.2
				break

		pub.publish(twist)
		pub.sleep(0.1)

	pub.publish(Twist())
	# new_dist = Distance()
	# new_dist.distance = remaining_dist
	# drive(new_dist)

def update_remaining_dist(data):
	global remaining_dist
	global previous_remaining_dist
	
	remaining_dist = min(data.ranges[i] for i in range(360 - center_offset, 360 + center_offset))
	if (previous_remaining_dist < remaining_dist):
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
	# pub = rospy.Publisher('/komodo_1/diff_driver/command', Twist, queue_size=10)

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

	# for i in range(int(direction * data.radians * math.pi)):
	# # for i in frange(0, data.radians, 0.005):
	# 	print "Turning " + str(i + 1)
	# 	twist = Twist()
	# 	twist.angular.z = direction * 0.1
		
	# 	pub.publish(twist)
	# 	rospy.sleep(0.1)

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

	# arm_offset = -0.6

	# dest_x = 0.3 + data.distance * math.cos(data.radians) + arm_offset
	# dest_y = 0 + data.distance * math.sin(data.radians) + arm_offset

	odom_angle.unregister();

	# odom_pos = rospy.Subscriber('/komodo_1/diff_driver/odometry', Odometry, update_pos)
	# rospy.wait_for_message('/komodo_1/diff_driver/odometry', Odometry)

	laser = rospy.Subscriber('/komodo_1/scan', LaserScan, update_remaining_dist)
	rospy.wait_for_message('/komodo_1/scan', LaserScan)

	while remaining_dist > 0.63:
		print "Remaining distance: " + str(remaining_dist)
		twist = Twist()
	 	twist.linear.x = 0.05
	 	pub.publish(twist)
		rospy.sleep(0.1)
		rospy.wait_for_message('/komodo_1/scan', LaserScan)
	print
	# while abs(current_x) < abs(dest_x) and abs(current_y) < abs(dest_y):
	#  	twist = Twist()
	#  	twist.linear.x = 0.1
	#  	pub.publish(twist)
	# 	rospy.sleep(0.1)
	# 	rospy.wait_for_message('/komodo_1/diff_driver/odometry', Odometry)
	# for i in range(int(data.distance)):
	# 	print "Moving " + str(i + 1)
	# 	twist = Twist()
	# 	twist.linear.x = 0.1

	# 	pub.publish(twist)
	# 	rospy.sleep(0.1)

	pub.publish(Twist())

def wait_for_distance():
	rospy.sleep(60)
	rospy.init_node('mover', anonymous=True)
	rospy.Subscriber('advance', Distance, drive)
	rospy.spin()

if __name__ == '__main__':
	try:
		wait_for_distance()
		
	except rospy.ROSInterruptException:
		pub = rospy.Publisher('/komodo_1/diff_driver/command', Twist, queue_size=10)
		pub.publish(Twist())
