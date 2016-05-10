#!/usr/bin/env python

import rospy
from robot_miniproj.msg import *
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

shoulder= None
elevator = None
base_rotation = None
wrist = None
left_finger = None
right_finger = None
elbow = None

def armit(data):
    print "Arm is moving"
    rospy.sleep(int(abs(data.radians) * 20))
    rospy.sleep(int(abs(data.distance) * 15))
    global shoulder,elevator,base_rotation,wrist,left_finger,right_finger,elbow

    print 'Moving arm to initial position'
    for i in range(150):
        elevator.publish(1)
        shoulder.publish(1.7)
        base_rotation.publish(0)
        wrist.publish(0)
        left_finger.publish(-1)
        right_finger.publish(1)
        elbow.publish(0.4)
        rospy.sleep(0.1)

    print 'Lowering elevator to grip position'
    for i in range(200):
        elevator.publish(0)
        shoulder.publish(1.7)
        base_rotation.publish(0)
        wrist.publish(0)
        left_finger.publish(-1)
        right_finger.publish(1)
        elbow.publish(0.4)
        rospy.sleep(0.1)
     
    print 'Gripping object'
    for i in range(50):
        elevator.publish(0)
        shoulder.publish(1.7)
        base_rotation.publish(0)
        wrist.publish(0)
        left_finger.publish(0.33)
        right_finger.publish(-0.33)
        elbow.publish(0.4)
        rospy.sleep(0.1)

    print 'Moving to finish position'
    for i in range(400):
        elevator.publish(1)
        shoulder.publish(1.7)
        base_rotation.publish(0)
        wrist.publish(0)
        left_finger.publish(0.33)
        right_finger.publish(-0.33)
        elbow.publish(0.4)
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        rospy.init_node('armit', anonymous=True)
        shoulder = rospy.Publisher('/komodo_1/shoulder_controller/command', Float64, queue_size = 10)
        elevator = rospy.Publisher('/komodo_1/elevator_controller/command', Float64, queue_size=10)
        base_rotation = rospy.Publisher('/komodo_1/base_rotation_controller/command', Float64, queue_size = 10)
        wrist = rospy.Publisher('/komodo_1/wrist_controller/command', Float64, queue_size = 10)
        left_finger = rospy.Publisher('/komodo_1/left_finger_controller/command', Float64, queue_size = 10)
        right_finger = rospy.Publisher('/komodo_1/right_finger_controller/command', Float64, queue_size = 10)
        elbow = rospy.Publisher('/komodo_1/elbow2_controller/command', Float64, queue_size = 10)
        # print 'Raising elevator'
        for i in range(300):
            elevator.publish(1)
            shoulder.publish(0)
            base_rotation.publish(0)
            wrist.publish(0)
            left_finger.publish(-1)
            right_finger.publish(1)
            elbow.publish(0)
            rospy.sleep(0.1)
        print "Arm is now active"
        rospy.Subscriber('advance', Distance, armit)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
