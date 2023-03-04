#!/bin/python3

import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':

    rospy.init_node("test_node")

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(3)

    msg = Twist()
    msg.linear.x = 0.2

    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
