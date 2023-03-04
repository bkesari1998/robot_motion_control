import rospy
from robot_motion_control import RobotMotionControl
from geometry_msgs.msg import Twist, Vector3

class Turtlebot(object):
    """
    Interface for controlling velocity of Turtlebot2 with ROS.
    """


    def __init__(self):

        rospy.init_node('turtlebot_test', anonymous=True)

        self.motion_control = RobotMotionControl('/cmd_vel_mux/input/navi', 1, 1, 3, 1)

        self.motion_control.velocity_cmd(Twist(linear=Vector3(x=0.1, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)), duration=5)

        while not rospy.is_shutdown():
            rospy.spin()