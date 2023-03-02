import rospy
from geometry_msgs.msg import Twist
import rostopic

import threading


class RobotMotionControl(object):
    """
    Interface for controlling velocity of mobile robots with ROS.
    Velocity control uses the Twist message type.

    This class is must be instanciated within a Python file 
    running a ROS node and after rospy.init_node(...) is called.
    """

    def __init__(self, velocity_topic: str, 
                       linear_dof: int = 2, 
                       angular_dof: int = 1) -> None:
        """
        Initializes an instance of RobotMotionControl class.

        @param velocity_topic [str]: ROS topic to control robot with Twist messages.
        @param linear_dof [int]: linear degrees of freedom (between 0-3)
        @param angular_dof [int]: angular degrees of freedom (between 0-3)
        """

        super.__init__(self)

        self.velocity_topic = velocity_topic
        self.linear_dof = linear_dof
        self.angualr_dof = angular_dof

    
    @property
    def velocity_topic(self) -> str:
        """
        Getter for velocity_topic.

        @return [str]: string holding topic name.
        """

        return self._velocity_topic
    
    @velocity_topic.setter
    def velocity_topic(self, topic: str) -> None:
        """
        Setter for velocity_topic.
        
        @param topic [str]: ROS topic to control robot with Twist messages.

        @raise NameError: if topic is not found.
        """

        if topic not in rostopic.get_topic_list():
            raise NameError(f"Topic '{topic}' was not found.")
        
        self._velocity_control = topic

    @property
    def linear_dof(self) -> int:
        """
        Getter for linear_dof.

        @return [int]: int representing linear degrees of freedom of the robot.
        """

        return self._linear_dof
    
    @linear_dof.setter
    def linear_dof(self, dof: int) -> None:
        """
        Setter for linear_dof.

        @param dof [int]: linear degrees of freedom of the robot (between 0-3).

        @raise ValueError: if dof less than 0 or greater than 3.
        """

        if not 0 <= dof <= 3:
            raise ValueError(f"Parameter 'dof' must be between 0-3 but got {dof}")
        
        self._linear_dof = dof
        

    @property
    def angular_dof(self) -> int:
        """
        Getter for angular_dof.

        @return [int]: int representing angular degrees of freedom of the robot.
        """

        return self._linear_dof
    
    @angular_dof.setter
    def angualr_dof(self, dof: int) -> None:
        """
        Setter for angular_dof.

        @param dof [int]: angular degrees of freedom of the robot (between 0-3).

        @raise ValueError: if dof less than 0 or greater than 3.
        """

        if not 0 <= dof <= 3:
            raise ValueError(f"Parameter 'dof' must be between 0-3 but got {dof}")
        
        self._angular_dof = dof
        
