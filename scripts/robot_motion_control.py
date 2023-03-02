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
                       angular_dof: int = 1,
                       rate: float = 3,
                       queue_size: int = 1) -> None:
        """
        Initializes an instance of RobotMotionControl class.

        @param velocity_topic [str]: ROS topic to control robot with Twist messages.
        @param linear_dof [int]: linear degrees of freedom (between 0-3)
        @param angular_dof [int]: angular degrees of freedom (between 0-3)
        @param rate [float]: Hz to publish velocity messages (greater than 0)
        @param queue_size [int]: Size of velocity publisher queue (greater than 0)
        """

        super.__init__(self)

        self.velocity_topic = velocity_topic
        self.linear_dof = linear_dof
        self.angualr_dof = angular_dof
        self.rate = rate
        self.queue_size = queue_size

        # Initialize ROS publisher
        self.velocity_pub = rospy.Publisher(self.velocity_topic, Twist, queue_size=self._queue_size)

        self.__overwrite = False

        self.pub_lock = threading.Lock()
        self.overwrite_lock = threading.Lock()
 

    
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
        
        self._velocity_topic = topic

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

    @property
    def rate(self) -> int:
        """
        Getter for rate.

        @return [int]: Hz of velocity publisher.
        """

        return self._rate.hz
    
    @rate.setter
    def rate(self, rate: float) -> None:
        """
        Setter for rate.

        @param rate [float]: Hz of velocity publisher (greater than 0)

        @raise ValueError: if rate is less than or equal to 0
        """

        if rate <= 0:
            raise ValueError(f"Parameter rate must be positive, but got {rate}")
        
        self._rate = rospy.Rate(rate)

        
    @property
    def queue_size(self) -> int:
        """
        Getter for queue_size.

        @return [int]: velocity publisher queue size.
        """

        return self._queue_size

    @queue_size.setter
    def queue_size(self, size: int) -> None:
        """
        Setter for queue_size.

        @param size [int]: size of velocity publisher queue (greater than 0)

        @raise ValueError: if size less than 1.
        """

        if size < 1:
            raise ValueError(f"Parameter size must be greater than 0 but got {size}")
        
        self._queue_size = size

    def __publish_velocity(self, msg: Twist, 
                                 duration: rospy.Duration = None) -> None:
        """
        Thread function to publish velocity Twist msg on velocity topic.

        @param msg [Twist]: Twist velocity message
        @param duration [rospy.Duration]: Duration to publish message.
        """

        with self.pub_lock:
            
            if duration == None:
                while True:

                    with self.overwrite_lock():
                        if self.__overwrite:
                            break

                    self.velocity_pub.publish(msg)
                    self.rate.sleep()
            else:
                start = rospy.Time.now()

                while (rospy.Time.now() - start) < duration:
                    
                    with self.overwrite_lock():    
                        if self.__overwrite:
                            break

                    self.velocity_pub.publish(msg)
                    self.rate.sleep()


