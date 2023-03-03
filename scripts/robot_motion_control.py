import rospy
from geometry_msgs.msg import Twist
import rostopic

import threading
import queue


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


        self.__pub_lock = threading.Lock()
        self.__overwrite_lock = threading.Lock()
        self.__overwrite = False

        self.velocity_topic = velocity_topic

        self.linear_dof = linear_dof
        self.angualr_dof = angular_dof
        
        self.__rate_lock = threading.Lock()
        self.rate = rate
        
        self.queue_size = queue_size

        self.__q = queue()

    
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

        with self.__overwrite_lock:
            self.__overwrite = True

        with self.__pub_lock:
            self.__velocity_pub = rospy.Publisher(self.velocity_topic, Twist, queue_size=self.queue_size)

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
        
        while self.__rate_lock.locked():
            rospy.spin()

        return self._rate
    
    @rate.setter
    def rate(self, rate: float) -> None:
        """
        Setter for rate.

        @param rate [float]: Hz of velocity publisher (greater than 0)

        @raise ValueError: if rate is less than or equal to 0
        """

        if rate <= 0:
            raise ValueError(f"Parameter rate must be positive, but got {rate}")
        
        with self.__overwrite_lock:
            self.__overwrite = True

        with self.__rate_lock:
            self._rate = rate
            self.__rate = rospy.Rate(self.rate)
    
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
        
        with self.__overwrite_lock:
            self.__overwrite = True

        with self.__pub_lock:
            self.__velocity_pub = rospy.Publisher(self.velocity_topic, Twist, queue_size=self.queue_size)

    def __publish_velocity(self, msg: Twist, 
                                 duration: rospy.Duration = None) -> None:
        """
        Thread function to publish velocity commands.

        @param msg [Twist]: Twist velocity message
        @param duration [rospy.Duration]: Duration to publish message.
        """

        with self.__pub_lock:

            rate = self.__rate
            
            if duration == None:
                while True:

                    with self.__overwrite_lock:
                        if self.__overwrite:
                            break

                    self.__velocity_pub.publish(msg)
                    
                    # If rate is updated while thread is running,
                    # change may not be seen immediately
                    if self.__rate_lock.locked():
                        rate.sleep()
                    else:
                        rate = self.__rate()

                    rate.sleep()
            else:
                start = rospy.Time.now()

                while (rospy.Time.now() - start) < duration:
                    
                    with self.__overwrite_lock:    
                        if self.__overwrite:
                            break

                    self.__velocity_pub.publish(msg)

                    # If rate is updated while thread is running,
                    # change may not be seen immediately
                    self.__rate.sleep()

    def __published_queued_velocity(self) -> None:
        """
        Thread function to publish queued velocity commands.
        """

        while True:
            velocity, duration = self.__q.get()

            self.__publish_velocity(velocity, duration)

            self.__q.task_done()


    def velocity_cmd(self, velocity: Twist, 
                         duration: float = None,
                         overwrite: bool = False) -> None:
        """
        Sends a command to robot to travel at a velocity for a specified duration.
        If overwrite is False, the command is queue'd behind other assigned velocity
        commands. If overwrite is True, any currently running velocity command is 
        canceled, the queue is erased, and the new velocity command is sent to the robot.
        """

        if overwrite:
            with self.__overwrite_lock:
                self.__overwrite = True

            with self.__q.mutex:
                self.__q.queue.clear()
            
            thread = threading.Thread(target=self.__publish_velocity, args=(velocity, duration))
            thread.start()
        
        else:
            self.__q.put((velocity, duration))

            if self.__q.qsize() == 1:
                thread = threading.Thread(target=self.__published_queued_velocity)
                thread.start()
    