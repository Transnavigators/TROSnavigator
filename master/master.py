#!/usr/bin/env python

import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist



class Master:
    """Master Node
    
    Processes commands from alexa and sends appropriate actions to arduino_motor
    
    Uses PID control

    """
    # set up constants
    def __init__(self):
        # create a new node
        rospy.init_node("master", anonymous=True)
        rospy.Subscriber("odom", Odometry, self.odometry_callback)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=50)
        self.action_server = actionlib.SimpleActionServer('move_base', MoveBaseAction, self.alexa_callback)
        
        # get publish rate and PID constants
        self.rate = rospy.get_param("~rate", 100)
        self.Kp = rospy.get_param("~position_contant", 1.0)
        self.Ki = rospy.get_param("~derivative_contant", 1.0)
        self.Kd = rospy.get_param("~integral_contant", 1.0)
        
        # current positions
        self.desired.position = 0.0
        self.desired.orientation = 0.o
        
        # desired positions
        self.current.position = 0.0
        self.current.orientation = 0.0
        
    def begin(self):
        """Sends velocities to the motors depending on the current and desired positions and orientations
        
        """
        r = rospy.Rate(self.rate):
        
        # integral sum
        self.forward_integral = 0.0
        self.orientation_integral = 0.0
        
        # old error
        old_forward_error = 0.0
        old_orientation_error = 0.0
        
        # old time
        old_time = 0.0
        
        while not rospy.is_shutdown():
            # get current time
            new_time = rospy.get_time()
            time_diff = new_time - old_time
            old_time = new_time
            
            # generate message
            msg = Twist()
            
            # current velocity
            forward_vel = 0.0
            rotational_vel = 0.0
            
            
            ###### To test ignore PID output ########################
            
            # http://robotsforroboticists.com/pid-control/
            
            # forward PID
            forward_error = self.desired.position - self.current.position
            forward_integral = forward_integral + (forward_error*time_diff)
            forward_derivative = forward_error - old_forward_error)/time_diff
            forward_output = Kp*forward_error+Ki*forward_integral + Kd*forward_derivative
            
            # orientation PID
            orientation_error = self.desired.orientation - self.current.orientation
            orientation_integral = orientation_integral + (orientation_error*time_diff)
            orientation_derivative = orientation_error - old_orientation_error)/time_diff
            orientation_output = Kp*orientation_error+Ki*orientation_integral + Kd*orientation_derivative
            
            # set old errors
            old_forward_error = forward_error
            old_orientation_error = orientation_error
            
            ##################################################
            
            
            # lets do simple positional control for now
            forward_vel = Kp*(self.desired.position - self.current.position)
            rotational_vel = Kp*(self.desired.orientation - self.current.orientation)
            
            # fill in values for the Twist
            msg.linear = Vector3(forward_vel, 0, 0)
            msg.angular = Vector3(0, 0, rotational_vel)
           
            # publish the message
            self.pub.publish(msg)
            
            # sleep
            r.sleep()
           
        
    def odometry_callback(self, msg):
        """Updates the current position and orientation 
        
        The message definition can be found here: http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html
        
        Args:
            msg (nav_msgs.msg.Odometry): The current position

        """
        
        self.current.position = msg.pose.point.x
        self.current.orientation = math.asin(msg.pose.orientation.z) * 2
        
    def alexa_callback(self, goal):
        """Sets the desired position and orientation 
        The message definition can be found here: http://docs.ros.org/api/move_base_msgs/html/action/MoveBase.html
        
        Args:
            msg (move_base_msgs.msg.MoveBaseAction): The desired position

        """
        
        self.desired.position = goal.target_pose.pose.position.x
        self.desired.orientation = math.asin(goal.target_pose.pose.orientation.z) * 2
        

if __name__ == "__main__":
    try:
        controller = Master()
        controller.begin()
    except rospy.ROSInterruptException:
        pass
        
    