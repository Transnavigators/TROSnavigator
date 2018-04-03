#!/usr/bin/env python

import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3


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
        self.action_server = actionlib.SimpleActionServer('move_base', MoveBaseAction, self.alexa_callback, False)

        # get publish rate and PID constants
        self.rate = rospy.get_param("~rate", 100)
        # self.Kp = rospy.get_param("~position_contant", 1.0)
        # self.Ki = rospy.get_param("~derivative_contant", 1.0)
        # self.Kd = rospy.get_param("~integral_contant", 1.0)

        self.recv_msg = False

        # current positions
        self.current_position_x = 0.0
        self.current_position_y = 0.0
        self.current_orientation = 0.0

        # desired positions
        self.desired_position_x = 0.0
        self.desired_position_y = 0.0
        self.desired_orientation = 0.0
        self.first_run = True
        self.action_server.start()

    def begin(self):
        """Sends velocities to the motors depending on the current and desired positions and orientations
        
        """
        r = rospy.Rate(self.rate)

        # # integral sum
        # self.forward_integral = 0.0
        # self.orientation_integral = 0.0

        # # old error
        # old_forward_error = 0.0
        # old_orientation_error = 0.0

        # # old time
        # old_time = 0.0

        while not rospy.is_shutdown():
            # # get current time
            # new_time = rospy.get_time()
            # time_diff = new_time - old_time
            # old_time = new_time

            # generate message
            msg = Twist()

            # current velocity
            forward_vel = 0.0
            rotational_vel = 0.0

            ###### To test ignore PID output ########################

            # http://robotsforroboticists.com/pid-control/

            # # forward PID
            # forward_error = self.desired.position - self.current.position
            # forward_integral = forward_integral + (forward_error*time_diff)
            # forward_derivative = forward_error - old_forward_error)/time_diff
            # forward_output = Kp*forward_error+Ki*forward_integral + Kd*forward_derivative

            # # orientation PID
            # orientation_error = self.desired_orientation - self.current_orientation
            # orientation_integral = orientation_integral + (orientation_error*time_diff)
            # orientation_derivative = orientation_error - old_orientation_error)/time_diff
            # orientation_output = Kp*orientation_error+Ki*orientation_integral + Kd*orientation_derivative

            # # set old errors
            # old_forward_error = forward_error
            # old_orientation_error = orientation_error

            ##################################################

            # lets do simple positional control for now
            # if abs(self.desired_position_x - self.current_orientation)
            # forward_vel = 0.5*((self.desired_position_x/math.cos(self.current_orientation)+self.desired_position_y/math.sin(self.current_orientation))
            #                   - (self.current_position_x/math.cos(self.current_orientation) + self.current_position_x/math.cos(self.current_orientation)))

            
            dist = math.sqrt((self.desired_position_x-self.current_position_x)**2+(self.desired_position_y-self.current_position_y)**2)
            
            # get delta orientation in the range -pi to pi so we always take the short way around
            # orientation_err = (self.desired_orientation - self.current_orientation)
            # if orientation_err > math.pi:
                # orientation_err = orientation_err - 2*math.pi
            # elif orientation_err < -math.pi:
                # orientation_err = orientation_err + 2*math.pi
            

            # we are trying to move forward
            if dist >= 0.05: # 5 cm
                # update desired orientation to point in the correct direction
                self.desired_orientation = math.atan2(self.desired_position_y - self.current_position_y, self.desired_position_x - self.current_position_x)
                
                rotational_vel = 0.3*orientation_err
                
                
                # make sure we are in the correct orientation before moving forward
                if abs(orientation_err) < 0.087: # 5 degrees
                    forward_vel = min(1.1, 0.2*dist)
            
            # turn command
            else:
                # if self.recv_msg:
                    # self.action_server.set_succeeded()
                    # self.recv_msg = False
                # continue
            
                # orientation deadband if we are doing a rotate command
                if abs(orientation_err) >= 0.087: # 5 degrees
                    rotational_vel = 0.3*orientation_err
                 
            
            
            
            rospy.loginfo_throttle(1, "Dist=%f Forward vel=%f Rotational vel=%f" % (dist, forward_vel, rotational_vel))
            
            # fill in values for the Twist
            msg.linear = Vector3(forward_vel, 0, 0)
            msg.angular = Vector3(0, 0, rotational_vel)
           


            rospy.loginfo_throttle(1, "Desired Position: (%f,%f,%f) Current Position: (%f,%f,%f) Sending Velocity: (%f,%f)" % (self.desired_position_x,self.desired_position_y,self.desired_orientation,self.current_position_x,self.current_position_y,self.current_orientation,forward_vel,rotational_vel))

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

        self.current_position_x = msg.pose.pose.position.x
        self.current_position_y = msg.pose.pose.position.y
        self.current_orientation = math.asin(msg.pose.pose.orientation.z) * 2
        if self.first_run:
            self.desired_position_x = self.current_position_x
            self.desired_position_y = self.current_position_y
            self.desired_orientation = self.current_orientation
            self.first_run = False

    def alexa_callback(self, goal):
        """Sets the desired position and orientation 
        The message definition can be found here: http://docs.ros.org/api/move_base_msgs/html/action/MoveBase.html
        
        Args:
            goal (move_base_msgs.msg.MoveBaseAction): The desired position

        """

        self.desired_orientation = self.current_orientation + math.asin(goal.target_pose.pose.orientation.z) * 2
        
        # keep theta between -pi and pi
        if self.desired_orientation > math.pi:
            self.desired_orientation = self.desired_orientation - 2*math.pi
        elif self.desired_orientation < -math.pi:
            self.desired_orientation = self.desired_orientation + 2*math.pi
        
        # update desired positions
        self.desired_position_x = self.current_position_x + goal.target_pose.pose.position.x*math.cos(self.current_orientation)
        self.desired_position_y = self.current_position_y + goal.target_pose.pose.position.x*math.sin(self.current_orientation)
        
        
        rospy.loginfo("Desired position offset %f", goal.target_pose.pose.position.x)

        self.action_server.set_succeeded()
        self.recv_msg = True


if __name__ == "__main__":
    try:
        controller = Master()
        controller.begin()
    except rospy.ROSInterruptException:
        pass

