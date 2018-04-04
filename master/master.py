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

        self.linear_accel = float(rospy.get_param("~linear_accel",0.2))
        self.rot_accel = float(rospy.get_param("~rot_accel", 1))

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

        self.last_forward_vel = 0
        self.last_rot_vel = 0
        self.current_speed = 0

        self.rotational_vel = 0
        self.forward_vel = 0

        self.is_turning = False

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
        old_time = rospy.get_time()

        while not rospy.is_shutdown():
            # # get current time
            new_time = rospy.get_time()
            time_diff = new_time - old_time
            old_time = new_time

            # generate message
            msg = Twist()

            # current velocity
            self.forward_vel = 0.0
            self.rotational_vel = 0.0

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

            # update desired orientation to point in the correct direction
            if dist >= 0.05 and not self.is_turning:  # 5 cm
                self.desired_orientation = math.atan2(self.desired_position_y - self.current_position_y,
                                                      self.desired_position_x - self.current_position_x)
                rospy.loginfo_throttle(1, "updating desired orientation %f" % self.desired_orientation)
            else:
                self.desired_position_x = self.current_position_x
                self.desired_position_y = self.current_position_y
            # get delta orientation in the range -pi to pi so we always take the short way around
            orientation_err = self.desired_orientation - self.current_orientation
            if orientation_err > math.pi:
                orientation_err = -(orientation_err - 2*math.pi)
                rospy.loginfo_throttle(1, "Wrapping orientation error from %f to %f" % (orientation_err+2*math.pi, orientation_err))
            elif orientation_err < -math.pi:
                orientation_err = -(orientation_err + 2*math.pi)
                rospy.loginfo_throttle(1, "Wrapping orientation error from %f to %f" % (orientation_err-2*math.pi, orientation_err))

            # we are trying to move forward
            if dist >= 0.05: # 5 cm
                # rotate toward the correct location
                # if orientation_err>0.875:
                    # self.rotational_vel = min(0.875,orientation_err)
                # else:
                    # self.rotational_vel = 0.3*orientation_err

                self.rotational_vel = 0.5*orientation_err
                rospy.loginfo_throttle(1, "Scaling rotational velocity=%f=%f*0.5" % (self.rotational_vel,orientation_err))

                # make sure we are in the correct orientation before moving forward
                if abs(orientation_err) < 0.043: # 5 degrees/2
                    if dist > 1:
                        self.forward_vel = 1.1
                    else:
                        self.forward_vel = dist
            # turn command
            else:
                # if self.recv_msg:
                    # self.action_server.set_succeeded()
                    # self.recv_msg = False
                # continue
            
                # orientation deadband if we are doing a rotate command
                if abs(orientation_err) >= 0.043: # 5 degrees (abs = 2.5 degrees)
                    # if orientation_err > 0.172: # 10 degrees
                    #      self.rotational_vel = 0.875
                    #  elif orientation_err < -0.172:
                    #      self.rotational_vel = -0.875
                    #  else:
                    #      self.rotational_vel = 1.29/orientation_err
                    # self.rotational_vel = min(0.875,orientation_err)

                    # Prefer right turns when turning 180 degrees
                    self.rotational_vel = max(-0.875, min(0.875, 0.3*orientation_err))
                    rospy.loginfo_throttle(1, "Turning rotational velocity=%f=%f*0.3" % (self.rotational_vel, orientation_err))

            if self.rotational_vel == 0 and self.forward_vel == 0:
                self.is_turning = False
            max_forward_vel = self.last_forward_vel + self.linear_accel*time_diff
            min_forward_vel = self.last_forward_vel - self.linear_accel * time_diff
            self.forward_vel = max(min_forward_vel, min(max_forward_vel, self.forward_vel))

            max_rot_vel = self.last_rot_vel + self.rot_accel * time_diff
            min_rot_vel = self.last_rot_vel - self.rot_accel * time_diff
            self.rotational_vel = max(min_rot_vel, min(max_rot_vel, self.rotational_vel))

            # fill in values for the Twist
            msg.linear = Vector3(self.forward_vel, 0, 0)
            msg.angular = Vector3(0, 0, self.rotational_vel)
            self.last_forward_vel = self.forward_vel
            self.last_rot_vel = self.rotational_vel

            rospy.loginfo_throttle(1, "Desired Position: (%f,%f,%f) Current Position: (%f,%f,%f) Sending Velocity: (%f,%f)" % (self.desired_position_x,self.desired_position_y,self.desired_orientation,self.current_position_x,self.current_position_y,self.current_orientation,self.forward_vel,self.rotational_vel))

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
        self.current_speed = msg.twist.twist.linear.x
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

        self.is_turning = goal.target_pose.pose.position.x == 0

        rospy.loginfo("Getting updated goal P:%f R:%f" % (goal.target_pose.pose.position.x, math.asin(goal.target_pose.pose.orientation.z) * 2))

        self.action_server.set_succeeded()
        self.recv_msg = True


if __name__ == "__main__":
    try:
        controller = Master()
        controller.begin()
    except rospy.ROSInterruptException:
        pass

