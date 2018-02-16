#!/usr/bin/env python

import rospy
import evdev
import math
import actionlib
import sys
from asyncore import file_dispatcher
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion, Point, Twist, Vector3


class SixaxisPublisher(file_dispatcher):
    def __init__(self):
        # set up node
        rospy.init_node('sixaxis_pub', anonymous=True)

        rospy.loginfo("Finding PS3 controller.")
        ps3dev = None
        devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
        for device in devices:
            if device.name == 'PLAYSTATION(R)3 Controller':
                ps3dev = device.fn

        if ps3dev is None:
            rospy.logfatal("Could not find the PS3 controller.")
            sys.exit(1)
        else:
            self.gamepad = evdev.InputDevice(ps3dev)
            file_dispatcher.__init__(self, self.gamepad)
            rospy.loginfo("Found the PS3 controller.")

        # Either publish velocities to motor or send actions to move_base for assisted driving
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        if rospy.has_param("~max_speed"):
            self.MAX_SPEED = float(rospy.get_param("~max_speed"))
        else:
            self.MAX_SPEED = 2.2

        if rospy.has_param("~max_reverse_speed"):
            self.MAX_REVERSE_SPEED = float(rospy.get_param("~max_reverse_speed"))
        else:
            self.MAX_REVERSE_SPEED = 0.5

        if rospy.has_param("~max_rot_speed"):
            self.MAX_ROT_SPEED = float(rospy.get_param("~max_rot_speed"))
        else:
            self.MAX_ROT_SPEED = 1.75

        if rospy.has_param("~joystick_threshold"):
            self.threshold = int(rospy.get_param("~joystick_threshold"))
        else:
            self.threshold = 5
        self.used_key = False
        self.x_vel = 0
        self.rot_vel = 0

    # Some helpers
    def scale(self, val, src, dst):
        """
        Scale the given value from the scale of src to the scale of dst.
    
        val: float or int
        src: tuple
        dst: tuple
    
        example: print(scale(99, (0.0, 99.0), (-1.0, +1.0)))
        """
        return (float(val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]

    def scale_stick(self, value):
        return self.scale(value, (0, 255), (-1, 1))

    def recv(self, ign=None):
        return self.device.read()

    def handle_read(self):
        for event in self.recv():
            stop = False

            # Check for joystick inputs and if we're in joystick mode
            mag = abs(event.value - 128)
            if event.type == 3 and not self.used_key:
                if event.code == 5:
                    # right joystick y axis controls moving forward
                    if mag > self.threshold:
                        scaled = self.scale_stick(event.value)
                        if scaled < 0:
                            self.x_vel = -self.scale_stick(event.value) * self.MAX_SPEED
                        else:
                            self.x_vel = -self.scale_stick(event.value) * self.MAX_REVERSE_SPEED
                    else:
                        self.x_vel = 0
                    self.used_key = False
                elif event.code in [0, 2]:
                    # left and right joystick x-axis controls turning
                    if mag > self.threshold:
                        self.rot_vel = self.scale_stick(event.value) * self.MAX_ROT_SPEED
                    else:
                        self.rot_vel = 0
                    self.used_key = False
                if self.rot_vel == 0 and self.x_vel == 0:
                    # When both joysticks are centered, stop
                    stop = True
            # Key presses
            elif event.type == 1:
                if event.value == 1:
                    # Key down press
                    if event.code == 293:
                        # turn right
                        self.rot_vel = -self.MAX_ROT_SPEED / 2
                        self.used_key = True
                    elif event.code == 292:
                        # move forward
                        self.x_vel = self.MAX_SPEED / 2
                        self.used_key = True
                    elif event.code == 294:
                        # move back
                        self.x_vel = -self.MAX_SPEED / 2
                        self.used_key = True
                    elif event.code == 295:
                        # turn left
                        self.rot_vel = self.MAX_ROT_SPEED / 2
                        self.used_key = True
                    elif event.code in [302, 303]:
                        # x key, stop
                        stop = True
                        self.used_key = True
                    # elif event.code == 301:
                    # TODO: horn
                if event.value == 0:
                    # Key up press
                    stop = True
                    self.used_key = False
            # Construct message if valid command was read
            if stop:
                self.x_vel = 0
                self.rot_vel = 0
            if self.x_vel != 0 or self.rot_vel != 0 or stop:
                twist = Twist()
                twist.linear = Vector3(self.x_vel, 0, 0)
                twist.angular = Vector3(0, 0, self.rot_vel)
                self.pub.publish(twist)
    #
    # def begin(self):
    #     if hasattr(self, 'gamepad'):
    #         while not rospy.is_shutdown():
    #             event = self.gamepad.read_one()
    #             if event is not None:
    #                 new_x = 0
    #                 new_y = 0
    #                 new_rot = 0
    #                 stop = False
    #                 # Event key mapping can be found here http://www.ev3dev.org/docs/tutorials/using-ps3-sixaxis/
    #                 # Analog stick moved
    #                 if event.type == 3:
    #                     if event.code == 5:
    #                         # moving forward
    #                         new_x = 0.1
    #                     elif event.code == 0:
    #                         # turning
    #                         new_rot = self.scale_stick(event.value) * math.pi / 32
    #                 # Key presses
    #                 elif event.type == 1:
    #                     if event.code == 293 and event.value == 0:
    #                         # turn right
    #                         new_rot = -math.pi / 32
    #                     elif event.code == 292 and event.value == 0:
    #                         # move forward
    #                         new_x = 0.1
    #                     elif event.code == 294 and event.value == 0:
    #                         # turn right
    #                         new_rot = -math.pi / 32
    #                     elif event.code == 295 and event.value == 0:
    #                         # turn left
    #                         new_rot = math.pi / 32
    #                     if event.code == 302 and event.value == 1:
    #                         # stop
    #                         stop = True
    #                 # Construct message if valid command was read
    #                 if new_x != 0 or new_y != 0 or new_rot != 0 or stop:
    #                     goal = MoveBaseGoal()
    #                     goal.target_pose.header.frame_id = "base_link"
    #                     goal.target_pose.header.stamp = rospy.get_time()
    #                     goal.target_pose.pose.position = Point(new_x, new_y, 0)
    #                     quat = quaternion_from_euler(0, 0, new_rot)
    #                     goal.target_pose.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
    #                     self.action_client.send_goal(goal)
    #             self.rate.sleep()
    #     else:
    #         rospy.logfatal("Exiting...")


if __name__ == "__main__":
    try:
        sp = SixaxisPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
