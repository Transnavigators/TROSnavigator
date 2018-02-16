#!/usr/bin/env python

import rospy
import evdev
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion, Point, Twist, Vector3


class SixaxisPublisher:
    def __init__(self):
        # set up node
        rospy.init_node('sixaxis_pub', anonymous=True)

        # Either publish velocities to motor or send actions to move_base for assisted driving
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        rospy.loginfo("Finding PS3 controller.")
        ps3dev = None
        devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
        for device in devices:
            if device.name == 'PLAYSTATION(R)3 Controller':
                ps3dev = device.fn
        if ps3dev is None:
            rospy.logfatal("Could not find the PS3 controller.")
        else:
            self.gamepad = evdev.InputDevice(ps3dev)

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

        if rospy.has_param("~rate"):
            self.rate = rospy.Rate(int(rospy.get_param("~rate")))
        else:
            self.rate = rospy.Rate(300)

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

    def begin(self):
        if hasattr(self, 'gamepad'):
            while not rospy.is_shutdown():
                event = self.gamepad.read_one()
                if event is not None:
                    new_x = 0
                    new_y = 0
                    new_rot = 0
                    stop = False
                    rospy.loginfo("Received joystick event: " + str(event))
                    # TODO: optionally publish to cmd_vel topic instead and bypass move_base
                    # Event key mapping can be found here http://www.ev3dev.org/docs/tutorials/using-ps3-sixaxis/
                    # Analog stick moved
                    if event.type == 3:
                        if event.code == 5:
                            # moving forward
                            new_x = 0.1
                        elif event.code == 0:
                            # turning
                            new_rot = self.scale_stick(event.value) * math.pi / 32
                    # Key presses
                    elif event.type == 1:
                        if event.code == 293 and event.value == 0:
                            # turn right
                            new_rot = -math.pi / 32
                        elif event.code == 292 and event.value == 0:
                            # move forward
                            new_x = 0.1
                        elif event.code == 294 and event.value == 0:
                            # turn right
                            new_rot = -math.pi / 32
                        elif event.code == 295 and event.value == 0:
                            # turn left
                            new_rot = math.pi / 32
                        if event.code == 302 and event.value == 1:
                            # stop
                            stop = True
                    # Construct message if valid command was read
                    if new_x != 0 or new_y != 0 or new_rot != 0 or stop:
                        goal = MoveBaseGoal()
                        goal.target_pose.header.frame_id = "base_link"
                        goal.target_pose.header.stamp = rospy.get_time()
                        goal.target_pose.pose.position = Point(new_x, new_y, 0)
                        quat = quaternion_from_euler(0, 0, new_rot)
                        goal.target_pose.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
                        self.action_client.send_goal(goal)
                self.rate.sleep()
        else:
            rospy.logfatal("Exiting...")

    def begin_direct_control(self):
        if hasattr(self, 'gamepad'):
            used_key = False
            while not rospy.is_shutdown():
                event = self.gamepad.read_one()
                if event is not None:
                    x_vel = 0
                    rot_vel = 0
                    stop = False
                    # rospy.loginfo("Received joystick event: "+str(event))
                    # TODO: test if this works or only goes in one direction
                    # Analog stick moved a sufficient amount
                    mag = abs(event.value-128)
                    if event.type == 3 and (mag > 10 or not used_key):
                        if event.code == 5:
                            # moving forward
                            if(mag > 10):
                                x_vel = -self.scale_stick(event.value) * self.MAX_SPEED
                            else:
                                x_vel = 0
                            #rospy.loginfo("Moving forward: " + str(x_vel))
                            used_key = False
                        elif event.code == 0:
                            # turning
                            if(mag > 10):
                                rot_vel = self.scale_stick(event.value) * self.MAX_ROT_SPEED
                            else:
                                rot_vel = 0
                            #rospy.loginfo("Turning: " + str(rot_vel))
                            used_key = False
                    # Key presses
                    elif event.type == 1:
                        if event.value == 1:
                            if event.code == 293:
                                # turn right
                                rot_vel = -self.MAX_ROT_SPEED / 2
                                #rospy.loginfo("Turning right key: " + str(event.value))
                                used_key = True
                            elif event.code == 292:
                                # move forward
                                x_vel = self.MAX_SPEED / 2
                                #rospy.loginfo("Moving forward key: " + str(event.value))
                                used_key = True
                            elif event.code == 294:
                                # move back
                                x_vel = -self.MAX_SPEED / 2
                                #rospy.loginfo("Moving backwards key: " + str(event.value))
                                used_key = True
                            elif event.code == 295:
                                # turn left
                                rot_vel = self.MAX_ROT_SPEED / 2
                                #rospy.loginfo("Turning left key: " + str(event.value))
                                used_key = True
                            elif event.code == 302:
                                # stop
                                stop = True
                                used_key = True
                        if event.value == 0:
                            stop = True
                            used_key = True
                    # Construct message if valid command was read
                    if x_vel != 0 or rot_vel != 0 or stop:
                        twist = Twist()
                        twist.linear = Vector3(x_vel, 0, 0)
                        twist.angular = Vector3(0, 0, rot_vel)
                        self.pub.publish(twist)

                        if stop:
                            rospy.loginfo("Sent stop")
                        else:
                            rospy.loginfo("Sent command")
                        self.rate.sleep()

if __name__ == "__main__":
    try:
        sp = SixaxisPublisher()
        if rospy.has_param("~direct_control") and rospy.get_param("~direct_control"):
            sp.begin_direct_control()
        else:
            sp.begin()
    except rospy.ROSInterruptException:
        pass
