#!/usr/bin/env python

import rospy
import evdev
import actionlib
import sys
import asyncore
import pyaudio
import wave
import rospkg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion, Point, Twist, Vector3


class SixaxisPublisher(asyncore.file_dispatcher):
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
            asyncore.file_dispatcher.__init__(self, self.gamepad)
            rospy.loginfo("Found the PS3 controller.")

        rospack = rospkg.RosPack()
        self.wav = wave.open(rospack.get_path('sixaxis_publisher') + '/dixie-horn_daniel-simion.wav')
        self.audio = pyaudio.PyAudio()
        self.chunk = 1024

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

        self.stream = self.audio.open(format=self.audio.get_format_from_width(self.wav.getsampwidth()),
                                      channels=self.wav.getnchannels(),
                                      rate=self.wav.getframerate(),
                                      output=True)
        self.stopped = False
        self.rot_vel = 0
        self.x_vel = 0
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

    # Never need to write anything
    def writable(self):
        return 0

    # Close connection on error
    def handle_expt(self):
        self.close()

    def handle_close(self):
        self.close()

    def handle_read(self):
        for event in self.gamepad.read():
            stop = False

            # Check for joystick inputs and if we're in joystick mode
            mag = abs(event.value - 128)
            if event.type == 3 and not self.used_key:
                if event.code == 5:
                    # right joystick y axis controls moving forward
                    if mag > self.threshold:
                        scaled = self.scale_stick(event.value)
                        if scaled < 0:
                            self.x_vel = -scaled * self.MAX_SPEED
                        else:
                            self.x_vel = -scaled * self.MAX_REVERSE_SPEED
                    else:
                        self.x_vel = 0
                    self.used_key = False
                elif event.code == 2:
                    # right joystick x-axis controls turning
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
                        self.x_vel = -self.MAX_REVERSE_SPEED
                        self.used_key = True
                    elif event.code == 295:
                        # turn left
                        self.rot_vel = self.MAX_ROT_SPEED / 2
                        self.used_key = True
                    elif event.code in [302, 303]:
                        # x key, stop
                        stop = True
                        self.used_key = True
                    elif event.code == 301:
                        data = self.wav.readframes(self.chunk)
                        while data:
                            self.stream.write(data)
                            data = self.wav.readframes(self.chunk)
                        #self.stream.stop_stream()
                        #self.stream.close()
                if event.value == 0:
                    # Key up press
                    stop = True
                    self.used_key = False
            # Construct message if valid command was read

            # If it used to be stopped and is not moving at full speed, ignore the input
            if self.stopped and (abs(self.x_vel) == self.MAX_SPEED or abs(self.x_vel) == self.MAX_REVERSE_SPEED or self.rot_vel == self.MAX_ROT_SPEED):
                rospy.logwarn("Caught error from 0 to vx=%f vth=%f" % (self.x_vel, self.rot_vel))
                continue
            if (self.x_vel != 0 or self.rot_vel != 0) or stop:
                twist = Twist()
                twist.linear = Vector3(self.x_vel, 0, 0)
                twist.angular = Vector3(0, 0, self.rot_vel)
                self.pub.publish(twist)
            if stop:
                self.stopped = True
            elif self.x_vel != 0 or self.rot_vel != 0:
                self.stopped = False
                rospy.loginfo_throttle(1, "Sending vx=%f vth=%f" % (self.x_vel, self.rot_vel))
#               goal = MoveBaseGoal()
#               goal.target_pose.header.frame_id = "base_link"
#               goal.target_pose.header.stamp = rospy.get_time()
#               goal.target_pose.pose.position = Point(new_x, new_y, 0)
#               quat = quaternion_from_euler(0, 0, new_rot)
#               goal.target_pose.pose.orientation = Quaternion(math.cos(new_rot), 0, 0, math.sin(new_rot))
#               self.action_client.send_goal(goal)


if __name__ == "__main__":
    try:
        sp = SixaxisPublisher()
        while not rospy.is_shutdown():
            asyncore.loop(timeout=1, count=1)
    except rospy.ROSInterruptException:
        sp.stream.stop_stream()
        sp.stream.close()
        pass
