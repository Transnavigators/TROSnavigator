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
from geometry_msgs.msg import Twist, Vector3


class SixaxisPublisher(asyncore.file_dispatcher):
    """Sixaxis Publisher is a ROS joystick driver for PS3 controllers

    This module looks for connected PlayStation 3 controllers and registers a callback to translate joystick inputs to
    velocity commands.

    """
    def __init__(self):
        # set up node
        rospy.init_node('sixaxis_pub', anonymous=True)

        rospy.loginfo("Finding PS3 controller.")
        ps3dev = None
        err_count = 0
        max_err = 10
        while not rospy.is_shutdown() and err_count < max_err:
            devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
            for device in devices:
                if 'PLAYSTATION(R)3 Controller' == device.name:
                    ps3dev = device.fn

            if ps3dev is None:
                rospy.sleep(1.0)
                err_count += 1
            else:
                self.gamepad = evdev.InputDevice(ps3dev)
                asyncore.file_dispatcher.__init__(self, self.gamepad)
                rospy.loginfo("Found the PS3 controller.")
                break

        if ps3dev is None:
            rospy.logfatal("Could not find the PS3 controller.")
            sys.exit(1)

        # Setup audio
        rospack = rospkg.RosPack()
        self.wav = wave.open(rospack.get_path('sixaxis_publisher') + '/dixie-horn_daniel-simion.wav', 'r')
        self.audio = pyaudio.PyAudio()
        self.chunk = 1024
        # Open an audio stream for the horn
        self.stream = self.audio.open(format=self.audio.get_format_from_width(self.wav.getsampwidth()),
                                      channels=self.wav.getnchannels(),
                                      rate=self.wav.getframerate(),
                                      output=True)

        # Either publish velocities to motor or send actions to move_base for assisted driving
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # Default params
        self.MAX_SPEED = float(rospy.get_param("~max_speed", 2.2))
        self.MAX_REVERSE_SPEED = float(rospy.get_param("~max_reverse_speed", 0.5))
        self.MAX_ROT_SPEED = float(rospy.get_param("~max_rot_speed", 1.75))
        self.threshold = int(rospy.get_param("~joystick_threshold", 5))

        # Instance variables for helping the callback remember its state
        self.used_key = False
        self.stopped = False
        self.rot_vel = 0
        self.x_vel = 0

    @staticmethod
    def scale(val, src, dst):
        """Scale the given value from the scale of src to the scale of dst.
    
        Args:
            val (float or int): the value to scale
            src (tuple): the original range
            dst (tuple): the destination range
        
        Returns:
            float: the value normalized with the new range
        
        Examples:
            >>> print(SixaxisPublisher.scale(99, (0.0, 99.0), (-1.0, +1.0)))
            1.0
        
        """
        return (float(val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]

    @staticmethod
    def scale_stick(value):
        """Normalizes a joystick output byte (0-255) to the range -1 to 1
        
        Args:
            value (float or int): the value to scale from (0,255) to (-1,1)
        
        Returns:
            float: the normalized value
        
        """
        return SixaxisPublisher.scale(value, (0, 255), (-1, 1))

    # Never need to write anything
    def writable(self):
        return 0

    # Close connection on error
    def handle_expt(self):
        self.close()

    def handle_close(self):
        self.close()

    def handle_read(self):
        stop = False
        for event in self.gamepad.read():
            # Check for joystick inputs and if we're in joystick mode
            mag = abs(event.value - 128)
            if event.type == 3 and not self.used_key:
                if event.code == 4:
                    # right joystick y axis controls moving forward
                    if mag > self.threshold:
                        rospy.loginfo("Top joystick = %d" % event.value)
                        scaled = SixaxisPublisher.scale_stick(event.value)
                        if scaled < 0:
                            self.x_vel = -scaled * self.MAX_SPEED
                        else:
                            self.x_vel = -scaled * self.MAX_REVERSE_SPEED
                    else:
                        self.x_vel = 0
                    self.used_key = False
                elif event.code == 3:
                    # right joystick x-axis controls turning
                    rospy.loginfo("Right joystick = %d" % event.value)
                    if mag > self.threshold:
                        self.rot_vel = -SixaxisPublisher.scale_stick(event.value) * self.MAX_ROT_SPEED
                    else:
                        self.rot_vel = 0
                    self.used_key = False
            # Key presses
            elif event.type == 1:
                if event.value == 1:
                    # Key down press
                    if event.code in [293, 547]:
                        # turn right
                        self.rot_vel = -self.MAX_ROT_SPEED / 2
                        self.used_key = True
                    elif event.code in [292, 544]:
                        # move forward
                        self.x_vel = self.MAX_SPEED / 2
                        self.rot_vel = 0
                        self.used_key = True
                    elif event.code in [294, 545]:
                        # move back
                        self.x_vel = -self.MAX_REVERSE_SPEED
                        self.rot_vel = 0
                        self.used_key = True
                    elif event.code in [295, 546]:
                        # turn left
                        self.rot_vel = self.MAX_ROT_SPEED / 2
                        self.used_key = True
                    elif event.code in [302, 303, 304]:
                        # x key, stop
                        stop = True
                        self.used_key = True
                    elif event.code in [301, 308]:
                        # Only play horn if not already playing
                        if self.wav.tell() == 0:
                            wav_data = self.wav.readframes(self.chunk)
                            while wav_data:
                                self.stream.write(wav_data)
                                wav_data = self.wav.readframes(self.chunk)
                            # Reset wav file pointer
                            self.wav.rewind()
                if event.value == 0:
                    # Key up press
                    stop = True
                    self.used_key = False
                    # Construct message if valid command was read
        if self.rot_vel == 0 and self.x_vel == 0:
            # When both joysticks are centered, stop
            stop = True
        # If it used to be stopped and is suddenly moving at full speed, ignore the input
        if self.stopped and (abs(self.x_vel) in [self.MAX_SPEED, self.MAX_REVERSE_SPEED]
                             or abs(self.rot_vel) == self.MAX_ROT_SPEED):
            rospy.logwarn("Caught error from 0 to vx=%f vth=%f" % (self.x_vel, self.rot_vel))
            return
        # Send a new twist if we have a nonzero command or an explicit stop command
        twist = Twist()
        twist.linear = Vector3(self.x_vel, 0, 0)
        twist.angular = Vector3(0, 0, self.rot_vel)
        self.pub.publish(twist)

        # Update stopped variable
        if stop:
            self.stopped = True
        elif self.x_vel != 0 or self.rot_vel != 0:
            self.stopped = False
            rospy.loginfo_throttle(1, "Sending vx=%f vth=%f" % (self.x_vel, self.rot_vel))


# goal = MoveBaseGoal()
#               goal.target_pose.header.frame_id = "base_link"
#               goal.target_pose.header.stamp = rospy.get_time()
#               goal.target_pose.pose.position = Point(new_x, new_y, 0)
#               quat = quaternion_from_euler(0, 0, new_rot)
#               goal.target_pose.pose.orientation = Quaternion(math.cos(new_rot), 0, 0, math.sin(new_rot))
#               self.action_client.send_goal(goal)


if __name__ == "__main__":
    sp = None
    try:
        sp = SixaxisPublisher()
        while not rospy.is_shutdown():
            asyncore.loop(timeout=1, count=100)
    except rospy.ROSInterruptException:
        sp.stream.stop_stream()
        sp.stream.close()
        pass
