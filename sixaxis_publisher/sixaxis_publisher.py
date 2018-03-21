#!/usr/bin/env python

import rospy
import evdev
import asyncore
import pyaudio
import wave
import rospkg
import time
import sys
from geometry_msgs.msg import Twist, Vector3


class SixaxisPublisher(asyncore.file_dispatcher):
    """Sixaxis Publisher is a ROS joystick driver for PS3 controllers

    This module looks for connected PlayStation 3 controllers and registers a callback to translate joystick inputs to
    velocity commands.

    """

    def __init__(self):
        # set up node
        rospy.init_node('sixaxis_publisher', anonymous=True)

        # Publish velocities to motor
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Default params
        self.MAX_SPEED = float(rospy.get_param("~max_speed", 2.2))
        self.MAX_REVERSE_SPEED = float(rospy.get_param("~max_reverse_speed", 0.5))
        self.MAX_ROT_SPEED = float(rospy.get_param("~max_rot_speed", 1.75))
        self.threshold = int(rospy.get_param("~joystick_threshold", 5))
        search_time = float(rospy.get_param("~search_time",60))
        is_test = int(rospy.get_param("~test", 0))

        # Instance variables for helping the callback remember its state
        self.used_key = False
        # self.stopped = True
        self.stop = False
        self.rot_vel = 0
        self.x_vel = 0

        self.gamepad = None
        if is_test == 1:
            self.sub = rospy.Subscriber('joytest', Vector3, self.callback)
            self.is_test = True
        else:
            self.is_test = False
            rospy.loginfo("Finding PS3 controller.")
            # Check every second
            rate = rospy.Rate(1.0)
            for i in range(0, search_time):
                if self.connect_joystick():
                    break
                rate.sleep()
            if self.gamepad is None:
                rospy.logerr("Could not find the PS3 controller.")
                if is_test == 0:
                    sys.exit(1)

        # Setup audio
        rospack = rospkg.RosPack()
        self.wav = wave.open(rospack.get_path('sixaxis_publisher') + '/dixie-horn_daniel-simion.wav', 'r')
        self.audio = pyaudio.PyAudio()
        self.chunk = 1024
        # Open an audio stream for the horn
        try:
            self.stream = self.audio.open(format=self.audio.get_format_from_width(self.wav.getsampwidth()),
                                          channels=self.wav.getnchannels(),
                                          rate=self.wav.getframerate(),
                                          output=True)
        except IOError:
            self.stream = None

    def connect_joystick(self):
        devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
        for device in devices:
            if 'PLAYSTATION(R)3 Controller' == device.name:
                rospy.loginfo("Found the PS3 controller.")
                self.gamepad = device
                asyncore.file_dispatcher.__init__(self, self.gamepad)
                return True
        return False

    def scale_stick(self, value):
        """Normalizes a joystick output byte (0-255) to the range -1 to 1
        
        Args:
            value (float or int): the value to scale from (0,255) to (-1,1)
        
        Returns:
            float: the normalized value
        
        """
        return (float(value)*2/255) - 1.0

    # Never need to write anything
    def writable(self):
        return False

    # Close connection on error
    def handle_expt(self):
        self.close()
        rospy.loginfo("Closing asyncore")

    def handle_close(self):
        self.close()
        rospy.loginfo("Closing asyncore")

    def callback(self, msg):
        curr_time = time.time()
        secs = int(curr_time)
        usecs = int(((curr_time % 1) * 10e6))
        event = evdev.InputEvent(sec=secs, usec=usecs, type=int(msg.x), code=int(msg.y), value=int(msg.z))
        self.process_event(event)
        if event.type == 0:  # Syn packet, so done reading
            self.send_cmd()
            self.stop = False

    def process_event(self, event):
        #rospy.loginfo(event)
        # Check for joystick inputs and if we're in joystick mode
        mag = abs(event.value - 128)
        if event.type == 3 and not self.used_key:
            if event.code == 4:
                # right joystick y axis controls moving forward
                if mag > self.threshold:
                    #rospy.loginfo("y=%d" % event.value)
                    scaled = self.scale_stick(event.value)
                    if scaled < 0:
                        self.x_vel = -scaled * self.MAX_SPEED
                    else:
                        self.x_vel = -scaled * self.MAX_REVERSE_SPEED
                    self.used_key = False
                else:
                    self.x_vel = 0

            elif event.code == 3:
                # right joystick x-axis controls turning
                #rospy.loginfo("x=%d" % event.value)
                if mag > self.threshold:
                    self.rot_vel = -self.scale_stick(event.value) * self.MAX_ROT_SPEED
                    self.used_key = False
                else:
                    self.rot_vel = 0
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
                    self.stop = True
                    self.used_key = True
                elif event.code in [301, 308]:
                    # Only play horn if not already playing
                    if self.stream is not None and self.wav.tell() == 0:
                        wav_data = self.wav.readframes(self.chunk)
                        while wav_data:
                            self.stream.write(wav_data)
                            wav_data = self.wav.readframes(self.chunk)
                        # Reset wav file pointer
                        self.wav.rewind()
            if event.value == 0:
                # Key up press
                self.stop = True
                self.used_key = False
                # Construct message if valid command was read
        if self.x_vel != 0 or self.rot_vel != 0 or self.stop:
            rospy.loginfo("Set x_vel=%f rot_vel=%f stop=%r" % (self.x_vel, self.rot_vel, self.stop))

    def send_cmd(self):
        rospy.loginfo("Sending x_vel=%d rot_vel=%d stop=%r" % (self.x_vel, self.rot_vel, self.stop))
        if self.rot_vel == 0 and self.x_vel == 0:
            # When both joysticks are centered, stop
            self.stop = True
        elif self.stop:
            self.rot_vel = 0
            self.x_vel = 0
        # If it used to be stopped and is suddenly moving at full speed, ignore the input
        # if self.stopped and (abs(self.x_vel) in [self.MAX_SPEED, self.MAX_REVERSE_SPEED]
        #                     or abs(self.rot_vel) == self.MAX_ROT_SPEED):
        #    rospy.logwarn("Caught error from 0 to vx=%f vth=%f" % (self.x_vel, self.rot_vel))
        # else:
        # Send a new twist if we have a nonzero command or an explicit stop command
        twist = Twist()
        twist.linear = Vector3(self.x_vel, 0, 0)
        twist.angular = Vector3(0, 0, self.rot_vel)
        # rospy.loginfo(str(twist))
        self.pub.publish(twist)

        # Update stopped variable
        # if self.stop:
        #    self.stopped = True
        if self.x_vel != 0 or self.rot_vel != 0:
            # self.stopped = False
            rospy.loginfo_throttle(1, "Sending vx=%f vth=%f" % (self.x_vel, self.rot_vel))
        # goal = MoveBaseGoal()
        # goal.target_pose.header.frame_id = "base_link"
        # goal.target_pose.header.stamp = rospy.get_time()
        # goal.target_pose.pose.position = Point(new_x, new_y, 0)
        # quat = quaternion_from_euler(0, 0, new_rot)
        # goal.target_pose.pose.orientation = Quaternion(math.cos(new_rot), 0, 0, math.sin(new_rot))
        # self.action_client.send_goal(goal)

    def handle_read(self):
        self.stop = False
        for event in self.gamepad.read():
            self.process_event(event)
        self.send_cmd()
        if rospy.is_shutdown():
            raise asyncore.ExitNow('ROS is quitting')

if __name__ == "__main__":
    sp = None
    try:
        sp = SixaxisPublisher()
        if sp.is_test:
            rospy.spin()
        else:
            asyncore.loop()
    except (rospy.ROSInterruptException, asyncore.ExitNow):
        sp.stream.stop_stream()
        sp.stream.close()
        pass
