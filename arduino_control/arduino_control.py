#!/usr/bin/env python

import rospy
import serial
import math
from geometry_msgs.msg import Twist


class ArduinoController:
    def __init__(self):
        # Initialize the serial port
        rospy.init_node('arduino_controller', anonymous=True)
        # Subscribe to the arduino_commands topic
        rospy.Subscriber("cmd_vel", Twist, self.callback)

        # set up the port
        port_name = rospy.get_param("~port")
        self.ser = serial.Serial(port=port_name, baudrate=115200, timeout=0)

        # make sure the port is closed on exit
        rospy.on_shutdown(self.close_port)

        if rospy.has_param("~width"):
            self.radius = float(rospy.get_param("~width")) / 2
        else:
            self.radius = 31.5 * 0.0254 / 2

    def int_to_bytes(self, value):
        result = []
        for i in range(0, 4):
            result.append(value >> (i * 8) & 0xff)
        result.reverse()
        return result

    # callback for receiving data from the Arduino
    def callback(self, msg):
        # Convert message to m/s for each wheel
        lin_vel = math.sqrt(msg.linear.x ** 2 + msg.linear.y ** 2) * 1e6
        ang_vel = msg.angular.z * self.radius * 1e6
        vel_l = self.int_to_bytes(int(lin_vel - ang_vel))
        vel_r = self.int_to_bytes(int(lin_vel + ang_vel))

        # TODO: make sure the linear direction is same as actual bearing
        if msg.linear.x == 0 and msg.linear.y == 0 and msg.linear.z == 0 and msg.angular.x == 0 and msg.angular.y == 0 and msg.angular.z == 0:
            # Stop
            self.ser.write(b'\xEE\x00')
        else:
            buf = bytearray()
            buf.extend(b'\xEE\x20')
            buf.extend(vel_l)
            buf.extend(vel_r)
            self.ser.write(bytes(buf))

    def close_port(self):
        self.ser.close()

    # start the node: spin forever
    def begin(self):
        # never exit
        rospy.spin()


if __name__ == "__main__":
    try:
        controller = ArduinoController()
        controller.begin()
    except rospy.ROSInterruptException:
        pass
