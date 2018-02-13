#!/usr/bin/env python

import rospy
import serial
import math
import os
from geometry_msgs.msg import Twist
from PyCRC.CRC16 import CRC16
from struct import pack


class ArduinoController:
    def __init__(self):
        # Initialize the serial port
        rospy.init_node('arduino_controller', anonymous=True)
        # Subscribe to the arduino_commands topic
        rospy.Subscriber("cmd_vel", Twist, self.callback)

        # The width between the wheels
        if rospy.has_param("~width"):
            self.radius = float(rospy.get_param("~width")) / 2
        else:
            self.radius = 31.5 * 0.0254 / 2

        # Check serial port name
        if rospy.has_param("~port"):
            self.port_name = rospy.get_param("~port")
        else:
            self.port_name = '/dev/ttyACM0'

        # Check serial baud rate
        if rospy.has_param("~baud_rate"):
            self.baud_rate = int(rospy.get_param("~baud_rate"))
        else:
            self.baud_rate = 115200

        if rospy.has_param("~virtual_port"):
            self.has_virtual_port = True
        else:
            self.has_virtual_port = False
        self.STOP_CMD = b'\xEE\x00'
        self.STOP_CRC = CRC16().calculate(bytes(self.STOP_CMD))
        self.GO_CMD = b'\xEE\x20'

    # callback for receiving data from the Arduino
    def callback(self, msg):
        # Convert message to m/s for each wheel
        lin_vel = math.sqrt(msg.linear.x ** 2 + msg.linear.y ** 2) * 1e6
        ang_vel = msg.angular.z * self.radius * 1e6
        vel_l = int(lin_vel - ang_vel)
        vel_r = int(lin_vel + ang_vel)
        if msg.linear.x == 0 and msg.linear.y == 0 and msg.linear.z == 0 and msg.angular.x == 0 and msg.angular.y == 0 and msg.angular.z == 0:
            # Stop
            packet = pack(self.STOP_CMD, self.STOP_CRC)
        else:
            packet = pack(self.GO_CMD, vel_l, vel_r)
            calc_crc = CRC16().calculate(bytes(packet))
            packet = pack(self.GO_CMD, vel_l, vel_r, calc_crc)
        # Write packet to Arduino's serial port
        self.ser.write(packet)

    def close_port(self):
        self.ser.close()

    # start the node: spin forever
    def begin(self):
        # Change how ports are configured if in a docker container with virtual ports
        if 'INSIDEDOCKER' in os.environ or self.has_virtual_port:
            self.ser = serial.Serial(port=self.port_name, baudrate=self.baud_rate, timeout=0, rtscts=True, dsrdtr=True)
        else:
            self.ser = serial.Serial(port=self.port_name, baudrate=self.baud_rate, timeout=0)

        # make sure the port is closed on exit
        rospy.on_shutdown(self.close_port)

        # never exit
        rospy.spin()


if __name__ == "__main__":
    try:
        controller = ArduinoController()
        controller.begin()
    except rospy.ROSInterruptException:
        pass
