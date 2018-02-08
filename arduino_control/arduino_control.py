#!/usr/bin/env python

import rospy
import serial
import math
import os
from geometry_msgs.msg import Twist
from PyCRC.CRC16 import CRC16

class ArduinoController:
    def __init__(self):
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

        self.stop_crc = CRC16().calculate(bytes(b'\xEE\x00'))
        self.STOP_CMD = b'\xEE\x00'
        self.GO_CMD = b'\xEE\x20'
        
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
        buf = bytearray()

        if msg.linear.x == 0 and msg.linear.y == 0 and msg.linear.z == 0 and msg.angular.x == 0 and msg.angular.y == 0 and msg.angular.z == 0:
            # Stop
            buf.extend(self.STOP_CMD)
            buf.extend(self.stop_crc)
        else:
            buf.extend(self.GO_CMD)
            buf.extend(vel_l)
            buf.extend(vel_r)
            calc_crc = CRC16().calculate(bytes(buf))
            buf.extend(calc_crc)

        # Write packet to Arduino's serial port
        self.ser.write(bytes(buf))

    def close_port(self):
        self.ser.close()

    # start the node: spin forever
    def begin(self):
        # Initialize the serial port
        rospy.init_node('arduino_controller', anonymous=True)
        # Subscribe to the arduino_commands topic
        rospy.Subscriber("cmd_vel", Twist, self.callback)
        
        # Change how ports are configured if in a docker container with virtual ports
        if 'INSIDEDOCKER' in os.environ:
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
