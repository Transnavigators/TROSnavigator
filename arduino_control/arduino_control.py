#!/usr/bin/env python

import rospy
import serial
import os
import re
from std_msgs.msg import String

port_name = '/dev/ttyUSB1'
if os.name == 'nt':
    port_name = 'COM6'
ser = serial.Serial(port=port_name, baudrate=115200, timeout=0)
move_pattern = re.compile('^Move r=(.*), a=(.*)$')
set_pattern = re.compile('^Set s=(.*), a=(.*), ac=(.*)$')


def callback(data):
    global ser, move_pattern, set_pattern
    if data.startswith("Stop"):
        # Stop after the current command is done
        if data.endswith("next"):
            ser.write("\xEE\x00")
            # Stop now
        else:
            ser.write("\xEE\x01")
    elif data.startswith("Check battery"):
        # Check battery levels
        ser.write("\xEE\x30")
    # Example: Set r=NUMBER, a=NUMBER
    elif data.startswith("Set"):
        # Set speed, accel, centrip accel

        # Use regex to parse floats from the command string
        match = set_pattern.match(data)

        # Put everything into a byte array for serial communication
        buf = bytearray()
        buf.extend(b'\xEE\x11')

        # Cast the floats in the string to floats and then bytes for packetization
        for i in range(0, 3):
            buf.extend(bytes(float(match.group(i))))
        ser.write(bytes(buffer))
    elif data.startswith("Move"):
        # Move along a circular path with a radius and ang;e
        match = move_pattern.match(data)
        match.group(0)
        buf = bytearray()
        buf.extend(b'\xEE\x21')
        for i in range(0, 2):
            buf.extend(bytes(float(match.group(i))))
        ser.write(bytes(buf))


def listener():
    # Initialize the serial port

    rospy.init_node('arduino_controller', anonymous=True)

    # Subscribe to the arduino_commands topic and never exit
    rospy.Subscriber("arduino_commands", String, callback)
    rospy.spin()


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
pass
