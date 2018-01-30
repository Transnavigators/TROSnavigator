#!/usr/bin/env python

import rospy
import serial
import os
import re
from PyCRC.CRC16 import CRC16
from std_msgs.msg import String

port_name = '/dev/ttyUSB1'
if os.name == 'nt':
    port_name = 'COM6'
ser = serial.Serial(port=port_name, baudrate=115200, timeout=0)
move_pattern = re.compile('^Move r=(.*), a=(.*)$')
set_pattern = re.compile('^Set s=(.*), a=(.*), ac=(.*)$')
rospy.init_node('arduino', anonymous=True)


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
    # Subscribe to the arduino_commands topic and never exit
    rospy.Subscriber("arduino_commands", String, callback)

# TODO: multithread
def talker():
    global ser
    pub = rospy.Publisher('arduino_publisher', String, queue_size=10)
    rate = rospy.Rate(100)  # 100Hz, increase if needed
    while not rospy.is_shutdown():
        data = ser.read()
        if data == 0xEE:
            data = ser.read()
            if data == 0x01:
                speed1 = ser.read(4)
                speed2 = ser.read(4)
                delta_time = ser.read(4)
                crc = ser.read(2)
                buf = bytearray()
                buf.extend(speed1)
                buf.extend(speed2)
                buf.extend(delta_time)
                calc_crc = CRC16().calculate(bytes(buffer))
                if calc_crc != crc:
                    rospy.loginfo("Error: packet didn't pass checksum")
                else:
                    pub.publish(
                        "speed1=" + int(speed1) + ", speed2=" + int(speed2) + ", time=" + int(delta_time))
        rate.sleep()


if __name__ == "__main__":
    try:
        listener()
        talker()
    except rospy.ROSInterruptException:
        pass
