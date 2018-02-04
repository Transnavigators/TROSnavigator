#!/usr/bin/env python

import rospy
import serial
import re
from std_msgs.msg import String


class ArduinoController:
    def __init__(self):
        # Initialize the serial port
        rospy.init_node('arduino_controller', anonymous=True)
        # Subscribe to the arduino_commands topic
        rospy.Subscriber("arduino_commands", String, self.callback)

        # set up the port
        port_name = rospy.get_param("~port")
        self.ser = serial.Serial(port=port_name, baudrate=115200, timeout=0)

        # create regex
        self.move_pattern = re.compile('^Move r=(.*), a=(.*)$')
        self.set_pattern = re.compile('^Set s=(.*), a=(.*), ac=(.*)$')

    # callback for receiving data from the arduino
    def callback(self, data):
        if data.startswith("Stop"):
            # Stop after the current command is done
            if data.endswith("next"):
                self.ser.write("\xEE\x00")
                # Stop now
            else:
                self.ser.write("\xEE\x01")
        elif data.startswith("Check battery"):
            # Check battery levels
            self.ser.write("\xEE\x30")
        # Example: Set r=NUMBER, a=NUMBER
        elif data.startswith("Move"):
            # Move along a circular path with a radius and angle
            match = self.move_pattern.match(data)
            match.group(0)
            buf = bytearray()
            buf.extend(b'\xEE\x21')
            for i in range(0, 2):
                buf.extend(bytes(float(match.group(i))))
            self.ser.write(bytes(buf))

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
