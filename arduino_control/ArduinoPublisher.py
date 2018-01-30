import os
import rospy
import serial
from PyCRC.CRC16 import CRC16


class ArduinoPublisher:
    def __init__(self):
        port_name = '/dev/ttyUSB1'
        if os.name == 'nt':
            port_name = 'COM6'
        self.ser = serial.Serial(port=port_name, baudrate=115200, timeout=0)
        self.pub = rospy.Publisher('arduino_publisher', String, queue_size=10)
        rospy.init_node('arduino', anonymous=True)

    def talker(self):
        rate = rospy.Rate(100)  # 100Hz, increase if needed
        while not rospy.is_shutdown():
            data = self.ser.read()
            if data == 0xEE:
                data = self.ser.read()
                if data == 0x01:
                    speed1 = self.ser.read(4)
                    speed2 = self.ser.read(4)
                    delta_time = self.ser.read(4)
                    crc = self.ser.read(2)
                    buf = bytearray()
                    buf.extend(speed1)
                    buf.extend(speed2)
                    buf.extend(delta_time)
                    calc_crc = CRC16().calculate(bytes(buffer))
                    if calc_crc != crc:
                        rospy.loginfo("Error: packet didn't pass checksum")
                    else:
                        self.pub.publish(
                            "speed1=" + int(speed1) + ", speed2=" + int(speed2) + ", time=" + int(delta_time))
            rate.sleep()


if __name__ == "__main__":
    try:
        ap = ArduinoPublisher()
        ap.talker()
    except rospy.ROSInterruptException:
        pass
