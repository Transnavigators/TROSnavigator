#!/usr/bin/env python

# modified from https://answers.ros.org/question/209963/cmd_veltwist-transform-twist-message-into-left-and-right-motor-commands/
import smbus
import rospy
import struct
from geometry_msgs.msg import Twist, TransformStamped, Quaternion


class ArduinoMotor:
    # commands for sending and receiving it

    def __init__(self):
        # Initialize the serial port
        rospy.init_node('arduino_motor', anonymous=True)
        rospy.loginfo("%s started" % rospy.get_name())

        self.width = rospy.get_param("~width", 31.5 * 0.0254)

        self.constant = 45.0
        self.bus = smbus.SMBus(1)

        self.move_cmd = ord('m')
        # self.encoder_cmd = ord('e')
        self.address = 0x04

        self.dx = 0
        self.dr = 0
        self.dy = 0

        rospy.Subscriber("cmd_vel", Twist, self.callback)

        self.rate = int(rospy.get_param("~rate", 20))
        # self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self.left = 0
        self.right = 0
        # self.ticks_since_target = 0

    def callback(self, msg):
        # rospy.loginfo("twist to motors:: twistCallback raw msg: %s" % str(msg))
        # self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y
        if self.dx != 0 or self.dr != 0 or self.dy != 0:
            rospy.loginfo("linear.x = " + str(self.dx) + " angular.z: " + str(self.dr) + " linear.y: " + str(self.dy))

    # send data to arduino
    def send_speed_to_motor(self, m1, m2):
        val1 = struct.unpack("=l", struct.pack("=ff", m1, m2))
        self.bus.write_i2c_block_data(self.address, self.move_cmd, [val1])

    # start the node: spin forever
    def begin(self):

        r = rospy.Rate(self.rate)
        # self.ticks_since_target = self.timeout_ticks

        # main loop
        while not rospy.is_shutdown():  # and self.ticks_since_target < self.timeout_ticks:
            self.spin_once()
            r.sleep()

    def spin_once(self):
        self.right = 1.0 * self.dx + self.dr * self.width / 2
        self.left = 1.0 * self.dx - self.dr * self.width / 2
        # rospy.loginfo("twist to motors:: spinOnce (dx:%f, dr: %f)", self.dx,self.dr)
        # rospy.loginfo("twist to motors:: spinOnce (self.left:%f,self.right %f)" % (self.left,self.right) )
        # rospy.loginfo("LEFT: " +str(int(self.left*self.constant))+"RIGHT: " +str(int(self.right*self.constant)))
        while not rospy.is_shutdown():
            try:
                self.send_speed_to_motor(float(self.left), float(self.right))
                rospy.loginfo_throttle(1, "Sent speed successfully")
                break
            except IOError as e:
                rospy.logwarn(e)
                pass
                # self.ticks_since_target += 1


if __name__ == "__main__":
    try:
        controller = ArduinoMotor()
        controller.begin()
    except rospy.ROSInterruptException:
        pass
