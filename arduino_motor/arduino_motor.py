#!/usr/bin/env python

# modified from https://answers.ros.org/question/209963/cmd_veltwist-transform-twist-message-into-left-and-right-motor-commands/
import smbus
import rospy
import struct

from geometry_msgs.msg import Twist, TransformStamped, Quaternion


class ArduinoMotor:
    """Sends commands to the Arduino to relay to the motor controller
    
    """
    def __init__(self):
        # Initialize the serial port
        rospy.init_node('arduino_motor', anonymous=True)
        rospy.loginfo("%s started" % rospy.get_name())

        # Get params
        self.width = float(rospy.get_param("~width", 31.5 * 0.0254))
        self.rate = rospy.Rate(int(rospy.get_param("~rate", 20)))
        self.retry_limit = int(rospy.get_param("~retry_limit", 10))
        reset_pin = int(rospy.get_param("reset_pin", 4))

        try:
            import RPi.GPIO as GPIO
            # Setup pin 4 as an output pin for resetting the Arduino
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(4, GPIO.OUT)

            # Reset the Arduino
            GPIO.output(reset_pin, GPIO.LOW)
            rospy.sleep(1.0)
            GPIO.output(reset_pin, GPIO.HIGH)
            rospy.sleep(1.0)
        except (ImportError, RuntimeError):
            rospy.logwarn("Not running on Raspberry Pi, so cannot reset Arduino")
            pass

        # Setup the i2c bus
        while not rospy.is_shutdown():
            try:
                self.bus = smbus.SMBus(1)
            except IOError:
                self.rate.sleep()

        # The address and command byte to use for i2c communication
        self.address = 0x04
        self.move_cmd = ord('m')

        rospy.Subscriber("cmd_vel", Twist, self.callback)

        self.left = 0
        self.right = 0

    def callback(self, msg):
        """Updates the linear and angular velocity instance variables
        
        The message definition can be found here: http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
        
        Args:
            msg (geometry_msgs.msg.Twist): The desired speed

        """
        dx = msg.linear.x
        dr = msg.angular.z
        self.right = 1.0 * dx + dr * self.width / 2
        self.left = 1.0 * dx - dr * self.width / 2
        if dx != 0 or dr != 0:
            rospy.loginfo("linear.x = " + str(dx) + " angular.z: " + str(dr))

    # send data to arduino
    def send_speed_to_motor(self, m1, m2):
        """
        
        Args:
            m1 (float): The left motor's speed in m/s 
            m2 (float): The right motor's speed in m/s 

        """
        val = list(bytearray(struct.pack("=ff", m1, m2)))
        self.bus.write_i2c_block_data(self.address, self.move_cmd, list(val))

    def begin(self):
        """Start the node and spin forever

        """

        # main loop
        while not rospy.is_shutdown():  # and self.ticks_since_target < self.timeout_ticks:
            self.spin_once()
            self.rate.sleep()

    def spin_once(self):
        """Sends the speeds to the motor once and retries if it fails

        """
        count = 0
        while not rospy.is_shutdown() and count < self.retry_limit:
            try:
                self.send_speed_to_motor(float(self.left), float(self.right))
                rospy.loginfo_throttle(1, "Sent speed successfully")
                break
            except IOError:
                count += 1
                pass


if __name__ == "__main__":
    try:
        controller = ArduinoMotor()
        controller.begin()
    except rospy.ROSInterruptException:
        pass
