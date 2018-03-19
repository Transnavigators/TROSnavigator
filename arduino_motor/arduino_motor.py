#!/usr/bin/env python

# modified from https://answers.ros.org/question/209963/cmd_veltwist-transform-twist-message-into-left-and-right-motor-commands/
import smbus
import rospy
import struct
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from std_msgs.msg import UInt8MultiArray, MultiArrayDimension


class ArduinoMotor:
    """Sends commands to the Arduino to relay to the motor controller
    
    """
    def __init__(self):
        # Initialize the serial port
        rospy.init_node('arduino_motor', anonymous=True)
        rospy.loginfo("%s started" % rospy.get_name())

        # Get params
        self.width = float(rospy.get_param("~width", 31.5 * 0.0254))
        self.rate = rospy.Rate(int(rospy.get_param("~poll_rate", 10)))
        self.retry_limit = int(rospy.get_param("~retry_limit", 1))
        self.reset_pin = int(rospy.get_param("reset_pin", 4))
        self.err_count = 0
        try:
            global GPIO
            import RPi.GPIO as GPIO
            # Setup pin 4 as an output pin for resetting the Arduino
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(4, GPIO.OUT)
            self.on_arduino = True
        except (ImportError, RuntimeError):
            rospy.logwarn("Not running on Raspberry Pi, so cannot reset Arduino")
            self.on_arduino = False
            pass
        # Reset the Arduino
        self.reset_arduino()
        self.is_virtual = int(rospy.get_param("~is_virtual", 0))
        # Setup the i2c bus
        if self.is_virtual:
            self.bus = smbus.SMBus(0)
            self.pub = rospy.Publisher('motorcmd', UInt8MultiArray, queue_size=10)
        else:
            self.bus = smbus.SMBus(1)

        # The address and command byte to use for i2c communication
        self.address = 0x04
        self.move_cmd = ord('m')

        rospy.Subscriber("cmd_vel", Twist, self.callback)

        self.left = 0
        self.right = 0

    def reset_arduino(self):
        if self.on_arduino:
            GPIO.output(self.reset_pin, GPIO.LOW)
            rospy.sleep(0.1)
            GPIO.output(self.reset_pin, GPIO.HIGH)
            rospy.sleep(0.1)

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
        if self.is_virtual:
            self.pub.publish(UInt8MultiArray(data=val))
            # rospy.loginfo(str(val))
        else:
            self.bus.write_i2c_block_data(self.address, self.move_cmd, val)

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
        last_err = ''
        while not rospy.is_shutdown() and count < self.retry_limit:
            try:
                if rospy.is_shutdown():
                    break
                self.send_speed_to_motor(float(self.left), float(self.right))
                rospy.loginfo_throttle(1, "Sent speed successfully")
                break
            except IOError as e:
                count += 1
                last_err = str(e)
                pass
        if count > 0:
            self.err_count += count
            rospy.logwarn("Failed to send speed %d times: %s" % (count, last_err))
            if self.err_count > 10:
                self.reset_arduino()


if __name__ == "__main__":
    try:
        controller = ArduinoMotor()
        controller.begin()
    except rospy.ROSInterruptException:
        pass
