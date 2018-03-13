#!/usr/bin/env python

# Modified From https://github.com/mwilliams03/BerryIMU
import math
import IMU
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Vector3, Quaternion


class BerryIMUPublisher:
    def __init__(self):
        rospy.init_node('berryimu', anonymous=True)
        self.RAD_TO_DEG = 180 / math.pi
        self.M_PI = 3.14159265358979323846
        self.ACC_TO_MS2 = (1.0 / (0.101972 * 2 ** 11))  # 2^15 = 16G, 1G ~= 9.8m/s^2
        self.G_GAIN = 0.070  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
        self.GYRO_TO_RADS = 1 / self.RAD_TO_DEG
        self.pub = rospy.Publisher("imu_data", Imu, queue_size=1000)

        self.rate = rospy.Rate(int(rospy.get_param("~poll_rate",20)))
        # wait for other i2c devices
        rospy.sleep(2.)
        # IMU.detectIMU()  # Detect if BerryIMUv1 or BerryIMUv2 is connected.

        IMU.initIMU()  # Initialise the accelerometer, gyroscope and compass

    def begin(self):

        gyro_x_angle = 0.0
        gyro_y_angle = 0.0
        gyro_z_angle = 0.0

        last_timestamp = rospy.get_time()

        while not rospy.is_shutdown():

            # Read the accelerometer,gyroscope and magnetometer values
            acc_x = IMU.readACCx()
            acc_y = IMU.readACCy()
            acc_z = IMU.readACCz()
            gyr_x = IMU.readGYRx()
            gyr_y = IMU.readGYRy()
            gyr_z = IMU.readGYRz()
            mag_x = IMU.readMAGx()
            mag_y = IMU.readMAGy()
            mag_z = IMU.readMAGz()

            # Calculate loop Period(LP). How long between Gyro Reads
            period = (rospy.get_time() - last_timestamp)
            last_timestamp = rospy.get_time()

            # Convert Gyro raw to degrees per second
            rate_gyr_x = gyr_x * self.G_GAIN
            rate_gyr_y = gyr_y * self.G_GAIN
            rate_gyr_z = gyr_z * self.G_GAIN

            # Calculate the angles from the gyro.
            gyro_x_angle += rate_gyr_x * period
            gyro_y_angle += rate_gyr_y * period
            gyro_z_angle += rate_gyr_z * period

            # Calculate heading
            heading = math.atan2(mag_y, mag_x) * self.RAD_TO_DEG

            # Only have our heading between 0 and 360
            if heading < 0:
                heading += 360

            # Normalize accelerometer raw values.
            norm = math.sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z)
            if norm > 1e-9:
                acc_x_norm = acc_x / norm
                acc_y_norm = acc_y / norm

                # Calculate pitch and roll
                pitch = math.asin(acc_x_norm)
                roll = -math.asin(acc_y_norm / math.cos(pitch))

                # Calculate the new tilt compensated values
                mag_x_comp = mag_x * math.cos(pitch) + mag_z * math.sin(pitch)
                mag_y_comp = mag_x * math.sin(roll) * math.sin(pitch) + mag_y * math.cos(roll) - mag_z * math.sin(
                    roll) * math.cos(pitch)

                # Calculate tilt compensated heading
                tilt_compensated_heading = math.atan2(mag_y_comp, mag_x_comp) * self.RAD_TO_DEG

                if tilt_compensated_heading < 0:
                    tilt_compensated_heading += 360

                msg = Imu()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "base_link"

                # Convert from euler to quaternion for ROS
                q = quaternion_from_euler(gyro_x_angle, gyro_y_angle, gyro_z_angle)
                msg.orientation = Quaternion(q[0], q[1], q[2], q[3])
                # TODO: measure covariance
                msg.orientation_covariance = [99999, 0, 0,  # covariance on x axis
                                              0, 99999, 0,  # covariance on y axis
                                              0, 0, 99999]  # covariance on z axis

                msg.angular_velocity = Vector3(rate_gyr_x * self.GYRO_TO_RADS, rate_gyr_y * self.GYRO_TO_RADS,
                                               rate_gyr_z * self.GYRO_TO_RADS)
                msg.angular_velocity_covariance = [99999, 0, 0,  # covariance on x axis
                                                   0, 99999, 0,  # covariance on y axis
                                                   0, 0, 99999]  # covariance on z axis

                msg.orientation_covariance = [99999, 0, 0,  # covariance on x axis
                                              0, 99999, 0,  # covariance on y axis
                                              0, 0, 99999]  # covariance on z axis

                msg.linear_acceleration = Vector3(acc_x * self.ACC_TO_MS2, acc_y * self.ACC_TO_MS2, acc_z * self.ACC_TO_MS2)
                msg.linear_acceleration_covariance = [99999, 0, 0,  # covariance on x axis
                                                      0, 99999, 0,  # covariance on y axis
                                                      0, 0, 99999]  # covariance on z axis

                rospy.loginfo_throttle(1, "Heading: %s" % tilt_compensated_heading)

                # self.imu_broadcaster.sendTransform((x, y, 0.), odom_quat, current_time, "base_link", "odom")
                # rospy.logdebug("BerryIMU orientation,%5.2f,%5.2f,%5.2f" % gyro_x_angle, gyro_y_angle, gyro_z_angle)
                # rospy.logdebug("BerryIMU velocity,%5.2f,%5.2f,5.2f" % msg.angular_velocity.x, msg.angular_velocity.y,
                #               msg.angular_velocity.z)
                # rospy.logdebug("BerryIMU acceleration,%5.2f,%5.2f,%5.2f" % msg.linear_acceleration.x,
                #               msg.linear_acceleration.y, msg.linear_acceleration.z)
                self.pub.publish(msg)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        bimu = BerryIMUPublisher()
        bimu.begin()
    except rospy.ROSInterruptException:
        pass
