#!/usr/bin/env python
import sys
import unittest
import rospy
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
import json
import os
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

package_name = 'alexa_voice_control'
test_name = 'alexa_voice_control'
package_name = 'test_alexa_voice_control'


# list of all expected poses



class TestAlexaVoiceControl(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_alexa', anonymous=True)

        # set up AWS constants
        if rospy.has_param("~host"):
            host = rospy.get_param("~host")
        else:
            host = 'a1vgqh9vgvjzyh.iot.us-east-1.amazonaws.com'
        if rospy.has_param("~rootCAPath"):
            root_ca_path = os.path.dirname(os.path.abspath(__file__)) + rospy.get_param("~rootCAPath")
        else:
            root_ca_path = os.path.dirname(os.path.abspath(__file__)) + '/Certificates/root-CA.crt'
        if rospy.has_param("~certificatePath"):
            certificate_path = os.path.dirname(os.path.abspath(__file__)) + rospy.get_param("~certificatePath")
        else:
            certificate_path = os.path.dirname(os.path.abspath(__file__)) + '/Certificates/Pi.cert.pem'
        if rospy.has_param("~privateKeyPath"):
            private_key_path = os.path.dirname(os.path.abspath(__file__)) + rospy.get_param("~privateKeyPath")
        else:
            private_key_path = os.path.dirname(os.path.abspath(__file__)) + '/Certificates/Pi.private.key'
        if rospy.has_param("~clientId"):
            client_id = rospy.get_param("~clientId")
        else:
            client_id = 'test_pi'
        if rospy.has_param("~topic"):
            self.topic = rospy.get_param("~topic")
        else:
            self.topic = '/Transnavigators/Pi'

        self.aws_iot_mqtt_client = AWSIoTMQTTClient(client_id)
        self.aws_iot_mqtt_client.configureEndpoint(host, 8883)
        self.aws_iot_mqtt_client.configureCredentials(root_ca_path, private_key_path, certificate_path)

        # AWSIoTMQTTClient connection configuration
        self.aws_iot_mqtt_client.configureAutoReconnectBackoffTime(1, 32, 20)
        self.aws_iot_mqtt_client.configureOfflinePublishQueueing(-1)  # Infinite offline Publish queueing
        self.aws_iot_mqtt_client.configureDrainingFrequency(2)  # Draining: 2 Hz
        self.aws_iot_mqtt_client.configureConnectDisconnectTimeout(10)  # 10 sec
        self.aws_iot_mqtt_client.configureMQTTOperationTimeout(5)  # 5 sec

        # Connect to AWS IoT
        self.aws_iot_mqtt_client.connect()
        rospy.loginfo("Connecting to AWS")

        self.action_server = actionlib.SimpleActionServer('move_base', MoveBaseAction, self.callback)
        self.POSE_LIST = []

    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

    def test_move_forward(self):
        # self.done = False
        # set up aws iot
        # self.action_server = actionlib.SimpleActionServer('move_base', MoveBaseAction, self.callback)
        # sub = rospy.Subscriber("/cmd_vel/goal", MoveBaseAction, self.callback)
        # start client
        rospy.sleep(3)

        # test forward

        # create and add the expected pose
        pose = Pose()
        pose.position = Point(100000, 0, 0)
        pose.orientation = Quaternion(1, 0, 0, 0)

        self.POSE_LIST.append(pose)

        message = json.dumps({"type": "forward"})
        self.aws_iot_mqtt_client.publish(self.topic, message, 1)

        rospy.sleep(3)
        for new_pose in self.POSE_LIST:
            if self.pose_equals(pose, new_pose):
                self.fail("Message wasn't received")

    def test_stop(self):
        # set up aws iot

        # sub = rospy.Subscriber("/cmd_vel/goal", MoveBaseAction, self.callback)
        # start client
        rospy.sleep(3)

        # test stop

        # create and add the expected pose
        pose = Pose()
        pose.position = Point(0, 0, 0)
        pose.orientation = Quaternion(1, 0, 0, 0)

        self.POSE_LIST.append(pose)

        message = json.dumps({"type": "stop"})
        self.aws_iot_mqtt_client.publish(self.topic, message, 1)

        rospy.sleep(3)
        for new_pose in self.POSE_LIST:
            if self.pose_equals(pose, new_pose):
                self.fail("Message wasn't received")

    def callback(self, goal):
        rospy.loginfo("In callback")
        recv_pose = goal.target_pose.pose
        for pose in self.POSE_LIST:
            if self.pose_equals(pose, recv_pose):
                self.POSE_LIST.remove(pose)
                #self.action_server.set_succeeded()

    def pose_equals(self, pose1, pose2):
        if abs(pose1.position.x - pose2.position.x) < 0.01 \
                and abs(pose1.position.y == pose2.position.y) < 0.01 \
                and abs(pose1.position.z == pose2.position.z) < 0.01 \
                and abs(pose1.orientation.x == pose2.orientation.x) < 0.01 \
                and abs(pose1.orientation.y == pose2.orientation.y) < 0.01 \
                and abs(pose1.orientation.z == pose2.orientation.z) < 0.01 \
                and abs(pose1.orientation.w == pose2.orientation.w) < 0.01:
            return True
        else:
            return False


if __name__ == '__main__':
    import rostest

    # run tests
    rostest.rosrun(package_name, test_name, TestAlexaVoiceControl)
