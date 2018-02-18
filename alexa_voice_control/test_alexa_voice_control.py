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
from geometry_msgs.msg import Pose

package_name = 'alexa_voice_control'
test_name = 'alexa_voice_control'
package_name = 'test_alexa_voice_control'


# list of all expected poses
POSE_LIST = []

class TestAlexaVoiceControl(unittest.TestCase):
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

    def test_move_forward(self):
        global POSE_LIST
        self.done = False
        # set up aws iot
        self.action_server = actionlib.SimpleActionServer('move_base', MoveBaseAction, self.callback)
        # sub = rospy.Subscriber("/cmd_vel/goal", MoveBaseAction, self.callback)
        # start client
        rospy.sleep(3)

        # test forward
        
        # create and add the expected pose
        pose = Pose()
        pose.position.x = 100000
        pose.position.y = 0
        pose.position.z = 0
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 0
        
        
        POSE_LIST.append(pose)
        
        message = json.dumps({"type": "forward"})
        aws_iot_mqtt_client.publish(topic, message, 1)

        rospy.sleep(3)
        self.assertTrue(self.done)
        
    def test_stop(self):
        global POSE_LIST
        self.done = False
        # set up aws iot
        self.action_server = actionlib.SimpleActionServer('move_base', MoveBaseAction, self.callback)
        # sub = rospy.Subscriber("/cmd_vel/goal", MoveBaseAction, self.callback)
        # start client
        rospy.sleep(3)

        # test stop
        
        # create and add the expected pose
        pose = Pose()
        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = 0
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 0
        
        
        POSE_LIST.append(pose)
        
        message = json.dumps({"type": "stop"})
        aws_iot_mqtt_client.publish(topic, message, 1)

        rospy.sleep(3)
        self.assertTrue(self.done)
        
        
    def callback(self, goal):
        global POSE_LIST
        rospy.loginfo("In callback")
        for pose in POSE_LIST:
            if pose.position.x == goal.target_pose.pose.position.x \
            and pose.position.y == goal.target_pose.pose.position.y \
            and pose.position.z == goal.target_pose.pose.position.z \
            and pose.orientation.x == goal.target_pose.pose.orientation.x \
            and pose.orientation.y == goal.target_pose.pose.orientation.y \
            and pose.orientation.z == goal.target_pose.pose.orientation.z \
            and pose.orientation.w == goal.target_pose.pose.orientation.w:
                POSE_LIST.remove(pose)
                self.done = True
                # self.action_server.set_succeeded()


if __name__ == '__main__':
    import rostest

    rospy.init_node('test_alexa', anonymous=True)
    
    # set up AWS constants
    if rospy.has_param("~host"):
        host = rospy.get_param("~host")
    else:
        host = 'a1vgqh9vgvjzyh.iot.us-east-1.amazonaws.com'
    if rospy.has_param("~rootCAPath"):
        rootCAPath = os.path.dirname(os.path.abspath(__file__)) + rospy.get_param("~rootCAPath")
    else:
        rootCAPath = os.path.dirname(os.path.abspath(__file__)) + '/Certificates/root-CA.crt'
    if rospy.has_param("~certificatePath"):
        certificatePath = os.path.dirname(os.path.abspath(__file__)) + rospy.get_param("~certificatePath")
    else:
        certificatePath = os.path.dirname(os.path.abspath(__file__)) + '/Certificates/Pi.cert.pem'
    if rospy.has_param("~privateKeyPath"):
        privateKeyPath = os.path.dirname(os.path.abspath(__file__)) + rospy.get_param("~privateKeyPath")
    else:
        privateKeyPath = os.path.dirname(os.path.abspath(__file__)) + '/Certificates/Pi.private.key'
    if rospy.has_param("~clientId"):
        clientId = rospy.get_param("~clientId")
    else:
        clientId = 'test_pi'
    if rospy.has_param("~topic"):
        topic = rospy.get_param("~topic")
    else:
        topic = '/Transnavigators/Pi'
    
    aws_iot_mqtt_client = AWSIoTMQTTClient(clientId)
    aws_iot_mqtt_client.configureEndpoint(host, 8883)
    aws_iot_mqtt_client.configureCredentials(rootCAPath, privateKeyPath, certificatePath)

    # AWSIoTMQTTClient connection configuration
    aws_iot_mqtt_client.configureAutoReconnectBackoffTime(1, 32, 20)
    aws_iot_mqtt_client.configureOfflinePublishQueueing(-1)  # Infinite offline Publish queueing
    aws_iot_mqtt_client.configureDrainingFrequency(2)  # Draining: 2 Hz
    aws_iot_mqtt_client.configureConnectDisconnectTimeout(10)  # 10 sec
    aws_iot_mqtt_client.configureMQTTOperationTimeout(5)  # 5 sec
    
    # Connect to AWS IoT
    aws_iot_mqtt_client.connect()
    rospy.loginfo("Connecting to AWS")
    # run tests
    rostest.rosrun(package_name, test_name, TestAlexaVoiceControl)
