#!/usr/bin/env python
import sys
import unittest
import rospy
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
import logging
import json
import os
import time
from move_base_msgs.msg import MoveBaseActionGoal    

package_name = 'alexa_voice_control'
test_name = 'alexa_voice_control'
package_name = 'test_alexa_voice_control'

class TestAlexaVoiceControl(unittest.TestCase):
    
    ## test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

    def test_alexa(self):
        self.done = False
        sub = rospy.Subscriber("/cmd_vel/goal", MoveBaseActionGoal, self.callback)
        
        
        # test forward
        message = json.dumps({"type" : "forward"})
        aws_iot_mqtt_client.publish(topic, message, 1)
        
        
        # message = json.dumps({"type" : "stop"})
        # aws_iot_mqtt_client.publish(topic, message, 1)
        
        time.sleep(10)
        self.assertEqual(self.done,"Timeout")
        
    def callback(self, msg):
        self.assertEqual(msg.goal.pose.x, 100000.0,"move_forward pose.x")
        self.assertEqual(msg.goal.pose.y, 0.0,"move_forward pose.y")
        self.assertEqual(msg.goal.pose.z, 0.0,"move_forward pose.z")
        self.assertEqual(msg.goal.orientation.x, 0.0,"move_forward orientation.x")
        self.assertEqual(msg.goal.orientation.y, 0.0,"move_forward orientation.y")
        self.assertEqual(msg.goal.orientation.z, 0.0,"move_forward orientation.z")
        self.assertEqual(msg.goal.orientation.w, 0.0,"move_forward orientation.w")
        self.done = True

        
    # def stop_callback(self, msg):
        
        # self.assertEqual(msg.goal.pose.x, 0.0,"stop pose.x")
        # self.assertEqual(msg.goal.pose.y, 0.0,"stop pose.y")
        # self.assertEqual(msg.goal.pose.z, 0.0,"stop pose.z")
        # self.assertEqual(msg.goal.orientation.x, 0.0,"stop orientation.x")
        # self.assertEqual(msg.goal.orientation.y, 0.0,"stop orientation.y")
        # self.assertEqual(msg.goal.orientation.z, 0.0,"stop orientation.z")
        # self.assertEqual(msg.goal.orientation.w, 0.0,"stop orientation.w")

        
if __name__ == '__main__':
    import rostest
    rospy.init_node('test_alexa', anonymous=True)

    # set up AWS constants
    if rospy.has_param("~host"):
        host = rospy.get_param("host")
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

    
    
    # set up aws iot
    rospy.loginfo("Connecting to AWS")
    
    # Configure logging
    logger = logging.getLogger("AWSIoTPythonSDK.core")
    logger.setLevel(logging.WARN)
    stream_handler = logging.StreamHandler()
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    stream_handler.setFormatter(formatter)
    logger.addHandler(stream_handler)

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
    rostest.rosrun(package_name, test_name, TestAlexaVoiceControl)