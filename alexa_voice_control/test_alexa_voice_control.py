#!/usr/bin/env python
import sys
import unittest
import rospy
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
import logging
import json
import os
from move_base_msgs.msg import MoveBaseActionGoal    

package_name = 'alexa_voice_control'
test_name = 'alexa_voice_control'
package_name = 'test_alexa_voice_control'

class TestAlexaVoiceControl(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestAlexaVoiceControl, self).__init__(*args, **kwargs)
        
        
        rospy.init_node('test_alexa', anonymous=True)

        # set up AWS constants
        if rospy.has_param("~host"):
            self.host = rospy.get_param("host")
        else:
            self.host = 'a1vgqh9vgvjzyh.iot.us-east-1.amazonaws.com'
        if rospy.has_param("~rootCAPath"):
            self.rootCAPath = os.path.dirname(os.path.abspath(__file__)) + rospy.get_param("~rootCAPath")
        else:
            self.rootCAPath = os.path.dirname(os.path.abspath(__file__)) + '/Certificates/root-CA.crt'
        if rospy.has_param("~certificatePath"):
            self.certificatePath = os.path.dirname(os.path.abspath(__file__)) + rospy.get_param("~certificatePath")
        else:
            self.certificatePath = os.path.dirname(os.path.abspath(__file__)) + '/Certificates/Pi.cert.pem'
        if rospy.has_param("~privateKeyPath"):
            self.privateKeyPath = os.path.dirname(os.path.abspath(__file__)) + rospy.get_param("~privateKeyPath")
        else:
            self.privateKeyPath = os.path.dirname(os.path.abspath(__file__)) + '/Certificates/Pi.private.key'
        if rospy.has_param("~clientId"):
            self.clientId = rospy.get_param("~clientId")
        else:
            self.clientId = 'test_pi'
        if rospy.has_param("~topic"):
            self.topic = rospy.get_param("~topic")
        else:
            self.topic = '/Transnavigators/Pi'

        
        
        # set up aws iot
        rospy.loginfo("Connecting to AWS")
    
        # Configure logging
        logger = logging.getLogger("AWSIoTPythonSDK.core")
        logger.setLevel(logging.WARN)
        stream_handler = logging.StreamHandler()
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        stream_handler.setFormatter(formatter)
        logger.addHandler(stream_handler)

        self.aws_iot_mqtt_client = AWSIoTMQTTClient(self.clientId)
        self.aws_iot_mqtt_client.configureEndpoint(self.host, 8883)
        self.aws_iot_mqtt_client.configureCredentials(self.rootCAPath, self.privateKeyPath, self.certificatePath)

        # AWSIoTMQTTClient connection configuration
        self.aws_iot_mqtt_client.configureAutoReconnectBackoffTime(1, 32, 20)
        self.aws_iot_mqtt_client.configureOfflinePublishQueueing(-1)  # Infinite offline Publish queueing
        self.aws_iot_mqtt_client.configureDrainingFrequency(2)  # Draining: 2 Hz
        self.aws_iot_mqtt_client.configureConnectDisconnectTimeout(10)  # 10 sec
        self.aws_iot_mqtt_client.configureMQTTOperationTimeout(5)  # 5 sec

        # Connect to AWS IoT
        self.aws_iot_mqtt_client.connect()
    
    ## test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

    def test_move_forward(self):
        sub = rospy.Subscriber("/cmd_vel/goal", MoveBaseActionGoal, self.move_forward_callback)
        message = json.dumps({"type" : "forward"})
        self.aws_iot_mqtt_client.publish(self.topic, message, 1)
        sub.unregister()
        
    def move_forward_callback(self, msg):
        
        assertEqual(msg.goal.pose.x, 100000.0,"pose.x")
        assertEqual(msg.goal.pose.y, 0.0,"pose.y")
        assertEqual(msg.goal.pose.z, 0.0,"pose.z")
        assertEqual(msg.goal.orientation.x, 0.0,"orientation.x")
        assertEqual(msg.goal.orientation.y 0.0,"orientation.y")
        assertEqual(msg.goal.orientation.z, 0.0,"orientation.z")
        assertEqual(msg.goal.orientation.w, 0.0,"orientation.w")
        
    
    def test_stop(self):
        sub = rospy.Subscriber("/cmd_vel/goal", MoveBaseActionGoal, self.stop_callback)
        message = json.dumps({"type" : "stop"})
        self.aws_iot_mqtt_client.publish(self.topic, message, 1)
        sub.unregister()
        
    def stop_callback(self, msg):
        
        assertEqual(msg.goal.pose.x, 0.0,"pose.x")
        assertEqual(msg.goal.pose.y, 0.0,"pose.y")
        assertEqual(msg.goal.pose.z, 0.0,"pose.z")
        assertEqual(msg.goal.orientation.x, 0.0,"orientation.x")
        assertEqual(msg.goal.orientation.y, 0.0,"orientation.y")
        assertEqual(msg.goal.orientation.z, 0.0,"orientation.z")
        assertEqual(msg.goal.orientation.w, 0.0,"orientation.w")
        
if __name__ == '__main__':
    import rostest
    rostest.rosrun(package_name, test_name, TestAlexaVoiceControl)