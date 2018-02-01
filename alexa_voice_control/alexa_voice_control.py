#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTShadowClient
import logging
import time
import argparse
import json
import os

class Alexa:
    def __init__(self):
        # create a new node that publishes on topic voice_control
        self.pub = rospy.Publisher('voice_control', String, queue_size=10)
        rospy.init_node('alexa', anonymous=True)
        
        # set up constants
        self.host = 'a1vgqh9vgvjzyh.iot.us-east-1.amazonaws.com'
        self.rootCAPath = os.path.dirname(os.path.abspath(__file__)) + '/Certificates/root-CA.crt'
        self.certificatePath = os.path.dirname(os.path.abspath(__file__)) + '/Certificates/Pi.cert.pem'
        self.privateKeyPath = os.path.dirname(os.path.abspath(__file__)) + '/Certificates/Pi.private.key'
        self.clientId = 'Pi'
        self.topic = '/get/accepted'
        
    def callback(self, payload, responseStatus, token):
        data = json.loads(payload)['state']

        # Delete shadow JSON doc
        self.deviceShadowHandler.shadowDelete(self.callbackDelete, 5)

        # publish data
        data = json.dumps(data)
        rospy.loginfo(data)
        self.pub.publish(data)


    def callbackDelete(self, payload, responseStatus, token):
        if responseStatus == "accepted":
            pass
        if responseStatus == "timeout":
            print("Delete request " + token + " time out")
        if responseStatus == "rejected":
            print("Delete request " + token + " rejected")


    def begin(self):

        # Configure logging
        logger = logging.getLogger("AWSIoTPythonSDK.core")
        logger.setLevel(logging.WARN)
        streamHandler = logging.StreamHandler()
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        streamHandler.setFormatter(formatter)
        logger.addHandler(streamHandler)

        # Shadow Client
        # Init AWSIoTMQTTClient
        myAWSIoTMQTTShadowClient = AWSIoTMQTTShadowClient(self.clientId)
        myAWSIoTMQTTShadowClient.configureEndpoint(self.host, 8883)
        myAWSIoTMQTTShadowClient.configureCredentials(self.rootCAPath, self.privateKeyPath, self.certificatePath)

        # AWSIoTMQTTClient connection configuration
        myAWSIoTMQTTShadowClient.configureAutoReconnectBackoffTime(1, 32, 20)
        myAWSIoTMQTTShadowClient.configureConnectDisconnectTimeout(10)  # 10 sec
        myAWSIoTMQTTShadowClient.configureMQTTOperationTimeout(5)  # 5 sec

        myAWSIoTMQTTShadowClient.connect()
        self.deviceShadowHandler = myAWSIoTMQTTShadowClient.createShadowHandlerWithName("Pi", True)
        self.deviceShadowHandler.shadowRegisterDeltaCallback(self.callback);

        # wait
        rospy.spin()


if __name__ == '__main__':
    try:
        alexa = Alexa();
        alexa.begin();
    except rospy.ROSInterruptException:
        pass
