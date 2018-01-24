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


deviceShadowHandler = None
pub = None

# Custom MQTT message callback
def customCallback(client, userdata, message):
    print("Received a new message: ")
    print(message.payload)
    print("from topic: ")
    print(message.topic)
    print("--------------\n\n")

def myCallback(payload, responseStatus, token):
    global deviceShadowHandler
    global pub

    data = json.loads(payload)['state']
    
    print(data);
    # Delete shadow JSON doc
    deviceShadowHandler.shadowDelete(customShadowCallback_Delete, 5)
    
    # publish data
    data = json.dumps(data)
    rospy.loginfo(data)
    pub.publish(data)

def customShadowCallback_Delete(payload, responseStatus, token):
    if responseStatus == "timeout":
        print("Delete request " + token + " time out!")
    if responseStatus == "accepted":
        print("~~~~~~~~~~~~~~~~~~~~~~~")
        print("Delete request with token: " + token + " accepted!")
        print("~~~~~~~~~~~~~~~~~~~~~~~\n\n")
    if responseStatus == "rejected":
        print("Delete request " + token + " rejected!")



def talker():
    global deviceShadowHandler
    global pub

    pub = rospy.Publisher('voice_control', String, queue_size=10)
    rospy.init_node('alexa', anonymous=True)
        

    host = 'a1vgqh9vgvjzyh.iot.us-east-1.amazonaws.com'
    rootCAPath = os.path.dirname(os.path.abspath(__file__)) + '/Certificates/root-CA.crt'
    certificatePath = os.path.dirname(os.path.abspath(__file__)) + '/Certificates/Pi.cert.pem'
    privateKeyPath = os.path.dirname(os.path.abspath(__file__)) + '/Certificates/Pi.private.key'
    clientId = 'Pi'
    topic = '/get/accepted'
    
    #Configure logging
    logger = logging.getLogger("AWSIoTPythonSDK.core")
    logger.setLevel(logging.INFO)
    streamHandler = logging.StreamHandler()
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    streamHandler.setFormatter(formatter)
    logger.addHandler(streamHandler)

    # Shadow Client
    # Init AWSIoTMQTTClient
    myAWSIoTMQTTShadowClient = AWSIoTMQTTShadowClient(clientId)
    myAWSIoTMQTTShadowClient.configureEndpoint(host, 8883)
    myAWSIoTMQTTShadowClient.configureCredentials(rootCAPath, privateKeyPath, certificatePath)

    # AWSIoTMQTTClient connection configuration
    myAWSIoTMQTTShadowClient.configureAutoReconnectBackoffTime(1, 32, 20)
    myAWSIoTMQTTShadowClient.configureConnectDisconnectTimeout(10)  # 10 sec
    myAWSIoTMQTTShadowClient.configureMQTTOperationTimeout(5)  # 5 sec

    myAWSIoTMQTTShadowClient.connect()
    deviceShadowHandler = myAWSIoTMQTTShadowClient.createShadowHandlerWithName("Pi", True)
    deviceShadowHandler.shadowRegisterDeltaCallback(myCallback);
    
    # wait
    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
