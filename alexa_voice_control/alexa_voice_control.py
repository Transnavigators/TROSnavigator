#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTShadowClient
import logging
import json
import os
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf import TransformListener

class Alexa:
    # set up constants
    def __init__(self):
        # create a new node
        rospy.init_node('alexa', anonymous=True)
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.tf = TransformListener()

        if rospy.has_param("~user_tag_id"):
            self.tag_id = rospy.get_param("~user_tag_id")
        else:
            self.tag_id = '1001'

        self.FEET_TO_M = 0.3048
        self.DEGREES_TO_RAD = math.pi/180

        # set up constants
        self.host = 'a1vgqh9vgvjzyh.iot.us-east-1.amazonaws.com'
        self.rootCAPath = os.path.dirname(os.path.abspath(__file__)) + '/Certificates/root-CA.crt'
        self.certificatePath = os.path.dirname(os.path.abspath(__file__)) + '/Certificates/Pi.cert.pem'
        self.privateKeyPath = os.path.dirname(os.path.abspath(__file__)) + '/Certificates/Pi.private.key'
        self.clientId = 'Pi'
        self.topic = '/get/accepted'
        self.deviceShadowHandler = None

        # callback for receiving AWS message
    def callback(self, payload, response_status, token):
        data = json.loads(payload)['state']

        # Delete shadow JSON doc
        self.deviceShadowHandler.shadowDelete(self.callback_delete, 5)

        # publish data
        dataStr = json.dumps(data)
        rospy.loginfo(dataStr)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.z = 0

        # Move forward
        if data.type == 'forward':
            if 'distance' in data and 'distanceUnit' in data:
                goal.target_pose.pose.position.x = float(data.distance)
                if data.angleUnit == 'feet':
                    goal.target_pose.pose.position.x *= self.FEET_TO_M
            else:
                goal.target_pose.pose.position.x = 100000.0

        # Turn the wheelchair
        elif data.type == 'turn':
            angle = float(data.angle)
            if data.angleUnit == 'degrees':
                angle *= self.DEGREES_TO_RAD
            goal_quat = quaternion_from_euler(0, 0, angle)
            goal.target_pose.pose.orientation = Quaternion(goal_quat[0], goal_quat[1], goal_quat[2], goal_quat[3])

        # Stop the wheelchair
        elif data.type == 'stop':
            goal.target_pose.pose.position = Point(0, 0, 0)
            goal.target_pose.pose.orientation.x = Quaternion(0, 0, 0, 1)

        # Go to the localino tag
        elif data.type == 'locateme' and self.tf.frameExists("/base_link") and self.tf.frameExists("/tag_"+self.tag_id):
            t = self.tf.getLatestCommonTime("/base_link", '/tag_' + self.tag_id)
            pos, quat = self.tf.lookupTransform("/base_link", "/tag_"+self.tag_id, t)

            # Go to the target, don't care about rotation
            goal.target_pose.pose.position = pos
            goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)

        # Go to the static landmark
        elif data.type == 'moveto' and self.tf.frameExists("/map") and self.tf.frameExists("/%s" % str(data.location)):
            t = self.tf.getLatestCommonTime("/base_link", "/%s" % str(data.location))
            pos, quat = self.tf.lookupTransform("/base_link", "/%s" % str(data.location), t)

            # Go to the target, don't care about rotation
            goal.target_pose.pose.position = pos
            goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        else:
            rospy.logerror("Could not find transform from base_link to map")

        self.action_client.send_goal(goal)

    # callback for removing message
    def callback_delete(self, payload, responseStatus, token):
        if responseStatus == "accepted":
            pass
        if responseStatus == "timeout":
            print("Delete request " + token + " time out")
        if responseStatus == "rejected":
            print("Delete request " + token + " rejected")

    # sets up communication with AWS
    def begin(self):


        # Configure logging
        logger = logging.getLogger("AWSIoTPythonSDK.core")
        logger.setLevel(logging.WARN)
        stream_handler = logging.StreamHandler()
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        stream_handler.setFormatter(formatter)
        logger.addHandler(stream_handler)

        # Shadow Client
        # Init AWSIoTMQTTClient
        aws_iot_mqtt_shadow_client = AWSIoTMQTTShadowClient(self.clientId)
        aws_iot_mqtt_shadow_client.configureEndpoint(self.host, 8883)
        aws_iot_mqtt_shadow_client.configureCredentials(self.rootCAPath, self.privateKeyPath, self.certificatePath)

        # AWSIoTMQTTClient connection configuration
        aws_iot_mqtt_shadow_client.configureAutoReconnectBackoffTime(1, 32, 20)
        aws_iot_mqtt_shadow_client.configureConnectDisconnectTimeout(10)  # 10 sec
        aws_iot_mqtt_shadow_client.configureMQTTOperationTimeout(5)  # 5 sec

        aws_iot_mqtt_shadow_client.connect()
        self.deviceShadowHandler = aws_iot_mqtt_shadow_client.createShadowHandlerWithName("Pi", True)
        self.deviceShadowHandler.shadowRegisterDeltaCallback(self.callback)

        self.action_client.wait_for_server()

        # wait
        rospy.spin()


if __name__ == '__main__':
    try:
        alexa = Alexa()
        alexa.begin()
    except rospy.ROSInterruptException:
        pass
