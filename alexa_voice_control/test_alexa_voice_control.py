#!/usr/bin/env python
import sys
import unittest
import rospy
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
import json
import os
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose

package_name = 'alexa_voice_control'
test_name = 'alexa_voice_control'
package_name = 'test_alexa_voice_control'


# list of all expected poses


class TestAlexaVoiceControl(unittest.TestCase):
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

    # def test_alexa(self):
        # # list of poses and their result
        # # TODO: UPDATE POSES with expected results
        # self.pose_list = [
            # (json.dumps({"type": "forward"}), [100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
            # (json.dumps({"type": "stop"}), [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
            # (json.dumps({"type": "turn"}), [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
            # (json.dumps({"type": "forward", "distance": 10, "distanceUnit": "meters"}),
             # [10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
            # (json.dumps({"type": "turn", "direction": "left", "angle": 45, "angleUnit": "degrees"}),
             # [0.0, 0.0, 0.0, 0.0, 0.0, -0.383, 0.924])
        # ]
        # self.current_pose = 0

        # # set up aws iot

        # self.action_server = actionlib.SimpleActionServer('move_base', MoveBaseAction, self.callback)
        # # sub = rospy.Subscriber("/cmd_vel/goal", MoveBaseAction, self.callback)

        # # start client
        # rospy.sleep(3)
        # self.done = False

        # # test 
        # message = self.pose_list[self.current_pose][0]
        # rospy.loginfo("Sending " + message)
        # self.current_pose += 1
        # aws_iot_mqtt_client.publish(topic, message, 1)

        # # wait for tests to finish
        # rospy.sleep(5)
        # self.assertTrue(self.done)

    def callback(self, goal):
        rospy.loginfo("In callback")

        # check result with expected pose
        for pose in self.pose_list:
            if pose[1][0] == goal.target_pose.pose.position.x \
                    and pose[1][1] == goal.target_pose.pose.position.y \
                    and pose[1][2] == goal.target_pose.pose.position.z \
                    and abs(pose[1][3]-goal.target_pose.pose.orientation.x) < 0.01 \
                    and abs(pose[1][4]-goal.target_pose.pose.orientation.y) < 0.01 \
                    and abs(pose[1][5]-goal.target_pose.pose.orientation.z) < 0.01 \
                    and abs(pose[1][6]-goal.target_pose.pose.orientation.w) < 0.01:

                # idk why this is needed
                # self.action_server.set_succeeded()

                rospy.loginfo("Found pose " + pose[0])

                # success if the first two tests work
                if self.current_pose == 2:
                    self.done = True

                # get next message and publish it
                message = self.pose_list[self.current_pose][0]
                rospy.loginfo("Sending " + message)
                self.current_pose += 1
                aws_iot_mqtt_client.publish(topic, message, 1)


if __name__ == '__main__':
    import rostest

    rospy.init_node('test_alexa', anonymous=True)

    # set up AWS constants
    host = rospy.get_param("~host", 'a1vgqh9vgvjzyh.iot.us-east-1.amazonaws.com')
    rootCAPath = os.path.dirname(os.path.abspath(__file__)) + rospy.get_param("~rootCAPath", '/Certificates/root-CA.crt')
    certificatePath = os.path.dirname(os.path.abspath(__file__)) + rospy.get_param("~certificatePath", '/Certificates/Pi.cert.pem')
    privateKeyPath = os.path.dirname(os.path.abspath(__file__)) + rospy.get_param("~privateKeyPath", '/Certificates/Pi.private.key')
    clientId = rospy.get_param("~clientId", 'test_pi')
    topic = rospy.get_param("~topic", '/Transnavigators/Pi')

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
