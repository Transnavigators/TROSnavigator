#!/usr/bin/env python

import socket
import configparser
import rospy
from std_msgs.msg import String


class LocalinoPublisher:
    def __init__(self):
        self.pub = rospy.Publisher('localino_server', String, queue_size=10)
        rospy.init_node('localino', anonymous=True)

    def talker(self):
        # Read the config file for anchor positions
        config = configparser.ConfigParser()
        config.read('/home/pi/catkin_ws/src/localino_server/config.ini')
        anchor_names = config.options('anchorpos')
        anchor_coords = {anchorName: None for anchorName in anchor_names}
        tag_ids = config.options('tagnames')
        for anchorName in anchor_names:
            coords = config['anchorpos'][anchorName].split(',')
            anchor_coords[anchorName] = {'x': float(coords[0]), 'y': float(coords[1]), 'z': float(coords[2])}

        # Read the config file for the number of cycles to keep data
        history_length = int(config['default']['historyLength'])

        # Read the config file for the maximum error when doing calculations
        epsilon = float(config['default']['epsilon'])

        # Create a list of 2D dictionaries to store distances reported from each anchor\tag pair
        data = [{tagID: {anchorID: None for anchorID in anchor_names} for tagID in tag_ids}] * history_length

        num_anchors = len(anchor_names)

        # Bind to UDP port 10000 to capture the tag's traffic
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('', int(config['default']['port'])))
        last_timestamp = {tagName: 0 for tagName in tag_ids}
        num_anchors_reported = {tagName: 0 for tagName in tag_ids}

        rate = rospy.Rate(10)  # 10Hz, increase if needed
        while not rospy.is_shutdown():
            data, addr = sock.recvfrom(100)
            rospy.loginfo("Received message:" + data.decode("ascii"))

            data_arr = data.decode("ascii").split(",")
            if len(data_arr) == 5 and data_arr[0] in anchor_names and data_arr[1] in tag_ids:
                # Give array of results human readable names
                anchor_id = data_arr[0]
                tag_id = data_arr[1]
                dist = data_arr[2]
                timestamp = data_arr[3]
                tag_power = data_arr[4]
                # TODO: verify inputs to prevent crashing
                data[timestamp % history_length][tag_id][anchor_id] = float(dist)

                # Keep track of whether or not we can do calculations
                if last_timestamp[tag_id] == timestamp:
                    num_anchors_reported[tag_id] += 1
                else:
                    last_timestamp[tag_id] = timestamp
                    num_anchors_reported[tag_id] = 0
                # Each anchor responded with a distance
                if num_anchors_reported[tag_id] == num_anchors:
                    sum_dist = 0.0
                    for anchorName in anchor_names:
                        sum_dist += 1.0 / data[timestamp % history_length][tag_id][anchorName]
                    x = 0.0
                    y = 0.0
                    for anchorName in anchor_names:
                        x += 1.0 / (anchor_coords[anchorName]['x'] * (
                            data[timestamp % history_length][tag_id][anchorName]) / sum_dist)
                        y += 1.0 / (anchor_coords[anchorName]['y'] * (
                            data[timestamp % history_length][tag_id][anchorName]) / sum_dist)

                    self.pub.publish("Located Tag#=" + tag_id + " at t=" + timestamp + "@ p=(" + x + "," + y + ")")
            rate.sleep()


if __name__ == "__main__":
    try:
        lp = LocalinoPublisher()
        lp.talker()
    except rospy.ROSInterruptException:
        pass
