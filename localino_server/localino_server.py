#!/usr/bin/env python

import socket
import configparser
import math
import rospy
import rospkg
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


class Circle(object):
    def __init__(self, point, radius):
        self.center = point
        self.radius = radius


def get_two_circles_intersecting_points(c1, c2):
    p1 = c1.center
    p2 = c2.center
    r1 = c1.radius
    r2 = c2.radius

    d = get_two_points_distance(p1, p2)
    # if to far away, or self contained - can't be done
    if d >= (r1 + r2) or d <= math.fabs(r1 - r2):
        return None

    a = (pow(r1, 2) - pow(r2, 2) + pow(d, 2)) / (2 * d)
    h = math.sqrt(pow(r1, 2) - pow(a, 2))
    x0 = p1.x + a * (p2.x - p1.x) / d
    y0 = p1.y + a * (p2.y - p1.y) / d
    rx = -(p2.y - p1.y) * (h / d)
    ry = -(p2.x - p1.x) * (h / d)
    return [Point(x0 + rx, y0 - ry), Point(x0 - rx, y0 + ry)]


def get_all_intersecting_points(circles):
    points = []
    num = len(circles)
    for i in range(num):
        j = i + 1
        for k in range(j, num):
            res = get_two_circles_intersecting_points(circles[i], circles[k])
            if res:
                points.extend(res)
    return points


def get_two_points_distance(p1, p2):
    return math.sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2))


def is_contained_in_circles(point, circles):
    for i in range(len(circles)):
        if get_two_points_distance(point, circles[i].center) > (circles[i].radius):
            return False
    return True


def get_polygon_center(points):
    center = Point(0, 0, 0)
    num = len(points)
    for i in range(num):
        center.x += points[i].x
        center.y += points[i].y
    center.x = center.x / num
    center.y = center.y / num
    return center


class LocalinoPublisher:
    def __init__(self):
        # initialize the node
        self.pub = rospy.Publisher('vo', Odometry, queue_size=10)
        rospy.init_node('localino', anonymous=True)
        self.vo_broadcaster = tf.TransformBroadcaster()

        # The tag ID of the localino tag mounted to the wheelchair
        if rospy.has_param("~base_tag_id"):
            self.base_id = rospy.get_param("~base_tag_id")
        else:
            self.base_id = "1002"

        # The number of nanoseconds between samples before the data is ignored
        # 50 us is approx 2x the time between responses from all 3 anchors
        if rospy.has_param("~port"):
            self.timeout = float(rospy.get_param("~timeout"))
        else:
            self.timeout = 50e-6

        # The rate at which each localino sends packets
        if rospy.has_param("~poll_rate"):
            self.rate = rospy.Rate(int(rospy.get_param("~poll_rate")))
        else:
            self.rate = rospy.Rate(4000)

        # The UDP port to listen on for localino packets
        if rospy.has_param("~port"):
            self.port = int(rospy.get_param("~port"))
        else:
            self.port = 10000

        # The IP to bind on, either all interfaces (0.0.0.0) or the localino subnet (192.168.4.255)
        if rospy.has_param("~ip"):
            self.ip = rospy.get_param("~ip")
        else:
            self.ip = ''

        self.other_pubs = {}

    # start the triangulation
    def begin(self):
        # Read the config file for anchor positions
        config = configparser.ConfigParser()

        # Find the path to the config file and read it
        package_dir = rospkg.RosPack().get_path('localino_server')
        config.read(package_dir + '/config.ini')
        anchor_names = config.options('anchorpos')
        anchor_coords = {anchorName: None for anchorName in anchor_names}
        tag_ids = config.options('tagnames')

        # Initialize publishers for the other tags
        for tag_id in tag_ids:
            if tag_id != self.base_id:
                self.other_pubs[tag_id] = self.pub = rospy.Publisher('tag_' + tag_id, Point, queue_size=10)

        for anchorName in anchor_names:
            coords = config['anchorpos'][anchorName].split(',')
            anchor_coords[anchorName] = Point(float(coords[0]), float(coords[1]), 0.)

        # Create 2D dictionaries to store distances reported from each anchor\tag pair
        dists = {tagID: {anchorID: None for anchorID in anchor_names} for tagID in tag_ids}

        num_anchors = len(anchor_names)

        # Bind to IP 0.0.0.0 UDP port 10000 to capture the tag's traffic
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('', self.port))

        # Create dictionaries to hold the last timestamp and
        last_timestamp = {tagID: {anchorID: 0 for anchorID in anchor_names} for tagID in tag_ids}
        num_anchors_reported = {tagName: 0 for tagName in tag_ids}

        while not rospy.is_shutdown():
            data, addr = sock.recvfrom(100)
            rospy.loginfo("Received message:" + data.decode("ascii"))

            data_arr = data.decode("utf-8").split(",")
            if len(data_arr) == 5 and data_arr[0] in anchor_names and data_arr[1] in tag_ids:
                # Give array of results human readable names
                anchor_id = data_arr[0]
                tag_id = data_arr[1]
                dist = data_arr[2]
                seq_num = data_arr[3]
                tag_power = data_arr[4]
                # TODO: verify inputs to prevent crashing
                # TODO: scale distances to make sure we get a correct coordinate
                # TODO: test if expiring data via timeout or seq_num is better
                dists[tag_id][anchor_id] = Circle(anchor_coords[anchor_id], float(dist))
                last_timestamp[tag_id][anchor_id] = rospy.get_time()
                num_anchors_reported[tag_id] += 1

                # Each anchor responded with a distance to this tag
                if num_anchors_reported[tag_id] >= num_anchors:

                    # Check that the data isn't stale
                    if last_timestamp[tag_id][anchor_id] - min(last_timestamp[tag_id].values()) < self.TIMEOUT:
                        # Use trilateration to locate the tag
                        # Algorithm from https://github.com/noomrevlis/trilateration
                        num_anchors_reported[tag_id] = 0
                        inner_points = []
                        for p in get_all_intersecting_points(dists[tag_id]):
                            if is_contained_in_circles(p, dists[tag_id]):
                                inner_points.append(p)
                        center = get_polygon_center(inner_points)

                        if tag_id == self.base_id:

                            # An Odometry message generated at time=stamp and published on topic /vo
                            odom = Odometry()
                            odom.header.stamp = rospy.Time().now()
                            odom.header.frame_id = "vo"

                            # Give the XY coordinates and set the covariance high on the rest so they aren't used
                            odom.pose.pose = Pose(center, Quaternion(1, 0, 0, 0))

                            # TODO: measure covariance with experiment + statistics
                            # This should be less accurate than the Arduino encoder odometry
                            odom.pose.covariance = {1000, 0, 0, 0, 0, 0,  # covariance on gps_x
                                                    0, 1000, 0, 0, 0, 0,  # covariance on gps_y
                                                    0, 0, 0, 0, 0, 0,  # covariance on gps_z
                                                    0, 0, 0, 0, 0, 0,  # large covariance on rot x
                                                    0, 0, 0, 0, 0, 0,  # large covariance on rot y
                                                    0, 0, 0, 0, 0, 0}  # large covariance on rot z
                            odom.child_frame_id = str(tag_id)
                            odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
                            odom.twist.covariance = {0} * 36  # Ignore twist from this topic

                            self.pub.publish(odom)

                            # Send a copy of the data to tf
                            self.vo_broadcaster.sendTransform((center.x, center.y, 0.), (1, 0, 0, 0),
                                                              odom.header.stamp, "base_link", "vo")
                        else:
                            # Publish a the tag's location to let Alexa know where to send the wheelchair
                            self.other_pubs[tag_id].publish(center)
                        # Immediately after receiving all of a frame, the localinos will take 0.2-0.3ms before sending
                        # a new packet, so wait until then
                        self.rate.sleep()


if __name__ == "__main__":
    try:
        lp = LocalinoPublisher()
        lp.begin()
    except rospy.ROSInterruptException:
        pass
