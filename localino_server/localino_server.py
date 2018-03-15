#!/usr/bin/env python

import socket
import math
from collections import defaultdict
import asyncore
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


class Circle(object):
    """An object that holds a point and a radius
    
    """
    def __init__(self, point, radius):
        self.center = point
        self.radius = radius


class LocalinoPublisher(asyncore.dispatcher):
    """Listens for Localino packets, triangulates the tags, and publishers their location to ROS
    
    """
    def __init__(self):

        # initialize the node
        self.pub = rospy.Publisher('vo', Odometry, queue_size=10)
        rospy.init_node('localino', anonymous=True)
        self.vo_broadcaster = tf.TransformBroadcaster()

        # TODO: parse settings from yaml file instead
        # The tag ID of the localino tag mounted to the wheelchair
        if rospy.has_param("~base_tag_id"):
            self.base_id = str(rospy.get_param("~base_tag_id"))
        else:
            self.base_id = "1002"

        # The number of nanoseconds between samples before the data is ignored
        # 50 us is approx 2x the time between responses from all 3 anchors
        if rospy.has_param("~timeout"):
            self.timeout = float(rospy.get_param("~timeout"))
        else:
            self.timeout = 15e7

        # The rate at which each localino sends packets
        self.rate = rospy.Rate(int(rospy.get_param("~poll_rate", 4000)))

        # The UDP port to listen on for localino packets
        self.port = int(rospy.get_param("~port", 10000))

        # The IP to bind on, either all interfaces (0.0.0.0) or the localino subnet (192.168.4.255)
        self.ip = rospy.get_param("~ip", '')

        # Keep a list of the anchor names
        self.anchor_names = str(rospy.get_param("~anchor_names", "9002,9003,9005")).split(',')

        # Keep a list of the tag names
        self.tag_ids = str(rospy.get_param("~tag_names", "1002,1001")).split(',')

        self.anchor_coords = defaultdict(Point)

        # Make a dictionary from the anchor's name to its position
        if all(rospy.has_param("~anchor_%s" % anchor_name) for anchor_name in self.anchor_names):
            for anchor_name in self.anchor_names:
                coords = rospy.get_param("~anchor_%s" % anchor_name).split(',')
                self.anchor_coords[anchor_name] = Point(float(coords[0]), float(coords[1]), 0)
        else:
            self.anchor_coords = {'9002': Point(0, 0, 0),
                                  '9003': Point(3.78, 0.28, 0),
                                  '9005': Point(1.12, 2.03, 0)
                                  }
        # Create 2D dictionaries to store distances reported from each anchor\tag pair
        self.dists = {tagID: {anchorID: None for anchorID in self.anchor_names} for tagID in self.tag_ids}

        # Create dictionaries to hold the last timestamp and
        self.last_timestamp = {tagID: {anchorID: 0 for anchorID in self.anchor_names} for tagID in self.tag_ids}

        # Bind to IP 0.0.0.0 UDP port 10000 using asyncore to capture the tag's traffic
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.bind(('', self.port))

    # Never need to write, only read
    def writeable(self):
        """Never need to write
        
        Returns:
            bool: False
        """
        return False

    # start the triangulation
    def handle_read(self):
        """On a read, save the distance info and try to triangulate the tag
        
        Overrides asyncore.dispatcher's handler

        """
        data, addr = self.recvfrom(100)
        rospy.loginfo("Received message:" + data.decode("ascii"))

        data_arr = data.decode("utf-8").split(",")
        if len(data_arr) == 5 and data_arr[0] in self.anchor_names and data_arr[1] in self.tag_ids:
            # Give array of results human readable names
            anchor_id = str(data_arr[0])
            tag_id = str(data_arr[1])
            # rospy.loginfo("Found tag "+tag_id+" type")
            dist = data_arr[2]
            # seq_num = data_arr[3]
            # tag_power = data_arr[4]
            # TODO: verify inputs to prevent crashing
            # TODO: scale distances to make sure we get a correct coordinate
            # TODO: test if expiring data via timeout or seq_num is better
            self.dists[tag_id][anchor_id] = Circle(self.anchor_coords[anchor_id], float(dist))
            self.last_timestamp[tag_id][anchor_id] = rospy.get_rostime().nsecs

            # Each anchor responded with a distance to this tag
            if None not in self.dists[tag_id].values():
                # Check that the data isn't stale
                delta_time = self.last_timestamp[tag_id][anchor_id] - min(self.last_timestamp[tag_id].values())
                if delta_time < self.timeout:
                    # Use trilateration to locate the tag
                    # Algorithm from https://github.com/noomrevlis/trilateration

                    inner_points = []

                    for p in LocalinoPublisher.get_all_intersecting_points(self.dists[tag_id].values()):
                        if LocalinoPublisher.is_contained_in_circles(p, self.dists[tag_id].values()):
                            inner_points.append(p)
                    center = LocalinoPublisher.get_polygon_center(inner_points)
                    self.dists = {tagID: {anchorID: None for anchorID in self.anchor_names} for tagID in self.tag_ids}
                    # An Odometry message generated at time=stamp and published on topic /vo
                    odom = Odometry()
                    odom.header.stamp = rospy.Time.now()
                    if tag_id == self.base_id:
                        odom.header.frame_id = "vo"

                        # Give the XY coordinates and set the covariance high on the rest so they aren't used
                        odom.pose.pose = Pose(center, Quaternion(1, 0, 0, 0))

                        # TODO: measure covariance with experiment + statistics
                        # This should be less accurate than the Arduino encoder odometry
                        odom.pose.covariance = [1e-9, 0, 0, 0, 0, 0,
                                                0, 1e-3, 1e-9, 0, 0, 0,
                                                0, 0, 1e6, 0, 0, 0,
                                                0, 0, 0, 1e6, 0, 0,
                                                0, 0, 0, 0, 1e6, 0,
                                                0, 0, 0, 0, 0, 1e-9]
                        odom.child_frame_id = "map"
                        odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
                        odom.twist.covariance = [1e-9, 0, 0, 0, 0, 0,
                                                 0, 1e-3, 1e-9, 0, 0, 0,
                                                 0, 0, 1e6, 0, 0, 0,
                                                 0, 0, 0, 1e6, 0, 0,
                                                 0, 0, 0, 0, 1e6, 0,
                                                 0, 0, 0, 0, 0, 1e-9]

                        self.pub.publish(odom)

                        self.vo_broadcaster.sendTransform((center.x, center.y, 0.), (1, 0, 0, 0),
                                                          odom.header.stamp, "map", "vo")
                    else:
                        # Publish a the tag's location to let Alexa know where to send the wheelchair
                        self.vo_broadcaster.sendTransform((center.x, center.y, 0.), (1, 0, 0, 0),
                                                          odom.header.stamp, "map", 'tag_' + str(tag_id))
                    # Immediately after receiving all of a frame, the localinos will take 0.2-0.3ms before sending
                    # a new packet, so wait until then
                    self.rate.sleep()
                else:
                    rospy.logwarn("Localino packet timed out at " + str(delta_time) + " ns")
            else:
                for anchor, dist in self.dists[tag_id].items():
                    if dist is None:
                        rospy.loginfo("Waiting for packet from anchor %s" % anchor)

    @staticmethod
    def get_two_circles_intersecting_points(c1, c2):
        """Finds the intersecting points between two circles
        
        Args:
            c1 (Circle): The first circle
            c2 (Circle): The second circle

        Returns:
            list: two points where the circles intersect or None
        """
        p1 = c1.center
        p2 = c2.center
        r1 = c1.radius
        r2 = c2.radius

        d = LocalinoPublisher.get_two_points_distance(p1, p2)
        # if to far away, or self contained - can't be done
        if d >= (r1 + r2) or d <= math.fabs(r1 - r2):
            return None

        a = (pow(r1, 2) - pow(r2, 2) + pow(d, 2)) / (2 * d)
        h = math.sqrt(pow(r1, 2) - pow(a, 2))
        x0 = p1.x + a * (p2.x - p1.x) / d
        y0 = p1.y + a * (p2.y - p1.y) / d
        rx = -(p2.y - p1.y) * (h / d)
        ry = -(p2.x - p1.x) * (h / d)
        return [Point(x0 + rx, y0 - ry, 0), Point(x0 - rx, y0 + ry, 0)]

    @staticmethod
    def get_all_intersecting_points(circles):
        """Finds the intersecting points among a list of circles
        
        Args:
            circles (list): A list of Circle objects

        Returns:
            List: A list of points where at least two circles intersect
        """
        points = []
        num = len(circles)
        for i in range(num):
            j = i + 1
            for k in range(j, num):
                res = LocalinoPublisher.get_two_circles_intersecting_points(circles[i], circles[k])
                if res:
                    points.extend(res)
        return points

    @staticmethod
    def get_two_points_distance(p1, p2):
        """Returns the distance between two points
        
        Args:
            p1 (geometry_msgs.msg.Point): The first point
            p2 (geometry_msgs.msg.Point): The second point

        Returns:
            float: The distance between the two points
        """
        return math.sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2))

    @staticmethod
    def is_contained_in_circles(point, circles):
        """Checks if a point is inside every circle
        
        Args:
            point (geometry_msgs.msg.Point): a point
            circles (List): A list of Circles

        Returns:
            bool: True if the point is in the circles, false otherwise
        """
        for circle in circles:
            if LocalinoPublisher.get_two_points_distance(point, circle.center) > circle.radius:
                return False
        return True

    @staticmethod
    def get_polygon_center(points):
        """Returns a point in the middle of the other points
        
        Args:
            points (list): A list of Points

        Returns:
            geometry_msgs.msg.Point: The point at the center
        """
        center = Point(0, 0, 0)
        for point in points:
            center.x += point.x
            center.y += point.y
        center.x = center.x / len(points)
        center.y = center.y / len(points)
        return center


if __name__ == "__main__":
    lp = LocalinoPublisher()
    try:
        while not rospy.is_shutdown():
            asyncore.loop(timeout=1, count=100)
    except rospy.ROSInterruptException:
        pass
    asyncore.close_all()
