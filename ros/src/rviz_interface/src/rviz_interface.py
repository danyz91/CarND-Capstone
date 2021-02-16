#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Int32

class RvizInterface(object):
    def __init__(self):
        rospy.logdebug("[RvizInterface] Rviz Interface ........")

        rospy.init_node("rviz_interface")

        # Member variables definition
        self.pose = None
        self.map = None
        self.waypoints = None
        self.waypoint_marker = None
        self.map_marker = None
        self.pose_marker = None
        self.traffic_marker = None

        #####################
        # Subscribers
        #####################
        rospy.Subscriber("/current_pose", PoseStamped, self.pose_cb)
        rospy.Subscriber("/base_waypoints", Lane, self.map_cb)
        rospy.Subscriber("/final_waypoints", Lane, self.waypoints_cb)
        rospy.Subscriber("/traffic_waypoint", Int32, self.traffic_cb)
        #####################
        # Publishers
        #####################
        self.marker_pub = rospy.Publisher('waypoints', Marker, queue_size=1)
        self.map_marker_pub = rospy.Publisher('map_waypoints', Marker, queue_size=1)
        self.pose_marker_pub = rospy.Publisher('vehicle_pose', Marker, queue_size=1)
        self.traffic_marker_pub = rospy.Publisher('traffic_wp', Marker, queue_size=1)

        #  Main loop function call
        self.loop()

    def loop(self):
        # Published data are sent to waypoint_follower that runs at 30Hz
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            if self.waypoint_marker is not None:
                self.publish_waypoint_marker()
            if self.map_marker is not None:
                self.publish_map_waypoint_marker()
            if self.pose_marker is not None:
                self.publish_pose_marker()
            if self.traffic_marker is not None:
                self.publish_traffic_marker()

            rate.sleep()

    def publish_waypoint_marker(self):
        self.marker_pub.publish(self.waypoint_marker)

    def publish_map_waypoint_marker(self):
        self.map_marker_pub.publish(self.map_marker)

    def publish_pose_marker(self):
        self.pose_marker_pub.publish(self.pose_marker)

    def publish_traffic_marker(self):
        self.traffic_marker_pub.publish(self.traffic_marker)

    def pose_cb(self, msg):
        self.pose = msg

    def map_cb(self, msg):

        self.waypoints = msg.waypoints
        self.map_marker = Marker()
        self.map_marker.header.frame_id = "/world"
        self.map_marker.type = self.map_marker.LINE_STRIP
        self.map_marker.action = self.map_marker.ADD

        # marker scale
        self.map_marker.scale.x = 0.03
        self.map_marker.scale.y = 0.03
        self.map_marker.scale.z = 0.03

        # marker color
        self.map_marker.color.a = 1.0
        self.map_marker.color.r = 1.0
        self.map_marker.color.g = 1.0
        self.map_marker.color.b = 0.0

        # marker orientaiton
        self.map_marker.pose.orientation.x = 0.0
        self.map_marker.pose.orientation.y = 0.0
        self.map_marker.pose.orientation.z = 0.0
        self.map_marker.pose.orientation.w = 1.0

        # marker position
        self.map_marker.pose.position.x = 0.0
        self.map_marker.pose.position.y = 0.0
        self.map_marker.pose.position.z = 0.0

        # marker line points
        self.map_marker.points = []

        for wp in msg.waypoints:
            # first point
            curr_point = Point()
            curr_point.x = wp.pose.pose.position.x
            curr_point.y = wp.pose.pose.position.y
            curr_point.z = wp.pose.pose.position.z
            self.map_marker.points.append(curr_point)

    def waypoints_cb(self, msg):

        self.waypoint_marker = Marker()
        self.waypoint_marker.header.frame_id = "/world"
        self.waypoint_marker.type = self.waypoint_marker.LINE_STRIP
        self.waypoint_marker.action = self.waypoint_marker.ADD

        # marker scale
        self.waypoint_marker.scale.x = 0.03
        self.waypoint_marker.scale.y = 0.03
        self.waypoint_marker.scale.z = 0.03

        # marker color
        self.waypoint_marker.color.a = 1.0
        self.waypoint_marker.color.r = 0.0
        self.waypoint_marker.color.g = 1.0
        self.waypoint_marker.color.b = 0.0

        # marker orientaiton
        self.waypoint_marker.pose.orientation.x = 0.0
        self.waypoint_marker.pose.orientation.y = 0.0
        self.waypoint_marker.pose.orientation.z = 0.0
        self.waypoint_marker.pose.orientation.w = 1.0

        # marker position
        self.waypoint_marker.pose.position.x = 0.0
        self.waypoint_marker.pose.position.y = 0.0
        self.waypoint_marker.pose.position.z = 0.05

        # marker line points
        self.waypoint_marker.points = []

        for wp in msg.waypoints:
            # first point
            curr_point = Point()
            curr_point.x = wp.pose.pose.position.x
            curr_point.y = wp.pose.pose.position.y
            curr_point.z = wp.pose.pose.position.z
            self.waypoint_marker.points.append(curr_point)

    def traffic_cb(self, msg):
        stopline_wp_idx = msg.data

        self.traffic_marker = Marker()
        self.traffic_marker.header.frame_id = "/world"
        self.traffic_marker.type = self.traffic_marker.CYLINDER
        self.traffic_marker.action = self.traffic_marker.ADD

        # marker scale
        self.traffic_marker.scale.x = 2
        self.traffic_marker.scale.y = 2
        self.traffic_marker.scale.z = 2

        # marker color
        self.traffic_marker.color.a = 1.0

        self.traffic_marker.color.r = 1.0
        self.traffic_marker.color.g = 0.0

        self.traffic_marker.color.b = 0.0


        # marker orientaiton
        self.traffic_marker.pose.orientation.x = 0.0
        self.traffic_marker.pose.orientation.y = 0.0
        self.traffic_marker.pose.orientation.z = 0.0
        self.traffic_marker.pose.orientation.w = 1.0

        # marker position
        self.traffic_marker.pose.position = self.waypoints[stopline_wp_idx].pose.pose.position




if __name__ == "__main__":
    try:
        RvizInterface()
    except rospy.ROSInterruptException:
        rospy.logerr("Could not start rviz interface node.")
