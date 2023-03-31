#!/usr/bin/env python3
# Manage the exploring of the map
# Subscribe to the map (/grid_map) and current robot pose (/pose_robot)
# Calculate the frontier points and starts the navigation to the goal_point
# Published all calculated frontier points as MarkerArray (/frontier_points)
from enum import Enum

import numpy as np
import cv2

from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

import message_filters

import rclpy
from rclpy.node import Node

from frontier_exploration.navigation import BasicNavigator, NavigationResult
from frontier_exploration.frontiers import *
from frontier_exploration.mapping import GridMap

# ------------------------------------------------------------------------------------------------------------
# Global Constants
P_PIOR = -1 	# UNKNOWN cell state
P_OCC = 100     # OCCUPIED cell state
P_FREE = 0	    # FREE cell state

RESOLUTION = 0.05 # Grid resolution in [m]

MAP_NAME  = 'house' # map name without extension
# Set limits of the map in world coordinates [m]
if MAP_NAME == 'world':
    X_LIM = [-4, 4]
    Y_LIM = [-4, 4]
elif MAP_NAME == 'house':
    X_LIM = [-10, 10]
    Y_LIM = [-10, 10]
else:
    X_LIM = [-15, 15]
    Y_LIM = [-15, 15]  

#------------------------------------------------------------------------------------------------------------

def makeMarker(point, header, i, b_delete=False):
    """ Create the marker for rviz to show the frontier points """
    pose = Pose(position=Point(x=point[0], y=point[1], z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
    msg_marker = Marker(header=header, pose=pose)

    if b_delete == True:
        msg_marker.action = msg_marker.DELETEALL
    else:
        msg_marker.action = msg_marker.ADD

        msg_marker.id = i
        msg_marker.ns = str(i)
    msg_marker.type = msg_marker.CUBE
    msg_marker.scale.x = 0.2 
    msg_marker.scale.y = 0.2 
    msg_marker.scale.z = 0.2 
    msg_marker.color.r = 1.0
    msg_marker.color.g = 0.0
    msg_marker.color.b = 0.0
    msg_marker.color.a = 1.0

    return msg_marker 


class Minimal_PubSub(Node):
    def __init__(self):
        super().__init__('minimal_pubsub')
        # Navigation
        self.navigation = BasicNavigator()

        # Publisher
        qos_policy_pub = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        self.Publisher_marker = self.create_publisher(msg_type=MarkerArray, topic='/frontier_points', qos_profile=qos_policy_pub)
 
        # Subscribe to messages with message_filters due to synchronizatio of the topics.
        qos_policy_sub = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        qos_policy_sub_map = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=5)
        Subscription_map = message_filters.Subscriber(self, OccupancyGrid, '/grid_map', qos_profile=qos_policy_sub_map)
        Subscription_odom = message_filters.Subscriber(self, PoseStamped, '/pose_robot', qos_profile=qos_policy_sub)
        ts = message_filters.ApproximateTimeSynchronizer([Subscription_map, Subscription_odom], 10, 0.5)
        ts.registerCallback(self.callback)

        self.map = None
        self.msg_odom = None
        self.goal_pose = None
        self.marker_array = None


    def callback(self, msg_map, msg_odom):
        """ Callback function for Subscriber """
        try:
            # Read sensor values
            self.read_odom(msg_odom)
            self.read_map(msg_map)

            # Frontier exploration
            self.fct_frontier_exploration()

            # Write data (map, marker_array, current_pose)
            self.fct_write_data()

            # Navigation
            self.fct_start_navigation()
        except:
            print("Error - Waiting that everything is started..")


    def read_map(self, msg_map):
        """ Read map from /grid_map topic """
        try:
            width = msg_map.info.width
            height = msg_map.info.height
            resolution  = msg_map.info.resolution
            data = np.array(msg_map.data).reshape(width, height)
            data = cv2.flip(cv2.rotate(data, cv2.ROTATE_90_CLOCKWISE), 1)

            self.map = GridMap(X_lim=X_LIM, Y_lim=Y_LIM, resolution=resolution, p=P_PIOR)
            self.map.data = data
        except:
            print("Error - Waiting that /grid_map is started..")

    def read_odom(self, msg_odom):
        """ Read sensor message from /pose_robot topic """
        try:
            self.msg_odom = msg_odom
        except:
            print("Error - Waiting that /pose_robot ist started..")

    def fct_write_data(self):
        """
        Publish sensor data 
        Write:  - map: current occupancy grid map   topic: /grid_map
                - pose: current pose of the robot   topic: /pose_robot
                - marker_array: frontier points     topic: /frontier_points
        """
        # Delete old markers
        header = Header(stamp=Node.get_clock(self).now().to_msg(), frame_id='map')
        msg_array = MarkerArray()
        msg_marker = msg_array.markers.append(makeMarker(point=[0.0, 0.0], header=header, i=0, b_delete=True))
        self.Publisher_marker.publish(msg_array)  

        # Build MarkerArray for frontier points
        header = Header(stamp=Node.get_clock(self).now().to_msg(), frame_id='map')
        msg_array = MarkerArray()
        for i, point in enumerate(self.frontiers):
            msg_marker = makeMarker(point=point, header=header, i=i, b_delete=False)
            msg_array.markers.append(msg_marker)

        # Publish data
        self.Publisher_marker.publish(msg_array)        

        # Visualization with cv2 for debugging
        # np_map = self.map.to_numpy_uint8()
        # cv2.imshow('Gridmap', cv2.rotate(np_map, cv2.ROTATE_180))
        # cv2.waitKey(1)


    def fct_frontier_exploration(self):
        """ Start the frontier exploration """
        self.frontiers = getFrontier(self.msg_odom.pose, self.map)
        print("Number of found frontiers: ", len(self.frontiers))

        location = None
        largestDist = 0
        for f in self.frontiers:
            if not self.test_point(f):
                dist = np.linalg.norm(f-np.array([self.msg_odom.pose.position.x, self.msg_odom.pose.position.y]))
                if  dist > largestDist:
                    largestDist = dist
                    location = f

        self.set_goal_pose(location)


    def fct_start_navigation(self):
        """ Start navigation to goal_pose and publish data """
        # self.navigation.goThroughPoses(self.goal_pose)
        self.navigation.followWaypoints(self.goal_pose)

        while not self.navigation.isNavComplete():
            feedback = self.navigation.getFeedback()
            print(feedback)


        # Do something depending on the return code
        result = self.navigation.getResult()
        if result == NavigationResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == NavigationResult.CANCELED:
            print('Goal was canceled!')
        elif result == NavigationResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')


    def set_goal_pose(self, point):
        """ Set goal pose """
        self.goal_pose = []
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigation.get_clock().now().to_msg()
        goal_pose.pose.position.x = point[0]
        goal_pose.pose.position.y = point[1]
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0 
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        self.goal_pose.append(goal_pose)


    def test_point(self, point):
        """ Check if frontier point is outside the house """
        if point[0] <= 5 and point[0] >= -5:
            if point[1] < 0:
                return  True
        return False


def main(args=None):
    print("Start Node exploring..")
    rclpy.init(args=args)
    minimal_PubSub = Minimal_PubSub()

    rclpy.spin(minimal_PubSub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_PubSub.destroy_node()
    rclpy.shutdown()
    print("Stop Node exploring..")


if __name__ == '__main__':
    main()

