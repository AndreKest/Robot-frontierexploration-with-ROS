#!/usr/bin/env python3
# Navigate the robot
# Based on the library of Steve Macenski
# GitHub: https://github.com/SteveMacenski/nav2_rosdevday_2021

import time
from enum import Enum

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses, FollowWaypoints, ComputePathToPose, ComputePathThroughPoses

import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile


class NavigationResult(Enum):
    """ Enumeration for the result of the navigation """
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class BasicNavigator(Node):
    def __init__(self):
        """ Initialize BasicNavigator class (Variables, ActionClients, Subscriber) """
        super().__init__(node_name='basic_navigator')
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose_received = False
        
        self.result_future = None
        self.goal_handle = None
        self.feedback = None
        self.status = None

        # Initialize action-clients
        self.nav_through_poses_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.compute_path_to_pose_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.compute_path_through_poses_client = ActionClient(self, ComputePathThroughPoses, 'compute_path_through_poses')

        self.check_client_ready()

        # Initialize publisher for /initialpose topic
        qos_policy_pub = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        self.initial_pose_pub = self.create_publisher(msg_type=PoseWithCovarianceStamped, topic='initialpose', qos_profile=qos_policy_pub)
        self.setInitialPose()


    def check_client_ready(self):
        """ Check and wait till all ActionClients are ready """
        while not self.nav_through_poses_client.server_is_ready():
            self.info('nav_through_poses - client is not ready')
        else:
            self.info('nav_through_poses - client is ready')
        
        while not self.follow_waypoints_client.server_is_ready():
            self.info('follow_waypoints - client is not ready')
        else:
            self.info('follow_waypoints - client is ready')
            
        while not self.compute_path_to_pose_client.server_is_ready():
            self.info('compute_path_to_pose  - client is not ready')
        else:
            self.info('compute_path_to_pose - client is ready')

        while not self.compute_path_through_poses_client.server_is_ready():
            self.info('compute_path_through_poses - client is not ready')
        else:
            self.info('compute_path_through_poses - client is ready')
            


    def setInitialPose(self):
        """ Set initial pose of the robot in class """
        self.initial_pose.pose.position.x = -6.5
        self.initial_pose.pose.position.y =  2.0
        self.initial_pose.pose.position.z =  0.0
        self.initial_pose.pose.orientation.x = 0.0
        self.initial_pose.pose.orientation.y = 0.0
        self.initial_pose.pose.orientation.z = 0.0
        self.initial_pose.pose.orientation.w = 1.0
        self._setInitialPose()


    def _setInitialPose(self):
        """ Set intial pose of the robot -> publish it to /intialpose topic """
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose.pose
        msg.header.frame_id = self.initial_pose.header.frame_id
        msg.header.stamp = self.initial_pose.header.stamp
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)


    def goThroughPoses(self, poses):
        """ Go to goal point with ThroughPoses """
        # Sends a `NavThroughPoses` action request
        self.debug("Waiting for 'NavigateThroughPoses' action server")
        while not self.nav_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateThroughPoses' action server not available, waiting...")

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        self.info('Navigating with ' + str(len(goal_msg.poses)) + ' goals.' + '...')
        send_goal_future = self.nav_through_poses_client.send_goal_async(goal_msg,
                                                                         self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal with ' + str(len(poses)) + ' poses was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True


    def followWaypoints(self, poses):
        """ Go to goal point with followWaypoints """
        # Sends a `FollowWaypoints` action request
        self.debug("Waiting for 'FollowWaypoints' action server")
        while not self.follow_waypoints_client.wait_for_server(timeout_sec=1.0):
            self.info("'FollowWaypoints' action server not available, waiting...")

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self.info('Following ' + str(len(goal_msg.poses)) + ' goals.' + '...')
        send_goal_future = self.follow_waypoints_client.send_goal_async(goal_msg,
                                                                        self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Following ' + str(len(poses)) + ' waypoints request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True


    def cancelNav(self):
        """ Cancel the navigation process """
        self.info('Canceling current goal')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return


    def isNavComplete(self):
        """ Check if navigation is complete """
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.debug('Goal with failed with status code: {0}'.format(self.status))
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.debug('Goal succeeded!')
        return True


    def getFeedback(self):
        """ Get feedback of navigation """
        return self.feedback


    def _feedbackCallback(self, msg):
        self.debug('Received action feedback message')
        self.feedback = msg.feedback
        return


    def getResult(self):
        """ Get the result of the navigation (SUCCEEDED, ABORTED, CANCELED, UNKNOWN) """
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return NavigationResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return NavigationResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return NavigationResult.CANCELED
        else:
            return NavigationResult.UNKNOWN


    def getPath(self, start, goal):
        # Sends a `NavToPose` action request
        self.debug("Waiting for 'ComputePathToPose' action server")
        while not self.compute_path_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'ComputePathToPose' action server not available, waiting...")

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal
        goal_msg.start = start

        self.info('Getting path...')
        send_goal_future = self.compute_path_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Get path was rejected!')
            return None

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)
        self.status = self.result_future.result().status
        if self.status != GoalStatus.STATUS_SUCCEEDED:
            self.warn('Getting path failed with status code: {0}'.format(self.status))
            return None

        return self.result_future.result().result.path


    def getPathThroughPoses(self, start, goals):
        # Sends a `NavToPose` action request
        self.debug("Waiting for 'ComputePathThroughPoses' action server")
        while not self.compute_path_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.info("'ComputePathThroughPoses' action server not available, waiting...")

        goal_msg = ComputePathThroughPoses.Goal()
        goal_msg.goals = goals
        goal_msg.start = start

        self.info('Getting path...')
        send_goal_future = self.compute_path_through_poses_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Get path was rejected!')
            return None

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)
        self.status = self.result_future.result().status
        if self.status != GoalStatus.STATUS_SUCCEEDED:
            self.warn('Getting path failed with status code: {0}'.format(self.status))
            return None

        return self.result_future.result().result.path


    def info(self, msg):
        """ Logger for info messages """
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        """ Logger for warning messages """
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        """ Logger for error messages """
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        """ Logger for debug messages """
        self.get_logger().debug(msg)
        return
