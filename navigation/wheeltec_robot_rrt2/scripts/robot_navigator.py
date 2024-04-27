import os
from enum import Enum

from nav2_msgs.action import NavigateToPose, FollowWaypoints
from action_msgs.msg import GoalStatus
from nav2_msgs.srv import ManageLifecycleNodes
from ament_index_python import get_package_share_directory

import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node

class NavigationResult(Enum):
    UKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3 


class BasicNavigator(Node):
    def __init__(self):
        super().__init__(node_name='basic_navigator')
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'FollowWaypoints')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def goToPose(self, pose):
        # Sends a `NavToPose` action request
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = os.path.join(get_package_share_directory('sandwich_bot_2_0_control'), 'config', 'box_collector_behavior_tree.xml')

        # self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
        #               str(pose.pose.position.y) + '...')
        
        self.info('Navigating to goal...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                           str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def followWaypoints(self, poses):
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

    def _feedbackCallback(self, msg):
        self.debug('Received action feedback message')
        self.feedback = msg.feedback
        return

    def cancelNav(self):
        self.info('Canceling current goal.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def getResult(self):
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return NavigationResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return NavigationResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return NavigationResult.CANCELED
        else:
            return NavigationResult.UNKNOWN

    def isNavComplete(self):
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
        return self.feedback

    def lifecycleShutdown(self):
        self.info('Shutting down lifecycle nodes based on lifecycle_manager.')
        srvs = self.get_service_names_and_types()
        for srv in srvs:
            if srv[1][0] == 'nav2_msgs/srv/ManageLifecycleNodes':
                srv_name = srv[0]
                self.info('Shutting down ' + srv_name)
                mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
                while not mgr_client.wait_for_service(timeout_sec=1.0):
                    self.info(srv_name + ' service not available, waiting...')
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request().SHUTDOWN
                future = mgr_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                future.result()
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return
    
    def info(self, msg):
        self.get_logger().info(msg)
        return
