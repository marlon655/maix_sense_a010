#!/usr/bin/env python3
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import ComputeRoute, FollowPath, NavigateThroughPoses


class RouteToPoses(Node):
    def __init__(self):
        super().__init__('route_to_poses')

        self.declare_parameter('start_id', 0)
        self.declare_parameter('goal_id', 0)
        self.declare_parameter('use_start', True)
        self.declare_parameter('use_poses', False)
        self.declare_parameter('mode', 'follow_path')
        self.declare_parameter('max_poses', 0)

        self._route_client = ActionClient(self, ComputeRoute, 'compute_route')
        self._follow_path_client = ActionClient(self, FollowPath, 'follow_path')
        self._nav_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

    def run(self):
        start_id = self.get_parameter('start_id').value
        goal_id = self.get_parameter('goal_id').value
        use_start = self.get_parameter('use_start').value
        use_poses = self.get_parameter('use_poses').value

        self.get_logger().info(
            f'Requesting route: start_id={start_id}, goal_id={goal_id}, '
            f'use_start={use_start}, use_poses={use_poses}'
        )

        if not self._route_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server /compute_route is not available')
            return False

        route_goal = ComputeRoute.Goal()
        route_goal.start_id = int(start_id)
        route_goal.goal_id = int(goal_id)
        route_goal.use_start = bool(use_start)
        route_goal.use_poses = bool(use_poses)

        route_future = self._route_client.send_goal_async(route_goal)
        rclpy.spin_until_future_complete(self, route_future)
        route_handle = route_future.result()

        if route_handle is None or not route_handle.accepted:
            self.get_logger().error('Route goal was rejected')
            return False

        result_future = route_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        route_result = result_future.result().result

        if route_result.error_code != ComputeRoute.Result.NONE:
            self.get_logger().error(f'Route failed with error_code={route_result.error_code}')
            return False

        self._prepare_path(route_result.path)
        poses = self._reduce_poses(list(route_result.path.poses))
        route_result.path.poses = poses

        if not poses:
            self.get_logger().error('Route succeeded, but returned an empty path')
            return False

        self._log_path_summary(route_result.path)

        mode = self.get_parameter('mode').value
        if mode == 'navigate_through_poses':
            return self._navigate_through_poses(poses)
        if mode == 'follow_path':
            return self._follow_path(route_result.path)

        self.get_logger().error(
            f'Invalid mode={mode}. Use follow_path or navigate_through_poses.'
        )
        return False

    def _follow_path(self, path):
        if not self._follow_path_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server /follow_path is not available')
            return False

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'general_goal_checker'
        goal.progress_checker_id = 'progress_checker'

        self.get_logger().info(f'Sending path with {len(path.poses)} poses to /follow_path')

        future = self._follow_path_client.send_goal_async(goal, feedback_callback=self._follow_path_feedback)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()

        if handle is None or not handle.accepted:
            self.get_logger().error('FollowPath goal was rejected')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code == FollowPath.Result.NONE:
            self.get_logger().info('FollowPath completed')
            return True

        self.get_logger().error(
            f'FollowPath failed with error_code={result.error_code}: {result.error_msg}'
        )
        return False

    def _navigate_through_poses(self, poses):
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server /navigate_through_poses is not available')
            return False

        nav_goal = NavigateThroughPoses.Goal()
        nav_goal.poses = poses

        nav_future = self._nav_client.send_goal_async(nav_goal, feedback_callback=self._feedback)
        rclpy.spin_until_future_complete(self, nav_future)
        nav_handle = nav_future.result()

        if nav_handle is None or not nav_handle.accepted:
            self.get_logger().error('NavigateThroughPoses goal was rejected')
            return False

        nav_result_future = nav_handle.get_result_async()
        rclpy.spin_until_future_complete(self, nav_result_future)
        nav_result = nav_result_future.result().result

        if nav_result.error_code == NavigateThroughPoses.Result.NONE:
            self.get_logger().info('Navigation through route completed')
            return True

        self.get_logger().error(
            f'Navigation failed with error_code={nav_result.error_code}: {nav_result.error_msg}'
        )
        return False

    def _prepare_path(self, path):
        if not path.header.frame_id:
            path.header.frame_id = 'map'

        for pose in path.poses:
            if not pose.header.frame_id:
                pose.header.frame_id = path.header.frame_id
            q = pose.pose.orientation
            if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
                q.w = 1.0

    def _log_path_summary(self, path):
        first = path.poses[0].pose.position
        last = path.poses[-1].pose.position
        distance = math.hypot(last.x - first.x, last.y - first.y)
        self.get_logger().info(
            f'Route path: frame={path.header.frame_id}, poses={len(path.poses)}, '
            f'first=({first.x:.3f}, {first.y:.3f}), '
            f'last=({last.x:.3f}, {last.y:.3f}), straight_distance={distance:.3f} m'
        )

    def _reduce_poses(self, poses):
        max_poses = int(self.get_parameter('max_poses').value)
        if max_poses <= 0 or len(poses) <= max_poses:
            return poses

        if max_poses == 1:
            return [poses[-1]]

        step = (len(poses) - 1) / float(max_poses - 1)
        reduced = []
        for index in range(max_poses):
            reduced.append(poses[round(index * step)])
        return reduced

    def _feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        remaining = getattr(feedback, 'number_of_poses_remaining', None)
        if remaining is not None and math.isfinite(float(remaining)):
            self.get_logger().debug(f'Poses remaining: {remaining}')

    def _follow_path_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f'distance_to_goal={feedback.distance_to_goal:.3f}, speed={feedback.speed:.3f}'
        )


def main():
    rclpy.init()
    node = RouteToPoses()
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
