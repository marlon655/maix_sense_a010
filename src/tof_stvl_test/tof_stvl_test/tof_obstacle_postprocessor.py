from collections import deque

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

from tof_stvl_test.pointcloud_preprocessor import (
    filter_height,
    lateral_filter_mask,
    make_bounds_line_points,
    radius_outlier_mask,
    temporal_filter_mask,
    validate_lateral_limits,
)
from tof_stvl_test.tof_cloud_preprocessor import transform_points


class TofObstaclePostprocessor(Node):
    """Filter segmented obstacles and restore the A010 sensor frame."""

    def __init__(self):
        super().__init__('tof_obstacle_postprocessor')

        self.declare_parameter(
            'input_topic', '/ground_segmentation/obstacle_points_raw')
        self.declare_parameter(
            'output_topic', '/ground_segmentation/obstacle_points')
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('output_frame', 'tof')
        self.declare_parameter('transform_timeout', 0.10)
        self.declare_parameter('distance_max', 2.50)
        self.declare_parameter('height_min', 0.04)
        self.declare_parameter('height_max', 1.20)
        self.declare_parameter('lateral_min', -1.10)
        self.declare_parameter('lateral_max', 1.10)
        self.declare_parameter('radius_search', 0.05)
        self.declare_parameter('min_neighbors', 3)
        self.declare_parameter('temporal_required_frames', 2)
        self.declare_parameter('temporal_match_radius', 0.05)
        self.declare_parameter('publish_intermediate_clouds', True)
        self.declare_parameter('publish_filter_bounds', True)

        self._validate_configuration()

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.history = deque()

        self.output_pub = self.create_publisher(PointCloud2, output_topic, 10)
        self.input_pub = self.create_publisher(
            PointCloud2, '/tof_filters/obstacles_input', 10)
        self.height_pub = self.create_publisher(
            PointCloud2, '/tof_filters/height', 10)
        self.lateral_pub = self.create_publisher(
            PointCloud2, '/tof_filters/lateral', 10)
        self.spatial_pub = self.create_publisher(
            PointCloud2, '/tof_filters/spatial', 10)

        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.filter_bounds_pub = self.create_publisher(
            Marker, '/tof_filters/filter_bounds', marker_qos)
        self.filter_bounds_visible = False
        self.filter_bounds_timer = self.create_timer(
            1.0, self.publish_filter_bounds_marker)

        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.cloud_callback, 10)

        self.get_logger().info(
            f'Postprocessing {input_topic} -> {output_topic}')

    def _validate_configuration(self):
        height_min = float(self.get_parameter('height_min').value)
        height_max = float(self.get_parameter('height_max').value)
        if height_min >= height_max:
            raise ValueError('height_min must be less than height_max')
        validate_lateral_limits(
            float(self.get_parameter('lateral_min').value),
            float(self.get_parameter('lateral_max').value),
        )
        if float(self.get_parameter('radius_search').value) <= 0.0:
            raise ValueError('radius_search must be greater than zero')
        if float(self.get_parameter('temporal_match_radius').value) <= 0.0:
            raise ValueError('temporal_match_radius must be greater than zero')

    def lookup_transform(self, target_frame, source_frame, stamp):
        return self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time.from_msg(stamp),
            timeout=Duration(
                seconds=float(
                    self.get_parameter('transform_timeout').value)),
        )

    @staticmethod
    def make_cloud(points, stamp, frame_id):
        header = Header(stamp=stamp, frame_id=frame_id)
        xyz = np.asarray(points, dtype=np.float32).reshape((-1, 3))
        return point_cloud2.create_cloud_xyz32(header, xyz)

    def create_filter_bounds_marker(self, action=Marker.ADD):
        target_frame = self.get_parameter('target_frame').value
        sensor_frame = self.get_parameter('output_frame').value

        marker = Marker()
        marker.header.frame_id = target_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'tof_obstacle_filter_bounds'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = action
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        marker.color.r = 0.10
        marker.color.g = 0.90
        marker.color.b = 0.25
        marker.color.a = 0.75

        if action == Marker.ADD:
            try:
                sensor_transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    sensor_frame,
                    Time(),
                    timeout=Duration(
                        seconds=float(
                            self.get_parameter('transform_timeout').value)),
                )
            except TransformException as error:
                self.get_logger().warning(
                    f'Cannot place filter bounds: {error}',
                    throttle_duration_sec=2.0,
                )
                return None

            marker.points = make_bounds_line_points(
                float(self.get_parameter('distance_max').value),
                float(self.get_parameter('height_min').value),
                float(self.get_parameter('height_max').value),
                float(self.get_parameter('lateral_min').value),
                float(self.get_parameter('lateral_max').value),
                sensor_transform.transform.translation.x,
            )
        return marker

    def publish_filter_bounds_marker(self):
        enabled = bool(self.get_parameter('publish_filter_bounds').value)
        if enabled:
            marker = self.create_filter_bounds_marker(Marker.ADD)
            if marker is not None:
                self.filter_bounds_pub.publish(marker)
                self.filter_bounds_visible = True
        elif self.filter_bounds_visible:
            self.filter_bounds_pub.publish(
                self.create_filter_bounds_marker(Marker.DELETE))
            self.filter_bounds_visible = False

    def cloud_callback(self, msg):
        try:
            cloud = point_cloud2.read_points(
                msg, field_names=['x', 'y', 'z'], skip_nans=True)
            points = np.column_stack(
                (cloud['x'], cloud['y'], cloud['z'])).astype(
                    np.float32, copy=False)
        except (AssertionError, TypeError, ValueError) as error:
            self.get_logger().error(f'Invalid obstacle cloud: {error}')
            return

        target_frame = self.get_parameter('target_frame').value
        if msg.header.frame_id == target_frame:
            transformed = points
        else:
            try:
                transform = self.lookup_transform(
                    target_frame, msg.header.frame_id, msg.header.stamp)
            except TransformException as error:
                self.get_logger().warning(
                    f'Cannot transform {msg.header.frame_id} to '
                    f'{target_frame}: {error}',
                    throttle_duration_sec=2.0,
                )
                return
            transformed = transform_points(points, transform)

        height_points = filter_height(
            transformed,
            float(self.get_parameter('height_min').value),
            float(self.get_parameter('height_max').value),
        )
        lateral_mask = lateral_filter_mask(
            height_points,
            float(self.get_parameter('lateral_min').value),
            float(self.get_parameter('lateral_max').value),
        )
        lateral_points = height_points[lateral_mask]

        spatial_mask = radius_outlier_mask(
            lateral_points,
            float(self.get_parameter('radius_search').value),
            int(self.get_parameter('min_neighbors').value),
        )
        spatial_points = lateral_points[spatial_mask]

        required_frames = max(
            1, int(self.get_parameter('temporal_required_frames').value))
        temporal_mask = temporal_filter_mask(
            spatial_points,
            self.history,
            required_frames,
            float(self.get_parameter('temporal_match_radius').value),
        )
        output_points = spatial_points[temporal_mask]
        self.history.append(spatial_points.copy())
        while len(self.history) > max(0, required_frames - 1):
            self.history.popleft()

        if bool(self.get_parameter('publish_intermediate_clouds').value):
            self.input_pub.publish(self.make_cloud(
                transformed, msg.header.stamp, target_frame))
            self.height_pub.publish(self.make_cloud(
                height_points, msg.header.stamp, target_frame))
            self.lateral_pub.publish(self.make_cloud(
                lateral_points, msg.header.stamp, target_frame))
            self.spatial_pub.publish(self.make_cloud(
                spatial_points, msg.header.stamp, target_frame))

        output_frame = self.get_parameter('output_frame').value
        if output_frame != target_frame:
            try:
                output_transform = self.lookup_transform(
                    output_frame, target_frame, msg.header.stamp)
            except TransformException as error:
                self.get_logger().warning(
                    f'Cannot transform filtered obstacles to '
                    f'{output_frame}: {error}',
                    throttle_duration_sec=2.0,
                )
                return
            output_points = transform_points(output_points, output_transform)

        self.output_pub.publish(self.make_cloud(
            output_points, msg.header.stamp, output_frame))


def main(args=None):
    rclpy.init(args=args)
    node = TofObstaclePostprocessor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
