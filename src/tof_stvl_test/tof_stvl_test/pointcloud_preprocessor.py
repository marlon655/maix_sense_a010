from collections import deque

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


def filter_distance(points, minimum, maximum):
    distances = np.linalg.norm(points, axis=1)
    return points[(distances > minimum) & (distances < maximum)]


def filter_height(points, minimum, maximum):
    return points[(points[:, 2] >= minimum) & (points[:, 2] <= maximum)]


def filter_radius_outliers(points, radius, min_neighbors):
    if len(points) == 0 or min_neighbors <= 0:
        return points

    tree = cKDTree(points)
    # query_ball_point inclui o proprio ponto; ele nao conta como vizinho.
    neighbor_counts = tree.query_ball_point(points, radius, return_length=True) - 1
    return points[neighbor_counts >= min_neighbors]


def filter_temporal(points, history, required_frames, match_radius):
    if required_frames <= 1:
        return points
    if len(points) == 0 or len(history) < required_frames - 1:
        return np.empty((0, 3), dtype=np.float32)

    persistent = np.ones(len(points), dtype=bool)
    for previous_points in list(history)[-(required_frames - 1):]:
        if len(previous_points) == 0:
            persistent[:] = False
            break
        distances, _ = cKDTree(previous_points).query(
            points,
            k=1,
            distance_upper_bound=match_radius,
        )
        persistent &= np.isfinite(distances)

    return points[persistent]


class PointCloudPreprocessor(Node):

    def __init__(self):
        super().__init__('tof_pointcloud_preprocessor')

        self.declare_parameter('input_topic', '/cloud')
        self.declare_parameter(
            'output_topic', '/ground_segmentation/obstacle_points')
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('output_frame', 'tof')
        self.declare_parameter('transform_timeout', 0.10)
        self.declare_parameter('distance_min', 0.25)
        self.declare_parameter('distance_max', 1.50)
        self.declare_parameter('height_min', 0.05)
        self.declare_parameter('height_max', 0.60)
        self.declare_parameter('radius_search', 0.05)
        self.declare_parameter('min_neighbors', 3)
        self.declare_parameter('temporal_required_frames', 3)
        self.declare_parameter('temporal_match_radius', 0.05)
        self.declare_parameter('publish_intermediate_clouds', True)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.history = deque()

        self.output_pub = self.create_publisher(PointCloud2, output_topic, 10)
        self.distance_pub = self.create_publisher(
            PointCloud2, '/tof_filters/distance', 10)
        self.height_pub = self.create_publisher(
            PointCloud2, '/tof_filters/height', 10)
        self.spatial_pub = self.create_publisher(
            PointCloud2, '/tof_filters/spatial', 10)
        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.cloud_callback, 10)

        self.get_logger().info(
            f'Filtering {input_topic} -> {output_topic}')

    @staticmethod
    def transform_points(points, transform):
        q = transform.transform.rotation
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z

        rotation = np.array([
            [1 - 2 * (q.y * q.y + q.z * q.z),
             2 * (q.x * q.y - q.z * q.w),
             2 * (q.x * q.z + q.y * q.w)],
            [2 * (q.x * q.y + q.z * q.w),
             1 - 2 * (q.x * q.x + q.z * q.z),
             2 * (q.y * q.z - q.x * q.w)],
            [2 * (q.x * q.z - q.y * q.w),
             2 * (q.y * q.z + q.x * q.w),
             1 - 2 * (q.x * q.x + q.y * q.y)],
        ], dtype=np.float32)
        translation = np.array([tx, ty, tz], dtype=np.float32)
        return points @ rotation.T + translation

    @staticmethod
    def make_cloud(points, stamp, frame_id):
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        return point_cloud2.create_cloud_xyz32(
            header, np.asarray(points, dtype=np.float32).reshape((-1, 3)))

    def cloud_callback(self, msg):
        try:
            cloud = point_cloud2.read_points(
                msg, field_names=['x', 'y', 'z'], skip_nans=True)
            points = np.column_stack(
                (cloud['x'], cloud['y'], cloud['z'])).astype(
                    np.float32, copy=False)
        except (AssertionError, ValueError) as error:
            self.get_logger().error(f'Invalid input cloud: {error}')
            return

        finite = np.all(np.isfinite(points), axis=1)
        points = points[finite]
        distance_points = filter_distance(
            points,
            float(self.get_parameter('distance_min').value),
            float(self.get_parameter('distance_max').value),
        )

        target_frame = self.get_parameter('target_frame').value
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(
                    seconds=float(
                        self.get_parameter('transform_timeout').value)),
            )
        except TransformException as error:
            self.get_logger().warning(
                f'Cannot transform {msg.header.frame_id} to '
                f'{target_frame}: {error}',
                throttle_duration_sec=2.0,
            )
            return

        transformed = self.transform_points(distance_points, transform)
        height_points = filter_height(
            transformed,
            float(self.get_parameter('height_min').value),
            float(self.get_parameter('height_max').value),
        )
        spatial_points = filter_radius_outliers(
            height_points,
            float(self.get_parameter('radius_search').value),
            int(self.get_parameter('min_neighbors').value),
        )

        required_frames = max(
            1, int(self.get_parameter('temporal_required_frames').value))
        output_points = filter_temporal(
            spatial_points,
            self.history,
            required_frames,
            float(self.get_parameter('temporal_match_radius').value),
        )
        self.history.append(spatial_points.copy())
        while len(self.history) > max(0, required_frames - 1):
            self.history.popleft()

        if self.get_parameter('publish_intermediate_clouds').value:
            self.distance_pub.publish(self.make_cloud(
                distance_points, msg.header.stamp, msg.header.frame_id))
            self.height_pub.publish(self.make_cloud(
                height_points, msg.header.stamp, target_frame))
            self.spatial_pub.publish(self.make_cloud(
                spatial_points, msg.header.stamp, target_frame))

        output_frame = self.get_parameter('output_frame').value
        try:
            output_transform = self.tf_buffer.lookup_transform(
                output_frame,
                target_frame,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(
                    seconds=float(
                        self.get_parameter('transform_timeout').value)),
            )
        except TransformException as error:
            self.get_logger().warning(
                f'Cannot transform filtered cloud from {target_frame} to '
                f'{output_frame}: {error}',
                throttle_duration_sec=2.0,
            )
            return

        output_points = self.transform_points(
            output_points, output_transform)
        self.output_pub.publish(self.make_cloud(
            output_points, msg.header.stamp, output_frame))


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPreprocessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
