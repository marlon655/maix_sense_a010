import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


def select_finite_points_in_range(points, minimum, maximum):
    """Preserve finite XYZ points within the sensor's useful range."""
    xyz = np.asarray(points, dtype=np.float32).reshape((-1, 3))
    if minimum < 0.0:
        raise ValueError('distance_min must be greater than or equal to zero')
    if maximum <= minimum:
        raise ValueError('distance_max must be greater than distance_min')

    finite = np.all(np.isfinite(xyz), axis=1)
    distances = np.linalg.norm(xyz, axis=1)
    mask = finite & (distances >= minimum) & (distances <= maximum)
    return xyz[mask]


def transform_points(points, transform):
    """Apply a TransformStamped to an Nx3 array."""
    xyz = np.asarray(points, dtype=np.float32).reshape((-1, 3))
    q = transform.transform.rotation
    rotation = np.array([
        [
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            2.0 * (q.x * q.y - q.z * q.w),
            2.0 * (q.x * q.z + q.y * q.w),
        ],
        [
            2.0 * (q.x * q.y + q.z * q.w),
            1.0 - 2.0 * (q.x * q.x + q.z * q.z),
            2.0 * (q.y * q.z - q.x * q.w),
        ],
        [
            2.0 * (q.x * q.z - q.y * q.w),
            2.0 * (q.y * q.z + q.x * q.w),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y),
        ],
    ], dtype=np.float32)
    translation = np.array([
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z,
    ], dtype=np.float32)
    return xyz @ rotation.T + translation


class TofCloudPreprocessor(Node):
    """Prepare /cloud for ground segmentation without removing the floor."""

    def __init__(self):
        super().__init__('tof_cloud_preprocessor')

        self.declare_parameter('input_topic', '/cloud')
        self.declare_parameter(
            'output_topic', '/tof_preprocessed/ground_input')
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('distance_min', 0.05)
        self.declare_parameter('distance_max', 2.50)
        self.declare_parameter('transform_timeout', 0.10)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.output_pub = self.create_publisher(PointCloud2, output_topic, 10)
        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.cloud_callback, 10)

        self.get_logger().info(
            f'Preparing {input_topic} -> {output_topic}; floor is preserved')

    def cloud_callback(self, msg):
        try:
            cloud = point_cloud2.read_points(
                msg, field_names=['x', 'y', 'z'], skip_nans=False)
            points = np.column_stack(
                (cloud['x'], cloud['y'], cloud['z'])).astype(
                    np.float32, copy=False)
            points = select_finite_points_in_range(
                points,
                float(self.get_parameter('distance_min').value),
                float(self.get_parameter('distance_max').value),
            )
        except (AssertionError, TypeError, ValueError) as error:
            self.get_logger().error(f'Invalid input cloud: {error}')
            return

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

        transformed = transform_points(points, transform)
        header = Header(stamp=msg.header.stamp, frame_id=target_frame)
        self.output_pub.publish(
            point_cloud2.create_cloud_xyz32(header, transformed))


def main(args=None):
    rclpy.init(args=args)
    node = TofCloudPreprocessor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # DDS can finish shutting down between the executor wake-up and
        # take_message() when a launch receives SIGINT.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
