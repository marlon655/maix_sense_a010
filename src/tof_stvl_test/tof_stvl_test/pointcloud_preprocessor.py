from collections import deque

from geometry_msgs.msg import Point
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


def filter_distance(points, minimum, maximum):
    distances = np.linalg.norm(points, axis=1)
    return points[(distances > minimum) & (distances < maximum)]


def filter_height(points, minimum, maximum):
    return points[(points[:, 2] >= minimum) & (points[:, 2] <= maximum)]


def validate_lateral_limits(minimum, maximum):
    if not np.isfinite(minimum) or not np.isfinite(maximum):
        raise ValueError('lateral_min and lateral_max must be finite')
    if minimum >= maximum:
        raise ValueError(
            f'lateral_min ({minimum}) must be less than '
            f'lateral_max ({maximum})')


def lateral_filter_mask(points, minimum, maximum):
    validate_lateral_limits(minimum, maximum)
    return (points[:, 1] >= minimum) & (points[:, 1] <= maximum)


def filter_lateral(points, minimum, maximum):
    return points[lateral_filter_mask(points, minimum, maximum)]


def radius_outlier_mask(points, radius, min_neighbors):
    if len(points) == 0:
        return np.zeros(0, dtype=bool)
    if min_neighbors <= 0:
        return np.ones(len(points), dtype=bool)

    tree = cKDTree(points)
    # query_ball_point inclui o proprio ponto; ele nao conta como vizinho.
    neighbor_counts = tree.query_ball_point(points, radius, return_length=True) - 1
    return neighbor_counts >= min_neighbors


def filter_radius_outliers(points, radius, min_neighbors):
    return points[radius_outlier_mask(points, radius, min_neighbors)]


def temporal_filter_mask(points, history, required_frames, match_radius):
    if required_frames <= 1:
        return np.ones(len(points), dtype=bool)
    if len(points) == 0 or len(history) < required_frames - 1:
        return np.zeros(len(points), dtype=bool)

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

    return persistent


def filter_temporal(points, history, required_frames, match_radius):
    mask = temporal_filter_mask(
        points, history, required_frames, match_radius)
    return points[mask]


def make_bounds_line_points(distance_max, height_min, height_max,
                            lateral_min, lateral_max, origin_x=0.0):
    front_x = origin_x + distance_max
    corners = [
        (origin_x, lateral_min, height_min),
        (origin_x, lateral_max, height_min),
        (origin_x, lateral_max, height_max),
        (origin_x, lateral_min, height_max),
        (front_x, lateral_min, height_min),
        (front_x, lateral_max, height_min),
        (front_x, lateral_max, height_max),
        (front_x, lateral_min, height_max),
    ]
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]
    return [
        Point(x=corners[index][0],
              y=corners[index][1],
              z=corners[index][2])
        for edge in edges
        for index in edge
    ]


def validate_projected_wall_epsilon(epsilon):
    if not np.isfinite(epsilon) or epsilon <= 0.0:
        raise ValueError(
            f'projected_wall_epsilon ({epsilon}) must be finite and '
            f'greater than zero')


def project_rays_to_wall(points, approved_indices, sensor_origin, distance_max,
                         lateral_min, lateral_max, height_min, height_max,
                         epsilon):
    validate_projected_wall_epsilon(epsilon)
    points = np.asarray(points, dtype=np.float32).reshape((-1, 3))
    omitted = np.zeros(len(points), dtype=bool)
    approved_indices = np.asarray(approved_indices, dtype=np.int64)
    if len(approved_indices) > 0:
        if np.any(approved_indices < 0) or np.any(approved_indices >= len(points)):
            raise IndexError('approved ray index is outside the input cloud')
        omitted[approved_indices] = True

    sensor_origin = np.asarray(sensor_origin, dtype=np.float32).reshape((3,))
    ray_points = points[~omitted]
    directions = ray_points - sensor_origin
    valid = (
        np.all(np.isfinite(directions), axis=1) &
        (directions[:, 0] > epsilon)
    )
    directions = directions[valid]
    if len(directions) == 0:
        return np.empty((0, 3), dtype=np.float32)

    scale = distance_max / directions[:, 0]
    projected = sensor_origin + directions * scale[:, np.newaxis]
    inside_bounds = (
        (projected[:, 1] >= lateral_min) &
        (projected[:, 1] <= lateral_max) &
        (projected[:, 2] >= height_min) &
        (projected[:, 2] <= height_max)
    )
    return projected[inside_bounds].astype(np.float32, copy=False)


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
        self.declare_parameter('lateral_min', -0.25)
        self.declare_parameter('lateral_max', 0.25)
        self.declare_parameter('radius_search', 0.05)
        self.declare_parameter('min_neighbors', 3)
        self.declare_parameter('temporal_required_frames', 3)
        self.declare_parameter('temporal_match_radius', 0.05)
        self.declare_parameter('publish_intermediate_clouds', True)
        self.declare_parameter('publish_filter_bounds', True)
        self.declare_parameter('publish_projected_wall', True)
        self.declare_parameter('projected_wall_epsilon', 0.001)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.lateral_min = float(self.get_parameter('lateral_min').value)
        self.lateral_max = float(self.get_parameter('lateral_max').value)
        self.projected_wall_epsilon = float(
            self.get_parameter('projected_wall_epsilon').value)
        try:
            validate_lateral_limits(self.lateral_min, self.lateral_max)
            validate_projected_wall_epsilon(self.projected_wall_epsilon)
        except ValueError as error:
            self.get_logger().error(f'Invalid filter configuration: {error}')
            raise
        self.parameter_callback_handle = self.add_on_set_parameters_callback(
            self.parameter_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.history = deque()

        self.output_pub = self.create_publisher(PointCloud2, output_topic, 10)
        self.distance_pub = self.create_publisher(
            PointCloud2, '/tof_filters/distance', 10)
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
        self.projected_wall_pub = self.create_publisher(
            PointCloud2, '/tof_filters/projected_wall', marker_qos)
        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.cloud_callback, 10)
        self.filter_bounds_visible = False
        self.projected_wall_visible = False
        self.filter_bounds_timer = self.create_timer(
            1.0, self.publish_filter_bounds_marker)
        self.publish_filter_bounds_marker()

        self.get_logger().info(
            f'Filtering {input_topic} -> {output_topic}')

    def create_filter_bounds_marker(self, action=Marker.ADD):
        marker = Marker()
        marker.header.frame_id = 'base_footprint'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'tof_filter_bounds'
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
                    'base_footprint',
                    'tof',
                    Time(),
                    timeout=Duration(
                        seconds=float(
                            self.get_parameter('transform_timeout').value)),
                )
            except TransformException as error:
                self.get_logger().warning(
                    f'Cannot place filter bounds from tof in '
                    f'base_footprint: {error}',
                    throttle_duration_sec=2.0,
                )
                return None
            origin_x = sensor_transform.transform.translation.x
            marker.points = make_bounds_line_points(
                float(self.get_parameter('distance_max').value),
                float(self.get_parameter('height_min').value),
                float(self.get_parameter('height_max').value),
                self.lateral_min,
                self.lateral_max,
                origin_x,
            )
        return marker

    def publish_filter_bounds_marker(self):
        enabled = bool(self.get_parameter('publish_filter_bounds').value)
        if enabled:
            marker = self.create_filter_bounds_marker(Marker.ADD)
            if marker is None:
                return
            self.filter_bounds_pub.publish(marker)
            self.filter_bounds_visible = True
        elif self.filter_bounds_visible:
            self.filter_bounds_pub.publish(
                self.create_filter_bounds_marker(Marker.DELETE))
            self.filter_bounds_visible = False

    def create_projected_wall_cloud(self, points, approved_indices,
                                    sensor_origin, stamp, empty=False):
        if empty:
            projected = np.empty((0, 3), dtype=np.float32)
        else:
            projected = project_rays_to_wall(
                points,
                approved_indices,
                sensor_origin,
                float(self.get_parameter('distance_max').value),
                self.lateral_min,
                self.lateral_max,
                float(self.get_parameter('height_min').value),
                float(self.get_parameter('height_max').value),
                self.projected_wall_epsilon,
            )
        return self.make_cloud(
            projected,
            stamp,
            'base_footprint',
        )

    def publish_projected_wall_cloud(self, points, approved_indices,
                                     sensor_origin, stamp):
        enabled = bool(self.get_parameter('publish_projected_wall').value)
        if enabled:
            cloud = self.create_projected_wall_cloud(
                points, approved_indices, sensor_origin, stamp)
            self.projected_wall_pub.publish(cloud)
            self.projected_wall_visible = True
        elif self.projected_wall_visible:
            empty_cloud = self.create_projected_wall_cloud(
                points, approved_indices, sensor_origin, stamp, empty=True)
            self.projected_wall_pub.publish(empty_cloud)
            self.projected_wall_visible = False

    def parameter_callback(self, parameters):
        lateral_min = self.lateral_min
        lateral_max = self.lateral_max
        projected_wall_epsilon = self.projected_wall_epsilon
        for parameter in parameters:
            if parameter.name == 'lateral_min':
                lateral_min = float(parameter.value)
            elif parameter.name == 'lateral_max':
                lateral_max = float(parameter.value)
            elif parameter.name == 'projected_wall_epsilon':
                projected_wall_epsilon = float(parameter.value)

        try:
            validate_lateral_limits(lateral_min, lateral_max)
            validate_projected_wall_epsilon(projected_wall_epsilon)
        except (TypeError, ValueError) as error:
            return SetParametersResult(successful=False, reason=str(error))

        self.lateral_min = lateral_min
        self.lateral_max = lateral_max
        self.projected_wall_epsilon = projected_wall_epsilon
        return SetParametersResult(successful=True)

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
    def make_cloud(points, stamp, frame_id, rgb=None):
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        xyz = np.asarray(points, dtype=np.float32).reshape((-1, 3))
        if rgb is None:
            return point_cloud2.create_cloud_xyz32(header, xyz)

        rgb = np.asarray(rgb, dtype=np.uint32).reshape((-1,))
        if len(rgb) != len(xyz):
            raise ValueError('XYZ and RGB arrays must have the same length')
        fields = [
            PointField(
                name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(
                name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(
                name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        structured = np.empty(
            len(xyz),
            dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')],
        )
        structured['x'] = xyz[:, 0]
        structured['y'] = xyz[:, 1]
        structured['z'] = xyz[:, 2]
        structured['rgb'] = rgb
        return point_cloud2.create_cloud(header, fields, structured)

    def cloud_callback(self, msg):
        try:
            available_fields = {field.name for field in msg.fields}
            field_names = ['x', 'y', 'z']
            if 'rgb' in available_fields:
                field_names.append('rgb')
            cloud = point_cloud2.read_points(
                msg, field_names=field_names, skip_nans=False)
            points = np.column_stack(
                (cloud['x'], cloud['y'], cloud['z'])).astype(
                    np.float32, copy=False)
        except (AssertionError, ValueError) as error:
            self.get_logger().error(f'Invalid input cloud: {error}')
            return

        finite = np.all(np.isfinite(points), axis=1)
        points = points[finite]
        rgb = None
        if 'rgb' in cloud.dtype.names:
            rgb_values = cloud['rgb'][finite]
            if np.issubdtype(rgb_values.dtype, np.floating):
                rgb = rgb_values.astype(np.float32, copy=False).view(np.uint32)
            else:
                rgb = rgb_values.astype(np.uint32, copy=False)

        distances = np.linalg.norm(points, axis=1)
        distance_mask = (
            (distances > float(self.get_parameter('distance_min').value)) &
            (distances < float(self.get_parameter('distance_max').value))
        )
        distance_points = points[distance_mask]
        distance_indices = np.flatnonzero(distance_mask)
        distance_rgb = rgb[distance_mask] if rgb is not None else None

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

        transformed_all = self.transform_points(points, transform)
        transformed = transformed_all[distance_mask]
        height_points = filter_height(
            transformed,
            float(self.get_parameter('height_min').value),
            float(self.get_parameter('height_max').value),
        )
        height_mask = (
            (transformed[:, 2] >=
             float(self.get_parameter('height_min').value)) &
            (transformed[:, 2] <=
             float(self.get_parameter('height_max').value))
        )
        height_rgb = (
            distance_rgb[height_mask] if distance_rgb is not None else None)
        height_indices = distance_indices[height_mask]

        lateral_mask = lateral_filter_mask(
            height_points, self.lateral_min, self.lateral_max)
        lateral_points = height_points[lateral_mask]
        lateral_rgb = (
            height_rgb[lateral_mask] if height_rgb is not None else None)
        lateral_indices = height_indices[lateral_mask]

        spatial_mask = radius_outlier_mask(
            lateral_points,
            float(self.get_parameter('radius_search').value),
            int(self.get_parameter('min_neighbors').value),
        )
        spatial_points = lateral_points[spatial_mask]
        spatial_rgb = (
            lateral_rgb[spatial_mask] if lateral_rgb is not None else None)
        spatial_indices = lateral_indices[spatial_mask]

        required_frames = max(
            1, int(self.get_parameter('temporal_required_frames').value))
        temporal_mask = temporal_filter_mask(
            spatial_points,
            self.history,
            required_frames,
            float(self.get_parameter('temporal_match_radius').value),
        )
        output_points = spatial_points[temporal_mask]
        output_rgb = (
            spatial_rgb[temporal_mask] if spatial_rgb is not None else None)
        approved_indices = spatial_indices[temporal_mask]
        self.history.append(spatial_points.copy())
        while len(self.history) > max(0, required_frames - 1):
            self.history.popleft()

        sensor_origin = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ], dtype=np.float32)
        self.publish_projected_wall_cloud(
            transformed_all,
            approved_indices,
            sensor_origin,
            msg.header.stamp,
        )

        if self.get_parameter('publish_intermediate_clouds').value:
            self.distance_pub.publish(self.make_cloud(
                distance_points, msg.header.stamp, msg.header.frame_id,
                distance_rgb))
            self.height_pub.publish(self.make_cloud(
                height_points, msg.header.stamp, target_frame, height_rgb))
            self.lateral_pub.publish(self.make_cloud(
                lateral_points, msg.header.stamp, target_frame, lateral_rgb))
            self.spatial_pub.publish(self.make_cloud(
                spatial_points, msg.header.stamp, target_frame, spatial_rgb))

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
            output_points, msg.header.stamp, output_frame, output_rgb))


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
