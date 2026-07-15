from collections import deque

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
import numpy as np
import pytest
from sensor_msgs_py import point_cloud2

from tof_stvl_test.pointcloud_preprocessor import (
    filter_lateral,
    make_bounds_line_points,
    PointCloudPreprocessor,
    project_rays_to_wall,
    validate_lateral_limits,
    validate_projected_wall_epsilon,
)


class Parameter:

    def __init__(self, value):
        self.value = value


class Publisher:

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Logger:

    def error(self, _message):
        pass

    def warning(self, _message, **_kwargs):
        pass


class ClockReading:

    def to_msg(self):
        return Time(sec=123, nanosec=456)


class Clock:

    def now(self):
        return ClockReading()


class TransformBuffer:

    def lookup_transform(self, target, source, _stamp, timeout=None):
        transform = TransformStamped()
        transform.header.frame_id = target
        transform.child_frame_id = source
        transform.transform.rotation.w = 1.0
        return transform


class ProcessorHarness(PointCloudPreprocessor):

    def __init__(self):
        self.parameters = {
            'distance_min': 0.10,
            'distance_max': 1.20,
            'target_frame': 'base_footprint',
            'output_frame': 'tof',
            'transform_timeout': 0.10,
            'height_min': 0.05,
            'height_max': 0.60,
            'radius_search': 0.05,
            'min_neighbors': 0,
            'temporal_required_frames': 1,
            'temporal_match_radius': 0.05,
            'publish_intermediate_clouds': True,
            'publish_filter_bounds': True,
            'publish_projected_wall': True,
        }
        self.lateral_min = -0.25
        self.lateral_max = 0.25
        self.history = deque()
        self.tf_buffer = TransformBuffer()
        self.distance_pub = Publisher()
        self.height_pub = Publisher()
        self.lateral_pub = Publisher()
        self.spatial_pub = Publisher()
        self.output_pub = Publisher()
        self.filter_bounds_pub = Publisher()
        self.projected_wall_pub = Publisher()
        self.filter_bounds_visible = False
        self.projected_wall_visible = False
        self.projected_wall_epsilon = 0.001
        self.logger = Logger()
        self.clock = Clock()

    def get_parameter(self, name):
        return Parameter(self.parameters[name])

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return self.clock


def make_input_cloud(points):
    colors = np.arange(1, len(points) + 1, dtype=np.uint32)
    return PointCloudPreprocessor.make_cloud(
        np.asarray(points, dtype=np.float32), Time(), 'tof', colors)


def run_callback(points):
    processor = ProcessorHarness()
    message = make_input_cloud(points)
    PointCloudPreprocessor.cloud_callback(processor, message)
    return processor


def test_lateral_filter_boundaries():
    y_values = [0.0, 0.24, -0.24, 0.26, -0.26, -0.25, 0.25]
    points = np.array(
        [[0.5, y, 0.2] for y in y_values], dtype=np.float32)

    filtered = filter_lateral(points, -0.25, 0.25)

    assert filtered[:, 1] == pytest.approx(
        [0.0, 0.24, -0.24, -0.25, 0.25])


def test_lateral_filter_accepts_empty_cloud():
    points = np.empty((0, 3), dtype=np.float32)

    filtered = filter_lateral(points, -0.25, 0.25)

    assert filtered.shape == (0, 3)


@pytest.mark.parametrize('minimum,maximum', [
    (-0.25, -0.25),
    (0.25, -0.25),
    (np.nan, 0.25),
    (-0.25, np.inf),
])
def test_invalid_lateral_configuration_is_rejected(minimum, maximum):
    with pytest.raises(ValueError, match='lateral_'):
        validate_lateral_limits(minimum, maximum)


def test_debug_and_output_frames_and_rgb_are_preserved():
    processor = run_callback([
        [0.5, 0.0, 0.2],
        [0.5, 0.26, 0.2],
    ])

    lateral = processor.lateral_pub.messages[-1]
    output = processor.output_pub.messages[-1]
    assert lateral.header.frame_id == 'base_footprint'
    assert output.header.frame_id == 'tof'
    assert lateral.width == 1
    assert output.width == 1
    assert [field.name for field in lateral.fields] == ['x', 'y', 'z', 'rgb']
    assert [field.name for field in output.fields] == ['x', 'y', 'z', 'rgb']


def test_empty_lateral_cloud_is_published():
    processor = run_callback([[0.5, 0.26, 0.2]])

    lateral = processor.lateral_pub.messages[-1]
    output = processor.output_pub.messages[-1]
    assert lateral.header.frame_id == 'base_footprint'
    assert lateral.width == 0
    assert output.header.frame_id == 'tof'
    assert output.width == 0
    assert list(point_cloud2.read_points(lateral)) == []


def test_filter_bounds_line_list_geometry_starts_at_tof():
    points = make_bounds_line_points(
        1.20, 0.05, 0.60, -0.25, 0.25, origin_x=0.26)

    assert len(points) == 24
    assert min(point.x for point in points) == pytest.approx(0.26)
    assert max(point.x for point in points) == pytest.approx(1.46)
    assert min(point.y for point in points) == pytest.approx(-0.25)
    assert max(point.y for point in points) == pytest.approx(0.25)
    assert min(point.z for point in points) == pytest.approx(0.05)
    assert max(point.z for point in points) == pytest.approx(0.60)


def test_filter_bounds_marker_metadata_and_disable_cleanup():
    processor = ProcessorHarness()

    processor.publish_filter_bounds_marker()
    marker = processor.filter_bounds_pub.messages[-1]
    assert marker.header.frame_id == 'base_footprint'
    assert marker.header.stamp == Time(sec=123, nanosec=456)
    assert marker.ns == 'tof_filter_bounds'
    assert marker.type == marker.LINE_LIST
    assert marker.action == marker.ADD
    assert marker.pose.orientation.w == pytest.approx(1.0)
    assert marker.scale.x == pytest.approx(0.01)
    assert 0.0 < marker.color.a < 1.0
    assert len(marker.points) == 24

    processor.parameters['publish_filter_bounds'] = False
    processor.publish_filter_bounds_marker()
    assert processor.filter_bounds_pub.messages[-1].action == marker.DELETE
    published_count = len(processor.filter_bounds_pub.messages)
    processor.publish_filter_bounds_marker()
    assert len(processor.filter_bounds_pub.messages) == published_count


def test_projected_wall_uses_real_rays_and_omits_approved_indices():
    rays = np.array([
        [0.60, 0.00, 0.20],
        [0.60, 0.10, 0.20],
        [0.00, 0.00, 0.20],
        [0.60, 0.40, 0.20],
    ], dtype=np.float32)

    projected = project_rays_to_wall(
        rays, [1], [0.0, 0.0, 0.0],
        1.20, -0.25, 0.25, 0.05, 0.60, 0.001)

    assert projected.shape == (1, 3)
    assert projected[0] == pytest.approx([1.20, 0.00, 0.40])


@pytest.mark.parametrize('epsilon', [0.0, -0.001, np.nan, np.inf])
def test_invalid_projected_wall_epsilon_is_rejected(epsilon):
    with pytest.raises(ValueError, match='projected_wall_epsilon'):
        validate_projected_wall_epsilon(epsilon)


def test_projected_wall_cloud_frame_and_disable_cleanup():
    processor = ProcessorHarness()
    rays = np.array([[0.60, 0.00, 0.20]], dtype=np.float32)
    origin = np.zeros(3, dtype=np.float32)

    processor.publish_projected_wall_cloud(rays, [], origin, Time())
    cloud = processor.projected_wall_pub.messages[-1]
    assert cloud.header.frame_id == 'base_footprint'
    assert cloud.header.stamp == Time()
    assert cloud.width > 0
    assert [field.name for field in cloud.fields] == ['x', 'y', 'z']

    processor.parameters['publish_projected_wall'] = False
    processor.publish_projected_wall_cloud(rays, [], origin, Time())
    empty_cloud = processor.projected_wall_pub.messages[-1]
    assert empty_cloud.header.frame_id == 'base_footprint'
    assert empty_cloud.width == 0


def test_projected_wall_aligns_with_filter_bounds_front_face():
    origin = np.array([0.26, 0.0, 0.21], dtype=np.float32)
    rays = np.array([[0.86, 0.0, 0.31]], dtype=np.float32)

    projected = project_rays_to_wall(
        rays, [], origin, 1.20,
        -0.25, 0.25, 0.05, 0.60, 0.001)

    assert projected.shape == (1, 3)
    assert projected[0] == pytest.approx([1.46, 0.0, 0.41])


def test_cloud_callback_creates_silhouette_from_approved_ray_indices():
    processor = run_callback([
        [0.60, 0.00, 0.20],  # Aprovado: deve abrir um buraco.
        [0.60, 0.00, 0.04],  # Rejeitado em altura: deve ser projetado.
    ])

    wall = processor.projected_wall_pub.messages[-1]
    projected = point_cloud2.read_points(
        wall, field_names=['x', 'y', 'z'], skip_nans=True)
    xyz = np.column_stack(
        (projected['x'], projected['y'], projected['z']))
    assert wall.header.frame_id == 'base_footprint'
    assert wall.width == 1
    assert xyz[0] == pytest.approx([1.20, 0.00, 0.08])
