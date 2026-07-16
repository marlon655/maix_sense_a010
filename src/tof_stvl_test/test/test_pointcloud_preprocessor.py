from collections import deque
from pathlib import Path

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
import numpy as np
import pytest
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformException
import yaml

from tof_stvl_test.pointcloud_preprocessor import (
    filter_lateral,
    make_bounds_line_points,
    PointCloudPreprocessor,
    project_rays_to_wall,
    temporal_filter_mask,
    temporal_timestamp_reset_reason,
    temporal_transform_is_discontinuous,
    validate_lateral_limits,
    validate_projected_wall_epsilon,
)


class Parameter:

    def __init__(self, value):
        self.value = value


class ParameterUpdate:

    def __init__(self, name, value):
        self.name = name
        self.value = value


class Publisher:

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Logger:

    def __init__(self):
        self.warnings = []

    def error(self, _message):
        pass

    def warning(self, message, **_kwargs):
        self.warnings.append(message)


class ClockReading:

    def to_msg(self):
        return Time(sec=123, nanosec=456)


class Clock:

    def now(self):
        return ClockReading()


class TransformBuffer:

    def __init__(self):
        self.odom_x_by_stamp = {}
        self.temporal_tf_failures = set()

    def lookup_transform(self, target, source, stamp, timeout=None):
        stamp_ns = stamp.nanoseconds
        if target == 'odom' and source == 'base_footprint':
            if stamp_ns in self.temporal_tf_failures:
                raise TransformException('temporal transform unavailable')
            translation_x = self.odom_x_by_stamp.get(stamp_ns, 0.0)
        else:
            translation_x = 0.0
        transform = TransformStamped()
        transform.header.frame_id = target
        transform.child_frame_id = source
        transform.transform.translation.x = translation_x
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
            'temporal_reference_frame': 'odom',
            'temporal_max_frame_gap': 0.50,
            'temporal_clear_history_on_tf_failure': True,
            'temporal_max_translation_jump': 1.0,
            'temporal_max_rotation_jump': 1.5708,
            'publish_intermediate_clouds': True,
            'publish_filter_bounds': True,
            'publish_projected_wall': True,
        }
        self.lateral_min = -0.25
        self.lateral_max = 0.25
        self.temporal_history = deque()
        self.temporal_history_frame = 'odom'
        self.temporal_history_required_frames = 1
        self.temporal_last_stamp_ns = None
        self.temporal_last_transform = None
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


def make_input_cloud(points, stamp=None, colors=None):
    if stamp is None:
        stamp = Time(sec=1)
    if colors is None:
        colors = np.arange(1, len(points) + 1, dtype=np.uint32)
    return PointCloudPreprocessor.make_cloud(
        np.asarray(points, dtype=np.float32), stamp, 'tof', colors)


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


def test_temporal_filter_is_equivalent_with_stationary_base_footprint():
    processor = ProcessorHarness()
    processor.parameters['temporal_required_frames'] = 2
    first_stamp = Time(sec=1)
    second_stamp = Time(sec=1, nanosec=100_000_000)

    PointCloudPreprocessor.cloud_callback(
        processor, make_input_cloud([[0.80, 0.0, 0.20]], first_stamp))
    assert processor.output_pub.messages[-1].width == 0

    PointCloudPreprocessor.cloud_callback(
        processor, make_input_cloud([[0.80, 0.0, 0.20]], second_stamp))
    assert processor.output_pub.messages[-1].width == 1
    assert processor.output_pub.messages[-1].header.frame_id == 'tof'


def test_odom_compensation_matches_obstacle_while_robot_moves_016_m():
    processor = ProcessorHarness()
    processor.parameters['temporal_required_frames'] = 2
    first_stamp = Time(sec=1)
    second_stamp = Time(sec=1, nanosec=160_000_000)
    processor.tf_buffer.odom_x_by_stamp[1_160_000_000] = 0.16

    PointCloudPreprocessor.cloud_callback(
        processor, make_input_cloud([[1.00, 0.0, 0.20]], first_stamp))
    PointCloudPreprocessor.cloud_callback(
        processor, make_input_cloud([[0.84, 0.0, 0.20]], second_stamp))

    output = processor.output_pub.messages[-1]
    assert output.header.frame_id == 'tof'
    assert output.width == 1


def test_same_motion_without_odom_compensation_is_rejected():
    history_base_footprint = deque([
        np.array([[1.00, 0.0, 0.20]], dtype=np.float32),
    ])
    current_base_footprint = np.array(
        [[0.84, 0.0, 0.20]], dtype=np.float32)

    mask = temporal_filter_mask(
        current_base_footprint,
        history_base_footprint,
        required_frames=2,
        match_radius=0.05,
    )

    assert mask.tolist() == [False]


def test_temporal_mask_keeps_base_arrays_rgb_indices_and_wall_aligned():
    processor = ProcessorHarness()
    processor.parameters['temporal_required_frames'] = 2
    first_stamp = Time(sec=1)
    second_stamp = Time(sec=1, nanosec=160_000_000)
    processor.tf_buffer.odom_x_by_stamp[1_160_000_000] = 0.16

    PointCloudPreprocessor.cloud_callback(
        processor,
        make_input_cloud(
            [[1.00, 0.0, 0.20]], first_stamp,
            np.array([10], dtype=np.uint32)),
    )
    PointCloudPreprocessor.cloud_callback(
        processor,
        make_input_cloud(
            [[0.84, 0.0, 0.20], [0.84, 0.10, 0.20]],
            second_stamp,
            np.array([100, 200], dtype=np.uint32)),
    )

    output = processor.output_pub.messages[-1]
    output_data = point_cloud2.read_points(
        output, field_names=['x', 'y', 'z', 'rgb'])
    assert output.width == 1
    assert output_data['x'][0] == pytest.approx(0.84)
    assert output_data['rgb'][0] == 100

    # O indice 0 aprovado abre a silhueta; somente o raio de indice 1 fica.
    wall = processor.projected_wall_pub.messages[-1]
    assert wall.width == 1


def test_temporal_tf_failure_clears_history_and_publishes_no_match():
    processor = ProcessorHarness()
    processor.parameters['temporal_required_frames'] = 2
    first_stamp = Time(sec=1)
    failed_stamp = Time(sec=1, nanosec=100_000_000)

    PointCloudPreprocessor.cloud_callback(
        processor, make_input_cloud([[0.80, 0.0, 0.20]], first_stamp))
    assert len(processor.temporal_history) == 1

    processor.tf_buffer.temporal_tf_failures.add(1_100_000_000)
    PointCloudPreprocessor.cloud_callback(
        processor, make_input_cloud([[0.80, 0.0, 0.20]], failed_stamp))

    assert len(processor.temporal_history) == 0
    assert processor.output_pub.messages[-1].width == 0
    assert any('Cannot transform temporal cloud' in warning
               for warning in processor.logger.warnings)


@pytest.mark.parametrize('second_stamp', [
    Time(sec=0, nanosec=900_000_000),
    Time(sec=2),
])
def test_timestamp_return_or_excessive_gap_resets_history(second_stamp):
    processor = ProcessorHarness()
    processor.parameters['temporal_required_frames'] = 2

    PointCloudPreprocessor.cloud_callback(
        processor, make_input_cloud([[0.80, 0.0, 0.20]], Time(sec=1)))
    PointCloudPreprocessor.cloud_callback(
        processor, make_input_cloud([[0.80, 0.0, 0.20]], second_stamp))

    assert processor.output_pub.messages[-1].width == 0
    assert len(processor.temporal_history) == 1
    assert any('Clearing temporal history' in warning
               for warning in processor.logger.warnings)


def test_odometry_transform_jump_resets_history():
    processor = ProcessorHarness()
    processor.parameters['temporal_required_frames'] = 2
    first_stamp = Time(sec=1)
    second_stamp = Time(sec=1, nanosec=100_000_000)
    processor.tf_buffer.odom_x_by_stamp[1_100_000_000] = 2.0

    PointCloudPreprocessor.cloud_callback(
        processor, make_input_cloud([[0.80, 0.0, 0.20]], first_stamp))
    PointCloudPreprocessor.cloud_callback(
        processor, make_input_cloud([[0.80, 0.0, 0.20]], second_stamp))

    assert processor.output_pub.messages[-1].width == 0
    assert len(processor.temporal_history) == 1
    assert any('odometry transform discontinuity' in warning
               for warning in processor.logger.warnings)


def test_temporal_transform_discontinuity_detects_rotation_jump():
    previous = TransformStamped()
    previous.transform.rotation.w = 1.0
    current = TransformStamped()
    current.transform.rotation.z = np.sin(np.pi / 2.0)
    current.transform.rotation.w = np.cos(np.pi / 2.0)

    assert temporal_transform_is_discontinuous(
        previous, current, 1.0, 1.5708)


def test_temporal_timestamp_helper_reports_gap_and_return():
    assert temporal_timestamp_reset_reason(
        1_000_000_000, Time(sec=0, nanosec=900_000_000), 0.50)
    assert temporal_timestamp_reset_reason(
        1_000_000_000, Time(sec=2), 0.50)
    assert temporal_timestamp_reset_reason(
        1_000_000_000, Time(sec=1, nanosec=100_000_000), 0.50) is None


@pytest.mark.parametrize('update', [
    ParameterUpdate('temporal_reference_frame', 'map'),
    ParameterUpdate('temporal_required_frames', 3),
])
def test_temporal_frame_or_required_frames_change_clears_history(update):
    processor = ProcessorHarness()
    processor.temporal_history.append(
        np.array([[0.80, 0.0, 0.20]], dtype=np.float32))
    processor.temporal_last_stamp_ns = 1_000_000_000
    transform = TransformStamped()
    transform.transform.rotation.w = 1.0
    processor.temporal_last_transform = transform

    result = PointCloudPreprocessor.parameter_callback(processor, [update])

    assert result.successful
    assert len(processor.temporal_history) == 0
    assert processor.temporal_last_stamp_ns is None
    assert processor.temporal_last_transform is None


def test_stvl_subscribes_only_to_filtered_obstacle_cloud():
    package_root = Path(__file__).parents[1]
    with open(
            package_root / 'config' / 'tof_pointcloud_filters.yaml',
            encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file)

    stvl = config['local_costmap']['local_costmap']['ros__parameters'][
        'stvl_layer']
    assert stvl['observation_sources'] == 'pointcloud'
    assert stvl['pointcloud']['topic'] == \
        '/ground_segmentation/obstacle_points'
    assert 'projected_wall' not in str(stvl)


def test_launch_can_disable_static_test_odometry_tf():
    package_root = Path(__file__).parents[1]
    launch_source = (
        package_root / 'launch' / 'stvl_maixsense_test.launch.py'
    ).read_text(encoding='utf-8')

    assert "DeclareLaunchArgument(\n            'publish_static_odom_tf'" in \
        launch_source
    assert "LaunchConfiguration('publish_static_odom_tf')" in launch_source
