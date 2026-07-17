from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_postprocessor_connects_raw_obstacles_to_tof_output():
    config_path = PACKAGE_ROOT / 'config' / 'tof_obstacle_postprocessor.yaml'
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    parameters = config['tof_obstacle_postprocessor']['ros__parameters']

    assert parameters['input_topic'] == (
        '/ground_segmentation/obstacle_points_raw')
    assert parameters['output_topic'] == '/ground_segmentation/obstacle_points'
    assert parameters['target_frame'] == 'base_footprint'
    assert parameters['output_frame'] == 'tof'
    assert parameters['height_min'] < parameters['height_max']
    assert parameters['lateral_min'] < parameters['lateral_max']


def test_static_segmentation_launch_starts_postprocessor():
    launch_path = (
        PACKAGE_ROOT / 'launch' / 'tof_ground_segmentation_static.launch.py')
    launch_source = launch_path.read_text(encoding='utf-8')

    assert "executable='tof_obstacle_postprocessor'" in launch_source
    assert "name='tof_obstacle_postprocessor'" in launch_source
