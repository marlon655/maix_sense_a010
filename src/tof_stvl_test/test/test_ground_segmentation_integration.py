from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_a010_segmentation_uses_base_footprint_ground_plane():
    config_path = PACKAGE_ROOT / 'config' / 'ground_segmentation_a010.yaml'
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    parameters = config['ground_segmentation']['ros__parameters']

    assert parameters['robot_frame'] == 'base_footprint'
    assert parameters['use_imu_orientation'] is False
    assert parameters['lidar_to_ground'] == 0.0
    assert parameters['groundInlierThreshold'] < parameters['cellSizeZPhase2']


def test_launch_keeps_raw_and_final_obstacle_topics_separate():
    launch_path = (
        PACKAGE_ROOT / 'launch' / 'tof_ground_segmentation_static.launch.py')
    launch_source = launch_path.read_text(encoding='utf-8')

    assert "'/tof_preprocessed/ground_input'" in launch_source
    assert "'/ground_segmentation/obstacle_points_raw'" in launch_source
