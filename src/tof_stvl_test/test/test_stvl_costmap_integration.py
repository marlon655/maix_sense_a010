from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def load_costmap_parameters():
    config_path = PACKAGE_ROOT / 'config' / 'stvl_local_costmap_a010.yaml'
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    return config['local_costmap']['local_costmap']['ros__parameters']


def load_postprocessor_parameters():
    config_path = PACKAGE_ROOT / 'config' / 'tof_obstacle_postprocessor.yaml'
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    return config['tof_obstacle_postprocessor']['ros__parameters']


def test_stvl_consumes_final_obstacle_cloud():
    parameters = load_costmap_parameters()
    postprocessor = load_postprocessor_parameters()
    stvl = parameters['stvl_layer']
    source = stvl['tof_obstacles']

    assert parameters['global_frame'] == 'base_footprint'
    assert parameters['robot_base_frame'] == 'base_footprint'
    assert isinstance(parameters['width'], int)
    assert isinstance(parameters['height'], int)
    assert parameters['resolution'] == 0.03
    assert stvl['voxel_size'] == 0.03
    assert stvl['plugin'] == (
        'spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer')
    assert source['topic'] == '/ground_segmentation/obstacle_points'
    assert source['data_type'] == 'PointCloud2'
    assert source['marking'] is True
    assert source['clearing'] is False
    assert source['min_obstacle_height'] < source['max_obstacle_height']
    assert source['min_obstacle_height'] == postprocessor['height_min']


def test_main_launch_starts_costmap_and_lifecycle_manager():
    launch_path = (
        PACKAGE_ROOT / 'launch' / 'tof_ground_segmentation_static.launch.py')
    launch_source = launch_path.read_text(encoding='utf-8')

    assert "executable='nav2_costmap_2d'" in launch_source
    assert "name='local_costmap'" in launch_source
    assert "executable='lifecycle_manager'" in launch_source
    assert "'bond_timeout': 0.0" in launch_source
    assert "'/local_costmap/local_costmap'" in launch_source
