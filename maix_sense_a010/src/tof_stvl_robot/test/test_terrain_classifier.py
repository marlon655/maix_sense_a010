import math

import numpy as np
import pytest
from tof_stvl_robot.terrain_classifier import (
    analyze_terrain,
    build_elevation_grid,
    classify_terrain_cells,
    create_terrain_masks,
    find_seed_cells,
    TerrainAnalysisConfig,
    TerrainClass,
    validate_terrain_config,
)


def default_config(**overrides):
    values = {
        'cell_size': 0.10,
        'min_points_per_cell': 3,
        'seed_x_min': 0.20,
        'seed_x_max': 0.45,
        'seed_half_width': 0.30,
        'seed_height_tolerance': 0.05,
        'flat_max_slope_deg': 3.0,
        'max_traversable_slope_deg': 10.0,
        'slope_noise_tolerance': 0.008,
        'max_roughness': 0.020,
        'obstacle_clearance': 0.040,
        'min_ramp_length': 0.20,
        'unknown_is_obstacle': True,
        'allow_small_steps': False,
        'max_traversable_step_height': 0.025,
    }
    values.update(overrides)
    return TerrainAnalysisConfig(**values)


def surface_points(slope_deg=0.0, x_start=0.25, x_stop=1.05,
                   step_at=None, step_height=0.0, rough_cell=None,
                   object_at=None):
    points = []
    slope = math.tan(math.radians(slope_deg))
    for x in np.arange(x_start, x_stop, 0.10):
        base_z = max(0.0, x - x_start) * slope
        if step_at is not None and x >= step_at:
            base_z += step_height
        for offset in (-0.015, 0.0, 0.015):
            z = base_z
            if rough_cell is not None and abs(x - rough_cell) < 0.04:
                z += { -0.015: -0.035, 0.0: 0.0, 0.015: 0.035 }[offset]
            points.append([float(x), 0.025 + offset, float(z)])
        if object_at is not None and abs(x - object_at) < 0.04:
            for offset in (-0.018, -0.012, -0.006, 0.0, 0.006, 0.012, 0.018):
                points.append([float(x), 0.025 + offset, float(base_z)])
            points.append([float(x), 0.025, float(base_z + 0.10)])
    return np.asarray(points, dtype=np.float32)


def classes_for(points, config=None):
    if config is None:
        config = default_config()
    cells = build_elevation_grid(points, config.cell_size,
                                 config.min_points_per_cell)
    seeds = find_seed_cells(
        cells, config.seed_x_min, config.seed_x_max,
        config.seed_half_width, config.seed_height_tolerance,
        config.max_roughness)
    return classify_terrain_cells(cells, seeds, config), cells


def test_flat_ground_is_removed_from_obstacle_mask():
    points = surface_points(0.0)

    result = analyze_terrain(points, default_config())

    assert not result.fallback_to_height_filter
    assert np.count_nonzero(result.terrain_mask) == len(points)
    assert np.count_nonzero(result.obstacle_mask) == 0
    assert set(result.cell_classes.values()) == {TerrainClass.FLAT}


def test_traversable_8deg_ramp_is_removed():
    points = surface_points(8.0)

    result = analyze_terrain(points, default_config())

    assert not result.fallback_to_height_filter
    assert np.count_nonzero(result.ramp_mask) > 0
    assert np.count_nonzero(result.obstacle_mask) == 0
    assert TerrainClass.RAMP in set(result.cell_classes.values())


def test_traversable_10deg_ramp_near_limit_is_removed():
    points = surface_points(10.0)

    result = analyze_terrain(points, default_config())

    assert not result.fallback_to_height_filter
    assert np.count_nonzero(result.obstacle_mask) == 0
    assert set(result.cell_classes.values()) <= {
        TerrainClass.FLAT,
        TerrainClass.RAMP,
    }


def test_non_traversable_ramp_is_kept_as_obstacle():
    points = surface_points(15.0)

    result = analyze_terrain(points, default_config())

    assert not result.fallback_to_height_filter
    assert np.count_nonzero(result.obstacle_mask) > 0
    assert (TerrainClass.NON_TRAVERSABLE_SLOPE in
            set(result.cell_classes.values()) or
            TerrainClass.STEP in set(result.cell_classes.values()))


def test_abrupt_4cm_step_is_kept_as_obstacle():
    points = surface_points(0.0, step_at=0.55, step_height=0.04)
    classes, cells = classes_for(points)
    result = create_terrain_masks(points, cells, classes, default_config())

    assert TerrainClass.STEP in set(classes.values())
    assert np.count_nonzero(result.obstacle_mask) > 0
    assert TerrainClass.RAMP not in set(result.cell_classes.values())


def test_small_entry_lip_before_traversable_ramp_is_removed():
    floor = surface_points(0.0, x_start=0.05, x_stop=0.45)
    lip = np.asarray([
        [0.45, 0.010, 0.050],
        [0.45, 0.025, 0.050],
        [0.45, 0.040, 0.050],
    ], dtype=np.float32)
    ramp = surface_points(10.0, x_start=0.55, x_stop=1.15)
    points = np.vstack((floor, lip, ramp))
    config = default_config(
        seed_x_min=0.0,
        seed_x_max=0.35,
        seed_height_tolerance=0.08,
        max_traversable_slope_deg=20.0,
        slope_noise_tolerance=0.03,
    )

    result = analyze_terrain(points, config)

    assert not result.fallback_to_height_filter
    assert np.count_nonzero(result.step_mask) == 0
    assert np.count_nonzero(result.obstacle_mask) == 0
    assert TerrainClass.RAMP in set(result.cell_classes.values())


def test_small_step_to_flat_platform_remains_obstacle():
    floor = surface_points(0.0, x_start=0.05, x_stop=0.45)
    step = np.asarray([
        [0.45, 0.010, 0.100],
        [0.45, 0.025, 0.100],
        [0.45, 0.040, 0.100],
    ], dtype=np.float32)
    platform = surface_points(0.0, x_start=0.55, x_stop=1.15)
    platform[:, 2] += 0.100
    points = np.vstack((floor, step, platform))
    config = default_config(
        seed_x_min=0.0,
        seed_x_max=0.35,
        seed_height_tolerance=0.08,
        max_traversable_slope_deg=20.0,
        slope_noise_tolerance=0.03,
    )

    result = analyze_terrain(points, config)

    assert np.count_nonzero(result.obstacle_mask) > 0
    assert TerrainClass.RAMP not in set(result.cell_classes.values())


def test_high_seed_plateau_is_not_released_as_terrain():
    points = surface_points(0.0, x_start=0.05, x_stop=0.80)
    points[:, 2] += 0.10
    config = default_config(
        seed_x_min=0.0,
        seed_x_max=0.60,
        seed_height_tolerance=0.12,
    )

    result = analyze_terrain(points, config)

    assert result.fallback_to_height_filter
    assert np.count_nonzero(result.terrain_mask) == 0


def test_high_seed_on_continuous_ramp_is_released_after_plane_check():
    points = surface_points(10.0, x_start=0.05, x_stop=0.80)
    points[:, 2] += 0.05 * math.tan(math.radians(10.0))
    config = default_config(
        seed_x_min=0.0,
        seed_x_max=0.60,
        seed_height_tolerance=0.12,
        max_traversable_slope_deg=20.0,
    )

    result = analyze_terrain(points, config)

    assert not result.fallback_to_height_filter
    assert np.count_nonzero(result.obstacle_mask) == 0
    assert TerrainClass.RAMP in set(result.cell_classes.values())


def test_irregular_surface_is_kept_as_obstacle():
    points = surface_points(0.0, rough_cell=0.65)
    classes, cells = classes_for(points)
    result = create_terrain_masks(points, cells, classes, default_config())

    assert TerrainClass.IRREGULAR in set(classes.values())
    assert np.count_nonzero(result.irregular_mask) > 0
    assert np.count_nonzero(result.obstacle_mask) > 0


def test_object_on_8deg_ramp_remains_obstacle():
    points = surface_points(8.0, object_at=0.65)

    result = analyze_terrain(points, default_config(max_roughness=0.030))

    assert np.count_nonzero(result.ramp_mask) > 0
    assert np.count_nonzero(result.obstacle_mask) > 0
    assert np.max(points[result.obstacle_mask, 2]) > 0.12


def test_sparse_cells_trigger_height_filter_fallback():
    points = surface_points(0.0)[::3]

    result = analyze_terrain(points, default_config())

    assert result.fallback_to_height_filter


def test_smooth_low_plane_without_seed_cells_is_removed():
    points = surface_points(0.0, x_start=0.60, x_stop=1.20)

    result = analyze_terrain(points, default_config())

    assert not result.fallback_to_height_filter
    assert np.count_nonzero(result.obstacle_mask) == 0
    assert set(result.cell_classes.values()) == {TerrainClass.FLAT}


def test_smooth_low_5deg_ramp_without_seed_cells_is_removed():
    points = surface_points(5.0, x_start=0.60, x_stop=1.20)

    result = analyze_terrain(points, default_config())

    assert not result.fallback_to_height_filter
    assert np.count_nonzero(result.obstacle_mask) == 0
    assert TerrainClass.RAMP in set(result.cell_classes.values())


def test_close_ramp_uses_base_footprint_reference_without_floor_seed():
    points = surface_points(15.0, x_start=0.20, x_stop=0.90)
    points[:, 2] += 0.20 * math.tan(math.radians(15.0))
    config = default_config(
        seed_height_tolerance=0.05,
        max_traversable_slope_deg=20.0,
    )

    result = analyze_terrain(points, config)

    assert not result.fallback_to_height_filter
    assert np.count_nonzero(result.obstacle_mask) == 0
    assert TerrainClass.RAMP in set(result.cell_classes.values())


def test_ramp_starting_ahead_uses_ground_crossing_reference():
    points = surface_points(10.0, x_start=0.60, x_stop=1.30)
    config = default_config(
        seed_x_max=1.20,
        seed_height_tolerance=0.05,
        max_traversable_slope_deg=20.0,
    )

    result = analyze_terrain(points, config)

    assert not result.fallback_to_height_filter
    assert np.count_nonzero(result.obstacle_mask) == 0
    assert TerrainClass.RAMP in set(result.cell_classes.values())


def test_close_ramp_above_limit_without_floor_seed_is_kept_as_obstacle():
    points = surface_points(15.0, x_start=0.20, x_stop=0.90)
    points[:, 2] += 0.20 * math.tan(math.radians(15.0))

    result = analyze_terrain(points, default_config())

    assert result.fallback_to_height_filter


def test_high_plane_without_seed_cells_triggers_height_filter_fallback():
    points = surface_points(0.0, x_start=0.60, x_stop=1.20)
    points[:, 2] += 0.20

    result = analyze_terrain(points, default_config())

    assert result.fallback_to_height_filter


def test_invalid_terrain_config_is_rejected():
    with pytest.raises(ValueError, match='terrain_cell_size'):
        validate_terrain_config(default_config(cell_size=0.0))

    with pytest.raises(ValueError, match='seed x range'):
        validate_terrain_config(default_config(seed_x_min=1.0, seed_x_max=0.5))

    with pytest.raises(ValueError, match='slope'):
        validate_terrain_config(default_config(max_traversable_slope_deg=95.0))
