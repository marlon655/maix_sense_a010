import math

import numpy as np
import pytest
from tof_stvl_robot.terrain_classifier import (
    analyze_terrain,
    build_elevation_grid,
    classify_terrain_cells,
    create_terrain_masks,
    FallbackReason,
    find_seed_cells,
    TerrainAnalysisConfig,
    TerrainClass,
    validate_terrain_config,
)


def default_config(**overrides):
    values = {
        'cell_size': 0.10,
        'min_points_per_cell': 3,
        'min_reliable_points_per_cell': 2,
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
        'transition_enabled': True,
        'transition_max_length': 0.10,
        'transition_min_forward_cells': 3,
        'transition_min_lateral_width': 0.20,
        'transition_max_plane_residual': 0.025,
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


def wide_surface_points(slope_deg=0.0, x_start=0.05, x_stop=1.05,
                        y_values=None, ramp_start=None, x_step=0.025):
    if y_values is None:
        y_values = (-0.12, -0.06, 0.0, 0.06, 0.12)
    if ramp_start is None:
        ramp_start = x_start

    points = []
    slope = math.tan(math.radians(slope_deg))
    for x in np.arange(x_start, x_stop, x_step):
        z = max(0.0, x - ramp_start) * slope
        for y in y_values:
            points.append([float(x), float(y), float(z)])
    return np.asarray(points, dtype=np.float32)


def mixed_entry_ramp_points(slope_deg=8.0, entry_x=0.40,
                            grid_alignment=0.0, robot_shift=0.0,
                            single_point_transition=False,
                            object_on_ramp=False, narrow=False):
    y_values = (0.0,) if narrow else (-0.12, -0.06, 0.0, 0.06, 0.12)
    entry_x = entry_x + grid_alignment - robot_shift
    floor = wide_surface_points(
        0.0,
        x_start=max(0.0, entry_x - 0.30),
        x_stop=entry_x,
        y_values=y_values,
    )
    ramp = wide_surface_points(
        slope_deg,
        x_start=entry_x + 0.05,
        x_stop=entry_x + 0.55,
        y_values=y_values,
        ramp_start=entry_x,
    )

    slope = math.tan(math.radians(slope_deg))
    transition = []
    transition_x_values = (entry_x - 0.005, entry_x + 0.005)
    for y in y_values:
        transition.append([entry_x - 0.010, y, 0.0])
        transition.append([entry_x, y, 0.030])
        for x in transition_x_values:
            transition.append([x, y, max(0.0, x - entry_x) * slope])
    transition = np.asarray(transition, dtype=np.float32)
    if single_point_transition:
        transition = np.asarray([
            [entry_x, y, 0.030]
            for y in y_values
        ], dtype=np.float32)

    clouds = [floor, transition, ramp]
    if object_on_ramp:
        clouds.append(np.asarray([
            [entry_x + 0.25, -0.04, 0.10],
            [entry_x + 0.25, 0.0, 0.10],
            [entry_x + 0.25, 0.04, 0.10],
            [entry_x + 0.28, -0.04, 0.11],
            [entry_x + 0.28, 0.0, 0.11],
            [entry_x + 0.28, 0.04, 0.11],
        ], dtype=np.float32))
    return np.vstack([cloud for cloud in clouds if len(cloud) > 0])


def transition_config(**overrides):
    values = {
        'cell_size': 0.05,
        'min_points_per_cell': 1,
        'min_reliable_points_per_cell': 2,
        'seed_x_min': 0.0,
        'seed_x_max': 0.35,
        'seed_half_width': 0.18,
        'seed_height_tolerance': 0.12,
        'flat_max_slope_deg': 3.0,
        'max_traversable_slope_deg': 12.0,
        'slope_noise_tolerance': 0.03,
        'max_roughness': 0.020,
        'obstacle_clearance': 0.040,
        'min_ramp_length': 0.20,
        'unknown_is_obstacle': True,
        'allow_small_steps': False,
        'max_traversable_step_height': 0.025,
        'transition_enabled': True,
        'transition_max_length': 0.10,
        'transition_min_forward_cells': 3,
        'transition_min_lateral_width': 0.20,
        'transition_max_plane_residual': 0.025,
    }
    values.update(overrides)
    return TerrainAnalysisConfig(**values)


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


@pytest.mark.parametrize(
    'grid_alignment',
    [index * 0.005 for index in range(10)],
)
def test_mixed_floor_lip_and_8deg_ramp_is_removed_for_grid_alignments(
        grid_alignment):
    points = mixed_entry_ramp_points(
        slope_deg=8.0,
        grid_alignment=grid_alignment,
    )

    result = analyze_terrain(points, transition_config())

    assert not result.fallback_to_height_filter
    assert np.count_nonzero(result.obstacle_mask) == 0
    assert TerrainClass.RAMP in set(result.cell_classes.values())


@pytest.mark.parametrize('robot_shift', [0.0, 0.05, 0.10, 0.18])
def test_mixed_entry_ramp_is_removed_for_robot_positions(robot_shift):
    points = mixed_entry_ramp_points(
        slope_deg=8.0,
        entry_x=0.35,
        robot_shift=robot_shift,
    )

    result = analyze_terrain(points, transition_config())

    assert not result.fallback_to_height_filter
    assert np.count_nonzero(result.obstacle_mask) == 0
    assert TerrainClass.RAMP in set(result.cell_classes.values())


def test_single_point_transition_cell_needs_reliable_forward_ramp_support():
    points = mixed_entry_ramp_points(
        slope_deg=8.0,
        single_point_transition=True,
    )

    result = analyze_terrain(points, transition_config())

    assert not result.fallback_to_height_filter
    assert np.count_nonzero(result.obstacle_mask) == 0
    assert any(
        cell.count == 1 and
        not np.any(result.obstacle_mask[cell.indices])
        for cell in result.cells.values()
        if 0.35 <= cell.center[0] <= 0.50
    )


def test_transition_recovery_disabled_keeps_mixed_entry_conservative():
    points = mixed_entry_ramp_points(slope_deg=8.0)

    result = analyze_terrain(
        points,
        transition_config(transition_enabled=False),
    )

    assert np.count_nonzero(result.obstacle_mask) > 0


def test_no_seed_ramp_entry_uses_forward_support_before_height_fallback():
    points = mixed_entry_ramp_points(
        slope_deg=8.0,
        entry_x=0.08,
        robot_shift=0.06,
    )

    result = analyze_terrain(points, transition_config(seed_x_max=0.01))

    assert not result.fallback_to_height_filter
    assert np.count_nonzero(result.obstacle_mask) == 0
    assert TerrainClass.RAMP in set(result.cell_classes.values())


def test_flat_platform_after_transition_remains_obstacle():
    floor = wide_surface_points(0.0, x_start=0.10, x_stop=0.35)
    transition = wide_surface_points(0.0, x_start=0.40, x_stop=0.45)
    transition[:, 2] += 0.10
    platform = wide_surface_points(0.0, x_start=0.50, x_stop=0.95)
    platform[:, 2] += 0.10
    points = np.vstack((floor, transition, platform))

    result = analyze_terrain(points, transition_config())

    assert np.count_nonzero(result.obstacle_mask) > 0
    assert TerrainClass.RAMP not in set(result.cell_classes.values())


def test_narrow_ramp_like_object_is_not_recovered_as_transition():
    points = mixed_entry_ramp_points(slope_deg=8.0, narrow=True)

    result = analyze_terrain(points, transition_config())

    assert np.count_nonzero(result.obstacle_mask) > 0


def test_object_on_recovered_ramp_remains_obstacle():
    points = mixed_entry_ramp_points(
        slope_deg=8.0,
        object_on_ramp=True,
    )

    result = analyze_terrain(points, transition_config())

    assert np.count_nonzero(result.ramp_mask) > 0
    assert np.count_nonzero(result.obstacle_mask) > 0
    assert np.max(points[result.obstacle_mask, 2]) > 0.09


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
    assert result.fallback_reason in {
        FallbackReason.NO_VALID_CELLS.value,
        FallbackReason.NO_SAFE_SEEDS.value,
        FallbackReason.PLANE_FIT_FAILED.value,
    }


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
    assert result.fallback_reason == \
        FallbackReason.PLANE_NOT_GROUND_REFERENCED.value


def test_empty_cloud_reports_no_points_fallback_reason():
    result = analyze_terrain(np.empty((0, 3)), default_config())

    assert result.fallback_to_height_filter
    assert result.fallback_reason == FallbackReason.NO_POINTS.value


def test_no_valid_cells_reports_fallback_reason():
    points = np.asarray([[0.20, 0.0, 0.0]], dtype=np.float32)

    result = analyze_terrain(points, default_config(min_points_per_cell=3))

    assert result.fallback_to_height_filter
    assert result.fallback_reason == FallbackReason.NO_VALID_CELLS.value


def test_isolated_single_high_point_remains_obstacle():
    points = np.asarray([[0.30, 0.0, 0.10]], dtype=np.float32)

    result = analyze_terrain(points, transition_config())

    assert result.fallback_to_height_filter
    assert np.count_nonzero(result.obstacle_mask) == 1


def test_wall_like_surface_is_not_recovered_as_ramp():
    points = []
    for x in (0.40, 0.41):
        for y in (-0.12, -0.06, 0.0, 0.06, 0.12):
            for z in (0.02, 0.08, 0.14, 0.20):
                points.append([x, y, z])
    points = np.asarray(points, dtype=np.float32)

    result = analyze_terrain(points, transition_config())

    assert np.count_nonzero(result.obstacle_mask) > 0
    assert TerrainClass.RAMP not in set(result.cell_classes.values())


def test_invalid_terrain_config_is_rejected():
    with pytest.raises(ValueError, match='terrain_cell_size'):
        validate_terrain_config(default_config(cell_size=0.0))

    with pytest.raises(ValueError, match='seed x range'):
        validate_terrain_config(default_config(seed_x_min=1.0, seed_x_max=0.5))

    with pytest.raises(ValueError, match='slope'):
        validate_terrain_config(default_config(max_traversable_slope_deg=95.0))
