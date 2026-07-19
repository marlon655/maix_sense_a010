from collections import deque
from dataclasses import dataclass
from enum import IntEnum
import math

import numpy as np


class TerrainClass(IntEnum):
    UNKNOWN = 0
    FLAT = 1
    RAMP = 2
    STEP = 3
    IRREGULAR = 4
    NON_TRAVERSABLE_SLOPE = 5


@dataclass(frozen=True)
class ElevationCell:
    key: tuple
    center: tuple
    count: int
    z: float
    z_low: float
    z_high: float
    roughness: float
    indices: np.ndarray


@dataclass(frozen=True)
class TerrainAnalysisConfig:
    cell_size: float
    min_points_per_cell: int
    seed_x_min: float
    seed_x_max: float
    seed_half_width: float
    seed_height_tolerance: float
    flat_max_slope_deg: float
    max_traversable_slope_deg: float
    slope_noise_tolerance: float
    max_roughness: float
    obstacle_clearance: float
    min_ramp_length: float
    unknown_is_obstacle: bool
    allow_small_steps: bool
    max_traversable_step_height: float


@dataclass(frozen=True)
class TerrainAnalysisResult:
    obstacle_mask: np.ndarray
    terrain_mask: np.ndarray
    ramp_mask: np.ndarray
    step_mask: np.ndarray
    irregular_mask: np.ndarray
    non_traversable_mask: np.ndarray
    point_classes: np.ndarray
    cell_classes: dict
    cells: dict
    fallback_to_height_filter: bool = False


def validate_terrain_config(config):
    values = {
        'terrain_cell_size': config.cell_size,
        'terrain_seed_x_min': config.seed_x_min,
        'terrain_seed_x_max': config.seed_x_max,
        'terrain_seed_half_width': config.seed_half_width,
        'terrain_seed_height_tolerance': config.seed_height_tolerance,
        'terrain_flat_max_slope_deg': config.flat_max_slope_deg,
        'terrain_max_traversable_slope_deg': config.max_traversable_slope_deg,
        'terrain_slope_noise_tolerance': config.slope_noise_tolerance,
        'terrain_max_roughness': config.max_roughness,
        'terrain_obstacle_clearance': config.obstacle_clearance,
        'terrain_min_ramp_length': config.min_ramp_length,
        'terrain_max_traversable_step_height':
            config.max_traversable_step_height,
    }
    for name, value in values.items():
        if not np.isfinite(value):
            raise ValueError(f'{name} must be finite')
    if config.cell_size <= 0.0:
        raise ValueError('terrain_cell_size must be greater than zero')
    if config.min_points_per_cell < 1:
        raise ValueError('terrain_min_points_per_cell must be at least 1')
    if config.seed_x_min > config.seed_x_max:
        raise ValueError('terrain seed x range is inverted')
    non_negative = [
        'terrain_seed_half_width',
        'terrain_seed_height_tolerance',
        'terrain_slope_noise_tolerance',
        'terrain_max_roughness',
        'terrain_obstacle_clearance',
        'terrain_min_ramp_length',
        'terrain_max_traversable_step_height',
    ]
    for name in non_negative:
        if values[name] < 0.0:
            raise ValueError(f'{name} must not be negative')
    if not (0.0 <= config.flat_max_slope_deg <= 89.0):
        raise ValueError('terrain_flat_max_slope_deg must be in [0, 89]')
    if not (0.0 <= config.max_traversable_slope_deg <= 89.0):
        raise ValueError(
            'terrain_max_traversable_slope_deg must be in [0, 89]')
    if config.flat_max_slope_deg > config.max_traversable_slope_deg:
        raise ValueError(
            'terrain_flat_max_slope_deg must not exceed '
            'terrain_max_traversable_slope_deg')


def build_elevation_grid(points, cell_size, min_points_per_cell=1):
    points = np.asarray(points, dtype=np.float32).reshape((-1, 3))
    if len(points) == 0:
        return {}

    cell_xy = np.floor(points[:, :2] / cell_size).astype(np.int64)
    grouped = {}
    for index, key_xy in enumerate(cell_xy):
        key = (int(key_xy[0]), int(key_xy[1]))
        grouped.setdefault(key, []).append(index)

    cells = {}
    for key, indices_list in grouped.items():
        indices = np.asarray(indices_list, dtype=np.int64)
        if len(indices) < min_points_per_cell:
            continue
        z_values = points[indices, 2]
        z_low = float(np.percentile(z_values, 20.0))
        z_high = float(np.percentile(z_values, 80.0))
        z = float(np.percentile(z_values, 50.0))
        roughness = float(np.percentile(z_values, 90.0) -
                          np.percentile(z_values, 10.0))
        center = ((key[0] + 0.5) * cell_size,
                  (key[1] + 0.5) * cell_size)
        cells[key] = ElevationCell(
            key=key,
            center=center,
            count=len(indices),
            z=z,
            z_low=z_low,
            z_high=z_high,
            roughness=roughness,
            indices=indices,
        )
    return cells


def find_seed_cells(cells, seed_x_min, seed_x_max, seed_half_width,
                    seed_height_tolerance, max_roughness):
    seeds = []
    for key, cell in cells.items():
        x, y = cell.center
        if not (seed_x_min <= x <= seed_x_max):
            continue
        if abs(y) > seed_half_width:
            continue
        if abs(cell.z) > seed_height_tolerance:
            continue
        if cell.roughness > max_roughness:
            continue
        seeds.append(key)
    return seeds


def fit_smooth_plane(cells, config):
    fit_cells = [
        cell for cell in cells.values()
        if cell.roughness <= config.max_roughness
    ]
    if len(fit_cells) < 3:
        return None

    centers = np.asarray([cell.center for cell in fit_cells],
                         dtype=np.float64)
    heights = np.asarray([cell.z for cell in fit_cells],
                         dtype=np.float64)
    counts = np.asarray([cell.count for cell in fit_cells],
                        dtype=np.float64)
    if np.ptp(centers[:, 0]) < config.min_ramp_length:
        return None

    design = np.column_stack(
        (centers[:, 0], centers[:, 1], np.ones(len(centers))))
    weights = np.sqrt(np.maximum(counts, 1.0))
    try:
        coefficients, *_ = np.linalg.lstsq(
            design * weights[:, np.newaxis],
            heights * weights,
            rcond=None,
        )
    except np.linalg.LinAlgError:
        return None

    predicted = design @ coefficients
    residuals = np.abs(heights - predicted)
    residual_limit = config.max_roughness + config.slope_noise_tolerance
    if float(np.percentile(residuals, 90.0)) > residual_limit:
        return None

    slope = math.atan(math.hypot(coefficients[0], coefficients[1]))
    return coefficients, math.degrees(slope)


def plane_is_ground_referenced(coefficients, config):
    # Usa base_footprint como referencia. Se o plano nao nasce exatamente em
    # z=0 no robo, ele ainda pode ser uma rampa cujo inicio esta alguns
    # centimetros a frente; nesse caso o plano deve cruzar z=0 dentro da janela
    # de semente configurada.
    origin_height = float(coefficients[2])
    ground_limit = (
        config.max_traversable_step_height + config.slope_noise_tolerance)
    if abs(origin_height) <= ground_limit:
        return True

    forward_slope = float(coefficients[0])
    if forward_slope <= 1e-4:
        return False
    ground_crossing_x = -origin_height / forward_slope
    return 0.0 <= ground_crossing_x <= config.seed_x_max


def filter_safe_seed_cells(cells, seed_keys, config):
    low_seed_limit = (
        config.max_traversable_step_height + config.slope_noise_tolerance)
    low_seeds = [
        key for key in seed_keys
        if abs(cells[key].z) <= low_seed_limit
    ]
    elevated_seeds = [key for key in seed_keys if key not in low_seeds]
    if not elevated_seeds:
        return seed_keys

    plane = fit_smooth_plane(cells, config)
    if plane is None:
        return low_seeds

    coefficients, slope_deg = plane
    if slope_deg > config.max_traversable_slope_deg:
        return low_seeds
    if slope_deg <= config.flat_max_slope_deg:
        return low_seeds
    if not plane_is_ground_referenced(coefficients, config):
        return low_seeds
    return seed_keys


def neighbor_keys(key):
    x, y = key
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            yield (x + dx, y + dy)


def classify_neighbor(cell, parent, delta_xy, config):
    if cell.roughness > config.max_roughness:
        return TerrainClass.IRREGULAR

    delta_z = float(cell.z - parent.z)
    abs_delta_z = abs(delta_z)
    expected_delta_z = (
        delta_xy * math.tan(math.radians(config.max_traversable_slope_deg)))
    tolerated_delta_z = expected_delta_z + config.slope_noise_tolerance
    slope = math.atan2(abs_delta_z, delta_xy)
    slope_deg = math.degrees(slope)

    if abs_delta_z <= config.max_traversable_step_height:
        if (config.allow_small_steps or
                slope_deg <= config.flat_max_slope_deg):
            return TerrainClass.FLAT

    if abs_delta_z > tolerated_delta_z:
        if abs_delta_z > config.max_traversable_step_height:
            return TerrainClass.STEP
        return TerrainClass.NON_TRAVERSABLE_SLOPE

    if slope_deg <= config.flat_max_slope_deg:
        return TerrainClass.FLAT
    return TerrainClass.RAMP


def classify_terrain_cells(cells, seed_keys, config):
    classes = {key: TerrainClass.UNKNOWN for key in cells}
    if not seed_keys:
        return classes

    ramp_distance = {key: 0.0 for key in cells}
    queue = deque()
    for key in seed_keys:
        if key in cells:
            classes[key] = TerrainClass.FLAT
            queue.append(key)

    while queue:
        current_key = queue.popleft()
        current_cell = cells[current_key]
        for next_key in neighbor_keys(current_key):
            if next_key not in cells:
                continue
            if classes[next_key] != TerrainClass.UNKNOWN:
                continue

            next_cell = cells[next_key]
            dx = (next_key[0] - current_key[0]) * config.cell_size
            dy = (next_key[1] - current_key[1]) * config.cell_size
            delta_xy = math.hypot(dx, dy)
            if delta_xy <= 0.0:
                continue

            classification = classify_neighbor(
                next_cell, current_cell, delta_xy, config)
            classes[next_key] = classification
            if classification in (TerrainClass.FLAT, TerrainClass.RAMP):
                if classification == TerrainClass.RAMP:
                    ramp_distance[next_key] = (
                        ramp_distance[current_key] + delta_xy)
                else:
                    ramp_distance[next_key] = 0.0
                queue.append(next_key)

    for key, classification in list(classes.items()):
        if classification != TerrainClass.RAMP:
            continue
        if ramp_distance.get(key, 0.0) < config.min_ramp_length:
            classes[key] = TerrainClass.FLAT
    return classes


def suppress_supported_ramp_entry_steps(cells, cell_classes, config):
    updated = dict(cell_classes)
    forward_cells_count = max(
        3, int(math.ceil(config.min_ramp_length / config.cell_size)) + 2)
    entry_height_limit = (
        config.seed_height_tolerance + config.slope_noise_tolerance)

    for key, classification in cell_classes.items():
        if classification != TerrainClass.STEP:
            continue

        cell = cells[key]
        if abs(cell.z) > entry_height_limit:
            continue

        forward_cells = {}
        for dx in range(1, forward_cells_count + 1):
            for dy in (-1, 0, 1):
                forward_key = (key[0] + dx, key[1] + dy)
                forward_cell = cells.get(forward_key)
                if forward_cell is None:
                    continue
                if forward_cell.roughness > config.max_roughness:
                    continue
                forward_cells[forward_key] = forward_cell

        if len(forward_cells) < 3:
            continue

        plane = fit_smooth_plane(forward_cells, config)
        if plane is None:
            continue

        coefficients, slope_deg = plane
        if slope_deg <= config.flat_max_slope_deg:
            continue
        if slope_deg > config.max_traversable_slope_deg:
            continue

        min_forward_height = min(abs(cell.z) for cell in forward_cells.values())
        if (not plane_is_ground_referenced(coefficients, config) and
                min_forward_height > config.seed_height_tolerance):
            continue

        updated[key] = TerrainClass.RAMP
        for forward_key in forward_cells:
            if updated.get(forward_key) == TerrainClass.UNKNOWN:
                updated[forward_key] = TerrainClass.RAMP

    return updated


def classify_smooth_low_plane_without_seeds(cells, config):
    if len(cells) < 3:
        return None

    plane = fit_smooth_plane(cells, config)
    if plane is None:
        return None

    coefficients, slope_deg = plane
    if slope_deg > config.max_traversable_slope_deg:
        return None

    ground_referenced = plane_is_ground_referenced(coefficients, config)
    if not ground_referenced:
        if slope_deg <= config.flat_max_slope_deg:
            return None
        min_visible_height = min(abs(cell.z) for cell in cells.values())
        if min_visible_height > config.seed_height_tolerance:
            return None

    classification = (
        TerrainClass.FLAT
        if slope_deg <= config.flat_max_slope_deg
        else TerrainClass.RAMP
    )
    return {key: classification for key in cells}


def create_terrain_masks(points, cells, cell_classes, config):
    points = np.asarray(points, dtype=np.float32).reshape((-1, 3))
    total = len(points)
    point_classes = np.full(total, TerrainClass.UNKNOWN, dtype=np.int16)
    obstacle_mask = np.full(total, config.unknown_is_obstacle, dtype=bool)
    terrain_mask = np.zeros(total, dtype=bool)
    ramp_mask = np.zeros(total, dtype=bool)
    step_mask = np.zeros(total, dtype=bool)
    irregular_mask = np.zeros(total, dtype=bool)
    non_traversable_mask = np.zeros(total, dtype=bool)

    for key, cell in cells.items():
        classification = cell_classes.get(key, TerrainClass.UNKNOWN)
        indices = cell.indices
        point_classes[indices] = int(classification)

        if classification in (TerrainClass.FLAT, TerrainClass.RAMP):
            height_above_terrain = points[indices, 2] - cell.z
            obstacle_indices = indices[
                height_above_terrain > config.obstacle_clearance]
            terrain_indices = indices[
                height_above_terrain <= config.obstacle_clearance]
            obstacle_mask[indices] = False
            obstacle_mask[obstacle_indices] = True
            terrain_mask[terrain_indices] = True
            if classification == TerrainClass.RAMP:
                ramp_mask[terrain_indices] = True
            continue

        obstacle_mask[indices] = True
        if classification == TerrainClass.STEP:
            step_mask[indices] = True
        elif classification == TerrainClass.IRREGULAR:
            irregular_mask[indices] = True
        elif classification == TerrainClass.NON_TRAVERSABLE_SLOPE:
            non_traversable_mask[indices] = True

    return TerrainAnalysisResult(
        obstacle_mask=obstacle_mask,
        terrain_mask=terrain_mask,
        ramp_mask=ramp_mask,
        step_mask=step_mask,
        irregular_mask=irregular_mask,
        non_traversable_mask=non_traversable_mask,
        point_classes=point_classes,
        cell_classes=cell_classes,
        cells=cells,
    )


def analyze_terrain(points, config):
    validate_terrain_config(config)
    points = np.asarray(points, dtype=np.float32).reshape((-1, 3))
    empty_mask = np.zeros(len(points), dtype=bool)
    if len(points) == 0:
        return TerrainAnalysisResult(
            obstacle_mask=empty_mask,
            terrain_mask=empty_mask,
            ramp_mask=empty_mask,
            step_mask=empty_mask,
            irregular_mask=empty_mask,
            non_traversable_mask=empty_mask,
            point_classes=np.zeros(len(points), dtype=np.int16),
            cell_classes={},
            cells={},
            fallback_to_height_filter=True,
        )

    cells = build_elevation_grid(
        points, config.cell_size, config.min_points_per_cell)
    seed_keys = find_seed_cells(
        cells,
        config.seed_x_min,
        config.seed_x_max,
        config.seed_half_width,
        config.seed_height_tolerance,
        config.max_roughness,
    )
    seed_keys = filter_safe_seed_cells(cells, seed_keys, config)
    plane_classes = None
    if cells and not seed_keys:
        plane_classes = classify_smooth_low_plane_without_seeds(cells, config)
        if plane_classes is not None:
            return create_terrain_masks(points, cells, plane_classes, config)

    if not cells or not seed_keys:
        return TerrainAnalysisResult(
            obstacle_mask=np.ones(len(points), dtype=bool),
            terrain_mask=empty_mask,
            ramp_mask=empty_mask,
            step_mask=empty_mask,
            irregular_mask=empty_mask,
            non_traversable_mask=empty_mask,
            point_classes=np.full(
                len(points), TerrainClass.UNKNOWN, dtype=np.int16),
            cell_classes={key: TerrainClass.UNKNOWN for key in cells},
            cells=cells,
            fallback_to_height_filter=True,
        )

    cell_classes = classify_terrain_cells(cells, seed_keys, config)
    cell_classes = suppress_supported_ramp_entry_steps(
        cells, cell_classes, config)
    return create_terrain_masks(points, cells, cell_classes, config)
