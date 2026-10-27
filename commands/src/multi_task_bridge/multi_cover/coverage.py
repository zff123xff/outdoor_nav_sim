from __future__ import annotations

import math
from typing import List

import numpy as np
from shapely.geometry import Point as ShapelyPoint, Polygon

from multi_cover.geometry import (
    clip_line_to_polygon,
    path_length,
    project_points,
    sweep_basis,
    waypoints_from_coords,
)
from multi_cover.types import Waypoint


def estimate_boustrophedon_length(
    poly: Polygon, sweep_angle_rad: float, swath_width: float, overlap: float
) -> float:
    waypoints = boustrophedon_sweep(poly, sweep_angle_rad, swath_width, overlap)
    return path_length(waypoints)


def boustrophedon_sweep(
    poly: Polygon,
    sweep_angle_rad: float,
    swath_width: float,
    overlap: float,
) -> List[Waypoint]:
    step = swath_width * (1.0 - overlap)
    if step <= 0:
        raise ValueError("effective step must be > 0")

    direction, normal = sweep_basis(sweep_angle_rad)
    coords = np.array(poly.exterior.coords[:-1], dtype=float)
    if len(coords) == 0:
        return []

    span = _effective_projection_span(poly, normal, direction, step)
    if span is None:
        return []
    p_min, p_max = span

    centroid = np.array([poly.centroid.x, poly.centroid.y], dtype=float)
    centroid_proj = float(centroid @ normal)

    min_segment = step * 0.1
    segments: List[List[tuple[float, float]]] = []
    for offset in _sweep_offsets(p_min, p_max, step):
        point_on_line = centroid + (offset - centroid_proj) * normal
        segment = clip_line_to_polygon(poly, point_on_line, direction)
        if segment is not None and _segment_length(segment) >= min_segment:
            segments.append(segment)

    if not segments:
        return []

    waypoints: List[tuple[float, float]] = []
    for index, segment in enumerate(segments):
        ordered = segment if index % 2 == 0 else list(reversed(segment))
        if waypoints and ordered:
            if waypoints[-1] == ordered[0]:
                ordered = ordered[1:]
        waypoints.extend(ordered)

    clipped = _clip_waypoints_to_polygon(waypoints, poly)
    return waypoints_from_coords(clipped)


def _is_axis_aligned(poly: Polygon) -> bool:
    envelope = poly.envelope
    if not math.isclose(poly.area, envelope.area, rel_tol=1e-6, abs_tol=1e-6):
        return False
    rect = poly.minimum_rotated_rectangle
    return math.isclose(rect.area, envelope.area, rel_tol=1e-6, abs_tol=1e-6)


def best_sweep_angle(
    poly: Polygon, swath_width: float, overlap: float
) -> float:
    minx, miny, maxx, maxy = poly.bounds
    width = maxx - minx
    height = maxy - miny
    preferred = 0.0 if width >= height else math.pi / 2.0

    if _is_axis_aligned(poly):
        candidates = [0.0, math.pi / 2.0]
    else:
        candidates = [math.radians(degrees) for degrees in range(0, 180, 5)]
        if preferred not in candidates:
            candidates.append(preferred)

    best_angle = preferred
    best_score = float("inf")
    step = swath_width * (1.0 - overlap)
    for angle in candidates:
        count = _count_strips(poly, angle, step)
        if count == 0:
            continue
        length = estimate_boustrophedon_length(poly, angle, swath_width, overlap)
        score = length
        if score < best_score - 1e-6:
            best_score = score
            best_angle = angle
        elif abs(score - best_score) <= 1e-6 and _prefer_partition_sweep_angle(
            width, height, angle
        ):
            best_angle = angle
    return best_angle


def _prefer_partition_sweep_angle(width: float, height: float, angle: float) -> bool:
    """Prefer strip partitions along the long task axis for multi-drone balancing."""
    if width >= height:
        return math.isclose(angle, math.pi / 2.0)
    return math.isclose(angle, 0.0)


def _sweep_offsets(p_min: float, p_max: float, step: float) -> List[float]:
    span = p_max - p_min
    if span < 1e-9:
        return []
    if span <= step:
        return [0.5 * (p_min + p_max)]

    count = max(2, int(math.ceil(span / step)) + 1)
    return [p_min + index * span / (count - 1) for index in range(count)]


def _count_strips(poly: Polygon, sweep_angle_rad: float, step: float) -> int:
    direction, normal = sweep_basis(sweep_angle_rad)
    span = _effective_projection_span(poly, normal, direction, step)
    if span is None:
        return 0
    return len(_sweep_offsets(span[0], span[1], step))


def _segment_length(segment: List[tuple[float, float]]) -> float:
    if len(segment) < 2:
        return 0.0
    return math.hypot(segment[1][0] - segment[0][0], segment[1][1] - segment[0][1])


def _effective_projection_span(
    poly: Polygon,
    normal: np.ndarray,
    direction: np.ndarray,
    step: float,
) -> tuple[float, float] | None:
    """Projection span where sweep lines actually cross the polygon interior."""
    coords = np.array(poly.exterior.coords[:-1], dtype=float)
    if len(coords) == 0:
        return None

    coarse_min = float(project_points(coords, normal).min())
    coarse_max = float(project_points(coords, normal).max())
    centroid = np.array([poly.centroid.x, poly.centroid.y], dtype=float)
    centroid_proj = float(centroid @ normal)
    min_segment = step * 0.1
    scan = max(step / 50.0, 0.5)

    valid_offsets: List[float] = []
    offset = coarse_min
    while offset <= coarse_max + 1e-9:
        point_on_line = centroid + (offset - centroid_proj) * normal
        segment = clip_line_to_polygon(poly, point_on_line, direction)
        if segment is not None and _segment_length(segment) >= min_segment:
            valid_offsets.append(offset)
        offset += scan

    if not valid_offsets:
        return None
    margin = min(step * 0.05, 1.0)
    p_min = valid_offsets[0]
    p_max = valid_offsets[-1]
    if p_max - p_min > 2.0 * margin:
        return p_min + margin, p_max - margin
    return p_min, p_max


def _clip_waypoints_to_polygon(
    waypoints: List[tuple[float, float]], poly: Polygon
) -> List[tuple[float, float]]:
    clipped: List[tuple[float, float]] = []
    for x, y in waypoints:
        point = ShapelyPoint(x, y)
        if poly.contains(point) or poly.boundary.distance(point) < 1e-8:
            clipped.append((x, y))
        else:
            nearest = poly.boundary.interpolate(poly.boundary.project(point))
            clipped.append((float(nearest.x), float(nearest.y)))
    return clipped
