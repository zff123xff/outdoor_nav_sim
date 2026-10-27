from __future__ import annotations

import math
from typing import List, Sequence

import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union

from multi_cover.coverage import boustrophedon_sweep, estimate_boustrophedon_length
from multi_cover.geometry import project_points, shapely_to_polygon_region, sweep_basis
from multi_cover.types import Point, PolygonRegion


def circle_sector_angles(num_sectors: int) -> List[float]:
    return [2.0 * math.pi * i / num_sectors for i in range(num_sectors + 1)]


def make_circle_sector(center: Point, radius: float, angle_start: float, angle_end: float) -> Polygon:
    cx, cy = center
    arc_count = max(8, int(abs(angle_end - angle_start) / (math.pi / 16)))
    points: List[Point] = [(cx, cy)]
    for angle in np.linspace(angle_start, angle_end, arc_count):
        points.append(_circle_point(cx, cy, radius, float(angle)))
    points.append((cx, cy))
    return Polygon(points)


def _circle_point(cx: float, cy: float, radius: float, angle: float) -> Point:
    x = cx + radius * math.cos(angle)
    y = cy + radius * math.sin(angle)
    if abs(x - cx) < 1e-10:
        x = cx
    if abs(y - cy) < 1e-10:
        y = cy
    return (float(x), float(y))


def circle_sectors(center: Point, radius: float, boundaries: Sequence[float]) -> List[Polygon]:
    sectors: List[Polygon] = []
    for index in range(len(boundaries) - 1):
        sectors.append(make_circle_sector(center, radius, boundaries[index], boundaries[index + 1]))
    return sectors


def sector_sweep_angle(angle_start: float, angle_end: float) -> float:
    mid = 0.5 * (angle_start + angle_end)
    return mid + math.pi / 2.0


def sweep_sector(poly: Polygon, angle_start: float, angle_end: float, swath_width: float, overlap: float):
    radial_angle = sector_sweep_angle(angle_start, angle_end)
    arc_angle = mid_angle(angle_start, angle_end)
    radial_len = estimate_boustrophedon_length(poly, radial_angle, swath_width, overlap)
    arc_len = estimate_boustrophedon_length(poly, arc_angle, swath_width, overlap)
    chosen = radial_angle if radial_len <= arc_len else arc_angle
    return boustrophedon_sweep(poly, chosen, swath_width, overlap), chosen


def mid_angle(angle_start: float, angle_end: float) -> float:
    return 0.5 * (angle_start + angle_end)


def decompose_strips(poly: Polygon, sweep_angle_rad: float, step: float) -> List[Polygon]:
    _, normal = sweep_basis(sweep_angle_rad)
    direction = np.array([math.cos(sweep_angle_rad), math.sin(sweep_angle_rad)], dtype=float)

    coords = np.array(poly.exterior.coords[:-1], dtype=float)
    projections = project_points(coords, normal)
    p_min = float(projections.min())
    p_max = float(projections.max())

    centroid = np.array([poly.centroid.x, poly.centroid.y], dtype=float)
    centroid_proj = float(centroid @ normal)

    strips: List[Polygon] = []
    offset = p_min
    while offset < p_max - 1e-9:
        upper = min(offset + step, p_max)
        strip = _strip_polygon(poly, normal, direction, centroid, centroid_proj, offset, upper)
        if strip is not None and not strip.is_empty and strip.area > 1e-8:
            strips.append(strip)
        offset += step
    return strips


def _strip_polygon(
    poly: Polygon,
    normal: np.ndarray,
    direction: np.ndarray,
    centroid: np.ndarray,
    centroid_proj: float,
    lower: float,
    upper: float,
) -> Polygon | None:
    span = poly.envelope.length * 2.0
    lower_line = [
        tuple(centroid + (lower - centroid_proj) * normal - direction * span),
        tuple(centroid + (lower - centroid_proj) * normal + direction * span),
    ]
    upper_line = [
        tuple(centroid + (upper - centroid_proj) * normal - direction * span),
        tuple(centroid + (upper - centroid_proj) * normal + direction * span),
    ]
    strip_rect = Polygon(lower_line + list(reversed(upper_line)))
    clipped = poly.intersection(strip_rect)
    if clipped.is_empty:
        return None
    if clipped.geom_type == "Polygon":
        return clipped
    if clipped.geom_type == "MultiPolygon":
        return max(clipped.geoms, key=lambda geom: geom.area)
    return None


def bundle_strips(strips: Sequence[Polygon], groups: Sequence[slice]) -> List[Polygon]:
    bundled: List[Polygon] = []
    for group in groups:
        parts = [strips[i] for i in range(len(strips))[group]]
        if not parts:
            bundled.append(Polygon())
            continue
        merged = unary_union(parts)
        if merged.geom_type == "Polygon":
            bundled.append(merged)
        elif merged.geom_type == "MultiPolygon":
            bundled.append(max(merged.geoms, key=lambda geom: geom.area))
        else:
            bundled.append(Polygon())
    return bundled


def initial_group_slices(num_strips: int, num_groups: int) -> List[slice]:
    if num_groups <= 0:
        raise ValueError("num_groups must be positive")
    if num_strips == 0:
        return [slice(0, 0) for _ in range(num_groups)]
    base = num_strips // num_groups
    remainder = num_strips % num_groups
    slices: List[slice] = []
    start = 0
    for index in range(num_groups):
        size = base + (1 if index < remainder else 0)
        end = start + size
        slices.append(slice(start, end))
        start = end
    return slices


def area_partition(poly: Polygon, num_parts: int, sweep_angle_rad: float) -> List[Polygon]:
    _, normal = sweep_basis(sweep_angle_rad)
    coords = np.array(poly.exterior.coords[:-1], dtype=float)
    projections = project_points(coords, normal)
    p_min = float(projections.min())
    p_max = float(projections.max())
    step = (p_max - p_min) / num_parts

    direction = np.array([math.cos(sweep_angle_rad), math.sin(sweep_angle_rad)], dtype=float)
    centroid = np.array([poly.centroid.x, poly.centroid.y], dtype=float)
    centroid_proj = float(centroid @ normal)

    parts: List[Polygon] = []
    for index in range(num_parts):
        lower = p_min + index * step
        upper = p_min + (index + 1) * step
        part = _strip_polygon(poly, normal, direction, centroid, centroid_proj, lower, upper)
        if part is None or part.is_empty:
            parts.append(Polygon())
        else:
            parts.append(part)
    return parts


def polygons_to_regions(polygons: Sequence[Polygon]) -> List[PolygonRegion]:
    return [shapely_to_polygon_region(poly) for poly in polygons]
