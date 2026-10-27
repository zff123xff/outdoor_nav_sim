from __future__ import annotations

import math
from typing import List, Sequence, Tuple

from shapely.geometry import LineString, Point, Polygon

from multi_cover.geometry import coords_from_waypoints, polygon_to_shapely
from multi_cover.types import PolygonRegion, Waypoint


def segments_intersect(
    a0: Tuple[float, float],
    a1: Tuple[float, float],
    b0: Tuple[float, float],
    b1: Tuple[float, float],
    eps: float = 1e-9,
) -> bool:
    if _points_close(a0, b0, eps) or _points_close(a0, b1, eps):
        return False
    if _points_close(a1, b0, eps) or _points_close(a1, b1, eps):
        return False
    line_a = LineString([a0, a1])
    line_b = LineString([b0, b1])
    inter = line_a.intersection(line_b)
    if inter.is_empty:
        return False
    if inter.geom_type == "Point":
        x, y = inter.x, inter.y
        if _points_close((x, y), a0, eps) or _points_close((x, y), a1, eps):
            return False
        if _points_close((x, y), b0, eps) or _points_close((x, y), b1, eps):
            return False
        return True
    return inter.geom_type in {"LineString", "MultiLineString"}


def _points_close(p: Tuple[float, float], q: Tuple[float, float], eps: float) -> bool:
    return math.hypot(p[0] - q[0], p[1] - q[1]) <= eps


def paths_cross(paths: Sequence[Sequence[Waypoint]]) -> bool:
    for i, path_a in enumerate(paths):
        coords_a = coords_from_waypoints(path_a)
        for j in range(i + 1, len(paths)):
            coords_b = coords_from_waypoints(paths[j])
            for a0, a1 in zip(coords_a, coords_a[1:]):
                for b0, b1 in zip(coords_b, coords_b[1:]):
                    if segments_intersect(a0, a1, b0, b1):
                        return True
    return False


def sub_regions_disjoint(regions: Sequence[PolygonRegion], tol_area: float = 1e-6) -> bool:
    polygons: List[Polygon] = []
    for region in regions:
        if not region.vertices:
            continue
        polygons.append(polygon_to_shapely(region))
    for i, poly_a in enumerate(polygons):
        for poly_b in polygons[i + 1 :]:
            inter = poly_a.intersection(poly_b)
            if not inter.is_empty and inter.area > tol_area:
                return False
    return True


def compute_balance_ratio(lengths: Sequence[float]) -> float:
    positive = [value for value in lengths if value > 0]
    if len(positive) <= 1:
        return 1.0
    return max(positive) / min(positive)


def max_coverage_distance(
    region: Polygon,
    paths: Sequence[Sequence[Waypoint]],
    sample_step: float = 5.0,
) -> float:
    """Max distance from region interior sample points to the nearest path segment."""
    segments: List[LineString] = []
    for path in paths:
        coords = coords_from_waypoints(path)
        for start, end in zip(coords, coords[1:]):
            segments.append(LineString([start, end]))
    if not segments:
        return float("inf")

    minx, miny, maxx, maxy = region.bounds
    max_distance = 0.0
    x = minx
    while x <= maxx:
        y = miny
        while y <= maxy:
            point = Point(x, y)
            if region.contains(point):
                distance = min(segment.distance(point) for segment in segments)
                max_distance = max(max_distance, distance)
            y += sample_step
        x += sample_step
    return max_distance
