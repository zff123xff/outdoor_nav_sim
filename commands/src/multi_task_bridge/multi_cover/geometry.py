from __future__ import annotations

import math
from typing import List, Sequence, Tuple

import numpy as np
from shapely.geometry import LineString, Point as ShapelyPoint, Polygon

from multi_cover.types import CircleRegion, Point, PolygonRegion, Region, Waypoint


def validate_params(num_drones: int, swath_width: float, sweep_overlap: float) -> None:
    if num_drones < 1:
        raise ValueError("num_drones must be >= 1")
    if swath_width <= 0:
        raise ValueError("swath_width must be > 0")
    if not 0.0 <= sweep_overlap < 1.0:
        raise ValueError("sweep_overlap must be in [0, 1)")


def polygon_to_shapely(region: PolygonRegion) -> Polygon:
    if len(region.vertices) < 3:
        raise ValueError("polygon must have at least 3 vertices")
    poly = Polygon(region.vertices)
    if not poly.is_valid:
        raise ValueError("invalid polygon geometry")
    return poly


def circle_to_shapely(region: CircleRegion) -> Polygon:
    if region.radius <= 0:
        raise ValueError("circle radius must be > 0")
    cx, cy = region.center
    return ShapelyPoint(cx, cy).buffer(region.radius, quad_segs=64)


def region_to_shapely(region: Region) -> Polygon:
    if isinstance(region, CircleRegion):
        return circle_to_shapely(region)
    return polygon_to_shapely(region)


def shapely_to_polygon_region(poly: Polygon) -> PolygonRegion:
    if poly.is_empty:
        return PolygonRegion(vertices=[])
    coords = list(poly.exterior.coords[:-1])
    return PolygonRegion(vertices=[(float(x), float(y)) for x, y in coords])


def is_convex_polygon(region: PolygonRegion) -> bool:
    poly = polygon_to_shapely(region)
    return poly.equals(poly.convex_hull)


def assert_convex_polygon(region: PolygonRegion) -> None:
    if not is_convex_polygon(region):
        raise ValueError("concave polygons are not supported")


def waypoints_from_coords(coords: Sequence[Tuple[float, float]]) -> List[Waypoint]:
    return [Waypoint(float(x), float(y)) for x, y in coords]


def coords_from_waypoints(waypoints: Sequence[Waypoint]) -> List[Point]:
    return [(wp.x, wp.y) for wp in waypoints]


def path_length(waypoints: Sequence[Waypoint]) -> float:
    if len(waypoints) < 2:
        return 0.0
    total = 0.0
    for a, b in zip(waypoints, waypoints[1:]):
        total += math.hypot(b.x - a.x, b.y - a.y)
    return total


def min_bounding_extent(poly: Polygon) -> float:
    rect = poly.minimum_rotated_rectangle
    coords = list(rect.exterior.coords)
    edges = [
        math.hypot(coords[i + 1][0] - coords[i][0], coords[i + 1][1] - coords[i][1])
        for i in range(4)
    ]
    return min(edges)


def sweep_basis(angle_rad: float) -> Tuple[np.ndarray, np.ndarray]:
    direction = np.array([math.cos(angle_rad), math.sin(angle_rad)], dtype=float)
    normal = np.array([-direction[1], direction[0]], dtype=float)
    return direction, normal


def project_points(points: np.ndarray, axis: np.ndarray) -> np.ndarray:
    return points @ axis


def clip_line_to_polygon(
    poly: Polygon, point_on_line: np.ndarray, direction: np.ndarray
) -> List[Point] | None:
    anchor = _anchor_inside_polygon(poly, point_on_line, direction)
    span = poly.envelope.length * 2.0 + poly.area
    line = LineString(
        [
            tuple(anchor - direction * span),
            tuple(anchor + direction * span),
        ]
    )
    inter = poly.intersection(line)
    if inter.is_empty:
        return None
    if inter.geom_type == "LineString":
        coords = list(inter.coords)
    elif inter.geom_type == "MultiLineString":
        coords = list(max(inter.geoms, key=lambda geom: geom.length).coords)
    elif inter.geom_type == "Point":
        c = inter.coords[0]
        return [(float(c[0]), float(c[1]))]
    else:
        return None
    if len(coords) < 2:
        return None
    return [(float(x), float(y)) for x, y in coords]


def _anchor_inside_polygon(
    poly: Polygon, point_on_line: np.ndarray, direction: np.ndarray
) -> np.ndarray:
    """Nudge sweep anchors off the boundary so clipping returns full segments."""
    anchor = np.asarray(point_on_line, dtype=float)
    probe = ShapelyPoint(anchor)
    if poly.contains(probe):
        return anchor

    normal = np.array([-direction[1], direction[0]], dtype=float)
    centroid = np.array([poly.centroid.x, poly.centroid.y], dtype=float)
    sign = 1.0 if (centroid - anchor) @ normal >= 0.0 else -1.0
    for scale in (1e-4, 1e-6, 1e-8):
        trial = anchor + sign * normal * scale
        if poly.contains(ShapelyPoint(trial)):
            return trial
    return anchor


def narrowest_chord_through_centroid(poly: Polygon) -> List[Point]:
    coords = np.array(poly.exterior.coords[:-1], dtype=float)
    centroid = np.array([poly.centroid.x, poly.centroid.y], dtype=float)

    best: List[Point] | None = None
    best_len = float("inf")
    for angle in np.linspace(0.0, math.pi, 36, endpoint=False):
        _, normal = sweep_basis(angle)
        point_on_line = centroid
        direction = np.array([normal[1], -normal[0]], dtype=float)
        segment = clip_line_to_polygon(poly, point_on_line, direction)
        if segment is None:
            continue
        length = math.hypot(segment[1][0] - segment[0][0], segment[1][1] - segment[0][1])
        if length < best_len:
            best_len = length
            best = segment
    if best is None:
        raise ValueError("unable to compute centroid chord")
    return best


def circle_diameter_chord(center: Point, radius: float) -> List[Point]:
    cx, cy = center
    return [(cx - radius, cy), (cx + radius, cy)]
