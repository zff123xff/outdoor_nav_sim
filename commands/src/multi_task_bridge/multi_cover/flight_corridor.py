from __future__ import annotations

import math
from typing import List, Sequence, Tuple

from shapely.geometry import Polygon
from shapely.geometry.base import BaseGeometry

from multi_cover.types import Waypoint


def default_path_clearance(swath_width: float) -> float:
    """Clearance band removed along inter-drone shared boundaries (per side)."""
    return max(2.0, swath_width * 0.1)


def resolve_path_clearance(params_clearance: float | None, swath_width: float) -> float:
    if params_clearance is None:
        return default_path_clearance(swath_width)
    return max(0.0, params_clearance)


def make_flight_polygon(
    assigned_region: Polygon,
    clearance: float,
    all_assigned: Sequence[Polygon] | None = None,
) -> Polygon:
    """Build flight corridor inside an assigned sub-region.

    Only boundaries shared with another assigned sub-region are inset.
    Outer task boundaries are kept so perimeter coverage is not lost.
    """
    if clearance <= 0 or assigned_region.is_empty:
        return assigned_region

    if not all_assigned:
        return _uniform_inset(assigned_region, clearance)

    flight = assigned_region
    for other in all_assigned:
        if other.equals(assigned_region) or other.is_empty:
            continue
        for shared in _shared_boundaries(assigned_region, other):
            flight = _subtract_shared_band(flight, shared, clearance)

    if flight.is_empty or flight.area < 1e-6:
        return assigned_region
    if flight.geom_type == "MultiPolygon":
        return max(flight.geoms, key=lambda geom: geom.area)
    return flight


def _uniform_inset(assigned_region: Polygon, clearance: float) -> Polygon:
    inset = assigned_region.buffer(-clearance, join_style=2, quad_segs=16)
    if inset.is_empty or inset.area < 1e-6:
        return assigned_region
    if inset.geom_type == "MultiPolygon":
        return max(inset.geoms, key=lambda geom: geom.area)
    return inset


def _subtract_shared_band(region: Polygon, shared: BaseGeometry, clearance: float) -> Polygon:
    if shared.geom_type == "Point":
        removal = shared.buffer(clearance, quad_segs=8)
    else:
        removal = shared.buffer(clearance, cap_style=2, join_style=2, quad_segs=8)
    cut = region.intersection(removal)
    if cut.is_empty:
        return region
    result = region.difference(cut)
    if result.is_empty:
        return region
    return result


def _shared_boundaries(
    poly_a: Polygon, poly_b: Polygon, tol: float = 1e-3
) -> List[BaseGeometry]:
    """Find coincident boundary segments, tolerating floating-point sector seams."""
    exact = poly_a.boundary.intersection(poly_b.boundary)
    parts = _line_shared_parts(exact, tol)
    if parts:
        return parts

    near = poly_a.boundary.buffer(tol, cap_style=2, join_style=2).intersection(
        poly_b.boundary
    )
    parts = _line_shared_parts(near, tol)
    if parts:
        return parts

    if not exact.is_empty and exact.geom_type == "Point":
        return [exact]
    return []


def _line_shared_parts(geometry: BaseGeometry, tol: float) -> List[BaseGeometry]:
    if geometry.is_empty:
        return []
    if geometry.geom_type == "LineString" and geometry.length > tol:
        return [geometry]
    if geometry.geom_type == "MultiLineString":
        return [geom for geom in geometry.geoms if geom.length > tol]
    if geometry.geom_type == "GeometryCollection":
        parts: List[BaseGeometry] = []
        for geom in geometry.geoms:
            parts.extend(_line_shared_parts(geom, tol))
        return parts
    return []


def endpoint_conflicts(
    paths: Sequence[Sequence[Waypoint]],
    min_separation: float,
) -> List[Tuple[int, str, int, int, float]]:
    """Detect drone endpoints closer than min_separation to another drone's waypoint."""
    conflicts: List[Tuple[int, str, int, int, float]] = []
    for i, path_i in enumerate(paths):
        if not path_i:
            continue
        endpoints: List[Tuple[str, Waypoint]] = [("start", path_i[0])]
        if len(path_i) > 1:
            endpoints.append(("end", path_i[-1]))

        for label, endpoint in endpoints:
            for j, path_j in enumerate(paths):
                if i == j or not path_j:
                    continue
                for k, waypoint in enumerate(path_j):
                    dist = math.hypot(endpoint.x - waypoint.x, endpoint.y - waypoint.y)
                    if dist <= 1e-6:
                        continue
                    if dist < min_separation:
                        conflicts.append((i, label, j, k, dist))
    return conflicts
