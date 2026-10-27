from __future__ import annotations

from typing import List

from shapely.geometry import Polygon

from multi_cover.coverage import _count_strips, best_sweep_angle
from multi_cover.geometry import (
    circle_diameter_chord,
    min_bounding_extent,
    narrowest_chord_through_centroid,
    path_length,
    region_to_shapely,
    waypoints_from_coords,
)
from multi_cover.types import (
    CircleRegion,
    CoverParams,
    CoverResult,
    PolygonRegion,
    Region,
    Waypoint,
)


def is_region_too_small(region: Region, params: CoverParams, poly: Polygon | None = None) -> bool:
    if poly is None:
        poly = region_to_shapely(region)
    if min_bounding_extent(poly) < params.swath_width:
        return True

    sweep_angle = best_sweep_angle(poly, params.swath_width, params.sweep_overlap)
    step = params.swath_width * (1.0 - params.sweep_overlap)
    strip_count = _count_strips(poly, sweep_angle, step)
    return strip_count == 0


def degraded_single_pass(region: Region, params: CoverParams) -> CoverResult:
    poly = region_to_shapely(region)
    if isinstance(region, CircleRegion):
        chord = circle_diameter_chord(region.center, region.radius)
        full_region = _circle_region_to_polygon(region)
    else:
        chord = narrowest_chord_through_centroid(poly)
        full_region = region

    waypoints = waypoints_from_coords(chord)
    chord_len = path_length(waypoints)

    paths: List[List[Waypoint]] = [waypoints] + [[] for _ in range(params.num_drones - 1)]
    path_lengths = [chord_len] + [0.0] * (params.num_drones - 1)
    sub_regions = [full_region] + [PolygonRegion(vertices=[]) for _ in range(params.num_drones - 1)]

    return CoverResult(
        paths=paths,
        path_lengths=path_lengths,
        sub_regions=sub_regions,
        max_path_length=chord_len,
        balance_ratio=1.0,
        status="degraded_single_pass",
        active_drones=1,
    )


def _circle_region_to_polygon(region: CircleRegion) -> PolygonRegion:
    poly = region_to_shapely(region)
    coords = list(poly.exterior.coords[:-1])
    return PolygonRegion(vertices=[(float(x), float(y)) for x, y in coords])
