from __future__ import annotations

from typing import List

from shapely.geometry import Polygon

from multi_cover.balancer import balance_circle_boundaries, balance_slices
from multi_cover.coverage import best_sweep_angle, boustrophedon_sweep
from multi_cover.flight_corridor import make_flight_polygon, resolve_path_clearance
from multi_cover.geometry import (
    assert_convex_polygon,
    path_length,
    region_to_shapely,
    shapely_to_polygon_region,
    validate_params,
)
from multi_cover.partitioner import (
    area_partition,
    bundle_strips,
    circle_sector_angles,
    circle_sectors,
    decompose_strips,
    polygons_to_regions,
    sweep_sector,
)
from multi_cover.small_region import degraded_single_pass, is_region_too_small
from multi_cover.types import (
    CircleRegion,
    CoverParams,
    CoverResult,
    PolygonRegion,
    Region,
    Waypoint,
)
from multi_cover.validator import compute_balance_ratio


def plan_coverage(region: Region, params: CoverParams) -> CoverResult:
    validate_params(params.num_drones, params.swath_width, params.sweep_overlap)

    if isinstance(region, PolygonRegion):
        assert_convex_polygon(region)

    poly = region_to_shapely(region)

    if is_region_too_small(region, params, poly):
        return degraded_single_pass(region, params)

    if params.num_drones == 1:
        return _plan_single(poly, region, params)

    if isinstance(region, CircleRegion):
        return _plan_circle(region, params)

    return _plan_polygon(region, poly, params)


def _path_clearance(params: CoverParams) -> float:
    return resolve_path_clearance(params.path_clearance, params.swath_width)


def _flight_region(
    assigned: Polygon,
    params: CoverParams,
    all_assigned: List[Polygon],
) -> Polygon:
    if params.num_drones <= 1:
        return assigned
    return make_flight_polygon(assigned, _path_clearance(params), all_assigned)


def _sweep_assigned(
    assigned: Polygon,
    sweep_angle: float,
    params: CoverParams,
    all_assigned: List[Polygon],
) -> List[Waypoint]:
    if assigned.is_empty:
        return []
    flight = _flight_region(assigned, params, all_assigned)
    return boustrophedon_sweep(
        flight, sweep_angle, params.swath_width, params.sweep_overlap
    )


def _plan_single(poly: Polygon, region: Region, params: CoverParams) -> CoverResult:
    sweep_angle = best_sweep_angle(poly, params.swath_width, params.sweep_overlap)
    waypoints = boustrophedon_sweep(poly, sweep_angle, params.swath_width, params.sweep_overlap)
    if isinstance(region, CircleRegion):
        sub_region = shapely_to_polygon_region(poly)
    else:
        sub_region = region
    return _build_result(
        paths=[waypoints],
        sub_regions=[sub_region],
        num_drones=params.num_drones,
        status="normal",
    )


def _plan_circle(region: CircleRegion, params: CoverParams) -> CoverResult:
    boundaries = circle_sector_angles(params.num_drones)
    clearance = _path_clearance(params)

    def sectors_for(angles: List[float]) -> List[Polygon]:
        return circle_sectors(region.center, region.radius, angles)

    def lengths_for_angles(angles: List[float]) -> List[float]:
        sectors = sectors_for(angles)
        lengths: List[float] = []
        for index, sector_poly in enumerate(sectors):
            flight = make_flight_polygon(sector_poly, clearance, sectors)
            path, _ = sweep_sector(
                flight,
                angles[index],
                angles[index + 1],
                params.swath_width,
                params.sweep_overlap,
            )
            lengths.append(path_length(path))
        return lengths

    balanced_angles, _ = balance_circle_boundaries(boundaries, lengths_for_angles)

    sectors = sectors_for(balanced_angles)
    paths: List[List[Waypoint]] = []
    for index, sector_poly in enumerate(sectors):
        flight = make_flight_polygon(sector_poly, clearance, sectors)
        path, _ = sweep_sector(
            flight,
            balanced_angles[index],
            balanced_angles[index + 1],
            params.swath_width,
            params.sweep_overlap,
        )
        paths.append(path)

    sub_regions = polygons_to_regions(sectors)
    return _build_result(paths, sub_regions, params.num_drones, "normal")


def _plan_polygon(region: PolygonRegion, poly: Polygon, params: CoverParams) -> CoverResult:
    sweep_angle = best_sweep_angle(poly, params.swath_width, params.sweep_overlap)
    step = params.swath_width * (1.0 - params.sweep_overlap)
    strips = decompose_strips(poly, sweep_angle, step)

    if len(strips) < params.num_drones:
        parts = area_partition(poly, params.num_drones, sweep_angle)
        paths = [
            _sweep_assigned(part, sweep_angle, params, parts) for part in parts
        ]
        sub_regions = polygons_to_regions(parts)
        return _build_result(paths, sub_regions, params.num_drones, "normal")

    def length_for_slice(group: slice) -> float:
        bundled = bundle_strips(strips, [group])[0]
        if bundled.is_empty:
            return 0.0
        return path_length(
            boustrophedon_sweep(
                bundled, sweep_angle, params.swath_width, params.sweep_overlap
            )
        )

    slices, _ = balance_slices(len(strips), params.num_drones, length_for_slice)
    bundled_polys = [bundle_strips(strips, [group])[0] for group in slices]
    paths = [
        _sweep_assigned(part, sweep_angle, params, bundled_polys)
        for part in bundled_polys
    ]
    sub_regions = polygons_to_regions(bundled_polys)
    return _build_result(paths, sub_regions, params.num_drones, "normal")


def _build_result(
    paths: List[List[Waypoint]],
    sub_regions: List[PolygonRegion],
    num_drones: int,
    status: str,
) -> CoverResult:
    while len(paths) < num_drones:
        paths.append([])
    while len(sub_regions) < num_drones:
        sub_regions.append(PolygonRegion(vertices=[]))

    path_lengths = [path_length(path) for path in paths]
    active = sum(1 for path in paths if path)
    positive_lengths = [value for value in path_lengths if value > 0]
    max_len = max(positive_lengths) if positive_lengths else 0.0

    return CoverResult(
        paths=paths[:num_drones],
        path_lengths=path_lengths[:num_drones],
        sub_regions=sub_regions[:num_drones],
        max_path_length=max_len,
        balance_ratio=compute_balance_ratio(path_lengths),
        status=status,  # type: ignore[arg-type]
        active_drones=active,
    )
