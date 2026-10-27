from __future__ import annotations

from dataclasses import dataclass
from typing import List, Literal, Tuple, Union

Point = Tuple[float, float]
CoverStatus = Literal["normal", "degraded_single_pass"]


@dataclass
class PolygonRegion:
    vertices: List[Point]


@dataclass
class CircleRegion:
    center: Point
    radius: float


Region = Union[PolygonRegion, CircleRegion]


@dataclass
class CoverParams:
    num_drones: int
    swath_width: float
    sweep_overlap: float = 0.0
    path_clearance: float | None = None  # None=auto per-side inset; 0=off; >0=meters


@dataclass
class Waypoint:
    x: float
    y: float


@dataclass
class CoverResult:
    paths: List[List[Waypoint]]
    path_lengths: List[float]
    sub_regions: List[PolygonRegion]
    max_path_length: float
    balance_ratio: float
    status: CoverStatus
    active_drones: int
