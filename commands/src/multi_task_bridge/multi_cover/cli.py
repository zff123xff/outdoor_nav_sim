"""CLI for visualizing multi-UAV coverage plans."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import List

from multi_cover.planner import plan_coverage
from multi_cover.types import CircleRegion, CoverParams, PolygonRegion
from multi_cover.viz import save_coverage, show_coverage


def _parse_point(text: str) -> tuple[float, float]:
    parts = text.split(",")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError(f"expected x,y got {text!r}")
    return float(parts[0]), float(parts[1])


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Visualize multi-UAV coverage plans")
    region = parser.add_mutually_exclusive_group(required=True)
    region.add_argument(
        "--circle",
        nargs=3,
        type=float,
        metavar=("CX", "CY", "R"),
        help="circle: center x, center y, radius (m)",
    )
    region.add_argument(
        "--polygon",
        nargs="+",
        type=_parse_point,
        metavar="X,Y",
        help="convex polygon vertices, e.g. 0,0 100,0 100,50 0,50",
    )
    region.add_argument(
        "--json",
        type=Path,
        metavar="FILE",
        help='input JSON file with {"region": ..., "params": ...}',
    )

    parser.add_argument("-n", "--drones", type=int, default=3, help="number of drones")
    parser.add_argument("-w", "--swath", type=float, default=20.0, help="swath width (m)")
    parser.add_argument("--overlap", type=float, default=0.0, help="sweep overlap ratio")
    parser.add_argument("-o", "--output", type=Path, help="save PNG/SVG instead of opening a window")
    parser.add_argument("--dpi", type=int, default=150, help="output image DPI")
    parser.add_argument("--no-subregions", action="store_true", help="hide sub-region fills")
    parser.add_argument("--no-waypoints", action="store_true", help="hide waypoint markers")
    return parser


def _load_json(path: Path):
    data = json.loads(path.read_text(encoding="utf-8"))
    region_data = data["region"]
    params_data = data["params"]
    region_type = region_data["type"]
    if region_type == "circle":
        region = CircleRegion(
            center=tuple(region_data["center"]),
            radius=float(region_data["radius"]),
        )
    elif region_type == "polygon":
        region = PolygonRegion(vertices=[tuple(v) for v in region_data["vertices"]])
    else:
        raise ValueError(f"unknown region type: {region_type}")
    params = CoverParams(
        num_drones=int(params_data["num_drones"]),
        swath_width=float(params_data["swath_width"]),
        sweep_overlap=float(params_data.get("sweep_overlap", 0.0)),
    )
    return region, params


def main(argv: List[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    if args.json is not None:
        region, params = _load_json(args.json)
    elif args.circle is not None:
        cx, cy, radius = args.circle
        region = CircleRegion(center=(cx, cy), radius=radius)
        params = CoverParams(num_drones=args.drones, swath_width=args.swath, sweep_overlap=args.overlap)
    else:
        region = PolygonRegion(vertices=list(args.polygon))
        params = CoverParams(num_drones=args.drones, swath_width=args.swath, sweep_overlap=args.overlap)

    result = plan_coverage(region, params)
    plot_kwargs = {
        "show_subregions": not args.no_subregions,
        "show_waypoints": not args.no_waypoints,
    }

    print(
        f"status={result.status} active={result.active_drones} "
        f"balance={result.balance_ratio:.3f} lengths={[round(v, 1) for v in result.path_lengths]}"
    )

    if args.output is not None:
        save_coverage(str(args.output), region, result, params=params, dpi=args.dpi, **plot_kwargs)
        print(f"saved: {args.output}")
    else:
        show_coverage(region, result, params=params, **plot_kwargs)
    return 0
