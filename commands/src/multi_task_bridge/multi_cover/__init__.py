from multi_cover.planner import plan_coverage
from multi_cover.types import (
    CircleRegion,
    CoverParams,
    CoverResult,
    PolygonRegion,
    Waypoint,
)

__all__ = [
    "CircleRegion",
    "CoverParams",
    "CoverResult",
    "PolygonRegion",
    "Waypoint",
    "plan_coverage",
    "plot_coverage",
    "save_coverage",
    "show_coverage",
]


def __getattr__(name: str):
    if name in {"plot_coverage", "save_coverage", "show_coverage"}:
        from multi_cover import viz

        return getattr(viz, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
