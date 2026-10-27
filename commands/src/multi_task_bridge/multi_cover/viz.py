from __future__ import annotations

from typing import TYPE_CHECKING, List, Optional, Sequence, Tuple

if TYPE_CHECKING:
    from matplotlib.axes import Axes
    from matplotlib.figure import Figure

from shapely.geometry import Polygon as ShapelyPolygon

from multi_cover.geometry import region_to_shapely
from multi_cover.types import CoverParams, CoverResult, PolygonRegion, Region, Waypoint

DroneColor = Tuple[float, float, float, float]

DEFAULT_COLORS: List[DroneColor] = [
    (0.12, 0.47, 0.71, 1.0),
    (1.0, 0.50, 0.05, 1.0),
    (0.17, 0.63, 0.17, 1.0),
    (0.84, 0.15, 0.16, 1.0),
    (0.58, 0.40, 0.74, 1.0),
    (0.55, 0.34, 0.29, 1.0),
    (0.89, 0.47, 0.76, 1.0),
    (0.50, 0.50, 0.50, 1.0),
]


def _mpl():
    try:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon as MplPolygon
    except (ImportError, AttributeError) as exc:
        raise ImportError(
            "matplotlib is required for visualization. Install with: pip install 'multi-cover[viz]'"
        ) from exc
    return plt, MplPolygon


def _color(index: int) -> tuple[DroneColor, DroneColor]:
    base = DEFAULT_COLORS[index % len(DEFAULT_COLORS)]
    fill = (base[0], base[1], base[2], 0.18)
    return fill, base


def _polygon_patch(vertices, facecolor, edgecolor):
    _, MplPolygon = _mpl()
    if not vertices:
        return None
    return MplPolygon(
        vertices,
        closed=True,
        facecolor=facecolor,
        edgecolor=edgecolor,
        linewidth=1.5,
        linestyle="--",
    )


def _path_coords(path: Sequence[Waypoint]) -> Tuple[List[float], List[float]]:
    xs = [wp.x for wp in path]
    ys = [wp.y for wp in path]
    return xs, ys


def plot_coverage(
    region: Region,
    result: CoverResult,
    params: Optional[CoverParams] = None,
    ax: Optional["Axes"] = None,
    *,
    show_subregions: bool = True,
    show_waypoints: bool = True,
    show_legend: bool = True,
    title: Optional[str] = None,
) -> "Figure":
    """Draw task region, sub-regions, and drone paths on a matplotlib figure."""
    plt, _ = _mpl()
    if ax is None:
        fig, ax = plt.subplots(figsize=(10, 8))
    else:
        fig = ax.figure

    task_poly = region_to_shapely(region)
    task_coords = list(task_poly.exterior.coords)
    ax.plot(
        [c[0] for c in task_coords],
        [c[1] for c in task_coords],
        color="black",
        linewidth=2.0,
        label="task region",
        zorder=1,
    )

    for index, path in enumerate(result.paths):
        fill_color, line_color = _color(index)
        sub = result.sub_regions[index] if index < len(result.sub_regions) else PolygonRegion(vertices=[])

        if show_subregions and sub.vertices:
            patch = _polygon_patch(sub.vertices, fill_color, line_color)
            if patch is not None:
                ax.add_patch(patch)

        if not path:
            continue

        xs, ys = _path_coords(path)
        length = result.path_lengths[index] if index < len(result.path_lengths) else 0.0
        ax.plot(
            xs,
            ys,
            color=line_color[:3],
            linewidth=2.0,
            marker="o" if show_waypoints else None,
            markersize=4,
            label=f"drone {index} ({length:.1f} m)",
            zorder=3,
        )

        for seg_index in range(len(path) - 1):
            x0, y0 = path[seg_index].x, path[seg_index].y
            x1, y1 = path[seg_index + 1].x, path[seg_index + 1].y
            dx, dy = x1 - x0, y1 - y0
            if abs(dx) + abs(dy) < 1e-9:
                continue
            ax.annotate(
                "",
                xy=(x1, y1),
                xytext=(x0, y0),
                arrowprops=dict(arrowstyle="->", color=line_color[:3], lw=1.2),
                zorder=4,
            )

    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle=":", alpha=0.5)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")

    if title is None:
        title = _default_title(result, params)
    ax.set_title(title)

    if show_legend:
        ax.legend(loc="upper right", fontsize=9)

    _autoscale(ax, task_poly)
    fig.tight_layout()
    return fig


def _default_title(result: CoverResult, params: Optional[CoverParams]) -> str:
    parts = [
        f"status={result.status}",
        f"active={result.active_drones}",
        f"balance={result.balance_ratio:.2f}",
        f"max_len={result.max_path_length:.1f}m",
    ]
    if params is not None:
        parts.insert(0, f"N={params.num_drones} W={params.swath_width:.1f}m")
    return " | ".join(parts)


def _autoscale(ax: "Axes", task_poly: ShapelyPolygon, padding_ratio: float = 0.08) -> None:
    minx, miny, maxx, maxy = task_poly.bounds
    pad_x = (maxx - minx) * padding_ratio or 1.0
    pad_y = (maxy - miny) * padding_ratio or 1.0
    ax.set_xlim(minx - pad_x, maxx + pad_x)
    ax.set_ylim(miny - pad_y, maxy + pad_y)


def show_coverage(
    region: Region,
    result: CoverResult,
    params: Optional[CoverParams] = None,
    **kwargs,
) -> None:
    """Open an interactive window with the coverage plot."""
    plt, _ = _mpl()
    plot_coverage(region, result, params=params, **kwargs)
    plt.show()


def save_coverage(
    path: str,
    region: Region,
    result: CoverResult,
    params: Optional[CoverParams] = None,
    dpi: int = 150,
    **kwargs,
) -> str:
    """Save the coverage plot to an image file. Returns the output path."""
    plt, _ = _mpl()
    fig = plot_coverage(region, result, params=params, **kwargs)
    fig.savefig(path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    return path
