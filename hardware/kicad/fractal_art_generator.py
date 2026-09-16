from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np


DEFAULT_OUTPUT = Path(__file__).with_name("koch_snowflake_demo.svg")


def equilateral_triangle(side_length: float) -> np.ndarray:
    height = np.sqrt(3.0) * side_length / 2.0
    return np.array(
        [
            [-side_length / 2.0, -height / 3.0],
            [side_length / 2.0, -height / 3.0],
            [0.0, 2.0 * height / 3.0],
            [-side_length / 2.0, -height / 3.0],
        ],
        dtype=float,
    )


def koch_segment(start: np.ndarray, end: np.ndarray) -> np.ndarray:
    vector = end - start
    first_third = start + vector / 3.0
    second_third = start + 2.0 * vector / 3.0
    rotation = np.array(
        [
            [0.5, np.sqrt(3.0) / 2.0],
            [-np.sqrt(3.0) / 2.0, 0.5],
        ],
        dtype=float,
    )
    peak = first_third + rotation @ (vector / 3.0)
    return np.vstack((start, first_third, peak, second_third))


def generate_koch_snowflake(depth: int, side_length: float) -> np.ndarray:
    if depth < 0:
        raise ValueError("depth must be non-negative")
    if side_length <= 0:
        raise ValueError("side_length must be positive")

    points = equilateral_triangle(side_length)
    for _ in range(depth):
        segments = [
            koch_segment(start, end)
            for start, end in zip(points[:-1], points[1:], strict=True)
        ]
        points = np.vstack((*segments, points[:1]))
    return points


def save_fractal(points: np.ndarray, output_path: Path, *, stroke_width: float, dpi: int) -> None:
    if stroke_width <= 0:
        raise ValueError("stroke_width must be positive")
    if dpi <= 0:
        raise ValueError("dpi must be positive")

    min_x, min_y = points.min(axis=0)
    max_x, max_y = points.max(axis=0)
    extent = max(max_x - min_x, max_y - min_y)
    margin = max(extent * 0.08, stroke_width * 2.0)

    plt.rcParams["svg.fonttype"] = "none"
    figure, axis = plt.subplots(figsize=(4, 4))
    axis.plot(
        points[:, 0],
        points[:, 1],
        color="black",
        linewidth=stroke_width,
        solid_capstyle="round",
        solid_joinstyle="round",
    )
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlim(min_x - margin, max_x + margin)
    axis.set_ylim(min_y - margin, max_y + margin)
    axis.axis("off")
    axis.set_facecolor("none")
    figure.patch.set_alpha(0.0)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(
        output_path,
        dpi=dpi,
        transparent=True,
        bbox_inches="tight",
        pad_inches=0.0,
    )
    plt.close(figure)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate a Koch snowflake as SVG or another Matplotlib-supported format.",
    )
    parser.add_argument(
        "--depth",
        type=int,
        default=3,
        help="Recursion depth for the Koch snowflake.",
    )
    parser.add_argument(
        "--size",
        type=float,
        default=40.0,
        help="Side length of the seed triangle in plot units.",
    )
    parser.add_argument(
        "--stroke-width",
        type=float,
        default=1.5,
        help="Line width used when rendering the fractal.",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=1200,
        help="Raster DPI used when the output format is raster; harmless for SVG.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"Output file path. Defaults to {DEFAULT_OUTPUT.name}.",
    )
    return parser


def main() -> int:
    args = build_argument_parser().parse_args()
    points = generate_koch_snowflake(args.depth, args.size)
    save_fractal(
        points,
        args.output,
        stroke_width=args.stroke_width,
        dpi=args.dpi,
    )
    print(
        f"wrote {args.output} "
        f"(depth={args.depth}, points={len(points)}, format={args.output.suffix or 'auto'})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
