from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from hardware.kicad.curves.multifold_rosette import multifold_rosette_points
from hardware.kicad.curves.phyllotaxis_spiral import phyllotaxis_spiral_points
from hardware.kicad.curves.superformula_motif import superformula_motif_points
from hardware.kicad.curves.truchet_weave import truchet_weave_points
from hardware.kicad.curves.venation_branch import venation_branch_points


OUTPUT_PATH = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("demo_new_patterns.png")


def _plot_phyllotaxis(axis: plt.Axes) -> None:
    for arm in phyllotaxis_spiral_points(
        point_count=89,
        primary_step=5,
        secondary_step=8,
        tertiary_step=13,
        primary_offset=0,
        secondary_offset=2,
        tertiary_offset=5,
    ):
        xs = [x for x, _ in arm]
        ys = [y for _, y in arm]
        axis.plot(xs, ys, linewidth=1.4, marker="o", markersize=2.5)
    axis.set_title("Phyllotaxis parastichy")


def _plot_rosette(axis: plt.Axes) -> None:
    points = multifold_rosette_points(k_numerator=7, k_denominator=1, num_points=320)
    axis.plot([x for x, _ in points], [y for _, y in points], linewidth=1.4)
    axis.set_title("Multifold rosette")


def _plot_truchet(axis: plt.Axes) -> None:
    for arc in truchet_weave_points(8, 6, seed=21, arc_segments=10):
        axis.plot([x for x, _ in arc], [y for _, y in arc], linewidth=1.0, color="#1f77b4")
    axis.set_title("Truchet weave")


def _plot_venation(axis: plt.Axes) -> None:
    for start, end in venation_branch_points(
        seed=13,
        attraction_count=180,
        growth_step=0.035,
        attraction_radius=0.16,
        kill_radius=0.04,
        max_iterations=550,
    ):
        axis.plot([start[0], end[0]], [start[1], end[1]], linewidth=1.1, color="#1f77b4")
    axis.set_title("Venation branching")


def _plot_superformula(axis: plt.Axes) -> None:
    points = superformula_motif_points(a=1.0, b=1.0, m=5.0, n1=0.3, n2=1.7, n3=1.7, num_points=420)
    axis.plot([x for x, _ in points], [y for _, y in points], linewidth=1.4)
    axis.set_title("Superformula motif")


def main() -> None:
    figure, axes = plt.subplots(2, 3, figsize=(14, 9))
    used_axes = [axes[0, 0], axes[0, 1], axes[0, 2], axes[1, 0], axes[1, 1]]
    plotters = (_plot_phyllotaxis, _plot_rosette, _plot_truchet, _plot_venation, _plot_superformula)

    for axis, plotter in zip(used_axes, plotters, strict=True):
        plotter(axis)
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlim(-0.02, 1.02)
        axis.set_ylim(-0.02, 1.02)
        axis.set_xticks([])
        axis.set_yticks([])

    axes[1, 2].axis("off")
    figure.suptitle("New KiCad curve generators", fontsize=16)
    figure.tight_layout()
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(OUTPUT_PATH, dpi=200, bbox_inches="tight")
    print(f"saved {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
