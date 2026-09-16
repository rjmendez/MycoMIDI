from __future__ import annotations

"""
Heighway dragon-curve point generator.

Unlike the Hilbert curve's uniform grid-filling look, the dragon curve produces
an irregular, asymmetric, self-similar jagged silhouette. That makes it useful
when a future integrator wants a fractal family with a more organic outline
instead of a near-even rectangular fill.
"""

from typing import Iterable


Point = tuple[float, float]
RawPoint = tuple[int, int]

_AXIOM = "FX"
_RULES = {
    "X": "X+YF+",
    "Y": "-FX-Y",
}


def _expand_dragon_lsystem(order: int) -> str:
    sequence = _AXIOM
    for _ in range(order):
        sequence = "".join(_RULES.get(symbol, symbol) for symbol in sequence)
    return sequence


def _raw_dragon_points(order: int) -> list[RawPoint]:
    if order < 1:
        raise ValueError("order must be at least 1")

    sequence = _expand_dragon_lsystem(order)
    x = 0
    y = 0
    heading = 0
    directions = (
        (1, 0),
        (0, 1),
        (-1, 0),
        (0, -1),
    )
    points: list[RawPoint] = [(x, y)]

    for symbol in sequence:
        if symbol == "F":
            dx, dy = directions[heading]
            x += dx
            y += dy
            points.append((x, y))
        elif symbol == "+":
            heading = (heading + 1) % 4
        elif symbol == "-":
            heading = (heading - 1) % 4

    return points


def _normalize_points(points: Iterable[RawPoint]) -> list[Point]:
    raw_points = list(points)
    if len(raw_points) < 2:
        raise ValueError("curve requires at least two points")

    xs = [x for x, _ in raw_points]
    ys = [y for _, y in raw_points]
    min_x = min(xs)
    max_x = max(xs)
    min_y = min(ys)
    max_y = max(ys)
    width = max_x - min_x
    height = max_y - min_y
    if width <= 0 or height <= 0:
        raise ValueError("curve bounds must span both axes for normalization")

    return [
        ((x - min_x) / width, (y - min_y) / height)
        for x, y in raw_points
    ]


def dragon_curve_points(order: int) -> list[Point]:
    """Return normalized Heighway dragon points for the given iteration order."""

    return _normalize_points(_raw_dragon_points(order))


def _consecutive_points_are_adjacent(points: Iterable[RawPoint]) -> bool:
    raw_points = list(points)
    return all(
        abs(x2 - x1) + abs(y2 - y1) == 1
        for (x1, y1), (x2, y2) in zip(raw_points[:-1], raw_points[1:], strict=True)
    )


def _run_self_test() -> None:
    for order in (8, 10):
        raw_points = _raw_dragon_points(order)
        normalized_points = dragon_curve_points(order)
        adjacent = _consecutive_points_are_adjacent(raw_points)
        min_x = min(x for x, _ in normalized_points)
        max_x = max(x for x, _ in normalized_points)
        min_y = min(y for _, y in normalized_points)
        max_y = max(y for _, y in normalized_points)

        assert adjacent, f"order {order} has a discontinuity"
        assert len(raw_points) == (1 << order) + 1, f"unexpected point count for order {order}"
        assert min_x == 0.0 and max_x == 1.0, f"order {order} x-range failed normalization"
        assert min_y == 0.0 and max_y == 1.0, f"order {order} y-range failed normalization"

        print(
            f"order={order} points={len(normalized_points)} "
            f"adjacent_steps={adjacent} normalized_bbox=[({min_x:.1f},{min_y:.1f}),({max_x:.1f},{max_y:.1f})]"
        )


if __name__ == "__main__":
    _run_self_test()
