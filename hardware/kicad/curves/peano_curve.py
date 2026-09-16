from __future__ import annotations

"""Peano-curve point generator for KiCad fractal routing experiments.

Compared with the repo's Hilbert generator, the classic Peano curve subdivides
space into a 3x3 lattice at each recursion step, so the resulting route looks
more woven and serpentine, with denser local turns than the squarer 2x2
Hilbert family.
"""

from dataclasses import dataclass


@dataclass
class _Turtle:
    x: int = 0
    y: int = 0
    direction: int = 0
    points: list[tuple[int, int]] | None = None

    def __post_init__(self) -> None:
        if self.points is None:
            self.points = [(self.x, self.y)]

    def forward(self) -> None:
        dx, dy = ((1, 0), (0, 1), (-1, 0), (0, -1))[self.direction]
        self.x += dx
        self.y += dy
        self.points.append((self.x, self.y))

    def turn_left(self) -> None:
        self.direction = (self.direction + 1) % 4

    def turn_right(self) -> None:
        self.direction = (self.direction - 1) % 4


def _draw_left_variant(order: int, turtle: _Turtle) -> None:
    if order == 0:
        return
    next_order = order - 1
    _draw_left_variant(next_order, turtle)
    turtle.forward()
    _draw_right_variant(next_order, turtle)
    turtle.forward()
    _draw_left_variant(next_order, turtle)
    turtle.turn_right()
    turtle.forward()
    turtle.turn_right()
    _draw_right_variant(next_order, turtle)
    turtle.forward()
    _draw_left_variant(next_order, turtle)
    turtle.forward()
    _draw_right_variant(next_order, turtle)
    turtle.turn_left()
    turtle.forward()
    turtle.turn_left()
    _draw_left_variant(next_order, turtle)
    turtle.forward()
    _draw_right_variant(next_order, turtle)
    turtle.forward()
    _draw_left_variant(next_order, turtle)


def _draw_right_variant(order: int, turtle: _Turtle) -> None:
    if order == 0:
        return
    next_order = order - 1
    _draw_right_variant(next_order, turtle)
    turtle.forward()
    _draw_left_variant(next_order, turtle)
    turtle.forward()
    _draw_right_variant(next_order, turtle)
    turtle.turn_left()
    turtle.forward()
    turtle.turn_left()
    _draw_left_variant(next_order, turtle)
    turtle.forward()
    _draw_right_variant(next_order, turtle)
    turtle.forward()
    _draw_left_variant(next_order, turtle)
    turtle.turn_right()
    turtle.forward()
    turtle.turn_right()
    _draw_right_variant(next_order, turtle)
    turtle.forward()
    _draw_left_variant(next_order, turtle)
    turtle.forward()
    _draw_right_variant(next_order, turtle)


def peano_curve_points(order: int) -> list[tuple[float, float]]:
    """Return classic Peano-curve points on a 0..(3**order-1) integer grid."""

    if order < 1:
        raise ValueError("order must be at least 1")

    turtle = _Turtle()
    _draw_left_variant(order, turtle)
    return [(float(x), float(-y)) for x, y in turtle.points]


def _validate_curve(order: int) -> tuple[int, bool]:
    points = peano_curve_points(order)
    expected_count = 9**order
    if len(points) != expected_count:
        raise AssertionError(f"order {order}: expected {expected_count} points, got {len(points)}")

    for (x1, y1), (x2, y2) in zip(points[:-1], points[1:], strict=True):
        if abs(x2 - x1) + abs(y2 - y1) != 1:
            raise AssertionError(
                f"order {order}: non-adjacent step from {(x1, y1)} to {(x2, y2)}"
            )
    return len(points), True


if __name__ == "__main__":
    for order in (1, 2):
        point_count, adjacent = _validate_curve(order)
        print(f"order={order} points={point_count} adjacent_steps={adjacent}")
