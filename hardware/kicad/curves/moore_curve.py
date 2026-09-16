from __future__ import annotations

"""Moore-curve point generation for KiCad geometry helpers.

The Moore curve is the loop/closed variant of the Hilbert curve: it fills the
same square grid, but wraps the path back to its starting side instead of
leaving two separated endpoints. That makes it a better fit for ring-like or
tileable layouts where a future consumer wants a closed traversal.
"""

from typing import Iterable

_AXIOM = "LFL+F+LFL"
_RULES = {
    "L": "-RF+LFL+FR-",
    "R": "+LF-RFR-FL+",
}
_DIRECTIONS: tuple[tuple[int, int], ...] = (
    (1, 0),
    (0, 1),
    (-1, 0),
    (0, -1),
)


def _expand_moore_commands(order: int) -> str:
    commands = _AXIOM
    for _ in range(order - 1):
        commands = "".join(_RULES.get(symbol, symbol) for symbol in commands)
    return commands


def _trace_commands(commands: Iterable[str]) -> list[tuple[int, int]]:
    direction_index = 0
    x = 0
    y = 0
    points = [(x, y)]

    for symbol in commands:
        if symbol == "F":
            dx, dy = _DIRECTIONS[direction_index]
            x += dx
            y += dy
            points.append((x, y))
        elif symbol == "-":
            direction_index = (direction_index + 1) % len(_DIRECTIONS)
        elif symbol == "+":
            direction_index = (direction_index - 1) % len(_DIRECTIONS)

    return points


def moore_curve_points(order: int) -> list[tuple[float, float]]:
    """Return Moore-curve points on the same 0..(2**order-1) lattice as Hilbert."""
    if order < 1:
        raise ValueError("order must be at least 1")

    points = _trace_commands(_expand_moore_commands(order))
    min_x = min(x for x, _ in points)
    min_y = min(y for _, y in points)
    translated = [(x - min_x, y - min_y) for x, y in points]

    start = translated[0]
    end = translated[-1]
    if abs(end[0] - start[0]) + abs(end[1] - start[1]) != 1:
        raise ValueError("generated Moore curve did not end adjacent to its start")

    closed = translated + [start]
    return [(float(x), float(y)) for x, y in closed]


def _has_adjacent_steps(points: list[tuple[float, float]]) -> bool:
    for (ax, ay), (bx, by) in zip(points[:-1], points[1:], strict=True):
        step = abs(ax - bx) + abs(ay - by)
        if step != 1:
            return False
    return True


def _is_closed(points: list[tuple[float, float]]) -> bool:
    if not points:
        return False
    ax, ay = points[0]
    bx, by = points[-1]
    return abs(ax - bx) < 1e-9 and abs(ay - by) < 1e-9


if __name__ == "__main__":
    for order in (2, 3):
        points = moore_curve_points(order)
        xs = [x for x, _ in points]
        ys = [y for _, y in points]
        print(
            f"order={order} point_count={len(points)} "
            f"bounds=({min(xs):.0f},{min(ys):.0f})-({max(xs):.0f},{max(ys):.0f}) "
            f"closed={_is_closed(points)} adjacent_steps={_has_adjacent_steps(points)}"
        )
