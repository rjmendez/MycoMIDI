from __future__ import annotations

import math

_AXIOM = "XF"
_RULES = {
    "X": "YF+XF+Y",
    "Y": "XF-YF-X",
}
_TURN_DEGREES = 60.0
_SQRT_3_OVER_2 = math.sqrt(3.0) / 2.0
_DIRECTION_VECTORS: tuple[tuple[float, float], ...] = (
    (1.0, 0.0),
    (0.5, _SQRT_3_OVER_2),
    (-0.5, _SQRT_3_OVER_2),
    (-1.0, 0.0),
    (-0.5, -_SQRT_3_OVER_2),
    (0.5, -_SQRT_3_OVER_2),
)


def _expand_l_system(order: int) -> str:
    state = _AXIOM
    for _ in range(order):
        state = "".join(_RULES.get(symbol, symbol) for symbol in state)
    return state


def _trace_curve(commands: str) -> list[tuple[float, float]]:
    direction = 0
    x = 0.0
    y = 0.0
    points = [(x, y)]

    for symbol in commands:
        if symbol == "F":
            dx, dy = _DIRECTION_VECTORS[direction]
            x += dx
            y += dy
            points.append((x, y))
        elif symbol == "+":
            direction = (direction + 1) % len(_DIRECTION_VECTORS)
        elif symbol == "-":
            direction = (direction - 1) % len(_DIRECTION_VECTORS)

    return points


def _snap_unit_value(value: float) -> float:
    if math.isclose(value, 0.0, abs_tol=1e-12):
        return 0.0
    if math.isclose(value, 1.0, abs_tol=1e-12):
        return 1.0
    return value


def _normalize_points(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    xs = [x for x, _ in points]
    ys = [y for _, y in points]
    min_x = min(xs)
    max_x = max(xs)
    min_y = min(ys)
    max_y = max(ys)
    width = max_x - min_x
    height = max_y - min_y

    if width <= 0.0 or height <= 0.0:
        raise ValueError("curve bounding box must have non-zero width and height")

    return [
        (
            _snap_unit_value((x - min_x) / width),
            _snap_unit_value((y - min_y) / height),
        )
        for x, y in points
    ]


def sierpinski_arrowhead_curve_points(order: int) -> list[tuple[float, float]]:
    """Return normalized Sierpinski arrowhead points for KiCad curve mapping.

    Unlike the Hilbert curve's square-grid right-angle walk, the Sierpinski
    arrowhead follows a triangular 60-degree fractal lattice, giving a more
    organic zigzag structure for regions that suit a less rectilinear fill.
    """

    if order < 1:
        raise ValueError("order must be at least 1")

    commands = _expand_l_system(order)
    return _normalize_points(_trace_curve(commands))


def _step_lengths(points: list[tuple[float, float]]) -> list[float]:
    return [
        math.hypot(end_x - start_x, end_y - start_y)
        for (start_x, start_y), (end_x, end_y) in zip(points[:-1], points[1:], strict=True)
    ]


def _self_test(order: int) -> None:
    points = sierpinski_arrowhead_curve_points(order)
    expected_points = (3**order) + 1
    if len(points) != expected_points:
        raise AssertionError(f"order {order}: expected {expected_points} points, got {len(points)}")

    steps = _step_lengths(points)
    if not steps or min(steps) <= 0.0:
        raise AssertionError(f"order {order}: non-positive step length detected")

    max_step = max(steps)
    min_step = min(steps)
    if max_step > (min_step * 2.0):
        raise AssertionError(
            f"order {order}: step discontinuity too large ({min_step:.6f}..{max_step:.6f})"
        )

    if any(not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0) for x, y in points):
        raise AssertionError(f"order {order}: normalized point outside unit square")

    print(
        f"order={order} points={len(points)} turns={_TURN_DEGREES:g}deg "
        f"step_range=[{min_step:.6f}, {max_step:.6f}]"
    )


if __name__ == "__main__":
    for sample_order in (4, 5):
        _self_test(sample_order)
