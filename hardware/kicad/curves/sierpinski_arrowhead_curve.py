from __future__ import annotations

import math


Point = tuple[float, float]


def _expand(order: int) -> str:
    state = "A"
    for _ in range(order):
        state = "".join("B-A-B" if symbol == "A" else "A+B+A" if symbol == "B" else symbol for symbol in state)
    return state


def _trace(order: int) -> list[Point]:
    program = _expand(order)
    x = 0.0
    y = 0.0
    heading_deg = 0.0
    points: list[Point] = [(x, y)]
    for symbol in program:
        if symbol in {"A", "B"}:
            x += math.cos(math.radians(heading_deg))
            y += math.sin(math.radians(heading_deg))
            points.append((round(x, 6), round(y, 6)))
        elif symbol == "+":
            heading_deg += 60.0
        elif symbol == "-":
            heading_deg -= 60.0
    return points


def _normalize(points: list[Point]) -> list[Point]:
    xs = [x for x, _ in points]
    ys = [y for _, y in points]
    min_x = min(xs)
    max_x = max(xs)
    min_y = min(ys)
    max_y = max(ys)
    span_x = max_x - min_x
    span_y = max_y - min_y
    if span_x <= 0 or span_y <= 0:
        raise ValueError("curve bounding box must have non-zero area")
    return [((x - min_x) / span_x, (y - min_y) / span_y) for x, y in points]


def sierpinski_arrowhead_curve_points(order: int) -> list[Point]:
    if order < 1:
        raise ValueError("order must be at least 1")
    return _normalize(_trace(order))


def _self_test(order: int) -> None:
    points = sierpinski_arrowhead_curve_points(order)
    expected = (3**order) + 1
    if len(points) != expected:
        raise AssertionError(f"order {order}: expected {expected} points, got {len(points)}")
    for (sx, sy), (ex, ey) in zip(points[:-1], points[1:], strict=True):
        if math.isclose(sx, ex, abs_tol=1e-12) and math.isclose(sy, ey, abs_tol=1e-12):
            raise AssertionError(f"order {order}: duplicate consecutive points")
    print(f"order={order} points={len(points)} adjacent_steps=True")


if __name__ == "__main__":
    for sample_order in (3, 5):
        _self_test(sample_order)
