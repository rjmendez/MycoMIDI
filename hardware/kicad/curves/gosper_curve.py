from __future__ import annotations

import math


Point = tuple[float, float]
_TURN_RADIANS = math.radians(60.0)
_STEP_DIRECTIONS = tuple(
    (math.cos(index * _TURN_RADIANS), math.sin(index * _TURN_RADIANS)) for index in range(6)
)


def _expand_gosper_lsystem(order: int) -> str:
    sequence = "A"
    for _ in range(order):
        sequence = "".join(
            "A-B--B+A++AA+B-" if symbol == "A" else "+A-BB--B-A++A+B" if symbol == "B" else symbol
            for symbol in sequence
        )
    return sequence


def _trace_gosper_points(order: int) -> list[Point]:
    sequence = _expand_gosper_lsystem(order)
    x = 0.0
    y = 0.0
    heading_index = 0
    points: list[Point] = [(x, y)]

    for symbol in sequence:
        if symbol in {"A", "B"}:
            dx, dy = _STEP_DIRECTIONS[heading_index % 6]
            x += dx
            y += dy
            points.append((x, y))
        elif symbol == "+":
            heading_index = (heading_index + 1) % 6
        elif symbol == "-":
            heading_index = (heading_index - 1) % 6

    return points


def _normalize_points(points: list[Point]) -> list[Point]:
    if len(points) < 2:
        raise ValueError("curve requires multiple points")

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


def gosper_curve_points(order: int) -> list[Point]:
    if order < 1:
        raise ValueError("order must be at least 1")
    return _normalize_points(_trace_gosper_points(order))


def _max_expected_normalized_step(raw_points: list[Point]) -> float:
    xs = [x for x, _ in raw_points]
    ys = [y for _, y in raw_points]
    span_x = max(xs) - min(xs)
    span_y = max(ys) - min(ys)
    if span_x <= 0 or span_y <= 0:
        raise ValueError("curve bounding box must have non-zero area")

    return max(
        math.hypot(dx / span_x, dy / span_y)
        for dx, dy in (
            (1.0, 0.0),
            (0.5, math.sqrt(3.0) / 2.0),
            (-0.5, math.sqrt(3.0) / 2.0),
            (-1.0, 0.0),
            (-0.5, -math.sqrt(3.0) / 2.0),
            (0.5, -math.sqrt(3.0) / 2.0),
        )
    )


def _self_test_order(order: int) -> None:
    raw_points = _trace_gosper_points(order)
    normalized_points = gosper_curve_points(order)

    raw_step_lengths = [
        math.hypot(ex - sx, ey - sy)
        for (sx, sy), (ex, ey) in zip(raw_points[:-1], raw_points[1:], strict=True)
    ]
    normalized_step_lengths = [
        math.hypot(ex - sx, ey - sy)
        for (sx, sy), (ex, ey) in zip(normalized_points[:-1], normalized_points[1:], strict=True)
    ]

    if not all(math.isclose(step, 1.0, rel_tol=1e-9, abs_tol=1e-9) for step in raw_step_lengths):
        raise AssertionError(f"order {order}: raw turtle path contains discontinuities")

    expected_step_ceiling = _max_expected_normalized_step(raw_points) * 1.000001
    max_normalized_step = max(normalized_step_lengths)
    if max_normalized_step > expected_step_ceiling:
        raise AssertionError(
            f"order {order}: normalized path step {max_normalized_step:.9f} exceeds expected "
            f"{expected_step_ceiling:.9f}"
        )

    print(
        f"order {order}: {len(normalized_points)} points, "
        f"raw max step={max(raw_step_lengths):.6f}, "
        f"normalized max step={max_normalized_step:.6f}, ok"
    )


if __name__ == "__main__":
    for test_order in (2, 3):
        _self_test_order(test_order)
