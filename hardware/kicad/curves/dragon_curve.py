from __future__ import annotations

import math


Point = tuple[float, float]


def _dragon_turns(order: int) -> list[int]:
    if order < 1:
        raise ValueError("order must be at least 1")
    turns = [1]
    for _ in range(1, order):
        turns = turns + [1] + [-turn for turn in reversed(turns)]
    return turns


def _trace_points(order: int) -> list[Point]:
    heading = 0
    x = 0
    y = 0
    points: list[Point] = [(0.0, 0.0)]
    directions = ((1, 0), (0, 1), (-1, 0), (0, -1))

    for turn in _dragon_turns(order):
        dx, dy = directions[heading]
        x += dx
        y += dy
        points.append((float(x), float(y)))
        heading = (heading + turn) % 4

    dx, dy = directions[heading]
    points.append((float(x + dx), float(y + dy)))
    return points


def _normalize_points(points: list[Point]) -> list[Point]:
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


def _orientation(a: Point, b: Point, c: Point) -> float:
    return ((b[0] - a[0]) * (c[1] - a[1])) - ((b[1] - a[1]) * (c[0] - a[0]))


def _point_on_segment(start: Point, point: Point, end: Point, tolerance: float = 1e-9) -> bool:
    return (
        min(start[0], end[0]) - tolerance <= point[0] <= max(start[0], end[0]) + tolerance
        and min(start[1], end[1]) - tolerance <= point[1] <= max(start[1], end[1]) + tolerance
        and math.isclose(_orientation(start, end, point), 0.0, abs_tol=tolerance)
    )


def _segments_intersect(first_start: Point, first_end: Point, second_start: Point, second_end: Point) -> bool:
    tolerance = 1e-9
    o1 = _orientation(first_start, first_end, second_start)
    o2 = _orientation(first_start, first_end, second_end)
    o3 = _orientation(second_start, second_end, first_start)
    o4 = _orientation(second_start, second_end, first_end)

    if ((o1 > tolerance and o2 < -tolerance) or (o1 < -tolerance and o2 > tolerance)) and (
        (o3 > tolerance and o4 < -tolerance) or (o3 < -tolerance and o4 > tolerance)
    ):
        return True

    collinear_or_touching = (
        (math.isclose(o1, 0.0, abs_tol=tolerance) and _point_on_segment(first_start, second_start, first_end))
        or (math.isclose(o2, 0.0, abs_tol=tolerance) and _point_on_segment(first_start, second_end, first_end))
        or (math.isclose(o3, 0.0, abs_tol=tolerance) and _point_on_segment(second_start, first_start, second_end))
        or (math.isclose(o4, 0.0, abs_tol=tolerance) and _point_on_segment(second_start, first_end, second_end))
    )
    if not collinear_or_touching:
        return False

    shared_points = {first_start, first_end}.intersection({second_start, second_end})
    return not shared_points


def _has_non_adjacent_self_intersection(points: list[Point]) -> bool:
    segment_count = len(points) - 1
    for first_index in range(segment_count):
        first_segment = (points[first_index], points[first_index + 1])
        for second_index in range(first_index + 2, segment_count):
            if second_index == first_index + 1:
                continue
            second_segment = (points[second_index], points[second_index + 1])
            if second_index == first_index + 1:
                continue
            if _segments_intersect(*first_segment, *second_segment):
                return True
    return False


def dragon_curve_points(order: int) -> list[Point]:
    return _normalize_points(_trace_points(order))


def _self_test(order: int) -> None:
    points = dragon_curve_points(order)
    if len(points) != (2**order) + 1:
        raise AssertionError(f"order {order}: unexpected point count {len(points)}")
    for (sx, sy), (ex, ey) in zip(points[:-1], points[1:], strict=True):
        if math.isclose(sx, ex, abs_tol=1e-12) and math.isclose(sy, ey, abs_tol=1e-12):
            raise AssertionError(f"order {order}: duplicate consecutive point {(sx, sy)}")
    if _has_non_adjacent_self_intersection(points):
        raise AssertionError(f"order {order}: dragon curve self-intersected")
    print(f"order={order} points={len(points)} self_intersections=False")


if __name__ == "__main__":
    for sample_order in (6, 8):
        _self_test(sample_order)
