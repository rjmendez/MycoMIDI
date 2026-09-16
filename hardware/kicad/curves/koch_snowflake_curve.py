from __future__ import annotations

"""Koch snowflake boundary generator for KiCad copper-pour shaping.

This module produces a closed polygon boundary for a solid filled region
such as a GND copper pour outline or an internal cutout. Unlike the repo's
open, wandering decorative/fractal path generators that suit trace-like
routes, the Koch snowflake here is meant to be consumed as a single simple
polygon boundary.
"""

import math


Point = tuple[float, float]
_SQRT_3_OVER_2 = math.sqrt(3.0) / 2.0
_TURN_RADIANS = math.pi / 3.0
_BASE_TRIANGLE: tuple[Point, Point, Point] = (
    (0.0, 0.0),
    (1.0, 0.0),
    (0.5, _SQRT_3_OVER_2),
)
_BASE_HEXAGON: tuple[Point, Point, Point, Point, Point, Point] = tuple(
    (math.cos(index * _TURN_RADIANS), math.sin(index * _TURN_RADIANS)) for index in range(6)
)


def _rotate(vector: Point, radians: float) -> Point:
    x, y = vector
    cosine = math.cos(radians)
    sine = math.sin(radians)
    return (
        (x * cosine) - (y * sine),
        (x * sine) + (y * cosine),
    )


def _koch_edge_points(start: Point, end: Point, order: int, turn_sign: int) -> list[Point]:
    if order == 0:
        return [start, end]

    dx = end[0] - start[0]
    dy = end[1] - start[1]
    one_third = (dx / 3.0, dy / 3.0)

    first = (start[0] + one_third[0], start[1] + one_third[1])
    third = (start[0] + (2.0 * one_third[0]), start[1] + (2.0 * one_third[1]))
    peak_offset = _rotate(one_third, turn_sign * _TURN_RADIANS)
    second = (first[0] + peak_offset[0], first[1] + peak_offset[1])

    first_leg = _koch_edge_points(start, first, order - 1, turn_sign)
    second_leg = _koch_edge_points(first, second, order - 1, turn_sign)
    third_leg = _koch_edge_points(second, third, order - 1, turn_sign)
    fourth_leg = _koch_edge_points(third, end, order - 1, turn_sign)
    return first_leg + second_leg[1:] + third_leg[1:] + fourth_leg[1:]


def _snap_unit_value(value: float) -> float:
    if math.isclose(value, 0.0, abs_tol=1e-12):
        return 0.0
    if math.isclose(value, 1.0, abs_tol=1e-12):
        return 1.0
    return value


def _normalize_points(points: list[Point]) -> list[Point]:
    if len(points) < 4:
        raise ValueError("closed polygon requires at least three edges")

    xs = [x for x, _ in points]
    ys = [y for _, y in points]
    min_x = min(xs)
    max_x = max(xs)
    min_y = min(ys)
    max_y = max(ys)
    width = max_x - min_x
    height = max_y - min_y
    if width <= 0.0 or height <= 0.0:
        raise ValueError("polygon bounds must span both axes for normalization")

    return [
        (
            _snap_unit_value((x - min_x) / width),
            _snap_unit_value((y - min_y) / height),
        )
        for x, y in points
    ]


def _base_polygon(anti: bool) -> tuple[Point, ...]:
    if anti:
        return _BASE_HEXAGON
    return _BASE_TRIANGLE


def koch_snowflake_points(order: int, anti: bool = False) -> list[Point]:
    """Return a normalized closed Koch snowflake polygon boundary.

    The returned polygon is explicitly closed: the last point repeats the
    first point. `anti=True` switches to an inward-dented anti-snowflake style
    built from a hexagonal seed, which keeps the boundary simple/non-self-
    intersecting while still producing a concave pour/cutout outline.
    """

    if order < 0:
        raise ValueError("order must be non-negative")

    turn_sign = 1 if anti else -1
    vertices = _base_polygon(anti)
    raw_points: list[Point] = []
    for index, start in enumerate(vertices):
        end = vertices[(index + 1) % len(vertices)]
        segment_points = _koch_edge_points(start, end, order, turn_sign)
        raw_points.extend(segment_points if not raw_points else segment_points[1:])
    return _normalize_points(raw_points)


def _points_are_close(first: Point, second: Point, tolerance: float = 1e-9) -> bool:
    return math.isclose(first[0], second[0], abs_tol=tolerance) and math.isclose(
        first[1], second[1], abs_tol=tolerance
    )


def _is_closed(points: list[Point]) -> bool:
    return bool(points) and _points_are_close(points[0], points[-1])


def _expected_point_count(order: int, anti: bool = False) -> int:
    edge_count = len(_base_polygon(anti))
    return (edge_count * (4**order)) + 1


def _segment_orientation(a: Point, b: Point, c: Point) -> float:
    return ((b[0] - a[0]) * (c[1] - a[1])) - ((b[1] - a[1]) * (c[0] - a[0]))


def _point_on_segment(start: Point, point: Point, end: Point, tolerance: float = 1e-9) -> bool:
    return (
        min(start[0], end[0]) - tolerance <= point[0] <= max(start[0], end[0]) + tolerance
        and min(start[1], end[1]) - tolerance <= point[1] <= max(start[1], end[1]) + tolerance
        and math.isclose(_segment_orientation(start, end, point), 0.0, abs_tol=tolerance)
    )


def _segments_intersect(first_start: Point, first_end: Point, second_start: Point, second_end: Point) -> bool:
    tolerance = 1e-9
    o1 = _segment_orientation(first_start, first_end, second_start)
    o2 = _segment_orientation(first_start, first_end, second_end)
    o3 = _segment_orientation(second_start, second_end, first_start)
    o4 = _segment_orientation(second_start, second_end, first_end)

    if ((o1 > tolerance and o2 < -tolerance) or (o1 < -tolerance and o2 > tolerance)) and (
        (o3 > tolerance and o4 < -tolerance) or (o3 < -tolerance and o4 > tolerance)
    ):
        return True

    return (
        (math.isclose(o1, 0.0, abs_tol=tolerance) and _point_on_segment(first_start, second_start, first_end))
        or (math.isclose(o2, 0.0, abs_tol=tolerance) and _point_on_segment(first_start, second_end, first_end))
        or (math.isclose(o3, 0.0, abs_tol=tolerance) and _point_on_segment(second_start, first_start, second_end))
        or (math.isclose(o4, 0.0, abs_tol=tolerance) and _point_on_segment(second_start, first_end, second_end))
    )


def _has_non_adjacent_self_intersection(points: list[Point]) -> bool:
    segment_count = len(points) - 1
    if segment_count < 3:
        return False

    for first_index in range(segment_count):
        first_segment = (points[first_index], points[first_index + 1])
        for second_index in range(first_index + 1, segment_count):
            if second_index == first_index + 1:
                continue
            if first_index == 0 and second_index == segment_count - 1:
                continue

            second_segment = (points[second_index], points[second_index + 1])
            if _segments_intersect(*first_segment, *second_segment):
                return True
    return False


def _self_test(order: int, anti: bool = False) -> None:
    variant = "anti-snowflake" if anti else "snowflake"
    points = koch_snowflake_points(order, anti=anti)
    expected_points = _expected_point_count(order, anti=anti)
    if len(points) != expected_points:
        raise AssertionError(f"{variant} order {order}: expected {expected_points} points, got {len(points)}")
    if not _is_closed(points):
        raise AssertionError(f"{variant} order {order}: polygon is not closed")
    if any(not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0) for x, y in points):
        raise AssertionError(f"{variant} order {order}: point fell outside normalized bounds")

    intersects = _has_non_adjacent_self_intersection(points)
    if intersects:
        raise AssertionError(f"{variant} order {order}: non-adjacent segments intersect")

    print(
        f"variant={variant} order={order} points={len(points)} "
        f"closed={_is_closed(points)} self_intersections={intersects}"
    )


if __name__ == "__main__":
    for sample_order in (2, 4):
        _self_test(sample_order)
    _self_test(2, anti=True)
