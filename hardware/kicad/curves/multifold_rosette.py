from __future__ import annotations

import math


Point = tuple[float, float]


def _normalize_points(points: list[Point]) -> list[Point]:
    xs = [x for x, _ in points]
    ys = [y for _, y in points]
    min_x = min(xs)
    max_x = max(xs)
    min_y = min(ys)
    max_y = max(ys)
    span_x = max_x - min_x
    span_y = max_y - min_y
    if span_x <= 0.0 or span_y <= 0.0:
        raise ValueError("rosette bounding box must have non-zero area")
    return [((x - min_x) / span_x, (y - min_y) / span_y) for x, y in points]


def _rose_period(k_numerator: int, k_denominator: int) -> float:
    reduced_gcd = math.gcd(k_numerator, k_denominator)
    p = k_numerator // reduced_gcd
    q = k_denominator // reduced_gcd
    if p % 2 == 1 and q % 2 == 1:
        return math.pi * q
    return 2.0 * math.pi * q


def _rose_curve_points(k_numerator: int, k_denominator: int, num_points: int) -> list[Point]:
    if k_denominator < 1:
        raise ValueError("k_denominator must be positive")
    if num_points < 32:
        raise ValueError("num_points must be at least 32")

    k = k_numerator / k_denominator
    period = _rose_period(k_numerator, k_denominator)
    raw_points: list[Point] = []
    for index in range(num_points):
        theta = period * index / num_points
        radius = math.cos(k * theta)
        raw_points.append((radius * math.cos(theta), radius * math.sin(theta)))
    raw_points.append(raw_points[0])
    return _normalize_points(raw_points)


def _star_polygon_points(vertices: int, step: int) -> list[Point]:
    if vertices < 5:
        raise ValueError("vertices must be at least 5")
    if step <= 1 or step >= vertices:
        raise ValueError("step must be between 2 and vertices - 1")
    if math.gcd(vertices, step) != 1:
        raise ValueError("vertices and step must be coprime for a single unicursal star polygon")

    circle_points = [
        (0.5 + (0.5 * math.cos((2.0 * math.pi * index) / vertices)), 0.5 + (0.5 * math.sin((2.0 * math.pi * index) / vertices)))
        for index in range(vertices)
    ]
    path = [circle_points[(index * step) % vertices] for index in range(vertices)]
    path.append(path[0])
    return path


def multifold_rosette_points(
    mode: str = "rose",
    k_numerator: int = 7,
    k_denominator: int = 1,
    num_points: int = 280,
    star_vertices: int = 7,
    star_step: int = 2,
) -> list[Point]:
    """Return a normalized multifold rosette in rose-curve or star-polygon mode."""

    if mode == "rose":
        return _rose_curve_points(k_numerator, k_denominator, num_points)
    if mode == "star":
        return _star_polygon_points(star_vertices, star_step)
    raise ValueError("mode must be 'rose' or 'star'")


def _is_closed(points: list[Point], tolerance: float = 1e-9) -> bool:
    return math.isclose(points[0][0], points[-1][0], abs_tol=tolerance) and math.isclose(
        points[0][1], points[-1][1], abs_tol=tolerance
    )


def _self_test() -> None:
    rose = multifold_rosette_points()
    star = multifold_rosette_points(mode="star", star_vertices=9, star_step=4)

    if len(rose) != 281:
        raise AssertionError(f"unexpected rose point count: {len(rose)}")
    if len(star) != 10:
        raise AssertionError(f"unexpected star point count: {len(star)}")
    if not _is_closed(rose) or not _is_closed(star):
        raise AssertionError("rosette output must be closed")
    if any(not math.isfinite(value) for point in rose + star for value in point):
        raise AssertionError("rosette output contains non-finite coordinates")

    print(f"rose_points={len(rose)} star_points={len(star)} closed=True")


if __name__ == "__main__":
    _self_test()
