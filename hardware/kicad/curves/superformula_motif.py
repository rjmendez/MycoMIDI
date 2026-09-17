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
        raise ValueError("superformula bounds must span both axes")
    return [((x - min_x) / span_x, (y - min_y) / span_y) for x, y in points]


def _radius(phi: float, a: float, b: float, m: float, n1: float, n2: float, n3: float) -> float:
    cosine_term = abs(math.cos((m * phi) / 4.0) / a) ** n2
    sine_term = abs(math.sin((m * phi) / 4.0) / b) ** n3
    denominator = cosine_term + sine_term
    if denominator <= 0.0 or not math.isfinite(denominator):
        raise ValueError("superformula denominator must stay finite and positive")
    radius = denominator ** (-1.0 / n1)
    if radius < 0.0 or not math.isfinite(radius):
        raise ValueError("superformula radius must stay finite and non-negative")
    return radius


def superformula_motif_points(
    a: float = 1.0,
    b: float = 1.0,
    m: float = 5.0,
    n1: float = 0.3,
    n2: float = 1.7,
    n3: float = 1.7,
    num_points: int = 360,
) -> list[Point]:
    """Return a normalized closed Gielis superformula silhouette."""

    if a == 0.0 or b == 0.0:
        raise ValueError("a and b must be non-zero")
    if n1 == 0.0:
        raise ValueError("n1 must be non-zero")
    if num_points < 64:
        raise ValueError("num_points must be at least 64")

    raw_points: list[Point] = []
    for index in range(num_points):
        phi = (2.0 * math.pi * index) / num_points
        radius = _radius(phi, a, b, m, n1, n2, n3)
        raw_points.append((radius * math.cos(phi), radius * math.sin(phi)))
    raw_points.append(raw_points[0])
    return _normalize_points(raw_points)


def _self_test() -> None:
    points = superformula_motif_points()
    if len(points) != 361:
        raise AssertionError(f"unexpected point count {len(points)}")
    if points[0] != points[-1]:
        raise AssertionError("superformula motif must be explicitly closed")
    if any(not math.isfinite(value) for point in points for value in point):
        raise AssertionError("superformula output contains non-finite coordinates")
    if any(not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0) for x, y in points):
        raise AssertionError("superformula output escaped normalized bounds")

    for sample_index in range(0, 360, 12):
        _radius((2.0 * math.pi * sample_index) / 360.0, 1.0, 1.0, 5.0, 0.3, 1.7, 1.7)

    print(f"points={len(points)} closed=True finite=True")


if __name__ == "__main__":
    _self_test()
