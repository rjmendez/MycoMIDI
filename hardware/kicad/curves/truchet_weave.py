from __future__ import annotations

import math
import random


Point = tuple[float, float]


def _arc(center_x: float, center_y: float, radius: float, start_angle: float, end_angle: float, segments: int) -> list[Point]:
    return [
        (
            center_x + (radius * math.cos(start_angle + ((end_angle - start_angle) * index / segments))),
            center_y + (radius * math.sin(start_angle + ((end_angle - start_angle) * index / segments))),
        )
        for index in range(segments + 1)
    ]


def _normalize_arc(points: list[Point], columns: int, rows: int) -> list[Point]:
    return [(x / columns, y / rows) for x, y in points]


def truchet_weave_points(columns: int, rows: int, *, seed: int = 0, arc_segments: int = 10) -> list[list[Point]]:
    """Return quarter-circle Truchet arcs for a randomized weave grid."""

    if columns < 1 or rows < 1:
        raise ValueError("columns and rows must both be positive")
    if arc_segments < 2:
        raise ValueError("arc_segments must be at least 2")

    rng = random.Random((columns * 7_919) + (rows * 104_729) + seed)
    arcs: list[list[Point]] = []

    for row in range(rows):
        for column in range(columns):
            x0 = float(column)
            y0 = float(row)
            orientation = rng.randint(0, 1)
            if orientation == 0:
                raw_arcs = (
                    _arc(x0 + 1.0, y0 + 1.0, 0.5, math.pi, 1.5 * math.pi, arc_segments),
                    _arc(x0, y0, 0.5, 0.0, 0.5 * math.pi, arc_segments),
                )
            else:
                raw_arcs = (
                    _arc(x0, y0 + 1.0, 0.5, -0.5 * math.pi, 0.0, arc_segments),
                    _arc(x0 + 1.0, y0, 0.5, math.pi, 0.5 * math.pi, arc_segments),
                )
            arcs.extend(_normalize_arc(points, columns, rows) for points in raw_arcs)
    return arcs


def _raw_endpoint(point: Point, columns: int, rows: int) -> tuple[float, float]:
    return (point[0] * columns, point[1] * rows)


def _endpoint_key(point: Point, columns: int, rows: int) -> tuple[int, int]:
    raw_x, raw_y = _raw_endpoint(point, columns, rows)
    return (round(raw_x * 1_000_000), round(raw_y * 1_000_000))


def _self_test(columns: int = 8, rows: int = 6, seed: int = 11) -> None:
    arcs = truchet_weave_points(columns, rows, seed=seed)
    expected_arc_count = columns * rows * 2
    if len(arcs) != expected_arc_count:
        raise AssertionError(f"expected {expected_arc_count} arcs, got {len(arcs)}")

    endpoint_counts: dict[tuple[int, int], int] = {}
    for arc in arcs:
        for endpoint in (arc[0], arc[-1]):
            if any(not math.isfinite(value) for value in endpoint):
                raise AssertionError("non-finite endpoint detected")
            endpoint_counts[_endpoint_key(endpoint, columns, rows)] = endpoint_counts.get(
                _endpoint_key(endpoint, columns, rows), 0
            ) + 1

    for key, count in endpoint_counts.items():
        raw_x = key[0] / 1_000_000
        raw_y = key[1] / 1_000_000
        on_boundary = math.isclose(raw_x, 0.0, abs_tol=1e-9) or math.isclose(raw_x, float(columns), abs_tol=1e-9) or math.isclose(raw_y, 0.0, abs_tol=1e-9) or math.isclose(raw_y, float(rows), abs_tol=1e-9)
        expected_count = 1 if on_boundary else 2
        if count != expected_count:
            raise AssertionError(
                f"endpoint {(raw_x, raw_y)} expected count {expected_count}, got {count}; weave is disconnected"
            )

    print(f"columns={columns} rows={rows} arcs={len(arcs)} endpoints_aligned=True")


if __name__ == "__main__":
    _self_test()
