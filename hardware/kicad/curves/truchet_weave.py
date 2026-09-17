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


def _quadratic_curve(start: Point, control: Point, end: Point, segments: int) -> list[Point]:
    points: list[Point] = []
    for index in range(segments + 1):
        t = index / segments
        one_minus_t = 1.0 - t
        x = (one_minus_t * one_minus_t * start[0]) + (2.0 * one_minus_t * t * control[0]) + (t * t * end[0])
        y = (one_minus_t * one_minus_t * start[1]) + (2.0 * one_minus_t * t * control[1]) + (t * t * end[1])
        points.append((x, y))
    return points


def _normalize_arc(points: list[Point], columns: int, rows: int) -> list[Point]:
    return [(x / columns, y / rows) for x, y in points]


def _point_key(point: Point) -> tuple[int, int]:
    return (round(point[0] * 1_000_000), round(point[1] * 1_000_000))


def _orientation_grid(columns: int, rows: int, *, seed: int) -> list[list[int]]:
    rng = random.Random((columns * 7_919) + (rows * 104_729) + seed)
    grid = [[rng.randint(0, 1) for _ in range(columns)] for _ in range(rows)]

    for _ in range(4):
        changed = False
        for row in range(rows - 1):
            for column in range(columns - 1):
                block = (
                    grid[row][column],
                    grid[row][column + 1],
                    grid[row + 1][column],
                    grid[row + 1][column + 1],
                )
                if block in ((0, 1, 1, 0), (1, 0, 0, 1)):
                    grid[row + 1][column + 1] = 1 - grid[row + 1][column + 1]
                    changed = True
        if not changed:
            break

    return grid


def _ordered_arc(points: list[Point], start_key: tuple[int, int]) -> list[Point]:
    if _point_key(points[0]) == start_key:
        return points
    return list(reversed(points))


def _append_points(target: list[Point], points: list[Point]) -> None:
    if not target:
        target.extend(points)
        return
    target.extend(points[1:] if _point_key(target[-1]) == _point_key(points[0]) else points)


def _trace_paths(arcs: list[list[Point]], *, small_loop_arc_count: int = 8) -> list[list[Point]]:
    endpoint_map: dict[tuple[int, int], list[tuple[int, bool]]] = {}
    for index, arc in enumerate(arcs):
        endpoint_map.setdefault(_point_key(arc[0]), []).append((index, True))
        endpoint_map.setdefault(_point_key(arc[-1]), []).append((index, False))

    unused = set(range(len(arcs)))
    paths: list[list[Point]] = []

    def walk(start_arc: int, start_key: tuple[int, int]) -> tuple[list[Point], int, bool]:
        path = _ordered_arc(arcs[start_arc], start_key)
        unused.remove(start_arc)
        arc_count = 1
        origin_key = start_key
        current_key = _point_key(path[-1])

        while True:
            candidates = [item for item in endpoint_map[current_key] if item[0] in unused]
            if not candidates:
                return path, arc_count, current_key == origin_key
            next_arc, next_is_start = candidates[0]
            ordered = arcs[next_arc] if next_is_start else list(reversed(arcs[next_arc]))
            _append_points(path, ordered)
            unused.remove(next_arc)
            arc_count += 1
            current_key = _point_key(path[-1])
            if current_key == origin_key:
                return path, arc_count, True

    for key, connected in endpoint_map.items():
        if len(connected) != 1:
            continue
        arc_index, is_start = connected[0]
        if arc_index not in unused:
            continue
        start_key = key if is_start else _point_key(arcs[arc_index][-1])
        path, _arc_count, _closed = walk(arc_index, start_key)
        paths.append(path)

    while unused:
        arc_index = next(iter(unused))
        start_key = _point_key(arcs[arc_index][0])
        path, arc_count, closed = walk(arc_index, start_key)
        if closed and arc_count <= small_loop_arc_count:
            continue
        paths.append(path)

    return paths


def truchet_weave_points(columns: int, rows: int, *, seed: int = 0, arc_segments: int = 10) -> list[list[Point]]:
    """Return continuous Truchet weave paths for a randomized weave grid."""

    if columns < 1 or rows < 1:
        raise ValueError("columns and rows must both be positive")
    if arc_segments < 2:
        raise ValueError("arc_segments must be at least 2")

    arcs: list[list[Point]] = []
    orientations = _orientation_grid(columns, rows, seed=seed)
    corner_inset = 0.26

    for row in range(rows):
        for column in range(columns):
            x0 = float(column)
            y0 = float(row)
            orientation = orientations[row][column]
            if orientation == 0:
                raw_arcs = (
                    _quadratic_curve(
                        (x0 + 0.5, y0),
                        (x0 + corner_inset, y0 + corner_inset),
                        (x0, y0 + 0.5),
                        arc_segments,
                    ),
                    _quadratic_curve(
                        (x0 + 0.5, y0 + 1.0),
                        (x0 + 1.0 - corner_inset, y0 + 1.0 - corner_inset),
                        (x0 + 1.0, y0 + 0.5),
                        arc_segments,
                    ),
                )
            else:
                raw_arcs = (
                    _quadratic_curve(
                        (x0, y0 + 0.5),
                        (x0 + corner_inset, y0 + 1.0 - corner_inset),
                        (x0 + 0.5, y0 + 1.0),
                        arc_segments,
                    ),
                    _quadratic_curve(
                        (x0 + 0.5, y0),
                        (x0 + 1.0 - corner_inset, y0 + corner_inset),
                        (x0 + 1.0, y0 + 0.5),
                        arc_segments,
                    ),
                )
            arcs.extend(_normalize_arc(points, columns, rows) for points in raw_arcs)
    return _trace_paths(arcs)


def _raw_endpoint(point: Point, columns: int, rows: int) -> tuple[float, float]:
    return (point[0] * columns, point[1] * rows)


def _endpoint_key(point: Point, columns: int, rows: int) -> tuple[int, int]:
    raw_x, raw_y = _raw_endpoint(point, columns, rows)
    return (round(raw_x * 1_000_000), round(raw_y * 1_000_000))


def _cross(ax: float, ay: float, bx: float, by: float, cx: float, cy: float) -> float:
    return ((bx - ax) * (cy - ay)) - ((by - ay) * (cx - ax))


def _point_on_segment(point: Point, start: Point, end: Point, *, eps: float = 1e-9) -> bool:
    px, py = point
    ax, ay = start
    bx, by = end
    if abs(_cross(ax, ay, bx, by, px, py)) > eps:
        return False
    return (
        min(ax, bx) - eps <= px <= max(ax, bx) + eps
        and min(ay, by) - eps <= py <= max(ay, by) + eps
    )


def _segment_intersection_kind(a0: Point, a1: Point, b0: Point, b1: Point, *, eps: float = 1e-9) -> str | None:
    shared = {_point_key(a0), _point_key(a1)} & {_point_key(b0), _point_key(b1)}
    d1 = _cross(*a0, *a1, *b0)
    d2 = _cross(*a0, *a1, *b1)
    d3 = _cross(*b0, *b1, *a0)
    d4 = _cross(*b0, *b1, *a1)

    if (
        ((d1 > eps and d2 < -eps) or (d1 < -eps and d2 > eps))
        and ((d3 > eps and d4 < -eps) or (d3 < -eps and d4 > eps))
    ):
        return "cross"

    for point in (b0, b1):
        if _point_on_segment(point, a0, a1, eps=eps):
            return None if _point_key(point) in shared else "touch"
    for point in (a0, a1):
        if _point_on_segment(point, b0, b1, eps=eps):
            return None if _point_key(point) in shared else "touch"
    return None


def _path_has_crossings(paths: list[list[Point]]) -> bool:
    segments: list[tuple[int, int, Point, Point]] = []
    for path_index, path in enumerate(paths):
        for segment_index in range(len(path) - 1):
            segments.append((path_index, segment_index, path[segment_index], path[segment_index + 1]))

    for index, (path_a, seg_a, a0, a1) in enumerate(segments):
        for path_b, seg_b, b0, b1 in segments[index + 1 :]:
            if path_a == path_b and abs(seg_a - seg_b) <= 1:
                continue
            kind = _segment_intersection_kind(a0, a1, b0, b1)
            if kind is not None:
                return True
    return False


def _self_test(columns: int = 8, rows: int = 6, seed: int = 11) -> None:
    arcs = truchet_weave_points(columns, rows, seed=seed)
    if not arcs:
        raise AssertionError("expected at least one traced Truchet path")

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
        expected_count = 1 if on_boundary else 0
        if count != expected_count:
            raise AssertionError(
                f"endpoint {(raw_x, raw_y)} expected count {expected_count}, got {count}; weave endpoints leaked into interior"
            )

    orientations = _orientation_grid(columns, rows, seed=seed)
    for row in range(rows - 1):
        for column in range(columns - 1):
            block = (
                orientations[row][column],
                orientations[row][column + 1],
                orientations[row + 1][column],
                orientations[row + 1][column + 1],
            )
            if block in ((0, 1, 1, 0), (1, 0, 0, 1)):
                raise AssertionError(f"checkerboard block survived at row={row} column={column}")

    if _path_has_crossings(arcs):
        raise AssertionError("centerline crossings detected in traced Truchet weave")

    print(f"columns={columns} rows={rows} paths={len(arcs)} endpoints_aligned=True no_checkerboards=True no_crossings=True")


if __name__ == "__main__":
    _self_test()
