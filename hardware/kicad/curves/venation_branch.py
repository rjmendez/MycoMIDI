from __future__ import annotations

import math
import random


Point = tuple[float, float]
Segment = tuple[Point, Point]


def _distance_squared(first: Point, second: Point) -> float:
    return ((first[0] - second[0]) ** 2) + ((first[1] - second[1]) ** 2)


def _normalize(vector_x: float, vector_y: float) -> Point:
    length = math.hypot(vector_x, vector_y)
    if length <= 0.0:
        return (0.0, 0.0)
    return (vector_x / length, vector_y / length)


def _orientation(a: Point, b: Point, c: Point) -> float:
    return ((b[0] - a[0]) * (c[1] - a[1])) - ((b[1] - a[1]) * (c[0] - a[0]))


def _point_on_segment(start: Point, point: Point, end: Point, tolerance: float = 1e-9) -> bool:
    return (
        min(start[0], end[0]) - tolerance <= point[0] <= max(start[0], end[0]) + tolerance
        and min(start[1], end[1]) - tolerance <= point[1] <= max(start[1], end[1]) + tolerance
        and math.isclose(_orientation(start, end, point), 0.0, abs_tol=tolerance)
    )


def _segments_intersect(first: Segment, second: Segment, tolerance: float = 1e-9) -> bool:
    first_start, first_end = first
    second_start, second_end = second
    shared_points = {first_start, first_end}.intersection({second_start, second_end})
    if shared_points:
        return False

    o1 = _orientation(first_start, first_end, second_start)
    o2 = _orientation(first_start, first_end, second_end)
    o3 = _orientation(second_start, second_end, first_start)
    o4 = _orientation(second_start, second_end, first_end)

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


def venation_branch_points(
    min_x: float = 0.0,
    min_y: float = 0.0,
    max_x: float = 1.0,
    max_y: float = 1.0,
    root_x: float = 0.5,
    root_y: float = 0.05,
    second_root_x: float | None = None,
    second_root_y: float | None = None,
    attraction_count: int = 160,
    seed: int = 0,
    growth_step: float = 0.04,
    attraction_radius: float = 0.18,
    kill_radius: float = 0.05,
    max_iterations: int = 500,
) -> list[Segment]:
    """Return non-crossing branch segments grown by space colonization."""

    if max_x <= min_x or max_y <= min_y:
        raise ValueError("bounding box must have positive area")
    if attraction_count < 1:
        raise ValueError("attraction_count must be positive")
    if growth_step <= 0.0:
        raise ValueError("growth_step must be positive")
    if attraction_radius <= 0.0 or kill_radius <= 0.0:
        raise ValueError("radii must be positive")
    if kill_radius >= attraction_radius:
        raise ValueError("kill_radius must be smaller than attraction_radius")

    rng = random.Random(
        seed
        + round(min_x * 10_000)
        + round(min_y * 100_000)
        + round(max_x * 1_000_000)
        + round(max_y * 10_000_000)
    )
    branch_points: list[Point] = [(root_x, root_y)]
    if second_root_x is not None or second_root_y is not None:
        if second_root_x is None or second_root_y is None:
            raise ValueError("second_root_x and second_root_y must both be provided")
        branch_points.append((second_root_x, second_root_y))

    attraction_points = [
        (rng.uniform(min_x, max_x), rng.uniform(min_y, max_y)) for _ in range(attraction_count)
    ]
    segments: list[Segment] = []

    for _ in range(max_iterations):
        assignments: dict[int, tuple[float, float, int]] = {}
        survivors: list[Point] = []
        for attraction_point in attraction_points:
            nearest_index = -1
            nearest_distance_squared = float("inf")
            for index, branch_point in enumerate(branch_points):
                candidate_distance_squared = _distance_squared(attraction_point, branch_point)
                if candidate_distance_squared < nearest_distance_squared:
                    nearest_distance_squared = candidate_distance_squared
                    nearest_index = index
            if nearest_distance_squared <= kill_radius * kill_radius:
                continue
            if nearest_distance_squared > attraction_radius * attraction_radius:
                survivors.append(attraction_point)
                continue

            branch_point = branch_points[nearest_index]
            direction_x, direction_y = _normalize(
                attraction_point[0] - branch_point[0], attraction_point[1] - branch_point[1]
            )
            sum_x, sum_y, count = assignments.get(nearest_index, (0.0, 0.0, 0))
            assignments[nearest_index] = (sum_x + direction_x, sum_y + direction_y, count + 1)
            survivors.append(attraction_point)

        attraction_points = survivors
        if not attraction_points or not assignments:
            break

        new_points: list[Point] = []
        existing_points = set(branch_points)
        for branch_index, (sum_x, sum_y, _count) in assignments.items():
            direction_x, direction_y = _normalize(sum_x, sum_y)
            if math.isclose(direction_x, 0.0, abs_tol=1e-12) and math.isclose(direction_y, 0.0, abs_tol=1e-12):
                continue

            start = branch_points[branch_index]
            candidate = (start[0] + (direction_x * growth_step), start[1] + (direction_y * growth_step))
            if not (min_x <= candidate[0] <= max_x and min_y <= candidate[1] <= max_y):
                continue
            if any(_distance_squared(candidate, point) < (growth_step * 0.35) ** 2 for point in existing_points.union(new_points)):
                continue

            new_segment = (start, candidate)
            if any(_segments_intersect(new_segment, existing_segment) for existing_segment in segments):
                continue

            segments.append(new_segment)
            new_points.append(candidate)

        if not new_points:
            break
        branch_points.extend(new_points)

    return segments


def _self_test() -> None:
    segments = venation_branch_points(seed=9)
    if len(segments) < 25:
        raise AssertionError(f"expected a branching structure, got only {len(segments)} segments")
    if any(not math.isfinite(value) for segment in segments for point in segment for value in point):
        raise AssertionError("venation contains non-finite coordinates")
    if any(_segments_intersect(first, second) for index, first in enumerate(segments) for second in segments[index + 1 :]):
        raise AssertionError("venation segments crossed")

    xs = [x for segment in segments for x, _ in segment]
    ys = [y for segment in segments for _, y in segment]
    if min(xs) < 0.0 or max(xs) > 1.0 or min(ys) < 0.0 or max(ys) > 1.0:
        raise AssertionError("venation escaped normalized bounds")

    print(f"segments={len(segments)} non_crossing=True normalized=True")


if __name__ == "__main__":
    _self_test()
