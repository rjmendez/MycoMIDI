from __future__ import annotations

"""Golden-angle phyllotaxis spiral arms for sparse PCB line-art accents.

The exported function returns a small list of polyline arms. Each arm is built
by connecting Vogel-sequence seed points in n-order with a fixed parastichy
step size k: points[offset], points[offset + k], points[offset + 2k], ...
"""

import math


Point = tuple[float, float]
_GOLDEN_ANGLE_RADIANS = 2.399963229728653


def _seed_points(point_count: int) -> list[Point]:
    scale = math.sqrt(max(point_count - 1, 1))
    seeds: list[Point] = []
    for index in range(point_count):
        radius = math.sqrt(index) / scale
        theta = index * _GOLDEN_ANGLE_RADIANS
        seeds.append((0.5 + (0.5 * radius * math.cos(theta)), 0.5 + (0.5 * radius * math.sin(theta))))
    return seeds


def phyllotaxis_spiral_points(
    point_count: int = 89,
    primary_step: int = 5,
    secondary_step: int = 8,
    tertiary_step: int = 13,
    primary_offset: int = 0,
    secondary_offset: int = 2,
    tertiary_offset: int = 5,
) -> list[list[Point]]:
    """Return normalized parastichy arms extracted from a Vogel spiral."""

    if point_count < 8:
        raise ValueError("point_count must be at least 8")

    seeds = _seed_points(point_count)
    requested_arms = (
        (primary_step, primary_offset),
        (secondary_step, secondary_offset),
        (tertiary_step, tertiary_offset),
    )
    arms: list[list[Point]] = []
    for step, offset in requested_arms:
        if step < 2 or step >= point_count:
            continue
        start_index = offset % step
        arm = seeds[start_index::step]
        if len(arm) < 4:
            continue
        arms.append(arm)

    if not arms:
        raise ValueError("at least one parastichy arm must contain four or more points")
    return arms


def _self_test() -> None:
    arms = phyllotaxis_spiral_points()
    if len(arms) != 3:
        raise AssertionError(f"expected three spiral arms, got {len(arms)}")

    point_total = sum(len(arm) for arm in arms)
    for arm in arms:
        radii = [math.hypot((x - 0.5) * 2.0, (y - 0.5) * 2.0) for x, y in arm]
        if any(not math.isfinite(value) for point in arm for value in point):
            raise AssertionError("found non-finite coordinate")
        if any(not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0) for x, y in arm):
            raise AssertionError("point escaped normalized bounds")
        if any(next_radius + 1e-12 < radius for radius, next_radius in zip(radii[:-1], radii[1:], strict=True)):
            raise AssertionError("arm radius regressed; expected outward spiral ordering")

    print(f"arms={len(arms)} total_points={point_total} normalized=True outward_spiral=True")


if __name__ == "__main__":
    _self_test()
