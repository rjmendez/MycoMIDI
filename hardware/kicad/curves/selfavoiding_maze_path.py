from __future__ import annotations

import random

Point = tuple[float, float]


def _neighbors(x: int, y: int, columns: int, rows: int) -> list[tuple[int, int]]:
    candidates = ((x + 1, y), (x, y + 1), (x - 1, y), (x, y - 1))
    return [(nx, ny) for nx, ny in candidates if 0 <= nx < columns and 0 <= ny < rows]


def self_avoiding_maze_path_points(columns: int, rows: int, *, seed: int = 0) -> list[Point]:
    """Return a deterministic self-avoiding Hamiltonian path across a grid."""

    if columns < 2 or rows < 2:
        raise ValueError("columns and rows must both be at least 2")

    total = columns * rows
    if total > 36:
        y_values = list(range(rows))
        if seed % 2:
            y_values.reverse()
        x_even = list(range(columns))
        x_odd = list(range(columns - 1, -1, -1))
        return [
            (float(x), float(y))
            for row_index, y in enumerate(y_values)
            for x in (x_even if row_index % 2 == 0 else x_odd)
        ]

    rng = random.Random((columns * 10_007) + (rows * 1_009) + (seed * 97))
    corners = ((0, 0), (0, rows - 1), (columns - 1, 0), (columns - 1, rows - 1))
    start = corners[seed % len(corners)]
    visited: set[tuple[int, int]] = {start}
    path: list[tuple[int, int]] = [start]

    def _candidate_key(candidate: tuple[int, int]) -> tuple[int, int, float]:
        onward = sum(1 for neighbor in _neighbors(*candidate, columns, rows) if neighbor not in visited)
        if len(path) < 2:
            turn_bias = 0
        else:
            last_dx = path[-1][0] - path[-2][0]
            last_dy = path[-1][1] - path[-2][1]
            next_dx = candidate[0] - path[-1][0]
            next_dy = candidate[1] - path[-1][1]
            turn_bias = 0 if (next_dx, next_dy) != (last_dx, last_dy) else 1
        return onward, turn_bias, rng.random()

    def _search() -> bool:
        if len(path) == total:
            return True
        candidates = [candidate for candidate in _neighbors(*path[-1], columns, rows) if candidate not in visited]
        candidates.sort(key=_candidate_key)
        for candidate in candidates:
            visited.add(candidate)
            path.append(candidate)
            if _search():
                return True
            path.pop()
            visited.remove(candidate)
        return False

    if not _search():
        raise RuntimeError(f"failed to build self-avoiding path for {columns}x{rows} grid")
    return [(float(x), float(y)) for x, y in path]


def _self_test(columns: int, rows: int, seed: int) -> None:
    points = self_avoiding_maze_path_points(columns, rows, seed=seed)
    if len(points) != len(set(points)):
        raise AssertionError("path revisited a grid point")
    for (sx, sy), (ex, ey) in zip(points[:-1], points[1:], strict=True):
        if abs(ex - sx) + abs(ey - sy) != 1.0:
            raise AssertionError(f"non-adjacent step from {(sx, sy)} to {(ex, ey)}")
    print(f"grid={columns}x{rows} seed={seed} points={len(points)} self_avoiding=True")


if __name__ == "__main__":
    for sample in ((6, 4, 7), (7, 5, 11)):
        _self_test(*sample)
