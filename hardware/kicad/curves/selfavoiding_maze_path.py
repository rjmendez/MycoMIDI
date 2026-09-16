from __future__ import annotations

Point = tuple[float, float]


def self_avoiding_maze_path_points(columns: int, rows: int, *, seed: int = 0) -> list[Point]:
    """Return a long simple serpentine path across a grid.

    The path is deterministic and self-avoiding by construction. `seed`
    intentionally only changes whether the sweep starts from the top or bottom
    row, so the interface stays stable without introducing backtracking
    failures.
    """

    if columns < 2 or rows < 2:
        raise ValueError("columns and rows must both be at least 2")

    y_values = list(range(rows))
    if seed % 2:
        y_values.reverse()

    path: list[Point] = []
    for row_index, y in enumerate(y_values):
        x_values = range(columns) if row_index % 2 == 0 else range(columns - 1, -1, -1)
        for x in x_values:
            path.append((float(x), float(y)))
    return path


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
