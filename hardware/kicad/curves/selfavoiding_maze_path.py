from __future__ import annotations

"""Randomized self-avoiding maze path generator for signal-trace routing.

This helper is for the "make it a mess" class of real, non-timing-critical PCB
signal nets: it builds a random orthogonal maze over a configurable rows-by-cols
grid, then returns the single continuous solution path from point A to point B.
The returned route is a simple path with no branches, no revisits, and no
self-crossings, so a future KiCad integration can convert it directly into one
continuous copper track.

That differs from the closed Koch snowflake generator used for pour boundaries,
and from the open decorative wandering curves in this directory whose purpose is
primarily visual variation instead of extracting one guaranteed maze-like trace
between two endpoints.
"""

import random
from collections import deque


Point = tuple[float, float]
Cell = tuple[int, int]

__all__ = ["selfavoiding_maze_path"]


def _validate_grid(rows: int, cols: int) -> None:
    if rows < 1 or cols < 1:
        raise ValueError("rows and cols must both be at least 1")
    if rows * cols < 2:
        raise ValueError("maze path requires at least two cells")


def _validate_endpoint(name: str, cell: Cell, rows: int, cols: int) -> None:
    row, col = cell
    if not (0 <= row < rows and 0 <= col < cols):
        raise ValueError(f"{name} {cell} is outside the {rows}x{cols} grid")


def _cardinal_neighbors(cell: Cell, rows: int, cols: int) -> list[Cell]:
    row, col = cell
    neighbors: list[Cell] = []
    for next_row, next_col in (
        (row - 1, col),
        (row + 1, col),
        (row, col - 1),
        (row, col + 1),
    ):
        if 0 <= next_row < rows and 0 <= next_col < cols:
            neighbors.append((next_row, next_col))
    return neighbors


def _generate_maze_tree(rows: int, cols: int, rng: random.Random) -> dict[Cell, set[Cell]]:
    passages = {(row, col): set() for row in range(rows) for col in range(cols)}
    origin = (rng.randrange(rows), rng.randrange(cols))
    stack = [origin]
    visited = {origin}

    while stack:
        current = stack[-1]
        unvisited_neighbors = [
            neighbor
            for neighbor in _cardinal_neighbors(current, rows, cols)
            if neighbor not in visited
        ]
        if not unvisited_neighbors:
            stack.pop()
            continue

        neighbor = rng.choice(unvisited_neighbors)
        passages[current].add(neighbor)
        passages[neighbor].add(current)
        visited.add(neighbor)
        stack.append(neighbor)

    return passages


def _extract_solution_path(passages: dict[Cell, set[Cell]], start: Cell, end: Cell) -> list[Cell]:
    frontier = deque([start])
    parent: dict[Cell, Cell | None] = {start: None}

    while frontier:
        current = frontier.popleft()
        if current == end:
            break
        for neighbor in passages[current]:
            if neighbor in parent:
                continue
            parent[neighbor] = current
            frontier.append(neighbor)

    if end not in parent:
        raise ValueError("generated maze did not connect the requested endpoints")

    path: list[Cell] = []
    cursor: Cell | None = end
    while cursor is not None:
        path.append(cursor)
        cursor = parent[cursor]
    path.reverse()
    return path


def _normalize_path(path: list[Cell], rows: int, cols: int) -> list[Point]:
    max_row = rows - 1
    max_col = cols - 1
    normalized: list[Point] = []
    for row, col in path:
        x = 0.0 if max_col == 0 else col / max_col
        y = 0.0 if max_row == 0 else row / max_row
        normalized.append((x, y))
    return normalized


def _maze_solution_cells(
    rows: int,
    cols: int,
    start: Cell | None = None,
    end: Cell | None = None,
    seed: int | None = None,
) -> list[Cell]:
    _validate_grid(rows, cols)
    start_cell = (0, 0) if start is None else start
    end_cell = (rows - 1, cols - 1) if end is None else end
    _validate_endpoint("start", start_cell, rows, cols)
    _validate_endpoint("end", end_cell, rows, cols)
    if start_cell == end_cell:
        raise ValueError("start and end must be different cells")

    rng = random.Random(seed)
    passages = _generate_maze_tree(rows, cols, rng)
    return _extract_solution_path(passages, start_cell, end_cell)


def selfavoiding_maze_path(
    rows: int,
    cols: int,
    start: Cell | None = None,
    end: Cell | None = None,
    seed: int | None = None,
) -> list[Point]:
    """Return a normalized simple maze path between two grid cells.

    The path is derived from a recursive-backtracker depth-first maze, then
    reduced to the tree's unique start-to-end route. Consecutive points are
    orthogonally adjacent on the underlying grid, and the returned coordinates
    are normalized into the unit square (or unit interval for 1xN / Nx1 grids).
    """

    path = _maze_solution_cells(rows, cols, start=start, end=end, seed=seed)
    return _normalize_path(path, rows, cols)


def _consecutive_steps_are_adjacent(path: list[Cell]) -> bool:
    return all(
        abs(next_row - row) + abs(next_col - col) == 1
        for (row, col), (next_row, next_col) in zip(path[:-1], path[1:], strict=True)
    )


def _is_simple_path(path: list[Cell]) -> bool:
    if len(path) < 2 or len(set(path)) != len(path):
        return False

    degrees = {cell: 0 for cell in path}
    for start_cell, end_cell in zip(path[:-1], path[1:], strict=True):
        degrees[start_cell] += 1
        degrees[end_cell] += 1

    endpoints = (path[0], path[-1])
    return all(
        degree == 1 if cell in endpoints else degree == 2
        for cell, degree in degrees.items()
    )


def _self_test_case(rows: int, cols: int, seed: int, start: Cell | None = None, end: Cell | None = None) -> None:
    raw_path = _maze_solution_cells(rows, cols, start=start, end=end, seed=seed)
    normalized_path = selfavoiding_maze_path(rows, cols, start=start, end=end, seed=seed)

    simple_path = _is_simple_path(raw_path)
    adjacent_steps = _consecutive_steps_are_adjacent(raw_path)
    in_unit_box = all(0.0 <= x <= 1.0 and 0.0 <= y <= 1.0 for x, y in normalized_path)

    if not simple_path:
        raise AssertionError(f"{rows}x{cols} seed={seed}: path was not simple")
    if not adjacent_steps:
        raise AssertionError(f"{rows}x{cols} seed={seed}: path had a jump")
    if not in_unit_box:
        raise AssertionError(f"{rows}x{cols} seed={seed}: normalized point escaped [0,1]")
    if len(raw_path) != len(normalized_path):
        raise AssertionError(f"{rows}x{cols} seed={seed}: normalized point count changed")

    print(
        f"rows={rows} cols={cols} seed={seed} path_length={len(raw_path)} "
        f"simple_path={simple_path} adjacent_steps={adjacent_steps} "
        f"start={raw_path[0]} end={raw_path[-1]}"
    )


if __name__ == "__main__":
    _self_test_case(8, 8, seed=7)
    _self_test_case(11, 7, seed=29, start=(0, 1), end=(10, 5))
