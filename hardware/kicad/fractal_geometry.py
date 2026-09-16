from __future__ import annotations

import math


def rot(n: int, x: int, y: int, rx: int, ry: int) -> tuple[int, int]:
    if ry == 0:
        if rx == 1:
            x = n - 1 - x
            y = n - 1 - y
        x, y = y, x
    return x, y


def d2xy(order: int, distance: int) -> tuple[int, int]:
    size = 1 << order
    x = 0
    y = 0
    t = distance
    step = 1
    while step < size:
        rx = 1 & (t // 2)
        ry = 1 & (t ^ rx)
        x, y = rot(step, x, y, rx, ry)
        x += step * rx
        y += step * ry
        t //= 4
        step <<= 1
    return x, y


def generate_hilbert_points(order: int) -> list[tuple[int, int]]:
    if order < 1:
        raise ValueError("order must be at least 1")
    size = 1 << order
    return [d2xy(order, distance) for distance in range(size * size)]


def expand_lsystem(axiom: str, rules: dict[str, str], iterations: int) -> str:
    if iterations < 0:
        raise ValueError("iterations must be non-negative")
    state = axiom
    for _ in range(iterations):
        state = "".join(rules.get(symbol, symbol) for symbol in state)
    return state


def trace_lsystem(
    program: str,
    *,
    angle_deg: float,
    draw_symbols: set[str],
    step: float = 1.0,
) -> list[tuple[float, float]]:
    if step <= 0:
        raise ValueError("step must be positive")
    x = 0.0
    y = 0.0
    heading_deg = 0.0
    points = [(x, y)]
    for symbol in program:
        if symbol in draw_symbols:
            x += step * math.cos(math.radians(heading_deg))
            y += step * math.sin(math.radians(heading_deg))
            points.append((round(x, 6), round(y, 6)))
        elif symbol == "+":
            heading_deg += angle_deg
        elif symbol == "-":
            heading_deg -= angle_deg
    return points


def generate_moore_points(order: int) -> list[tuple[float, float]]:
    if order < 1:
        raise ValueError("order must be at least 1")
    program = expand_lsystem(
        "LFL+F+LFL",
        {
            "L": "-RF+LFL+FR-",
            "R": "+LF-RFR-FL+",
        },
        order,
    )
    return trace_lsystem(program, angle_deg=90.0, draw_symbols={"F"})


def generate_peano_points(order: int) -> list[tuple[float, float]]:
    if order < 1:
        raise ValueError("order must be at least 1")
    program = expand_lsystem(
        "L",
        {
            "L": "LFRFL-F-RFLFR+F+LFRFL",
            "R": "RFLFR+F+LFRFL-F-RFLFR",
        },
        order,
    )
    return trace_lsystem(program, angle_deg=90.0, draw_symbols={"F", "L", "R"})


def format_mm(value: float) -> str:
    if math.isclose(value, 0.0, abs_tol=1e-9):
        value = 0.0
    return f"{value:.6f}".rstrip("0").rstrip(".")
