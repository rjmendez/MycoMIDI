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


def format_mm(value: float) -> str:
    if math.isclose(value, 0.0, abs_tol=1e-9):
        value = 0.0
    return f"{value:.6f}".rstrip("0").rstrip(".")
