from __future__ import annotations

import argparse
import math
import re
import uuid
from dataclasses import dataclass
from pathlib import Path

from curves.gosper_curve import gosper_curve_points
from curves.selfavoiding_maze_path import self_avoiding_maze_path_points
from curves.sierpinski_arrowhead_curve import sierpinski_arrowhead_curve_points
from fractal_geometry import format_mm, generate_hilbert_points, generate_moore_points, generate_peano_points
from lib.kicad_sexpr_cst import List, parse, serialize

DEFAULT_INPUT = Path("hardware/kicad/adc_board/adc_board_8ch.kicad_pcb")
DEFAULT_OUTPUT = DEFAULT_INPUT
SEGMENT_EPSILON_MM = 0.0005


@dataclass(frozen=True)
class RoutePlan:
    net_name: str
    family: str
    layer: str
    start: tuple[float, float]
    end: tuple[float, float]
    spread_mm: float
    curve_args: tuple[tuple[str, int], ...] = ()


ROUTE_PLANS: tuple[RoutePlan, ...] = (
    RoutePlan(
        net_name="AIN0N",
        family="maze",
        layer="B.Cu",
        start=(15.531, 12.0),
        end=(42.1075, 38.5765),
        spread_mm=0.55,
        curve_args=(("columns", 5), ("rows", 3), ("seed", 7)),
    ),
    RoutePlan(
        net_name="AIN1P",
        family="peano",
        layer="B.Cu",
        start=(14.728, 13.3655),
        end=(40.4106, 39.0481),
        spread_mm=-0.55,
        curve_args=(("order", 1),),
    ),
    RoutePlan(
        net_name="AIN2P",
        family="sierpinski",
        layer="B.Cu",
        start=(14.5129, 15.8754),
        end=(39.0382, 40.4007),
        spread_mm=0.45,
        curve_args=(("order", 3),),
    ),
    RoutePlan(
        net_name="AIN5P",
        family="gosper",
        layer="B.Cu",
        start=(13.3183, 28.3917),
        end=(31.5023, 46.5757),
        spread_mm=-0.9,
        curve_args=(("order", 1),),
    ),
)


def _curve_kwargs(plan: RoutePlan) -> dict[str, int]:
    return dict(plan.curve_args)


def curve_points(plan: RoutePlan) -> list[tuple[float, float]]:
    kwargs = _curve_kwargs(plan)
    if plan.family == "hilbert":
        return [(float(x), float(y)) for x, y in generate_hilbert_points(kwargs["order"])]
    if plan.family == "moore":
        return generate_moore_points(kwargs["order"])
    if plan.family == "peano":
        return generate_peano_points(kwargs["order"])
    if plan.family == "gosper":
        return gosper_curve_points(kwargs["order"])
    if plan.family == "sierpinski":
        return sierpinski_arrowhead_curve_points(kwargs["order"])
    if plan.family == "maze":
        return self_avoiding_maze_path_points(
            kwargs["columns"],
            kwargs["rows"],
            seed=kwargs.get("seed", 0),
        )
    raise ValueError(f"unsupported curve family {plan.family!r}")


def map_points(
    points: list[tuple[float, float]],
    *,
    start: tuple[float, float],
    end: tuple[float, float],
    spread: float,
) -> list[tuple[float, float]]:
    if math.isclose(spread, 0.0, abs_tol=1e-12):
        raise ValueError("spread must be non-zero")
    if len(points) < 2:
        raise ValueError("curve requires multiple points")

    sx, sy = start
    ex, ey = end
    dx = ex - sx
    dy = ey - sy
    length = math.hypot(dx, dy)
    if length <= 0:
        raise ValueError("start and end must differ")

    first_x, first_y = points[0]
    last_x, last_y = points[-1]
    base_dx = last_x - first_x
    base_dy = last_y - first_y
    base_length = math.hypot(base_dx, base_dy)
    if base_length <= 0:
        raise ValueError("curve endpoints must differ")

    base_tangent = (base_dx / base_length, base_dy / base_length)
    base_normal = (-base_tangent[1], base_tangent[0])
    tangent = (dx / length, dy / length)
    normal = (-tangent[1], tangent[0])

    relative_points = []
    normal_values = []
    for x, y in points:
        rel_x = x - first_x
        rel_y = y - first_y
        u = (rel_x * base_tangent[0] + rel_y * base_tangent[1]) / base_length
        v = rel_x * base_normal[0] + rel_y * base_normal[1]
        relative_points.append((u, v))
        normal_values.append(v)

    first_v = relative_points[0][1]
    last_v = relative_points[-1][1]
    adjusted_points = []
    for u, v in relative_points:
        baseline_v = ((1.0 - u) * first_v) + (u * last_v)
        adjusted_points.append((u, v - baseline_v))

    normal_values = [v for _, v in adjusted_points]
    min_v = min(normal_values)
    max_v = max(normal_values)
    scale_v = max_v - min_v
    if scale_v <= 0:
        raise ValueError("curve has no transverse spread to map")

    spread_sign = 1.0 if spread > 0 else -1.0
    spread_abs = abs(spread)
    mapped = []
    for u, v in adjusted_points:
        normalized_v = (v - min_v) / scale_v
        if spread_sign < 0:
            normalized_v = 1.0 - normalized_v
        mx = sx + tangent[0] * (length * u) + normal[0] * (spread_abs * normalized_v)
        my = sy + tangent[1] * (length * u) + normal[1] * (spread_abs * normalized_v)
        mapped.append((mx, my))
    mapped[0] = start
    mapped[-1] = end
    return mapped


def build_segments(
    points: list[tuple[float, float]],
    *,
    width: float,
    layer: str,
    net_id: int,
) -> str:
    chunks: list[str] = []
    for (sx, sy), (ex, ey) in zip(points[:-1], points[1:], strict=True):
        if math.isclose(sx, ex) and math.isclose(sy, ey):
            continue
        chunks.append(
            "\t(segment\n"
            f"\t\t(start {format_mm(sx)} {format_mm(sy)})\n"
            f"\t\t(end {format_mm(ex)} {format_mm(ey)})\n"
            f"\t\t(width {format_mm(width)})\n"
            f'\t\t(layer "{layer}")\n'
            f"\t\t(net {net_id})\n"
            f'\t\t(uuid "{uuid.uuid4()}")\n'
            "\t)\n"
        )
    return "".join(chunks)


def _node_float_pair(node: List, name: str) -> tuple[float, float]:
    child = node.find(name)
    if child is None or len(child.atoms) < 3:
        raise ValueError(f"missing {name} in {node.head}")
    return float(child.atoms[1].text), float(child.atoms[2].text)


def _node_text(node: List, name: str) -> str:
    child = node.find(name)
    if child is None or len(child.atoms) < 2:
        raise ValueError(f"missing {name} in {node.head}")
    return child.atoms[1].text


def _segment_matches(node: List, *, layer: str, start: tuple[float, float], end: tuple[float, float]) -> bool:
    if _node_text(node, "layer") != layer:
        return False
    actual_start = _node_float_pair(node, "start")
    actual_end = _node_float_pair(node, "end")
    return (
        _points_close(actual_start, start)
        and _points_close(actual_end, end)
    ) or (
        _points_close(actual_start, end)
        and _points_close(actual_end, start)
    )


def _points_close(first: tuple[float, float], second: tuple[float, float]) -> bool:
    return math.isclose(first[0], second[0], abs_tol=SEGMENT_EPSILON_MM) and math.isclose(
        first[1], second[1], abs_tol=SEGMENT_EPSILON_MM
    )


def _board_root(data: bytes) -> List:
    doc = parse(data)
    if not doc.lists:
        raise ValueError("board document did not contain a root list")
    return doc.lists[0]


def _net_table(board: List) -> dict[str, int]:
    net_ids: dict[str, int] = {}
    for child in board.find_all("net"):
        if len(child.atoms) < 3:
            continue
        net_ids[child.atoms[2].text] = int(child.atoms[1].text)
    return net_ids


def _remove_target_segments(board: List, net_ids: dict[str, int]) -> dict[str, float]:
    widths: dict[str, float] = {}
    for plan in ROUTE_PLANS:
        net_id = net_ids[plan.net_name]
        match = None
        for child in list(board.children):
            if not isinstance(child, List) or child.head != "segment":
                continue
            net_atom = child.find("net")
            if net_atom is None or int(net_atom.atoms[1].text) != net_id:
                continue
            if _segment_matches(child, layer=plan.layer, start=plan.start, end=plan.end):
                width_child = child.find("width")
                widths[plan.net_name] = float(width_child.atoms[1].text)
                match = child
                break
        if match is None:
            raise ValueError(
                f"failed to find target segment for {plan.net_name} on {plan.layer} "
                f"from {plan.start} to {plan.end}"
            )
        board.remove_child(match)
    return widths


def _insert_segments(text: str, segment_block: str) -> str:
    match = re.search(r"\n\)\s*$", text)
    if not match:
        raise ValueError("could not find the root closing parenthesis in the board file")
    return text[: match.start()] + "\n" + segment_block.rstrip() + text[match.start() :]


def rewrite_board(input_path: Path, output_path: Path) -> list[tuple[str, str]]:
    original = input_path.read_bytes()
    board = _board_root(original)
    net_ids = _net_table(board)
    widths = _remove_target_segments(board, net_ids)
    base_text = serialize(parse(serialize(board))).decode("utf-8", "surrogateescape")

    generated_blocks = []
    summary = []
    for plan in ROUTE_PLANS:
        mapped = map_points(
            curve_points(plan),
            start=plan.start,
            end=plan.end,
            spread=plan.spread_mm,
        )
        net_id = net_ids[plan.net_name]
        generated_blocks.append(
            build_segments(
                mapped,
                width=widths[plan.net_name],
                layer=plan.layer,
                net_id=net_id,
            )
        )
        summary.append((plan.net_name, plan.family))

    updated = _insert_segments(base_text, "".join(generated_blocks))
    output_path.write_text(updated, encoding="utf-8")
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Replace selected ADS131M08 signal segments with procedural chaos.")
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    summary = rewrite_board(args.input, args.output)
    for net_name, family in summary:
        print(f"{net_name}: routed with {family}")
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
