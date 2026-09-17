from __future__ import annotations

import argparse
import math
import re
import uuid
from dataclasses import dataclass
from pathlib import Path

from curves.dragon_curve import dragon_curve_points
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
    style: str = "window"
    left_lane_x: float | None = None
    band_start: tuple[float, float] | None = None
    band_end: tuple[float, float] | None = None
    right_lane_x: float | None = None
    width_mm: float = 0.2
    preserve_segments: tuple[tuple[tuple[float, float], tuple[float, float]], ...] = ()
    curve_args: tuple[tuple[str, int], ...] = ()


ROUTE_PLANS: tuple[RoutePlan, ...] = (
    RoutePlan(
        net_name="AIN0P",
        family="maze",
        layer="B.Cu",
        start=(15.8093, 10.7948),
        end=(43.6, 38.5855),
        style="corridor",
        spread_mm=-0.2,
        preserve_segments=(
            ((8.0, 12.0), (9.1517, 12.0)),
            ((9.1517, 12.0), (9.1517, 11.7121)),
            ((9.1517, 11.7121), (10.069, 10.7948)),
            ((10.069, 10.7948), (15.8093, 10.7948)),
        ),
        curve_args=(("columns", 12), ("rows", 2), ("seed", 3)),
    ),
    RoutePlan(
        net_name="AIN1N",
        family="sierpinski",
        layer="F.Cu",
        start=(14.5401, 14.54),
        end=(40.9188, 40.9187),
        spread_mm=0.8,
        style="direct",
        preserve_segments=(
            ((10.54, 14.54), (14.5401, 14.54)),
            ((40.9188, 40.9187), (41.51, 40.9187)),
            ((41.51, 40.9187), (41.9999, 40.4288)),
            ((41.9999, 40.4288), (41.9999, 39.8375)),
            ((41.9999, 39.8375), (42.0, 39.8375)),
        ),
        curve_args=(("order", 2),),
    ),
    RoutePlan(
        net_name="AIN2N",
        family="maze",
        layer="B.Cu",
        start=(13.8995, 17.08),
        end=(38.5952, 41.7757),
        style="corridor",
        spread_mm=0.12,
        preserve_segments=(
            ((10.54, 17.08), (13.8995, 17.08)),
        ),
        curve_args=(("columns", 12), ("rows", 2), ("seed", 3)),
    ),
    RoutePlan(
        net_name="AIN3N",
        family="gosper",
        layer="B.Cu",
        start=(13.3762, 19.62),
        end=(36.3319, 42.5757),
        style="direct",
        spread_mm=-0.4,
        preserve_segments=(
            ((10.54, 19.62), (13.3762, 19.62)),
        ),
        curve_args=(("order", 2),),
    ),
    RoutePlan(
        net_name="AIN3P",
        family="maze",
        layer="B.Cu",
        start=(13.4683, 23.3117),
        end=(35.8618, 45.7052),
        style="window",
        left_lane_x=13.8,
        band_start=(18.0, 46.0),
        band_end=(29.0, 46.0),
        right_lane_x=34.5,
        spread_mm=4.8,
        preserve_segments=(
            ((9.27, 20.89), (8.0, 19.62)),
            ((10.063, 23.3117), (9.27, 22.5187)),
            ((9.27, 22.5187), (9.27, 20.89)),
            ((13.4683, 23.3117), (10.063, 23.3117)),
            ((38.5975, 43.3757), (38.5975, 44.3338)),
            ((38.5975, 44.3338), (37.2261, 45.7052)),
            ((37.2261, 45.7052), (35.8618, 45.7052)),
        ),
        curve_args=(("columns", 10), ("rows", 5), ("seed", 21)),
    ),
    RoutePlan(
        net_name="AIN4N",
        family="sierpinski",
        layer="B.Cu",
        start=(14.1785, 22.16),
        end=(37.0458, 45.0273),
        style="direct",
        spread_mm=0.45,
        preserve_segments=(
            ((10.54, 22.16), (14.1785, 22.16)),
        ),
        curve_args=(("order", 3),),
    ),
    RoutePlan(
        net_name="AIN4P",
        family="peano",
        layer="F.Cu",
        start=(16.3297, 23.43),
        end=(37.2997, 44.4),
        spread_mm=0.5,
        style="direct",
        preserve_segments=(
            ((8.0, 22.16), (9.27, 23.43)),
            ((9.27, 23.43), (16.3297, 23.43)),
            ((37.2997, 44.4), (39.8375, 44.4)),
        ),
        curve_args=(("order", 1),),
    ),
    RoutePlan(
        net_name="AIN5N",
        family="peano",
        layer="F.Cu",
        start=(36.4677, 45.7772),
        end=(15.3905, 24.7),
        spread_mm=0.6,
        style="direct",
        preserve_segments=(
            ((10.54, 24.7), (15.3905, 24.7)),
            ((36.4677, 45.7772), (39.6147, 45.7772)),
            ((39.6147, 45.7772), (39.8375, 46.0)),
        ),
        curve_args=(("order", 1),),
    ),
    RoutePlan(
        family="maze",
        layer="B.Cu",
        net_name="AIN5P",
        start=(13.3183, 28.3917),
        end=(31.5023, 46.5757),
        style="window",
        left_lane_x=13.2,
        band_start=(18.0, 58.8),
        band_end=(29.8, 58.8),
        right_lane_x=29.8,
        spread_mm=5.4,
        preserve_segments=(
            ((8.0, 24.7), (9.27, 25.97)),
            ((9.27, 25.97), (9.27, 27.5987)),
            ((9.27, 27.5987), (10.063, 28.3917)),
            ((10.063, 28.3917), (13.3183, 28.3917)),
            ((31.5023, 46.5757), (37.395, 46.5757)),
        ),
        curve_args=(("columns", 10), ("rows", 5), ("seed", 21)),
    ),
    RoutePlan(
        net_name="AIN6N",
        family="peano",
        layer="F.Cu",
        start=(36.3262, 46.3484),
        end=(17.2178, 27.24),
        spread_mm=0.15,
        style="direct",
        preserve_segments=(
            ((10.54, 27.24), (17.2178, 27.24)),
            ((36.3262, 46.3484), (36.3262, 46.3759)),
            ((36.3262, 46.3759), (37.6014, 47.6511)),
            ((37.6014, 47.6511), (40.3489, 47.6511)),
            ((40.3489, 47.6511), (40.9089, 47.0911)),
            ((40.9089, 47.0911), (41.5508, 47.0911)),
            ((41.5508, 47.0911), (42.0, 47.5403)),
            ((42.0, 47.5403), (42.0, 48.1625)),
        ),
        curve_args=(("order", 1),),
    ),
    RoutePlan(
        net_name="AIN6P",
        family="sierpinski",
        layer="F.Cu",
        start=(17.8923, 28.51),
        end=(37.5448, 48.1625),
        spread_mm=2.8,
        style="direct",
        preserve_segments=(
            ((8.0, 27.24), (9.27, 28.51)),
            ((9.27, 28.51), (17.8923, 28.51)),
            ((37.5448, 48.1625), (41.2, 48.1625)),
        ),
        curve_args=(("order", 4),),
    ),
)


def _curve_kwargs(plan: RoutePlan) -> dict[str, int]:
    return dict(plan.curve_args)


def _selected_plans(net_names: set[str] | None) -> tuple[RoutePlan, ...]:
    if not net_names:
        return ROUTE_PLANS
    selected = tuple(plan for plan in ROUTE_PLANS if plan.net_name in net_names)
    missing = sorted(net_names - {plan.net_name for plan in selected})
    if missing:
        raise ValueError(f"unknown route-plan nets requested: {', '.join(missing)}")
    return selected


def curve_points(plan: RoutePlan) -> list[tuple[float, float]]:
    kwargs = _curve_kwargs(plan)
    if plan.family == "hilbert":
        return [(float(x), float(y)) for x, y in generate_hilbert_points(kwargs["order"])]
    if plan.family == "dragon":
        return dragon_curve_points(kwargs["order"])
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


def orthogonalize_points(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    orthogonal: list[tuple[float, float]] = [points[0]]
    horizontal_first = True
    for ex, ey in points[1:]:
        sx, sy = orthogonal[-1]
        if math.isclose(sx, ex, abs_tol=1e-9) or math.isclose(sy, ey, abs_tol=1e-9):
            orthogonal.append((ex, ey))
            continue
        corner = (ex, sy) if horizontal_first else (sx, ey)
        if not _points_close(orthogonal[-1], corner):
            orthogonal.append(corner)
        orthogonal.append((ex, ey))
        horizontal_first = not horizontal_first

    simplified: list[tuple[float, float]] = [orthogonal[0]]
    for point in orthogonal[1:]:
        if _points_close(point, simplified[-1]):
            continue
        if len(simplified) >= 2:
            ax, ay = simplified[-2]
            bx, by = simplified[-1]
            cx, cy = point
            if (math.isclose(ax, bx, abs_tol=1e-9) and math.isclose(bx, cx, abs_tol=1e-9)) or (
                math.isclose(ay, by, abs_tol=1e-9) and math.isclose(by, cy, abs_tol=1e-9)
            ):
                simplified[-1] = point
                continue
        simplified.append(point)
    return simplified


def build_route_points(plan: RoutePlan) -> list[tuple[float, float]]:
    if plan.style == "direct":
        return map_points(
            curve_points(plan),
            start=plan.start,
            end=plan.end,
            spread=plan.spread_mm,
        )
    if plan.style == "corridor":
        return orthogonalize_points(
            map_points(
                curve_points(plan),
                start=plan.start,
                end=plan.end,
                spread=plan.spread_mm,
            )
        )

    if plan.style != "window":
        raise ValueError(f"unsupported route style {plan.style!r}")
    if None in (plan.left_lane_x, plan.band_start, plan.band_end, plan.right_lane_x):
        raise ValueError(f"window route for {plan.net_name} is missing lane/band data")

    assert plan.band_start is not None
    assert plan.band_end is not None
    assert plan.left_lane_x is not None
    assert plan.right_lane_x is not None
    curve = orthogonalize_points(
        map_points(
            curve_points(plan),
            start=plan.band_start,
            end=plan.band_end,
            spread=plan.spread_mm,
        )
    )
    return [
        plan.start,
        (plan.left_lane_x, plan.start[1]),
        (plan.left_lane_x, plan.band_start[1]),
        plan.band_start,
        *curve[1:-1],
        plan.band_end,
        (plan.right_lane_x, plan.band_end[1]),
        (plan.right_lane_x, plan.end[1]),
        plan.end,
    ]


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


def _remove_target_segments(board: List, net_ids: dict[str, int], plans: tuple[RoutePlan, ...]) -> dict[str, float]:
    widths: dict[str, float] = {}
    for plan in plans:
        net_id = net_ids[plan.net_name]
        removed_any = False
        for child in list(board.children):
            if not isinstance(child, List) or child.head != "segment":
                continue
            net_atom = child.find("net")
            if net_atom is None or int(net_atom.atoms[1].text) != net_id:
                continue
            if _node_text(child, "layer") != plan.layer:
                continue
            segment_start = _node_float_pair(child, "start")
            segment_end = _node_float_pair(child, "end")
            preserved = any(
                _points_close(segment_start, keep_start) and _points_close(segment_end, keep_end)
                or _points_close(segment_start, keep_end) and _points_close(segment_end, keep_start)
                for keep_start, keep_end in plan.preserve_segments
            )
            if preserved:
                continue
            width_child = child.find("width")
            widths[plan.net_name] = float(width_child.atoms[1].text)
            board.remove_child(child)
            removed_any = True
        if not removed_any and plan.preserve_segments:
            raise ValueError(f"failed to find removable routed segments for {plan.net_name}")
        widths.setdefault(plan.net_name, plan.width_mm)
    return widths


def _insert_segments(text: str, segment_block: str) -> str:
    match = re.search(r"\n\)\s*$", text)
    if not match:
        raise ValueError("could not find the root closing parenthesis in the board file")
    return text[: match.start()] + "\n" + segment_block.rstrip() + text[match.start() :]


def rewrite_board(
    input_path: Path,
    output_path: Path,
    *,
    net_names: set[str] | None = None,
) -> list[tuple[str, str]]:
    original = input_path.read_bytes()
    board = _board_root(original)
    net_ids = _net_table(board)
    plans = _selected_plans(net_names)
    widths = _remove_target_segments(board, net_ids, plans)
    base_text = serialize(parse(serialize(board))).decode("utf-8", "surrogateescape")

    generated_blocks = []
    summary = []
    for plan in plans:
        mapped = build_route_points(plan)
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
    parser.add_argument(
        "--nets",
        nargs="*",
        help="Optional list of net names to rewrite. Defaults to every net with a defined route plan.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    net_names = set(args.nets) if args.nets else None
    summary = rewrite_board(args.input, args.output, net_names=net_names)
    for net_name, family in summary:
        print(f"{net_name}: routed with {family}")
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
