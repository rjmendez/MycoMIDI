from __future__ import annotations

import argparse
import math
import re
import uuid
from dataclasses import dataclass
from pathlib import Path

try:
    import pcbnew
except ImportError as exc:  # pragma: no cover - only hit outside KiCad runtime.
    raise SystemExit(
        "pcbnew is required. Run this script inside KiCad's Python environment or the kicad/kicad:9.0 container."
    ) from exc

from shapely.geometry import LineString
from shapely.ops import unary_union

from adc_texture_fill import (
    CLEARANCE_MM,
    _board_outline,
    _courtyard_or_body_polygon,
    _exact_pad_polygon,
    _track_polygon,
    _via_polygon,
)
from fractal_geometry import format_mm
from lib.kicad_sexpr_cst import List, parse, serialize

DEFAULT_INPUT = Path("hardware/kicad/adc_board/adc_board_8ch.kicad_pcb")
DEFAULT_OUTPUT = DEFAULT_INPUT
SEGMENT_EPSILON_MM = 0.0005
CLEARANCE_TOLERANCE_MM = 0.03


@dataclass(frozen=True)
class RoutePlan:
    net_name: str
    layer: str
    style: str
    start: tuple[float, float]
    end: tuple[float, float]
    left_lane_x: float | None = None
    band_start: tuple[float, float] | None = None
    band_end: tuple[float, float] | None = None
    right_lane_x: float | None = None
    width_mm: float = 0.2
    preserve_segments: tuple[tuple[tuple[float, float], tuple[float, float]], ...] = ()
    tooth_counts: tuple[int, ...] = (11, 10, 9, 8, 7, 6)
    amplitudes_mm: tuple[float, ...] = (2.4, 2.0, 1.6, 1.3, 1.0, 0.8)


ROUTE_PLANS: tuple[RoutePlan, ...] = (
    RoutePlan(
        net_name="AIN0N",
        layer="B.Cu",
        style="direct",
        start=(15.531, 12.0),
        end=(42.1075, 38.5765),
        preserve_segments=(
            ((10.54, 12.0), (15.531, 12.0)),
        ),
        tooth_counts=(10, 9, 8, 7, 6),
        amplitudes_mm=(1.9, 1.6, 1.3, 1.0, 0.8),
    ),
    RoutePlan(
        net_name="AIN1P",
        layer="B.Cu",
        style="direct",
        start=(14.728, 13.3655),
        end=(40.4106, 39.0481),
        preserve_segments=(
            ((8.0, 14.54), (9.1745, 13.3655)),
            ((9.1745, 13.3655), (14.728, 13.3655)),
        ),
        tooth_counts=(10, 9, 8, 7, 6),
        amplitudes_mm=(1.8, 1.5, 1.2, 1.0, 0.8),
    ),
    RoutePlan(
        net_name="AIN2P",
        layer="B.Cu",
        style="direct",
        start=(14.5129, 15.8754),
        end=(39.0382, 40.4007),
        preserve_segments=(
            ((8.0, 17.08), (9.2046, 15.8754)),
            ((9.2046, 15.8754), (14.5129, 15.8754)),
        ),
        tooth_counts=(10, 9, 8, 7, 6),
        amplitudes_mm=(1.7, 1.4, 1.15, 0.95, 0.75),
    ),
    RoutePlan(
        net_name="AIN3P",
        layer="B.Cu",
        style="band",
        start=(13.4683, 23.3117),
        end=(35.8618, 45.7052),
        left_lane_x=13.8,
        band_start=(18.0, 46.0),
        band_end=(29.0, 46.0),
        right_lane_x=34.5,
        preserve_segments=(
            ((9.27, 20.89), (8.0, 19.62)),
            ((10.063, 23.3117), (9.27, 22.5187)),
            ((9.27, 22.5187), (9.27, 20.89)),
            ((13.4683, 23.3117), (10.063, 23.3117)),
            ((38.5975, 43.3757), (38.5975, 44.3338)),
            ((38.5975, 44.3338), (37.2261, 45.7052)),
            ((37.2261, 45.7052), (35.8618, 45.7052)),
        ),
        tooth_counts=(8, 7, 6, 5),
        amplitudes_mm=(2.8, 2.4, 2.0, 1.6, 1.2),
    ),
    RoutePlan(
        net_name="AIN5P",
        layer="B.Cu",
        style="band",
        start=(13.3183, 28.3917),
        end=(31.5023, 46.5757),
        left_lane_x=13.2,
        band_start=(18.0, 58.8),
        band_end=(29.8, 58.8),
        right_lane_x=29.8,
        preserve_segments=(
            ((8.0, 24.7), (9.27, 25.97)),
            ((9.27, 25.97), (9.27, 27.5987)),
            ((9.27, 27.5987), (10.063, 28.3917)),
            ((10.063, 28.3917), (13.3183, 28.3917)),
            ((31.5023, 46.5757), (37.395, 46.5757)),
        ),
        tooth_counts=(7, 6, 5, 4),
        amplitudes_mm=(2.8, 2.4, 2.0, 1.6, 1.2),
    ),
)

def _board_obstacle_union(
    board: pcbnew.BOARD,
    net_name: str,
    layer_name: str,
    clearance_mm: float,
    *,
    include_courtyards: bool,
):
    layer_id = board.GetLayerID(layer_name)
    geometries = []
    for footprint in board.GetFootprints():
        if include_courtyards:
            geometries.append(_courtyard_or_body_polygon(footprint))
        for pad in footprint.Pads():
            if pad.GetNetname() == net_name:
                continue
            geometries.append(_exact_pad_polygon(pad, clearance_mm))
    for item in board.GetTracks():
        if isinstance(item, pcbnew.PCB_VIA):
            if item.GetNetname() == net_name:
                continue
            geometries.append(_via_polygon(item, clearance_mm))
            continue
        if item.GetLayer() != layer_id:
            continue
        if item.GetNetname() == net_name:
            continue
        geometries.append(_track_polygon(item, clearance_mm))
    return unary_union(geometries).buffer(0)


def _zigzag_body(
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    tooth_count: int,
    amplitude_mm: float,
    phase_sign: int,
) -> list[tuple[float, float]]:
    if tooth_count < 2:
        raise ValueError("tooth_count must be at least 2")
    sx, sy = start
    ex, ey = end
    dx = ex - sx
    dy = ey - sy
    length = math.hypot(dx, dy)
    if length <= 0:
        raise ValueError("route endpoints must differ")
    tangent = (dx / length, dy / length)
    normal = (-tangent[1], tangent[0])

    body = [start]
    for tooth_index in range(1, tooth_count):
        u = tooth_index / float(tooth_count)
        base_x = sx + (dx * u)
        base_y = sy + (dy * u)
        envelope = 0.72 + (0.28 * math.sin(math.pi * u))
        offset = amplitude_mm * envelope * (phase_sign if tooth_index % 2 else -phase_sign)
        body.append((base_x + (normal[0] * offset), base_y + (normal[1] * offset)))
    body.append(end)
    return body


def _append_point(points: list[tuple[float, float]], point: tuple[float, float]) -> None:
    if not points or not _points_close(points[-1], point):
        points.append(point)


def _candidate_route_points(
    plan: RoutePlan,
    *,
    tooth_count: int,
    amplitude_mm: float,
    phase_sign: int,
) -> list[tuple[float, float]]:
    if plan.style == "direct":
        return _zigzag_body(plan.start, plan.end, tooth_count=tooth_count, amplitude_mm=amplitude_mm, phase_sign=phase_sign)
    if plan.style != "band" or plan.band_start is None or plan.band_end is None or plan.left_lane_x is None or plan.right_lane_x is None:
        raise ValueError(f"unsupported route plan style {plan.style!r}")

    points: list[tuple[float, float]] = []
    for point in (
        plan.start,
        (plan.left_lane_x, plan.start[1]),
        (plan.left_lane_x, plan.band_start[1]),
        plan.band_start,
    ):
        _append_point(points, point)
    for point in _zigzag_body(plan.band_start, plan.band_end, tooth_count=tooth_count, amplitude_mm=amplitude_mm, phase_sign=phase_sign)[1:]:
        _append_point(points, point)
    for point in (
        (plan.right_lane_x, plan.band_end[1]),
        (plan.right_lane_x, plan.end[1]),
        plan.end,
    ):
        _append_point(points, point)
    return points


def _path_is_clear(
    points: list[tuple[float, float]],
    *,
    width_mm: float,
    obstacle_union,
    board_interior,
    clearance_mm: float,
) -> bool:
    if len(points) < 2 or not LineString(points).is_simple:
        return False
    safe_interior = board_interior.buffer(-((width_mm / 2.0) + CLEARANCE_TOLERANCE_MM))
    if safe_interior.is_empty:
        return False
    guard_mm = (width_mm / 2.0) + clearance_mm
    for start, end in zip(points[:-1], points[1:], strict=True):
        segment = LineString([start, end])
        segment_guard = segment.buffer(guard_mm, cap_style=1, join_style=1)
        if not segment_guard.within(safe_interior):
            return False
        if not obstacle_union.is_empty and segment_guard.intersects(obstacle_union):
            return False
    return True


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


def build_route_points(
    plan: RoutePlan,
    *,
    obstacle_union,
    board_interior,
    width_mm: float,
    clearance_mm: float,
) -> list[tuple[float, float]]:
    for tooth_count in plan.tooth_counts:
        for amplitude_mm in plan.amplitudes_mm:
            for phase_sign in (1, -1):
                points = _candidate_route_points(
                    plan,
                    tooth_count=tooth_count,
                    amplitude_mm=amplitude_mm,
                    phase_sign=phase_sign,
                )
                if _path_is_clear(
                    points,
                    width_mm=width_mm,
                    obstacle_union=obstacle_union,
                    board_interior=board_interior,
                    clearance_mm=clearance_mm,
                ):
                    return points
    raise ValueError(f"failed to find a DRC-safe acute zigzag route for {plan.net_name}")


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


def rewrite_board(input_path: Path, output_path: Path) -> list[tuple[str, int, float]]:
    original = input_path.read_bytes()
    board = _board_root(original)
    net_ids = _net_table(board)
    widths = _remove_target_segments(board, net_ids)
    base_text = serialize(parse(serialize(board))).decode("utf-8", "surrogateescape")
    kicad_board = pcbnew.LoadBoard(str(input_path))
    board_interior = _board_outline(kicad_board)

    generated_blocks = []
    summary = []
    for plan in ROUTE_PLANS:
        width_mm = widths[plan.net_name]
        obstacle_union = _board_obstacle_union(
            kicad_board,
            plan.net_name,
            plan.layer,
            CLEARANCE_MM,
            include_courtyards=(plan.style == "band"),
        )
        mapped = build_route_points(
            plan,
            obstacle_union=obstacle_union,
            board_interior=board_interior,
            width_mm=width_mm,
            clearance_mm=CLEARANCE_MM,
        )
        net_id = net_ids[plan.net_name]
        generated_blocks.append(
            build_segments(
                mapped,
                width=width_mm,
                layer=plan.layer,
                net_id=net_id,
            )
        )
        summary.append((plan.net_name, len(mapped) - 1, width_mm))

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
    for net_name, segment_count, width_mm in summary:
        print(f"{net_name}: routed with {segment_count} sharp segments at {width_mm:.3f} mm width")
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
