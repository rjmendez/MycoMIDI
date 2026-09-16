from __future__ import annotations

import argparse
import math
import re
import uuid
from pathlib import Path

from fractal_geometry import format_mm, generate_hilbert_points


DEFAULT_INPUT = Path("hardware/kicad/demo/demo.kicad_pcb")
DEFAULT_OUTPUT = Path("hardware/kicad/demo/fractal_demo.kicad_pcb")
DEFAULT_LAYER = "F.Cu"
DEFAULT_NET_ID = 1
DEFAULT_NET_NAME = "FRACTAL_FUN"


def map_points(
    points: list[tuple[int, int]],
    start: tuple[float, float],
    end: tuple[float, float],
    spread: float,
) -> list[tuple[float, float]]:
    if spread <= 0:
        raise ValueError("spread must be positive")
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

    min_v = min(normal_values)
    max_v = max(normal_values)
    scale_v = max_v - min_v
    if scale_v <= 0:
        raise ValueError("curve has no transverse spread to map")

    mapped = []
    for u, v in relative_points:
        normalized_v = (v - min_v) / scale_v
        mx = sx + tangent[0] * (length * u) + normal[0] * (spread * normalized_v)
        my = sy + tangent[1] * (length * u) + normal[1] * (spread * normalized_v)
        mapped.append((mx, my))
    return mapped


def build_segments(
    points: list[tuple[float, float]],
    *,
    width: float,
    layer: str,
    net_id: int,
) -> str:
    if width <= 0:
        raise ValueError("width must be positive")

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


def build_return_path(
    *,
    start: tuple[float, float],
    end: tuple[float, float],
    margin: float,
) -> list[tuple[float, float]]:
    if margin <= 0:
        raise ValueError("return margin must be positive")

    sx, sy = start
    ex, ey = end
    dx = ex - sx
    dy = ey - sy
    length = math.hypot(dx, dy)
    if length <= 0:
        raise ValueError("start and end must differ")

    tangent = (dx / length, dy / length)
    normal = (-tangent[1], tangent[0])
    return [
        end,
        (ex - normal[0] * margin, ey - normal[1] * margin),
        (sx - normal[0] * margin, sy - normal[1] * margin),
        start,
    ]


def build_copper_block(
    path_points: list[tuple[float, float]],
    *,
    start: tuple[float, float],
    end: tuple[float, float],
    net_id: int,
    width: float,
    layer: str,
    return_margin: float,
) -> str:
    return_points = build_return_path(start=start, end=end, margin=return_margin)
    return build_segments(path_points, width=width, layer=layer, net_id=net_id) + build_segments(
        return_points,
        width=width,
        layer=layer,
        net_id=net_id,
    )


def insert_net(text: str, *, net_id: int, net_name: str) -> str:
    pattern = re.compile(
        rf"(?m)^\s*\(net\s+{re.escape(str(net_id))}\s+\"{re.escape(net_name)}\"\)\s*$"
    )
    if pattern.search(text):
        return text

    matches = list(re.finditer(r"(?m)^\s*\(net\s+\d+\s+.*\)\s*$", text))
    if not matches:
        raise ValueError("could not find a top-level net declaration to anchor insertion")
    insert_at = matches[-1].end()
    return text[:insert_at] + f"\n\t(net {net_id} \"{net_name}\")" + text[insert_at:]


def insert_segments(text: str, segment_block: str) -> str:
    match = re.search(r"\n\)\s*$", text)
    if not match:
        raise ValueError("could not find the root closing parenthesis in the board file")
    return text[: match.start()] + "\n" + segment_block.rstrip() + text[match.start() :]


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Inject a Hilbert-curve novelty trace route into a KiCad board copy.",
    )
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT, help="Source .kicad_pcb file.")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="Destination .kicad_pcb file.")
    parser.add_argument("--start-x", type=float, default=1.0, help="Trace start X coordinate in mm.")
    parser.add_argument("--start-y", type=float, default=1.0, help="Trace start Y coordinate in mm.")
    parser.add_argument("--end-x", type=float, default=9.0, help="Trace end X coordinate in mm.")
    parser.add_argument("--end-y", type=float, default=1.0, help="Trace end Y coordinate in mm.")
    parser.add_argument(
        "--spread",
        type=float,
        default=7.2,
        help="Distance in mm that the space-filling curve extends away from the baseline.",
    )
    parser.add_argument("--order", type=int, default=5, help="Hilbert recursion order.")
    parser.add_argument("--width", type=float, default=0.08, help="Track width in mm.")
    parser.add_argument(
        "--return-margin",
        type=float,
        default=0.4,
        help="Distance in mm used to close the route with a short return path below the baseline.",
    )
    parser.add_argument("--layer", default=DEFAULT_LAYER, help="KiCad copper layer name.")
    parser.add_argument("--net-id", type=int, default=DEFAULT_NET_ID, help="Numeric KiCad net id.")
    parser.add_argument("--net-name", default=DEFAULT_NET_NAME, help="KiCad net name.")
    return parser


def main() -> int:
    args = build_argument_parser().parse_args()
    source_text = args.input.read_text(encoding="utf-8")
    board_with_net = insert_net(source_text, net_id=args.net_id, net_name=args.net_name)
    curve = generate_hilbert_points(args.order)
    mapped = map_points(
        curve,
        start=(args.start_x, args.start_y),
        end=(args.end_x, args.end_y),
        spread=args.spread,
    )
    copper_block = build_copper_block(
        mapped,
        start=(args.start_x, args.start_y),
        end=(args.end_x, args.end_y),
        net_id=args.net_id,
        width=args.width,
        layer=args.layer,
        return_margin=args.return_margin,
    )
    output_text = insert_segments(board_with_net, copper_block)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(output_text, encoding="utf-8")
    print(
        f"wrote {args.output} with {max(len(mapped) - 1, 0)} segments "
        f"(order={args.order}, width={args.width}mm, spread={args.spread}mm)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
