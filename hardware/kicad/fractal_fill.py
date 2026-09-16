from __future__ import annotations

import argparse
import math
import re
import uuid
from dataclasses import dataclass
from pathlib import Path

from fractal_geometry import format_mm, generate_hilbert_points

DEFAULT_INPUT = Path("hardware/kicad/demo/demo.kicad_pcb")
DEFAULT_OUTPUT = Path("hardware/kicad/demo/fractal_fill_demo.kicad_pcb")
DEFAULT_MASKED_NET_ID = 1
DEFAULT_MASKED_NET_NAME = "FRACTAL_FILL_MASKED"
DEFAULT_EXPOSED_NET_ID = 2
DEFAULT_EXPOSED_NET_NAME = "FRACTAL_FILL_EXPOSED"


@dataclass(frozen=True)
class Rect:
    left: float
    bottom: float
    right: float
    top: float

    @property
    def width(self) -> float:
        return self.right - self.left

    @property
    def height(self) -> float:
        return self.top - self.bottom

    def inset(self, margin: float) -> "Rect":
        return Rect(
            self.left + margin,
            self.bottom + margin,
            self.right - margin,
            self.top - margin,
        )


def map_points_to_rect(
    points: list[tuple[int, int]],
    rect: Rect,
    *,
    margin: float = 0.0,
    mirror_x: bool = False,
    mirror_y: bool = False,
) -> list[tuple[float, float]]:
    if rect.width <= 0 or rect.height <= 0:
        raise ValueError("rectangle must have positive dimensions")
    target = rect.inset(margin)
    if target.width <= 0 or target.height <= 0:
        raise ValueError("margin leaves no drawable area")

    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span_x = max_x - min_x
    span_y = max_y - min_y
    if span_x <= 0 or span_y <= 0:
        raise ValueError("curve needs non-zero X/Y span")

    mapped: list[tuple[float, float]] = []
    for x, y in points:
        u = (x - min_x) / span_x
        v = (y - min_y) / span_y
        if mirror_x:
            u = 1.0 - u
        if mirror_y:
            v = 1.0 - v
        mapped.append((target.left + target.width * u, target.bottom + target.height * v))
    return mapped


def build_graphic_lines(points: list[tuple[float, float]], *, width: float, layer: str) -> str:
    if width <= 0:
        raise ValueError("width must be positive")

    chunks: list[str] = []
    for (sx, sy), (ex, ey) in zip(points[:-1], points[1:], strict=True):
        if math.isclose(sx, ex) and math.isclose(sy, ey):
            continue
        chunks.append(
            "\t(gr_line\n"
            f"\t\t(start {format_mm(sx)} {format_mm(sy)})\n"
            f"\t\t(end {format_mm(ex)} {format_mm(ey)})\n"
            "\t\t(stroke\n"
            f"\t\t\t(width {format_mm(width)})\n"
            "\t\t\t(type solid)\n"
            "\t\t)\n"
            f'\t\t(layer "{layer}")\n'
            f'\t\t(uuid "{uuid.uuid4()}")\n'
            "\t)\n"
        )
    return "".join(chunks)


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


def build_via(point: tuple[float, float], *, net_id: int, size: float, drill: float) -> str:
    x, y = point
    if drill <= 0 or size <= drill:
        raise ValueError("via size must be larger than drill")
    return (
        "\t(via\n"
        f"\t\t(at {format_mm(x)} {format_mm(y)})\n"
        f"\t\t(size {format_mm(size)})\n"
        f"\t\t(drill {format_mm(drill)})\n"
        '\t\t(layers "F.Cu" "B.Cu")\n'
        f"\t\t(net {net_id})\n"
        f'\t\t(uuid "{uuid.uuid4()}")\n'
        "\t)\n"
    )


def insert_nets(text: str, nets: list[tuple[int, str]]) -> str:
    updated = text
    matches = list(re.finditer(r"(?m)^\s*\(net\s+\d+\s+.*\)\s*$", text))
    if not matches:
        raise ValueError("could not find a top-level net declaration to anchor insertion")
    insert_at = matches[-1].end()

    additions: list[str] = []
    for net_id, net_name in nets:
        pattern = re.compile(
            rf"(?m)^\s*\(net\s+{re.escape(str(net_id))}\s+\"{re.escape(net_name)}\"\)\s*$"
        )
        if not pattern.search(updated):
            additions.append(f"\n\t(net {net_id} \"{net_name}\")")
    if not additions:
        return updated
    return updated[:insert_at] + "".join(additions) + updated[insert_at:]


def insert_blocks(text: str, *blocks: str) -> str:
    payload = "\n".join(block.rstrip() for block in blocks if block.strip())
    match = re.search(r"\n\)\s*$", text)
    if not match:
        raise ValueError("could not find the root closing parenthesis in the board file")
    return text[: match.start()] + "\n" + payload + text[match.start() :]


def build_demo_blocks(
    *,
    order_silk: int,
    order_copper: int,
    silk_width: float,
    copper_width: float,
    mask_width: float,
    masked_net_id: int,
    exposed_net_id: int,
    via_size: float,
    via_drill: float,
) -> list[str]:
    base_silk = generate_hilbert_points(order_silk)
    base_copper = generate_hilbert_points(order_copper)

    silk_rect = Rect(0.55, 0.55, 3.05, 9.45)
    masked_rect = Rect(3.35, 0.55, 6.65, 9.45)
    exposed_rect = Rect(6.95, 0.85, 9.15, 9.15)

    silk_top = map_points_to_rect(base_silk, silk_rect, margin=0.12)
    silk_bottom = map_points_to_rect(base_silk, silk_rect, margin=0.22, mirror_x=True)

    masked_top = map_points_to_rect(base_copper, masked_rect, margin=0.25)
    masked_bottom = list(reversed(masked_top))

    exposed_top = map_points_to_rect(base_copper, exposed_rect, margin=0.05, mirror_y=True)
    exposed_bottom = list(reversed(exposed_top))

    blocks = [
        build_graphic_lines(silk_top, width=silk_width, layer="F.SilkS"),
        build_graphic_lines(silk_bottom, width=silk_width, layer="B.SilkS"),
        build_segments(masked_top, width=copper_width, layer="F.Cu", net_id=masked_net_id),
        build_segments(masked_bottom, width=copper_width, layer="B.Cu", net_id=masked_net_id),
        build_via(masked_top[0], net_id=masked_net_id, size=via_size, drill=via_drill),
        build_via(masked_top[-1], net_id=masked_net_id, size=via_size, drill=via_drill),
        build_segments(exposed_top, width=copper_width, layer="F.Cu", net_id=exposed_net_id),
        build_segments(exposed_bottom, width=copper_width, layer="B.Cu", net_id=exposed_net_id),
        build_via(exposed_top[0], net_id=exposed_net_id, size=via_size, drill=via_drill),
        build_via(exposed_top[-1], net_id=exposed_net_id, size=via_size, drill=via_drill),
        build_graphic_lines(exposed_top, width=mask_width, layer="F.Mask"),
        build_graphic_lines(exposed_bottom, width=mask_width, layer="B.Mask"),
    ]
    return blocks


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Write a KiCad board copy with three isolated fractal dead-space-fill variants: "
            "silkscreen-only, masked copper, and exposed copper."
        ),
    )
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT, help="Source .kicad_pcb file.")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="Destination .kicad_pcb file.")
    parser.add_argument("--masked-net-id", type=int, default=DEFAULT_MASKED_NET_ID, help="KiCad net id for tented copper art.")
    parser.add_argument("--masked-net-name", default=DEFAULT_MASKED_NET_NAME, help="Net name for tented copper art.")
    parser.add_argument("--exposed-net-id", type=int, default=DEFAULT_EXPOSED_NET_ID, help="KiCad net id for exposed copper art.")
    parser.add_argument("--exposed-net-name", default=DEFAULT_EXPOSED_NET_NAME, help="Net name for exposed copper art.")
    parser.add_argument("--silk-order", type=int, default=4, help="Hilbert order for the silkscreen fill.")
    parser.add_argument("--copper-order", type=int, default=3, help="Hilbert order for both copper variants.")
    parser.add_argument("--silk-width", type=float, default=0.12, help="Line width in mm for silk graphics.")
    parser.add_argument("--copper-width", type=float, default=0.2, help="Track width in mm for isolated copper art.")
    parser.add_argument("--mask-width", type=float, default=0.32, help="Mask opening width in mm for exposed copper art.")
    parser.add_argument("--via-size", type=float, default=0.6, help="Through-via diameter in mm for top/bottom continuity.")
    parser.add_argument("--via-drill", type=float, default=0.3, help="Through-via drill in mm for top/bottom continuity.")
    return parser


def main() -> int:
    args = build_argument_parser().parse_args()
    source_text = args.input.read_text(encoding="utf-8")
    board_with_nets = insert_nets(
        source_text,
        [
            (args.masked_net_id, args.masked_net_name),
            (args.exposed_net_id, args.exposed_net_name),
        ],
    )
    blocks = build_demo_blocks(
        order_silk=args.silk_order,
        order_copper=args.copper_order,
        silk_width=args.silk_width,
        copper_width=args.copper_width,
        mask_width=args.mask_width,
        masked_net_id=args.masked_net_id,
        exposed_net_id=args.exposed_net_id,
        via_size=args.via_size,
        via_drill=args.via_drill,
    )
    output_text = insert_blocks(board_with_nets, *blocks)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(output_text, encoding="utf-8")
    print(
        f"wrote {args.output} with silk_order={args.silk_order}, "
        f"copper_order={args.copper_order}, masked_net={args.masked_net_id}:{args.masked_net_name}, "
        f"exposed_net={args.exposed_net_id}:{args.exposed_net_name}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
