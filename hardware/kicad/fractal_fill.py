from __future__ import annotations

import argparse
import math
import re
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

from fractal_geometry import format_mm, generate_hilbert_points, generate_moore_points, generate_peano_points

DEFAULT_INPUT = Path("hardware/kicad/demo/demo.kicad_pcb")
DEFAULT_OUTPUT = Path("hardware/kicad/demo/fractal_fill_demo.kicad_pcb")
ADC_BOARD_INPUT = Path("hardware/kicad/adc_board/adc_board_8ch.kicad_pcb")
ADC_BOARD_OUTPUT = ADC_BOARD_INPUT
DEFAULT_MASKED_NET_ID = 1
DEFAULT_MASKED_NET_NAME = "FRACTAL_FILL_MASKED"
DEFAULT_EXPOSED_NET_ID = 2
DEFAULT_EXPOSED_NET_NAME = "FRACTAL_FILL_EXPOSED"
DEFAULT_GND_NET_NAME = "GND"
ADC_CLEARANCE_MM = 0.25
ADC_REGION_CLEARANCE_MM = 0.45
ADC_GRID_STEP_MM = 1.0
ADC_SILK_CLEARANCE_MM = 0.35
ADC_TILE_TARGET_MM = 9.5
ADC_TILE_MIN_MM = 5.0
ADC_TILE_MAX_MM = 12.5
ADC_MIN_SILK_REGION_WIDTH_MM = 4.0
ADC_MIN_SILK_REGION_HEIGHT_MM = 4.0
ADC_MIN_SILK_REGION_AREA_MM2 = 40.0
ADC_MIN_COPPER_REGION_WIDTH_MM = 6.0
ADC_MIN_COPPER_REGION_HEIGHT_MM = 6.0
ADC_MIN_COPPER_REGION_AREA_MM2 = 60.0
ADC_MAX_COPPER_CONNECTOR_MM = 32.0
ADC_SILK_TILE_MARGIN_MM = 0.4
ADC_COPPER_TILE_MARGIN_MM = 0.45
ADC_MASK_OPENING_WIDTH_MM = 0.28
ADC_TOP_LEVEL_EMBEDDED_FONTS = "\n\t(embedded_fonts no)\n"
ADC_REAL_BOARD_CURVES = ("moore", "peano", "hilbert")


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

    @property
    def area(self) -> float:
        return max(0.0, self.width) * max(0.0, self.height)

    def inset(self, margin: float) -> "Rect":
        return Rect(
            self.left + margin,
            self.bottom + margin,
            self.right - margin,
            self.top - margin,
        )

    def contains_point(self, point: tuple[float, float], *, margin: float = 0.0) -> bool:
        x, y = point
        return (self.left + margin) <= x <= (self.right - margin) and (self.bottom + margin) <= y <= (self.top - margin)

    def intersects(self, other: "Rect") -> bool:
        return not (
            self.right < other.left
            or other.right < self.left
            or self.top < other.bottom
            or other.top < self.bottom
        )


@dataclass(frozen=True)
class Obstacle:
    rect: Rect
    net_name: str
    kind: str
    ref: str = ""
    center: tuple[float, float] | None = None


@dataclass(frozen=True)
class CopperRegionPlan:
    rect: Rect
    anchor: tuple[float, float]
    path: list[tuple[float, float]]
    exposed: bool
    curve_name: str


@dataclass(frozen=True)
class RegionStats:
    silk_regions: list[Rect]
    copper_regions: list[CopperRegionPlan]
    free_silk_area_mm2: float
    free_copper_area_mm2: float


def generate_curve_points(curve_name: str) -> list[tuple[float, float]]:
    if curve_name == "hilbert":
        return generate_hilbert_points(3)
    if curve_name == "moore":
        return generate_moore_points(2)
    if curve_name == "peano":
        return generate_peano_points(2)
    raise ValueError(f"unknown curve family {curve_name!r}")


def preferred_curve_names(index: int) -> list[str]:
    offset = index % len(ADC_REAL_BOARD_CURVES)
    return list(ADC_REAL_BOARD_CURVES[offset:]) + list(ADC_REAL_BOARD_CURVES[:offset])


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


def extract_blocks(text: str, tag: str) -> list[str]:
    blocks: list[str] = []
    index = 0
    pattern = re.compile(rf"\({re.escape(tag)}(?=[\s])")
    while True:
        match = pattern.search(text, index)
        if not match:
            return blocks
        start = match.start()
        depth = 0
        for end in range(start, len(text)):
            char = text[end]
            if char == "(":
                depth += 1
            elif char == ")":
                depth -= 1
                if depth == 0:
                    blocks.append(text[start : end + 1])
                    index = end + 1
                    break
        else:
            raise ValueError(f"unterminated {tag} block")


def parse_float_list(block: str, key: str) -> tuple[float, ...] | None:
    match = re.search(rf"\({re.escape(key)} ([^\)]+)\)", block)
    if not match:
        return None
    return tuple(float(value) for value in match.group(1).split())


def rotate_point(x: float, y: float, rotation_deg: float) -> tuple[float, float]:
    theta = math.radians(rotation_deg)
    return (
        x * math.cos(theta) - y * math.sin(theta),
        x * math.sin(theta) + y * math.cos(theta),
    )


def rotated_half_extents(width: float, height: float, rotation_deg: float) -> tuple[float, float]:
    theta = math.radians(rotation_deg)
    cos_theta = abs(math.cos(theta))
    sin_theta = abs(math.sin(theta))
    return ((width * cos_theta + height * sin_theta) / 2.0, (width * sin_theta + height * cos_theta) / 2.0)


def board_net_map(text: str) -> dict[int, str]:
    return {int(match.group(1)): match.group(2) for match in re.finditer(r'\(net (\d+) "([^"]*)"\)', text)}


def board_net_id(text: str, net_name: str) -> int:
    for net_id, existing_name in board_net_map(text).items():
        if existing_name == net_name:
            return net_id
    raise ValueError(f"board does not define net {net_name!r}")


def graphic_width(block: str) -> float:
    width_match = re.search(r"\(width ([^\)]+)\)", block)
    return float(width_match.group(1)) if width_match else 0.0


def layer_name(block: str) -> str | None:
    match = re.search(r'\(layer "([^"]+)"\)', block)
    return match.group(1) if match else None


def text_dimensions(block: str) -> tuple[float, float]:
    match = re.search(r"\(font\s+\(size ([^\s\)]+) ([^\s\)]+)\)", block)
    if not match:
        return (1.0, 1.0)
    return (float(match.group(1)), float(match.group(2)))


def text_payload(block: str, *, footprint_text: bool) -> str:
    if footprint_text:
        match = re.search(r'\(fp_text\s+[^\s\)]+\s+"([^"]*)"', block)
    else:
        match = re.search(r'\(gr_text\s+"([^"]*)"', block)
    return match.group(1) if match else "TEXT"


def text_rect(text_value: str, center: tuple[float, float], *, size: tuple[float, float], rotation_deg: float) -> Rect:
    glyph_count = max(1, len(text_value.replace("${REFERENCE}", "REF")))
    width = max(size[0], glyph_count * size[0] * 0.8)
    height = size[1] * 2.0
    half_w, half_h = rotated_half_extents(width, height, rotation_deg)
    cx, cy = center
    return Rect(cx - half_w, cy - half_h, cx + half_w, cy + half_h)


def block_rect_from_line(block: str) -> Rect | None:
    start = parse_float_list(block, "start")
    end = parse_float_list(block, "end")
    if not (start and end):
        return None
    pad = graphic_width(block) / 2.0
    return Rect(
        min(start[0], end[0]) - pad,
        min(start[1], end[1]) - pad,
        max(start[0], end[0]) + pad,
        max(start[1], end[1]) + pad,
    )


def line_rects(start: tuple[float, float], end: tuple[float, float], *, pad: float) -> list[Rect]:
    if math.isclose(start[0], end[0]) or math.isclose(start[1], end[1]):
        return [
            Rect(
                min(start[0], end[0]) - pad,
                min(start[1], end[1]) - pad,
                max(start[0], end[0]) + pad,
                max(start[1], end[1]) + pad,
            )
        ]

    boxes: list[Rect] = []
    for sample_x, sample_y in segment_samples(start, end, step_mm=max(0.5, pad * 2.0)):
        boxes.append(Rect(sample_x - pad, sample_y - pad, sample_x + pad, sample_y + pad))
    return boxes


def block_rect_from_rect(block: str) -> Rect | None:
    start = parse_float_list(block, "start")
    end = parse_float_list(block, "end")
    if not (start and end):
        return None
    pad = graphic_width(block) / 2.0
    return Rect(
        min(start[0], end[0]) - pad,
        min(start[1], end[1]) - pad,
        max(start[0], end[0]) + pad,
        max(start[1], end[1]) + pad,
    )


def block_rect_from_circle(block: str) -> Rect | None:
    center = parse_float_list(block, "center")
    end = parse_float_list(block, "end")
    if not (center and end):
        return None
    radius = math.dist(center[:2], end[:2]) + (graphic_width(block) / 2.0)
    return Rect(center[0] - radius, center[1] - radius, center[0] + radius, center[1] + radius)


def block_rect_from_arc(block: str) -> Rect | None:
    points: list[tuple[float, float]] = []
    for key in ("start", "mid", "end"):
        values = parse_float_list(block, key)
        if values and len(values) >= 2:
            points.append((values[0], values[1]))
    if not points:
        return None
    pad = graphic_width(block) / 2.0
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return Rect(min(xs) - pad, min(ys) - pad, max(xs) + pad, max(ys) + pad)


def parse_obstacles(text: str) -> list[Obstacle]:
    nets = board_net_map(text)
    obstacles: list[Obstacle] = []

    for block in extract_blocks(text, "segment"):
        start = parse_float_list(block, "start")
        end = parse_float_list(block, "end")
        width_match = re.search(r"\(width ([^\)]+)\)", block)
        net_match = re.search(r"\(net (\d+)\)", block)
        if not (start and end and width_match and net_match):
            continue
        width = float(width_match.group(1))
        pad = width / 2.0
        center = ((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0)
        for rect in line_rects((start[0], start[1]), (end[0], end[1]), pad=pad):
            obstacles.append(Obstacle(rect=rect, net_name=nets[int(net_match.group(1))], kind="segment", center=center))

    for block in extract_blocks(text, "via"):
        at = parse_float_list(block, "at")
        size_match = re.search(r"\(size ([^\)]+)\)", block)
        net_match = re.search(r"\(net (\d+)\)", block)
        if not (at and size_match and net_match):
            continue
        radius = float(size_match.group(1)) / 2.0
        x, y = at[:2]
        rect = Rect(x - radius, y - radius, x + radius, y + radius)
        obstacles.append(Obstacle(rect=rect, net_name=nets[int(net_match.group(1))], kind="via", center=(x, y)))

    for tag, kind in (("gr_line", "graphic-line"), ("gr_rect", "graphic-rect"), ("gr_circle", "graphic-circle"), ("gr_arc", "graphic-arc")):
        for block in extract_blocks(text, tag):
            layer = layer_name(block)
            if layer in (None, "Edge.Cuts"):
                continue
            if tag == "gr_line":
                rect = block_rect_from_line(block)
            elif tag == "gr_rect":
                rect = block_rect_from_rect(block)
            elif tag == "gr_circle":
                rect = block_rect_from_circle(block)
            else:
                rect = block_rect_from_arc(block)
            if rect is None:
                continue
            obstacles.append(Obstacle(rect=rect, net_name="", kind=kind))

    for block in extract_blocks(text, "gr_text"):
        layer = layer_name(block)
        if layer in (None, "Edge.Cuts"):
            continue
        at = parse_float_list(block, "at")
        if not at:
            continue
        rotation = at[2] if len(at) >= 3 else 0.0
        rect = text_rect(text_payload(block, footprint_text=False), (at[0], at[1]), size=text_dimensions(block), rotation_deg=rotation)
        obstacles.append(Obstacle(rect=rect, net_name="", kind="graphic-text", center=(at[0], at[1])))

    for footprint_block in extract_blocks(text, "footprint"):
        ref_match = re.search(r'\(property "Reference" "([^"]+)"', footprint_block)
        at = parse_float_list(footprint_block, "at")
        if not (ref_match and at):
            continue
        ref = ref_match.group(1)
        fx, fy = at[:2]
        footprint_rotation = at[2] if len(at) >= 3 else 0.0
        footprint_rects: list[Rect] = []
        footprint_points: list[tuple[float, float]] = []

        for pad_block in extract_blocks(footprint_block, "pad"):
            pad_at = parse_float_list(pad_block, "at") or (0.0, 0.0, 0.0)
            pad_size = parse_float_list(pad_block, "size")
            pad_net = re.search(r'\(net (\d+) "([^"]*)"\)', pad_block)
            if not (pad_size and pad_net):
                continue
            local_x, local_y = pad_at[:2]
            pad_rotation = pad_at[2] if len(pad_at) >= 3 else 0.0
            offset_x, offset_y = rotate_point(local_x, local_y, footprint_rotation)
            gx = fx + offset_x
            gy = fy + offset_y
            half_w, half_h = rotated_half_extents(pad_size[0], pad_size[1], footprint_rotation + pad_rotation)
            rect = Rect(gx - half_w, gy - half_h, gx + half_w, gy + half_h)
            net_name = nets[int(pad_net.group(1))]
            obstacles.append(Obstacle(rect=rect, net_name=net_name, kind="pad", ref=ref, center=(gx, gy)))
            footprint_rects.append(rect)
            footprint_points.extend([(rect.left, rect.bottom), (rect.right, rect.top)])

        for graphic_tag in ("fp_line", "fp_rect", "fp_circle", "fp_arc"):
            for graphic_block in extract_blocks(footprint_block, graphic_tag):
                local_rect: Rect | None
                if graphic_tag == "fp_line":
                    local_rect = block_rect_from_line(graphic_block)
                elif graphic_tag == "fp_rect":
                    local_rect = block_rect_from_rect(graphic_block)
                elif graphic_tag == "fp_circle":
                    local_rect = block_rect_from_circle(graphic_block)
                else:
                    local_rect = block_rect_from_arc(graphic_block)
                if local_rect is None:
                    continue
                corners = [
                    (local_rect.left, local_rect.bottom),
                    (local_rect.left, local_rect.top),
                    (local_rect.right, local_rect.bottom),
                    (local_rect.right, local_rect.top),
                ]
                for local_x, local_y in corners:
                    offset_x, offset_y = rotate_point(local_x, local_y, footprint_rotation)
                    footprint_points.append((fx + offset_x, fy + offset_y))

        for text_block in extract_blocks(footprint_block, "fp_text"):
            text_at = parse_float_list(text_block, "at")
            if not text_at:
                continue
            local_x, local_y = text_at[:2]
            text_rotation = text_at[2] if len(text_at) >= 3 else 0.0
            offset_x, offset_y = rotate_point(local_x, local_y, footprint_rotation)
            center = (fx + offset_x, fy + offset_y)
            rect = text_rect(
                text_payload(text_block, footprint_text=True),
                center,
                size=text_dimensions(text_block),
                rotation_deg=footprint_rotation + text_rotation,
            )
            obstacles.append(Obstacle(rect=rect, net_name="", kind="footprint-text", ref=ref, center=center))
            footprint_points.extend([
                (rect.left, rect.bottom),
                (rect.left, rect.top),
                (rect.right, rect.bottom),
                (rect.right, rect.top),
            ])

        if footprint_points:
            xs = [point[0] for point in footprint_points]
            ys = [point[1] for point in footprint_points]
            obstacles.append(
                Obstacle(
                    rect=Rect(min(xs), min(ys), max(xs), max(ys)),
                    net_name="",
                    kind="footprint-body",
                    ref=ref,
                    center=((min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0),
                )
            )

    return obstacles


def inflate_rect(rect: Rect, amount: float) -> Rect:
    return Rect(rect.left - amount, rect.bottom - amount, rect.right + amount, rect.top + amount)


def allowed_obstacle(obstacle: Obstacle, allowed_anchor: tuple[float, float] | None, allowed_net_name: str) -> bool:
    if allowed_anchor is None:
        return False
    if obstacle.net_name != allowed_net_name:
        return False
    if obstacle.kind not in {"via", "segment", "pad"}:
        return False
    bubble = Rect(
        allowed_anchor[0] - 0.8,
        allowed_anchor[1] - 0.8,
        allowed_anchor[0] + 0.8,
        allowed_anchor[1] + 0.8,
    )
    return obstacle.rect.contains_point(allowed_anchor) or obstacle.rect.intersects(bubble)


def rect_clear(rect: Rect, obstacles: Iterable[Obstacle], *, clearance: float, allowed_anchor: tuple[float, float] | None, allowed_net_name: str) -> bool:
    inflated = inflate_rect(rect, clearance)
    for obstacle in obstacles:
        if allowed_obstacle(obstacle, allowed_anchor, allowed_net_name):
            continue
        if inflated.intersects(obstacle.rect):
            return False
    return True


def segment_samples(start: tuple[float, float], end: tuple[float, float], *, step_mm: float = 0.2) -> list[tuple[float, float]]:
    sx, sy = start
    ex, ey = end
    distance = math.dist(start, end)
    if math.isclose(distance, 0.0):
        return [start]
    count = max(2, int(math.ceil(distance / step_mm)) + 1)
    return [
        (sx + (ex - sx) * index / (count - 1), sy + (ey - sy) * index / (count - 1))
        for index in range(count)
    ]


def segment_clear(
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    width: float,
    clearance: float,
    obstacles: Iterable[Obstacle],
    allowed_anchor: tuple[float, float] | None,
    allowed_net_name: str,
) -> bool:
    probe_margin = (width / 2.0) + clearance
    for sample_x, sample_y in segment_samples(start, end):
        probe = Rect(sample_x - probe_margin, sample_y - probe_margin, sample_x + probe_margin, sample_y + probe_margin)
        for obstacle in obstacles:
            if (
                allowed_anchor is not None
                and allowed_obstacle(obstacle, allowed_anchor, allowed_net_name)
                and math.dist((sample_x, sample_y), allowed_anchor) <= 1.0
            ):
                continue
            if obstacle.rect.intersects(probe):
                return False
    return True


def via_clear(
    point: tuple[float, float],
    *,
    size: float,
    clearance: float,
    obstacles: Iterable[Obstacle],
    allowed_net_name: str,
) -> bool:
    radius = (size / 2.0) + clearance
    probe = Rect(point[0] - radius, point[1] - radius, point[0] + radius, point[1] + radius)
    for obstacle in obstacles:
        if obstacle.net_name == allowed_net_name and obstacle.kind == "via" and obstacle.rect.contains_point(point):
            continue
        if obstacle.rect.intersects(probe):
            return False
    return True


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


def strip_generated_adc_art(text: str) -> str:
    marker_index = text.rfind(ADC_TOP_LEVEL_EMBEDDED_FONTS)
    if marker_index < 0:
        return text
    closing_index = text.rfind("\n)")
    if closing_index < 0:
        return text
    return text[: marker_index + len(ADC_TOP_LEVEL_EMBEDDED_FONTS)] + text[closing_index:]


def parse_board_outline(text: str) -> list[tuple[float, float]]:
    segments: list[tuple[tuple[float, float], tuple[float, float]]] = []
    for block in extract_blocks(text, "gr_line"):
        if layer_name(block) != "Edge.Cuts":
            continue
        start = parse_float_list(block, "start")
        end = parse_float_list(block, "end")
        if not (start and end):
            continue
        segments.append(((start[0], start[1]), (end[0], end[1])))
    if not segments:
        raise ValueError("board outline parse found no Edge.Cuts gr_line segments")

    adjacency: dict[tuple[float, float], list[tuple[float, float]]] = {}
    for start, end in segments:
        adjacency.setdefault(start, []).append(end)
        adjacency.setdefault(end, []).append(start)
    if any(len(neighbors) != 2 for neighbors in adjacency.values()):
        raise ValueError("Edge.Cuts outline is not a simple closed loop")

    start_point = min(adjacency)
    polygon = [start_point]
    previous: tuple[float, float] | None = None
    current = start_point
    while True:
        neighbors = adjacency[current]
        next_point = neighbors[0] if neighbors[0] != previous else neighbors[1]
        if next_point == start_point:
            break
        polygon.append(next_point)
        previous, current = current, next_point
        if len(polygon) > len(segments) + 1:
            raise ValueError("Edge.Cuts outline traversal did not close")
    return polygon


def polygon_bounds(polygon: list[tuple[float, float]]) -> Rect:
    xs = [point[0] for point in polygon]
    ys = [point[1] for point in polygon]
    return Rect(min(xs), min(ys), max(xs), max(ys))


def point_in_polygon(point: tuple[float, float], polygon: list[tuple[float, float]]) -> bool:
    x, y = point
    inside = False
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1], strict=True):
        if ((y1 > y) != (y2 > y)) and (x < ((x2 - x1) * (y - y1) / (y2 - y1)) + x1):
            inside = not inside
    return inside


def rect_corners(rect: Rect) -> list[tuple[float, float]]:
    return [
        (rect.left, rect.bottom),
        (rect.left, rect.top),
        (rect.right, rect.bottom),
        (rect.right, rect.top),
    ]


def scan_free_regions(
    polygon: list[tuple[float, float]],
    *,
    obstacles: list[Obstacle],
    clearance: float,
    outline_clearance: float,
    step_mm: float,
    min_width_mm: float,
    min_height_mm: float,
    min_area_mm2: float,
) -> tuple[list[Rect], float]:
    bounds = polygon_bounds(polygon)
    width_steps = int(math.floor(bounds.width / step_mm))
    height_steps = int(math.floor(bounds.height / step_mm))
    xs = [bounds.left + step_mm * (index + 0.5) for index in range(width_steps)]
    ys = [bounds.bottom + step_mm * (index + 0.5) for index in range(height_steps)]
    free = [[False for _ in xs] for _ in ys]

    free_area = 0.0
    for y_index, y in enumerate(ys):
        for x_index, x in enumerate(xs):
            if not point_in_polygon((x, y), polygon):
                continue
            cell = Rect(x - (step_mm / 2.0), y - (step_mm / 2.0), x + (step_mm / 2.0), y + (step_mm / 2.0))
            outline_probe = inflate_rect(cell, outline_clearance)
            if not all(point_in_polygon(corner, polygon) for corner in rect_corners(outline_probe)):
                continue
            inflated = inflate_rect(cell, clearance)
            if any(inflated.intersects(obstacle.rect) for obstacle in obstacles):
                continue
            free[y_index][x_index] = True
            free_area += step_mm * step_mm

    used = [[False for _ in xs] for _ in ys]
    regions: list[Rect] = []
    for y_index in range(len(ys)):
        x_index = 0
        while x_index < len(xs):
            if not free[y_index][x_index] or used[y_index][x_index]:
                x_index += 1
                continue
            run_width = 0
            while x_index + run_width < len(xs) and free[y_index][x_index + run_width] and not used[y_index][x_index + run_width]:
                run_width += 1
            best_width = run_width
            height = 1
            while y_index + height < len(ys):
                next_width = 0
                while next_width < best_width and free[y_index + height][x_index + next_width] and not used[y_index + height][x_index + next_width]:
                    next_width += 1
                if next_width == 0:
                    break
                best_width = min(best_width, next_width)
                height += 1
            for used_y in range(y_index, y_index + height):
                for used_x in range(x_index, x_index + best_width):
                    used[used_y][used_x] = True
            rect = Rect(
                xs[x_index] - (step_mm / 2.0),
                ys[y_index] - (step_mm / 2.0),
                xs[x_index] + step_mm * (best_width - 0.5),
                ys[y_index] + step_mm * (height - 0.5),
            )
            if rect.width >= min_width_mm and rect.height >= min_height_mm and rect.area >= min_area_mm2:
                regions.append(rect)
            x_index += best_width

    regions.sort(key=lambda rect: rect.area, reverse=True)
    return regions, free_area


def choose_tile_count(span_mm: float) -> int:
    count = max(1, int(round(span_mm / ADC_TILE_TARGET_MM)))
    while count > 1 and (span_mm / count) < ADC_TILE_MIN_MM:
        count -= 1
    while (span_mm / count) > ADC_TILE_MAX_MM:
        count += 1
    return max(1, count)


def build_tiled_curve(points: list[tuple[float, float]], rect: Rect, *, start_corner: str, margin: float) -> list[tuple[float, float]]:
    cols = choose_tile_count(rect.width)
    rows = choose_tile_count(rect.height)
    tile_width = rect.width / cols
    tile_height = rect.height / rows
    start_left = start_corner in {"bl", "tl"}
    start_top = start_corner in {"tl", "tr"}
    row_indices = list(range(rows - 1, -1, -1)) if start_top else list(range(rows))

    path: list[tuple[float, float]] = []
    for visit_row, row_index in enumerate(row_indices):
        left_to_right = start_left if visit_row % 2 == 0 else not start_left
        col_indices = range(cols) if left_to_right else range(cols - 1, -1, -1)
        for col_index in col_indices:
            tile = Rect(
                rect.left + (col_index * tile_width),
                rect.bottom + (row_index * tile_height),
                rect.left + ((col_index + 1) * tile_width),
                rect.bottom + ((row_index + 1) * tile_height),
            )
            tile_points = map_points_to_rect(points, tile, margin=margin, mirror_x=not left_to_right, mirror_y=start_top)
            if path and path[-1] != tile_points[0]:
                path.append(tile_points[0])
            path.extend(tile_points if not path else tile_points[1:])
    return path


def connector_candidates(anchor: tuple[float, float], target: tuple[float, float]) -> list[list[tuple[float, float]]]:
    ax, ay = anchor
    tx, ty = target
    return [
        [anchor, target],
        [anchor, (ax, ty), target],
        [anchor, (tx, ay), target],
    ]


def path_clear(
    points: list[tuple[float, float]],
    *,
    width: float,
    clearance: float,
    obstacles: list[Obstacle],
    allowed_anchor: tuple[float, float] | None,
    allowed_net_name: str,
) -> bool:
    return all(
        segment_clear(
            start,
            end,
            width=width,
            clearance=clearance,
            obstacles=obstacles,
            allowed_anchor=allowed_anchor,
            allowed_net_name=allowed_net_name,
        )
        for start, end in zip(points[:-1], points[1:], strict=True)
    )


def gnd_anchor_points(text: str, gnd_net_name: str) -> list[tuple[float, float]]:
    anchors: list[tuple[float, float]] = []
    net_id = board_net_id(text, gnd_net_name)
    for block in extract_blocks(text, "via"):
        if f"(net {net_id})" not in block:
            continue
        at = parse_float_list(block, "at")
        if at and len(at) >= 2:
            anchors.append((at[0], at[1]))
    for footprint_block in extract_blocks(text, "footprint"):
        footprint_at = parse_float_list(footprint_block, "at")
        if not footprint_at:
            continue
        fx, fy = footprint_at[:2]
        footprint_rotation = footprint_at[2] if len(footprint_at) >= 3 else 0.0
        for pad_block in extract_blocks(footprint_block, "pad"):
            if f'(net {net_id} "{gnd_net_name}")' not in pad_block:
                continue
            layers_match = re.search(r"\(layers ([^\)]*)\)", pad_block)
            if not layers_match:
                continue
            layers = re.findall(r'"([^"]+)"', layers_match.group(1))
            if "*.Cu" not in layers and not {"F.Cu", "B.Cu"}.issubset(set(layers)):
                continue
            pad_at = parse_float_list(pad_block, "at") or (0.0, 0.0, 0.0)
            offset_x, offset_y = rotate_point(pad_at[0], pad_at[1], footprint_rotation)
            anchors.append((fx + offset_x, fy + offset_y))
    deduped: list[tuple[float, float]] = []
    seen: set[tuple[float, float]] = set()
    for x, y in anchors:
        key = (round(x, 4), round(y, 4))
        if key in seen:
            continue
        seen.add(key)
        deduped.append((x, y))
    return deduped


def copper_routing_obstacles(obstacles: list[Obstacle]) -> list[Obstacle]:
    return [obstacle for obstacle in obstacles if obstacle.kind in {"segment", "via", "pad"}]


def route_copper_region(
    rect: Rect,
    *,
    curve_names: list[str],
    anchors: list[tuple[float, float]],
    obstacles: list[Obstacle],
    gnd_net_name: str,
    copper_width: float,
    via_size: float,
) -> tuple[str, float, tuple[float, float], list[tuple[float, float]]] | None:
    for curve_name in curve_names:
        base_points = generate_curve_points(curve_name)
        best_choice: tuple[float, tuple[float, float], list[tuple[float, float]]] | None = None
        for start_corner in ("bl", "br", "tl", "tr"):
            core_path = build_tiled_curve(base_points, rect, start_corner=start_corner, margin=ADC_COPPER_TILE_MARGIN_MM)
            if not core_path:
                continue
            if not via_clear(core_path[-1], size=via_size, clearance=ADC_CLEARANCE_MM, obstacles=obstacles, allowed_net_name=gnd_net_name):
                continue
            for anchor in anchors:
                connector: list[tuple[float, float]] | None = None
                for candidate in connector_candidates(anchor, core_path[0]):
                    if path_clear(
                        candidate,
                        width=copper_width,
                        clearance=ADC_CLEARANCE_MM,
                        obstacles=obstacles,
                        allowed_anchor=anchor,
                        allowed_net_name=gnd_net_name,
                    ):
                        connector = candidate
                        break
                if connector is None:
                    continue
                candidate_path = connector + core_path[1:]
                if not path_clear(
                    candidate_path,
                    width=copper_width,
                    clearance=ADC_CLEARANCE_MM,
                    obstacles=obstacles,
                    allowed_anchor=anchor,
                    allowed_net_name=gnd_net_name,
                ):
                    continue
                length = sum(math.dist(start, end) for start, end in zip(connector[:-1], connector[1:], strict=True))
                if length > ADC_MAX_COPPER_CONNECTOR_MM:
                    continue
                if best_choice is None or length < best_choice[0]:
                    best_choice = (length, anchor, candidate_path)
        if best_choice is not None:
            return (curve_name, best_choice[0], best_choice[1], best_choice[2])
    return None


def split_copper_region(rect: Rect) -> list[tuple[Rect, Rect]]:
    pairs: list[tuple[Rect, Rect]] = []
    gap = 1.0
    if rect.width >= (2 * ADC_MIN_COPPER_REGION_WIDTH_MM) + gap:
        split_x = rect.left + (rect.width / 2.0)
        pairs.append(
            (
                Rect(rect.left, rect.bottom, split_x - (gap / 2.0), rect.top),
                Rect(split_x + (gap / 2.0), rect.bottom, rect.right, rect.top),
            )
        )
    if rect.height >= (2 * ADC_MIN_COPPER_REGION_HEIGHT_MM) + gap:
        split_y = rect.bottom + (rect.height / 2.0)
        pairs.append(
            (
                Rect(rect.left, rect.bottom, rect.right, split_y - (gap / 2.0)),
                Rect(rect.left, split_y + (gap / 2.0), rect.right, rect.top),
            )
        )
    return pairs


def choose_copper_regions(
    regions: list[Rect],
    *,
    anchors: list[tuple[float, float]],
    obstacles: list[Obstacle],
    gnd_net_name: str,
    copper_width: float,
    via_size: float,
) -> list[CopperRegionPlan]:
    if not anchors:
        raise ValueError("could not find any existing GND vias to anchor the decorative copper")

    routing_obstacles = copper_routing_obstacles(obstacles)
    routed_regions: list[tuple[Rect, str, float, tuple[float, float], list[tuple[float, float]]]] = []
    for index, rect in enumerate(regions):
        routed = route_copper_region(
            rect,
            curve_names=preferred_curve_names(index),
            anchors=anchors,
            obstacles=routing_obstacles,
            gnd_net_name=gnd_net_name,
            copper_width=copper_width,
            via_size=via_size,
        )
        if routed is None:
            continue
        routed_regions.append((rect, routed[0], routed[1], routed[2], routed[3]))

    if not routed_regions:
        raise ValueError("could not find any anchorable GND fill regions")

    plans: list[CopperRegionPlan] = []
    if len(routed_regions) == 1:
        rect, _, _, _, _ = routed_regions[0]
        best_pair: tuple[float, tuple[CopperRegionPlan, CopperRegionPlan]] | None = None
        for first_rect, second_rect in split_copper_region(rect):
            first = route_copper_region(
                first_rect,
                curve_names=preferred_curve_names(0),
                anchors=anchors,
                obstacles=routing_obstacles,
                gnd_net_name=gnd_net_name,
                copper_width=copper_width,
                via_size=via_size,
            )
            second = route_copper_region(
                second_rect,
                curve_names=preferred_curve_names(1),
                anchors=anchors,
                obstacles=routing_obstacles,
                gnd_net_name=gnd_net_name,
                copper_width=copper_width,
                via_size=via_size,
            )
            if first is None or second is None:
                continue
            pair = (
                first[1] + second[1],
                (
                    CopperRegionPlan(rect=first_rect, anchor=first[2], path=first[3], exposed=False, curve_name=first[0]),
                    CopperRegionPlan(rect=second_rect, anchor=second[2], path=second[3], exposed=True, curve_name=second[0]),
                ),
            )
            if best_pair is None or pair[0] < best_pair[0]:
                best_pair = pair
        if best_pair is not None:
            return [best_pair[1][0], best_pair[1][1]]

    for index, (rect, curve_name, _, anchor, path) in enumerate(routed_regions):
        plans.append(
            CopperRegionPlan(
                rect=rect,
                anchor=anchor,
                path=path,
                exposed=(index % 2) == 1,
                curve_name=curve_name,
            )
        )
    if not any(not plan.exposed for plan in plans):
        raise ValueError("could not find any anchorable masked GND fill regions")
    if not any(plan.exposed for plan in plans):
        raise ValueError("could not find any anchorable exposed GND fill regions")
    return plans


def adc_region_stats(
    source_text: str,
    *,
    gnd_net_name: str,
    copper_width: float,
    via_size: float,
) -> RegionStats:
    clean_text = strip_generated_adc_art(source_text)
    polygon = parse_board_outline(clean_text)
    obstacles = parse_obstacles(clean_text)
    silk_regions, free_silk_area = scan_free_regions(
        polygon,
        obstacles=obstacles,
        clearance=ADC_SILK_CLEARANCE_MM,
        outline_clearance=0.0,
        step_mm=ADC_GRID_STEP_MM,
        min_width_mm=ADC_MIN_SILK_REGION_WIDTH_MM,
        min_height_mm=ADC_MIN_SILK_REGION_HEIGHT_MM,
        min_area_mm2=ADC_MIN_SILK_REGION_AREA_MM2,
    )
    copper_rects, free_copper_area = scan_free_regions(
        polygon,
        obstacles=obstacles,
        clearance=ADC_REGION_CLEARANCE_MM,
        outline_clearance=max(0.0, 0.55 - ADC_COPPER_TILE_MARGIN_MM),
        step_mm=ADC_GRID_STEP_MM,
        min_width_mm=ADC_MIN_COPPER_REGION_WIDTH_MM,
        min_height_mm=ADC_MIN_COPPER_REGION_HEIGHT_MM,
        min_area_mm2=ADC_MIN_COPPER_REGION_AREA_MM2,
    )
    copper_regions = choose_copper_regions(
        copper_rects,
        anchors=gnd_anchor_points(clean_text, gnd_net_name),
        obstacles=obstacles,
        gnd_net_name=gnd_net_name,
        copper_width=copper_width,
        via_size=via_size,
    )
    return RegionStats(
        silk_regions=silk_regions,
        copper_regions=copper_regions,
        free_silk_area_mm2=free_silk_area,
        free_copper_area_mm2=free_copper_area,
    )


def build_adc_board_blocks(
    *,
    source_text: str,
    gnd_net_name: str,
    silk_width: float,
    copper_width: float,
    mask_width: float,
    via_size: float,
    via_drill: float,
) -> list[str]:
    clean_text = strip_generated_adc_art(source_text)
    gnd_net_id = board_net_id(clean_text, gnd_net_name)
    stats = adc_region_stats(clean_text, gnd_net_name=gnd_net_name, copper_width=copper_width, via_size=via_size)

    blocks: list[str] = []
    silk_regions = [rect for rect in stats.silk_regions if not any(rect.intersects(plan.rect) for plan in stats.copper_regions)]
    silk_start_corners = ("bl", "tr", "br", "tl")
    for index, rect in enumerate(silk_regions):
        curve_name = preferred_curve_names(index)[0]
        silk_points = generate_curve_points(curve_name)
        front = build_tiled_curve(silk_points, rect, start_corner=silk_start_corners[index % len(silk_start_corners)], margin=ADC_SILK_TILE_MARGIN_MM)
        back = build_tiled_curve(silk_points, rect, start_corner=silk_start_corners[(index + 1) % len(silk_start_corners)], margin=ADC_SILK_TILE_MARGIN_MM)
        blocks.append(build_graphic_lines(front, width=silk_width, layer="F.SilkS"))
        blocks.append(build_graphic_lines(back, width=silk_width, layer="B.SilkS"))

    for plan in stats.copper_regions:
        blocks.append(build_segments(plan.path, width=copper_width, layer="F.Cu", net_id=gnd_net_id))
        blocks.append(build_segments(plan.path, width=copper_width, layer="B.Cu", net_id=gnd_net_id))
        blocks.append(build_via(plan.path[-1], net_id=gnd_net_id, size=via_size, drill=via_drill))
        if plan.exposed:
            blocks.append(build_graphic_lines(plan.path[1:], width=mask_width, layer="F.Mask"))
            blocks.append(build_graphic_lines(plan.path[1:], width=mask_width, layer="B.Mask"))

    return blocks


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Write a KiCad board copy with decorative fractal fill. "
            "The demo profile uses isolated Hilbert art nets; the adc-board-gnd profile mixes multiple curve families on real GND."
        ),
    )
    parser.add_argument("--profile", choices=("demo", "adc-board-gnd"), default="demo", help="Board profile to decorate.")
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT, help="Source .kicad_pcb file.")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="Destination .kicad_pcb file.")
    parser.add_argument("--masked-net-id", type=int, default=DEFAULT_MASKED_NET_ID, help="KiCad net id for tented copper art (demo profile only).")
    parser.add_argument("--masked-net-name", default=DEFAULT_MASKED_NET_NAME, help="Net name for tented copper art (demo profile only).")
    parser.add_argument("--exposed-net-id", type=int, default=DEFAULT_EXPOSED_NET_ID, help="KiCad net id for exposed copper art (demo profile only).")
    parser.add_argument("--exposed-net-name", default=DEFAULT_EXPOSED_NET_NAME, help="Net name for exposed copper art (demo profile only).")
    parser.add_argument("--gnd-net-name", default=DEFAULT_GND_NET_NAME, help="Net name to use for adc-board-gnd copper art.")
    parser.add_argument("--silk-order", type=int, default=4, help="Hilbert order for the silkscreen fill (demo profile only).")
    parser.add_argument("--copper-order", type=int, default=3, help="Hilbert order for both copper variants (demo profile only).")
    parser.add_argument("--silk-width", type=float, default=0.12, help="Line width in mm for silk graphics.")
    parser.add_argument("--copper-width", type=float, default=0.1, help="Track width in mm for decorative copper art.")
    parser.add_argument("--mask-width", type=float, default=0.28, help="Mask opening width in mm for exposed copper art.")
    parser.add_argument("--via-size", type=float, default=0.6, help="Through-via diameter in mm for top/bottom continuity.")
    parser.add_argument("--via-drill", type=float, default=0.3, help="Through-via drill in mm for top/bottom continuity.")
    return parser


def main() -> int:
    args = build_argument_parser().parse_args()
    if args.profile == "adc-board-gnd":
        if args.input == DEFAULT_INPUT:
            args.input = ADC_BOARD_INPUT
        if args.output == DEFAULT_OUTPUT:
            args.output = ADC_BOARD_OUTPUT

    source_bytes = args.input.read_bytes()
    newline = "\r\n" if b"\r\n" in source_bytes else "\n"
    source_text = source_bytes.decode("utf-8")
    if args.profile == "demo":
        board_text = insert_nets(
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
    else:
        board_text = strip_generated_adc_art(source_text)
        blocks = build_adc_board_blocks(
            source_text=board_text,
            gnd_net_name=args.gnd_net_name,
            silk_width=args.silk_width,
            copper_width=args.copper_width,
            mask_width=args.mask_width,
            via_size=args.via_size,
            via_drill=args.via_drill,
        )

    output_text = insert_blocks(board_text, *blocks)
    if newline == "\r\n":
        output_text = output_text.replace("\n", "\r\n")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(output_text, encoding="utf-8")
    print(
        f"wrote {args.output} with profile={args.profile}, "
        f"copper_width={args.copper_width}, silk_width={args.silk_width}, mask_width={args.mask_width}"
    )
    if args.profile == "adc-board-gnd":
        stats = adc_region_stats(board_text, gnd_net_name=args.gnd_net_name, copper_width=args.copper_width, via_size=args.via_size)
        used_curves = sorted({plan.curve_name for plan in stats.copper_regions} | {preferred_curve_names(index)[0] for index, rect in enumerate(stats.silk_regions) if not any(rect.intersects(plan.rect) for plan in stats.copper_regions)})
        print(f"verified real copper art net: {args.gnd_net_name} (net {board_net_id(board_text, args.gnd_net_name)})")
        print(
            f"detected {len(stats.silk_regions)} silk regions covering ~{stats.free_silk_area_mm2:.0f} mm^2 free space; "
            f"anchored {len(stats.copper_regions)} GND copper regions across ~{sum(plan.rect.area for plan in stats.copper_regions):.0f} mm^2"
        )
        print(f"mixed curve families: {', '.join(used_curves)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
