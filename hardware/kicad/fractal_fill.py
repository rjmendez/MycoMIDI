from __future__ import annotations

import argparse
import math
import re
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

from fractal_geometry import format_mm, generate_hilbert_points

DEFAULT_INPUT = Path("hardware/kicad/demo/demo.kicad_pcb")
DEFAULT_OUTPUT = Path("hardware/kicad/demo/fractal_fill_demo.kicad_pcb")
ADC_BOARD_INPUT = Path("hardware/kicad/adc_board/adc_board_8ch.kicad_pcb")
ADC_BOARD_OUTPUT = ADC_BOARD_INPUT
DEFAULT_MASKED_NET_ID = 1
DEFAULT_MASKED_NET_NAME = "FRACTAL_FILL_MASKED"
DEFAULT_EXPOSED_NET_ID = 2
DEFAULT_EXPOSED_NET_NAME = "FRACTAL_FILL_EXPOSED"
DEFAULT_GND_NET_NAME = "GND"
ADC_BOARD_OUTLINE = (2.0, 2.0, 90.0, 68.0)
ADC_SILK_RECT = (12.0, 60.6, 31.0, 66.4)
ADC_MASKED_RECT = (56.0, 53.0, 64.0, 59.0)
ADC_EXPOSED_RECT = (66.0, 53.0, 74.0, 59.0)
ADC_MASKED_ANCHOR = (52.9509, 57.1759)
ADC_EXPOSED_ANCHOR = (52.9509, 57.1759)
ADC_CLEARANCE_MM = 0.25
ADC_REGION_CLEARANCE_MM = 0.45


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
class RegionSpec:
    name: str
    rect: Rect
    silk_order: int | None = None
    copper_order: int | None = None
    mirror_x: bool = False
    mirror_y: bool = False
    anchor: tuple[float, float] | None = None
    add_mask_opening: bool = False
    add_end_via: bool = False


@dataclass(frozen=True)
class Obstacle:
    rect: Rect
    net_name: str
    kind: str
    ref: str = ""


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
    needle = f"({tag} "
    while True:
        start = text.find(needle, index)
        if start < 0:
            return blocks
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
        rect = Rect(
            min(start[0], end[0]) - pad,
            min(start[1], end[1]) - pad,
            max(start[0], end[0]) + pad,
            max(start[1], end[1]) + pad,
        )
        obstacles.append(Obstacle(rect=rect, net_name=nets[int(net_match.group(1))], kind="segment"))

    for block in extract_blocks(text, "via"):
        at = parse_float_list(block, "at")
        size_match = re.search(r"\(size ([^\)]+)\)", block)
        net_match = re.search(r"\(net (\d+)\)", block)
        if not (at and size_match and net_match):
            continue
        radius = float(size_match.group(1)) / 2.0
        x, y = at[:2]
        rect = Rect(x - radius, y - radius, x + radius, y + radius)
        obstacles.append(Obstacle(rect=rect, net_name=nets[int(net_match.group(1))], kind="via"))

    for footprint_block in extract_blocks(text, "footprint"):
        ref_match = re.search(r'\(property "Reference" "([^"]+)"', footprint_block)
        at = parse_float_list(footprint_block, "at")
        if not (ref_match and at):
            continue
        ref = ref_match.group(1)
        fx, fy = at[:2]
        footprint_rotation = at[2] if len(at) >= 3 else 0.0
        footprint_rects: list[Rect] = []

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
            obstacles.append(Obstacle(rect=rect, net_name=net_name, kind="pad", ref=ref))
            footprint_rects.append(rect)

        local_points: list[tuple[float, float]] = []
        for graphic_tag in ("fp_line", "fp_rect"):
            for graphic_block in extract_blocks(footprint_block, graphic_tag):
                for key in ("start", "end"):
                    values = parse_float_list(graphic_block, key)
                    if values and len(values) >= 2:
                        local_points.append((values[0], values[1]))
        if local_points:
            global_points = []
            for local_x, local_y in local_points:
                offset_x, offset_y = rotate_point(local_x, local_y, footprint_rotation)
                global_points.append((fx + offset_x, fy + offset_y))
            xs = [point[0] for point in global_points]
            ys = [point[1] for point in global_points]
            obstacles.append(
                Obstacle(
                    rect=Rect(min(xs), min(ys), max(xs), max(ys)),
                    net_name="",
                    kind="footprint-body",
                    ref=ref,
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
    if obstacle.kind != "via":
        return False
    return obstacle.rect.contains_point(allowed_anchor)


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
            if allowed_obstacle(obstacle, allowed_anchor, allowed_net_name) and obstacle.rect.contains_point((sample_x, sample_y), margin=0.01):
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


def adc_regions() -> tuple[RegionSpec, RegionSpec, RegionSpec]:
    return (
        RegionSpec(name="silk", rect=Rect(*ADC_SILK_RECT), silk_order=4, mirror_x=False),
        RegionSpec(
            name="masked",
            rect=Rect(*ADC_MASKED_RECT),
            copper_order=3,
            mirror_x=True,
            anchor=ADC_MASKED_ANCHOR,
            add_end_via=True,
        ),
        RegionSpec(
            name="exposed",
            rect=Rect(*ADC_EXPOSED_RECT),
            copper_order=3,
            mirror_x=False,
            anchor=ADC_EXPOSED_ANCHOR,
            add_mask_opening=True,
            add_end_via=True,
        ),
    )


def adc_front_prefix(region_name: str, anchor: tuple[float, float], first_point: tuple[float, float]) -> list[tuple[float, float]]:
    return []


def validate_adc_profile(
    source_text: str,
    *,
    gnd_net_name: str,
    copper_width: float,
    via_size: float,
) -> None:
    net_id = board_net_id(source_text, gnd_net_name)
    if net_id <= 0:
        raise ValueError(f"unexpected ground net id {net_id} for {gnd_net_name}")

    board_rect = Rect(*ADC_BOARD_OUTLINE)
    obstacles = parse_obstacles(source_text)
    for region in adc_regions():
        if not board_rect.intersects(region.rect) or not board_rect.contains_point((region.rect.left, region.rect.bottom)) or not board_rect.contains_point((region.rect.right, region.rect.top)):
            raise ValueError(f"region {region.name} lies outside board outline")
        clearance = ADC_REGION_CLEARANCE_MM if region.copper_order is not None else 0.2
        allowed_anchor = region.anchor if region.copper_order is not None else None
        if not rect_clear(region.rect, obstacles, clearance=clearance, allowed_anchor=allowed_anchor, allowed_net_name=gnd_net_name):
            raise ValueError(f"region {region.name} no longer clears the routed board; adjust its rectangle")

    for region in adc_regions():
        if region.copper_order is None or region.anchor is None:
            continue
        mapped = map_points_to_rect(
            generate_hilbert_points(region.copper_order),
            region.rect,
            margin=0.18,
            mirror_x=region.mirror_x,
            mirror_y=region.mirror_y,
        )
        end_via = mapped[-1]
        if not via_clear(end_via, size=via_size, clearance=ADC_CLEARANCE_MM, obstacles=obstacles, allowed_net_name=gnd_net_name):
            raise ValueError(f"planned {region.name} end via at {end_via} collides with the board")
        front_path = [region.anchor, *adc_front_prefix(region.name, region.anchor, mapped[0]), *mapped]
        back_path = [region.anchor, *list(reversed(mapped))]
        for start, end in zip(front_path[:-1], front_path[1:], strict=True):
            if not segment_clear(
                start,
                end,
                width=copper_width,
                clearance=ADC_CLEARANCE_MM,
                obstacles=obstacles,
                allowed_anchor=region.anchor,
                allowed_net_name=gnd_net_name,
            ):
                raise ValueError(f"{region.name} front-copper path is too close to existing board geometry")
        for start, end in zip(back_path[:-1], back_path[1:], strict=True):
            if not segment_clear(
                start,
                end,
                width=copper_width,
                clearance=ADC_CLEARANCE_MM,
                obstacles=obstacles,
                allowed_anchor=region.anchor,
                allowed_net_name=gnd_net_name,
            ):
                raise ValueError(f"{region.name} back-copper path is too close to existing board geometry")
        if region.add_end_via:
            if not region.rect.contains_point(end_via, margin=0.15):
                raise ValueError(f"{region.name} end via must stay inside its decorative region")


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
    validate_adc_profile(source_text, gnd_net_name=gnd_net_name, copper_width=copper_width, via_size=via_size)
    gnd_net_id = board_net_id(source_text, gnd_net_name)
    silk_region, masked_region, exposed_region = adc_regions()
    silk_points = generate_hilbert_points(silk_region.silk_order or 4)
    copper_points = generate_hilbert_points(masked_region.copper_order or 3)

    silk_front = map_points_to_rect(silk_points, silk_region.rect, margin=0.12)
    silk_back = map_points_to_rect(silk_points, silk_region.rect, margin=0.2, mirror_x=True, mirror_y=True)

    masked_core = map_points_to_rect(
        copper_points,
        masked_region.rect,
        margin=0.18,
        mirror_x=masked_region.mirror_x,
        mirror_y=masked_region.mirror_y,
    )
    masked_front = [masked_region.anchor, *adc_front_prefix(masked_region.name, masked_region.anchor, masked_core[0]), *masked_core]
    masked_back = [masked_region.anchor, *list(reversed(masked_core))]
    masked_end_via = masked_core[-1]
    masked_front.append(masked_end_via)
    masked_back.append(masked_end_via)

    exposed_core = map_points_to_rect(
        copper_points,
        exposed_region.rect,
        margin=0.18,
        mirror_x=exposed_region.mirror_x,
        mirror_y=exposed_region.mirror_y,
    )
    exposed_front = [exposed_region.anchor, *exposed_core]
    exposed_back = [exposed_region.anchor, *list(reversed(exposed_core))]
    exposed_end_via = exposed_core[-1]
    exposed_front.append(exposed_end_via)
    exposed_back.append(exposed_end_via)

    blocks = [
        build_graphic_lines(silk_front, width=silk_width, layer="F.SilkS"),
        build_graphic_lines(silk_back, width=silk_width, layer="B.SilkS"),
        build_segments(masked_front, width=copper_width, layer="F.Cu", net_id=gnd_net_id),
        build_segments(masked_back, width=copper_width, layer="B.Cu", net_id=gnd_net_id),
        build_via(masked_end_via, net_id=gnd_net_id, size=via_size, drill=via_drill),
        build_segments(exposed_front, width=copper_width, layer="F.Cu", net_id=gnd_net_id),
        build_segments(exposed_back, width=copper_width, layer="B.Cu", net_id=gnd_net_id),
        build_via(exposed_end_via, net_id=gnd_net_id, size=via_size, drill=via_drill),
        build_graphic_lines(exposed_core, width=mask_width, layer="F.Mask"),
        build_graphic_lines(list(reversed(exposed_core)), width=mask_width, layer="B.Mask"),
    ]
    return blocks


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Write a KiCad board copy with Hilbert-curve decorative fill. "
            "The demo profile uses isolated art nets; the adc-board-gnd profile ties the copper art to real GND."
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
        board_text = source_text
        blocks = build_adc_board_blocks(
            source_text=source_text,
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
        print(f"verified real copper art net: {args.gnd_net_name} (net {board_net_id(source_text, args.gnd_net_name)})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
