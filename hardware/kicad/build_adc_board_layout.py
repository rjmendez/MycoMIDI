from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

try:
    import pcbnew
except ImportError as exc:  # pragma: no cover - only hit outside KiCad runtime.
    raise SystemExit(
        "pcbnew is required. Run this script inside KiCad's Python environment or the kicad/kicad:9.0 container."
    ) from exc

BOARD_WIDTH_MM = 60.0
BOARD_HEIGHT_MM = 40.0
OUTLINE_MARGIN_MM = 2.0
ANALOG_TRACK_WIDTH_MM = 0.20
SIGNAL_TRACK_WIDTH_MM = 0.25
POWER_TRACK_WIDTH_MM = 0.35
VIA_DIAMETER_MM = 0.60
VIA_DRILL_MM = 0.30
FOOTPRINT_LIB_ROOT = Path("/usr/share/kicad/footprints")
SCRIPT_DIR = Path(__file__).resolve().parent
ADC_BOARD_NETLIST = SCRIPT_DIR / "adc_board" / "adc_board_8ch.net"
ADC_BOARD_PCB = SCRIPT_DIR / "adc_board" / "adc_board_8ch.kicad_pcb"


@dataclass(frozen=True)
class ComponentSpec:
    ref: str
    value: str
    footprint: str


@dataclass(frozen=True)
class Placement:
    x_mm: float
    y_mm: float
    rotation_deg: float = 0.0


PLACEMENTS = {
    "U1": Placement(30.0, 20.0, 0.0),
    "J1": Placement(8.0, 11.0, 0.0),
    "J2": Placement(54.0, 7.5, 0.0),
    "J3": Placement(54.0, 23.5, 0.0),
    "C1": Placement(35.8, 25.6, 0.0),
    "C2": Placement(39.2, 25.6, 0.0),
    "C3": Placement(35.8, 14.0, 0.0),
    "C4": Placement(39.2, 14.0, 0.0),
    "C5": Placement(39.0, 21.6, 90.0),
    "C6": Placement(39.0, 18.2, 90.0),
}


TOP_ANALOG_LANES = {
    "AIN0P": 12.2,
    "AIN0N": 12.9,
    "AIN1N": 13.6,
    "AIN1P": 14.3,
}

BOTTOM_ANALOG_LANES = {
    "AIN6P": 26.2,
    "AIN6N": 26.9,
    "AIN7N": 27.6,
    "AIN7P": 28.3,
}

def mm(value: float) -> int:
    return pcbnew.FromMM(value)


def point(x_mm: float, y_mm: float) -> pcbnew.VECTOR2I:
    return pcbnew.VECTOR2I(mm(x_mm), mm(y_mm))


class NetlistData:
    def __init__(self, path: Path):
        self.path = path
        self.components: dict[str, ComponentSpec] = {}
        self.pin_to_net: dict[tuple[str, str], str] = {}
        self.net_names: list[str] = []

    def load(self) -> None:
        parsed = parse_sexpr(self.path.read_text(encoding="utf-8"))
        blocks = {entry[0]: entry[1:] for entry in parsed[1:] if isinstance(entry, list) and entry}

        for comp in blocks["components"]:
            ref = find_scalar(comp, "ref")
            self.components[ref] = ComponentSpec(
                ref=ref,
                value=(find_scalar(comp, "value") or ref),
                footprint=(find_scalar(comp, "footprint") or ""),
            )

        nets_sorted = sorted(blocks["nets"], key=lambda item: int(find_scalar(item, "code")))
        self.net_names = [find_scalar(net, "name") for net in nets_sorted if find_scalar(net, "name")]
        for net in nets_sorted:
            net_name = find_scalar(net, "name")
            for node in child_blocks(net, "node"):
                self.pin_to_net[(find_scalar(node, "ref"), find_scalar(node, "pin"))] = net_name


TOKEN_RE = re.compile(r'''\s*(?:(\()|(\))|"((?:[^"\\\\]|\\\\.)*)"|([^\s()]+))''')


def parse_sexpr(text: str) -> list:
    tokens = TOKEN_RE.finditer(text)
    stack: list[list] = []
    root: list | None = None
    for match in tokens:
        open_paren, close_paren, quoted, bare = match.groups()
        if open_paren:
            node: list = []
            if stack:
                stack[-1].append(node)
            stack.append(node)
            if root is None:
                root = node
        elif close_paren:
            if not stack:
                raise ValueError("Unbalanced closing parenthesis in netlist")
            stack.pop()
        else:
            value = quoted if quoted is not None else bare
            if not stack or value is None:
                continue
            stack[-1].append(value)
    if stack or root is None:
        raise ValueError("Failed to parse complete S-expression netlist")
    return root


def find_scalar(node: list, key: str) -> str:
    for child in node[1:]:
        if isinstance(child, list) and child and child[0] == key:
            if len(child) < 2:
                return ""
            return str(child[1]).strip('"')
    raise KeyError(f"Missing {key} in {node[:2]}")


def child_blocks(node: list, key: str) -> list[list]:
    return [child for child in node[1:] if isinstance(child, list) and child and child[0] == key]


def add_outline(board: pcbnew.BOARD) -> None:
    corners = [
        (OUTLINE_MARGIN_MM, OUTLINE_MARGIN_MM),
        (BOARD_WIDTH_MM - OUTLINE_MARGIN_MM, OUTLINE_MARGIN_MM),
        (BOARD_WIDTH_MM - OUTLINE_MARGIN_MM, BOARD_HEIGHT_MM - OUTLINE_MARGIN_MM),
        (OUTLINE_MARGIN_MM, BOARD_HEIGHT_MM - OUTLINE_MARGIN_MM),
    ]
    for start, end in zip(corners, corners[1:] + corners[:1], strict=True):
        shape = pcbnew.PCB_SHAPE(board)
        shape.SetShape(pcbnew.S_SEGMENT)
        shape.SetLayer(pcbnew.Edge_Cuts)
        shape.SetWidth(mm(0.15))
        shape.SetStart(point(*start))
        shape.SetEnd(point(*end))
        board.Add(shape)


def create_nets(board: pcbnew.BOARD, net_names: Iterable[str]) -> dict[str, pcbnew.NETINFO_ITEM]:
    nets: dict[str, pcbnew.NETINFO_ITEM] = {}
    for net_name in net_names:
        if not net_name:
            continue
        net = pcbnew.NETINFO_ITEM(board, net_name)
        board.Add(net)
        nets[net_name] = net
    return nets


def load_footprint(component: ComponentSpec) -> pcbnew.FOOTPRINT:
    lib_name, footprint_name = component.footprint.split(":", 1)
    lib_path = FOOTPRINT_LIB_ROOT / f"{lib_name}.pretty"
    footprint = pcbnew.FootprintLoad(str(lib_path), footprint_name)
    if footprint is None:
        raise FileNotFoundError(f"Could not load footprint {component.footprint} from {lib_path}")
    return footprint


def place_components(
    board: pcbnew.BOARD,
    netlist: NetlistData,
    nets: dict[str, pcbnew.NETINFO_ITEM],
) -> dict[str, pcbnew.FOOTPRINT]:
    footprints: dict[str, pcbnew.FOOTPRINT] = {}
    for ref in sorted(netlist.components):
        if ref not in PLACEMENTS:
            raise KeyError(f"No placement defined for {ref}")
        component = netlist.components[ref]
        placement = PLACEMENTS[ref]
        footprint = load_footprint(component)
        footprint.SetReference(ref)
        footprint.SetValue(component.value)
        footprint.SetPosition(point(placement.x_mm, placement.y_mm))
        footprint.SetOrientationDegrees(placement.rotation_deg)
        for pad in footprint.Pads():
            net_name = netlist.pin_to_net.get((ref, pad.GetNumber()))
            if net_name:
                pad.SetNet(nets[net_name])
        board.Add(footprint)
        footprints[ref] = footprint
    return footprints


def pad_xy_mm(footprint: pcbnew.FOOTPRINT, pad_number: str) -> tuple[float, float]:
    for pad in footprint.Pads():
        if pad.GetNumber() == pad_number:
            pos = pad.GetPosition()
            return pcbnew.ToMM(pos.x), pcbnew.ToMM(pos.y)
    raise KeyError(f"Pad {pad_number} not found on {footprint.GetReference()}")


def add_track(board: pcbnew.BOARD, net: pcbnew.NETINFO_ITEM, start: tuple[float, float], end: tuple[float, float], *, layer: int, width_mm: float) -> None:
    track = pcbnew.PCB_TRACK(board)
    track.SetLayer(layer)
    track.SetNet(net)
    track.SetWidth(mm(width_mm))
    track.SetStart(point(*start))
    track.SetEnd(point(*end))
    board.Add(track)


def add_via(board: pcbnew.BOARD, net: pcbnew.NETINFO_ITEM, at: tuple[float, float]) -> None:
    via = pcbnew.PCB_VIA(board)
    via.SetNet(net)
    via.SetPosition(point(*at))
    via.SetDrill(mm(VIA_DRILL_MM))
    via.SetWidth(mm(VIA_DIAMETER_MM))
    via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    board.Add(via)


def add_path(board: pcbnew.BOARD, net: pcbnew.NETINFO_ITEM, points_mm: list[tuple[float, float]], *, layer: int, width_mm: float) -> None:
    for start, end in zip(points_mm, points_mm[1:]):
        add_track(board, net, start, end, layer=layer, width_mm=width_mm)


def route_via_trunk(
    board: pcbnew.BOARD,
    net: pcbnew.NETINFO_ITEM,
    start: tuple[float, float],
    back_path: list[tuple[float, float]],
    end_via: tuple[float, float],
    front_path: list[tuple[float, float]],
    *,
    width_mm: float,
) -> None:
    add_path(board, net, [start, *back_path, end_via], layer=pcbnew.B_Cu, width_mm=width_mm)
    add_via(board, net, end_via)
    add_path(board, net, [end_via, *front_path], layer=pcbnew.F_Cu, width_mm=width_mm)


def route_analog_inputs(board: pcbnew.BOARD, nets: dict[str, pcbnew.NETINFO_ITEM], fps: dict[str, pcbnew.FOOTPRINT]) -> None:
    header = fps["J1"]
    adc = fps["U1"]
    trunk_x = [12.5 + (index * 0.8) for index in range(16)]
    analog_order = [
        ("AIN0P", "29", "1"),
        ("AIN0N", "30", "2"),
        ("AIN1P", "32", "3"),
        ("AIN1N", "31", "4"),
        ("AIN2P", "1", "5"),
        ("AIN2N", "2", "6"),
        ("AIN3P", "4", "7"),
        ("AIN3N", "3", "8"),
        ("AIN4P", "5", "9"),
        ("AIN4N", "6", "10"),
        ("AIN5P", "8", "11"),
        ("AIN5N", "7", "12"),
        ("AIN6P", "9", "13"),
        ("AIN6N", "10", "14"),
        ("AIN7P", "12", "15"),
        ("AIN7N", "11", "16"),
    ]

    for index, (net_name, adc_pad, header_pad) in enumerate(analog_order):
        net = nets[net_name]
        start = pad_xy_mm(header, header_pad)
        pad_x, pad_y = pad_xy_mm(adc, adc_pad)
        bus_x = trunk_x[index]
        is_positive_column = int(header_pad) % 2 == 1
        start_lane_y = start[1] - 0.7 if is_positive_column else start[1] + 0.7

        if net_name in TOP_ANALOG_LANES:
            lane_y = TOP_ANALOG_LANES[net_name]
            end_via = (bus_x, lane_y)
            front_path = [(pad_x, lane_y), (pad_x, pad_y)]
        elif net_name in BOTTOM_ANALOG_LANES:
            lane_y = BOTTOM_ANALOG_LANES[net_name]
            end_via = (bus_x, lane_y)
            front_path = [(pad_x, lane_y), (pad_x, pad_y)]
        else:
            end_via = (bus_x, pad_y)
            front_path = [(pad_x, pad_y)]
        route_via_trunk(
            board,
            net,
            start,
            (
                [(6.2, start[1]), (6.2, start_lane_y), (bus_x, start_lane_y), (bus_x, end_via[1])]
                if is_positive_column
                else [(start[0], start_lane_y), (bus_x, start_lane_y), (bus_x, end_via[1])]
            ),
            end_via,
            front_path,
            width_mm=ANALOG_TRACK_WIDTH_MM,
        )


def route_digital_headers(board: pcbnew.BOARD, nets: dict[str, pcbnew.NETINFO_ITEM], fps: dict[str, pcbnew.FOOTPRINT]) -> None:
    adc = fps["U1"]
    spi = fps["J2"]
    power = fps["J3"]
    routes = {
        "CS": (spi, "1", adc, "17", 50.0, "right"),
        "DRDY": (spi, "5", adc, "18", 49.0, "right"),
        "SCLK": (spi, "2", adc, "19", 48.0, "right"),
        "DOUT": (spi, "4", adc, "20", 47.0, "right"),
        "DIN": (spi, "3", adc, "21", 46.0, "right"),
        "CLKIN": (power, "4", adc, "23", 45.0, "right"),
        "SYNC_RESET": (power, "5", adc, "16", 44.0, "bottom"),
    }
    for net_name, (src_fp, src_pad, dst_fp, dst_pad, bus_x, side) in routes.items():
        net = nets[net_name]
        start = pad_xy_mm(src_fp, src_pad)
        pad_x, pad_y = pad_xy_mm(dst_fp, dst_pad)
        if side == "bottom":
            lane_y = 28.8
            end_via = (bus_x, lane_y)
            front_path = [(pad_x, lane_y), (pad_x, pad_y)]
        else:
            end_via = (bus_x, pad_y)
            front_path = [(pad_x, pad_y)]
        route_via_trunk(
            board,
            net,
            start,
            [(bus_x, start[1]), (bus_x, end_via[1])],
            end_via,
            front_path,
            width_mm=SIGNAL_TRACK_WIDTH_MM,
        )


def route_power_and_caps(board: pcbnew.BOARD, nets: dict[str, pcbnew.NETINFO_ITEM], fps: dict[str, pcbnew.FOOTPRINT]) -> None:
    adc = fps["U1"]
    power = fps["J3"]

    avdd = nets["AVDD"]
    dvdd = nets["DVDD"]
    gnd = nets["GND"]
    refp = nets["REFP"]
    cap = nets["CAP"]

    # AVDD header -> caps -> ADC pin 15.
    j3_avdd = pad_xy_mm(power, "1")
    c1_p1 = pad_xy_mm(fps["C1"], "1")
    c2_p1 = pad_xy_mm(fps["C2"], "1")
    u1_avdd = pad_xy_mm(adc, "15")
    add_path(board, avdd, [j3_avdd, (44.0, j3_avdd[1]), (44.0, c1_p1[1]), c1_p1], layer=pcbnew.F_Cu, width_mm=POWER_TRACK_WIDTH_MM)
    add_path(board, avdd, [c1_p1, (37.4, c1_p1[1]), c2_p1], layer=pcbnew.F_Cu, width_mm=POWER_TRACK_WIDTH_MM)
    add_path(board, avdd, [c1_p1, (u1_avdd[0], c1_p1[1]), u1_avdd], layer=pcbnew.F_Cu, width_mm=POWER_TRACK_WIDTH_MM)

    # DVDD header -> caps -> ADC pin 26.
    j3_dvdd = pad_xy_mm(power, "2")
    c3_p1 = pad_xy_mm(fps["C3"], "1")
    c4_p1 = pad_xy_mm(fps["C4"], "1")
    u1_dvdd = pad_xy_mm(adc, "26")
    add_path(board, dvdd, [j3_dvdd, (46.0, j3_dvdd[1]), (46.0, c3_p1[1]), c3_p1], layer=pcbnew.F_Cu, width_mm=POWER_TRACK_WIDTH_MM)
    add_path(board, dvdd, [c3_p1, (37.4, c3_p1[1]), c4_p1], layer=pcbnew.F_Cu, width_mm=POWER_TRACK_WIDTH_MM)
    add_path(board, dvdd, [c3_p1, (u1_dvdd[0], c3_p1[1]), u1_dvdd], layer=pcbnew.F_Cu, width_mm=POWER_TRACK_WIDTH_MM)

    # REFIN alias (REFP) and CAP support capacitors.
    c5_p1 = pad_xy_mm(fps["C5"], "1")
    u1_ref = pad_xy_mm(adc, "14")
    add_path(board, refp, [c5_p1, (u1_ref[0], c5_p1[1]), u1_ref], layer=pcbnew.F_Cu, width_mm=SIGNAL_TRACK_WIDTH_MM)

    c6_p1 = pad_xy_mm(fps["C6"], "1")
    u1_cap = pad_xy_mm(adc, "24")
    add_path(board, cap, [u1_cap, (36.6, u1_cap[1]), c6_p1], layer=pcbnew.F_Cu, width_mm=SIGNAL_TRACK_WIDTH_MM)

    # Ground trunk on B.Cu, then fan out with F.Cu stubs through dedicated vias.
    j3_gnd = pad_xy_mm(power, "3")
    trunk_x = 52.0
    trunk = [j3_gnd, (trunk_x, j3_gnd[1]), (trunk_x, 12.6)]
    add_path(board, gnd, trunk, layer=pcbnew.B_Cu, width_mm=POWER_TRACK_WIDTH_MM)

    branch_vias = {
        "C1": ((42.2, 27.0), "2"),
        "C2": ((44.2, 27.0), "2"),
        "C3": ((42.2, 12.6), "2"),
        "C4": ((44.2, 12.6), "2"),
        "C5": ((36.2, 23.0), "2"),
        "C6": ((41.0, 16.6), "2"),
    }
    for ref, (via_point, pad_num) in branch_vias.items():
        add_path(board, gnd, [(trunk_x, via_point[1]), via_point], layer=pcbnew.B_Cu, width_mm=POWER_TRACK_WIDTH_MM)
        add_via(board, gnd, via_point)
        pad2 = pad_xy_mm(fps[ref], pad_num)
        add_path(board, gnd, [via_point, pad2], layer=pcbnew.F_Cu, width_mm=POWER_TRACK_WIDTH_MM)

    adc_ground_vias = {
        "13": (30.4, 25.8),
        "25": (32.8, 15.2),
        "28": (30.4, 15.2),
    }
    for pad_num, via_point in adc_ground_vias.items():
        add_path(board, gnd, [(trunk_x, via_point[1]), via_point], layer=pcbnew.B_Cu, width_mm=POWER_TRACK_WIDTH_MM)
        add_via(board, gnd, via_point)
        pad = pad_xy_mm(adc, pad_num)
        add_path(board, gnd, [via_point, pad], layer=pcbnew.F_Cu, width_mm=POWER_TRACK_WIDTH_MM)


def annotate_board(board: pcbnew.BOARD) -> None:
    text = pcbnew.PCB_TEXT(board)
    text.SetText("MycoMIDI ADS131M08 8ch prototype")
    text.SetLayer(pcbnew.F_SilkS)
    text.SetTextHeight(mm(1.2))
    text.SetTextWidth(mm(1.2))
    text.SetPosition(point(16.0, 31.0))
    board.Add(text)


def build_board(netlist_path: Path, output_path: Path) -> None:
    netlist = NetlistData(netlist_path)
    netlist.load()

    board = pcbnew.BOARD()
    add_outline(board)
    nets = create_nets(board, netlist.net_names)
    footprints = place_components(board, netlist, nets)
    route_analog_inputs(board, nets, footprints)
    route_digital_headers(board, nets, footprints)
    route_power_and_caps(board, nets, footprints)
    annotate_board(board)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    pcbnew.SaveBoard(str(output_path), board)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build the placed+routed ADS131M08 KiCad PCB from the generated SKiDL netlist.")
    parser.add_argument("--netlist", type=Path, default=ADC_BOARD_NETLIST)
    parser.add_argument("--output", type=Path, default=ADC_BOARD_PCB)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.netlist.exists():
        print(f"error: netlist does not exist: {args.netlist}", file=sys.stderr)
        return 66
    build_board(args.netlist, args.output)
    print(f"Wrote {args.output}")
    print("Placed U1 centrally, J1 on the left edge, J2/J3 on the right edge, and routed analog/power/SPI support traces.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
