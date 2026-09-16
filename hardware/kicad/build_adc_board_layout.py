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

BOARD_WIDTH_MM = 92.0
BOARD_HEIGHT_MM = 70.0
OUTLINE_MARGIN_MM = 2.0
FOOTPRINT_LIB_ROOT = Path("/usr/share/kicad/footprints")
SCRIPT_DIR = Path(__file__).resolve().parent
ADC_BOARD_NETLIST = SCRIPT_DIR / "adc_board" / "adc_board_8ch.net"
ADC_BOARD_PCB = SCRIPT_DIR / "adc_board" / "adc_board_8ch.kicad_pcb"
ADC_BOARD_DSN = SCRIPT_DIR / "adc_board" / "adc_board_8ch.dsn"


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
    "U1": Placement(44.0, 44.0, 0.0),
    "J1": Placement(8.0, 12.0, 0.0),
    "J2": Placement(82.0, 10.0, 0.0),
    "J3": Placement(82.0, 32.0, 0.0),
    "C1": Placement(49.0, 58.0, 0.0),
    "C2": Placement(53.0, 58.0, 0.0),
    "C3": Placement(49.0, 30.0, 0.0),
    "C4": Placement(53.0, 30.0, 0.0),
    "C5": Placement(60.0, 49.5, 90.0),
    "C6": Placement(60.0, 38.5, 90.0),
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

def annotate_board(board: pcbnew.BOARD) -> None:
    return None


def build_board(netlist_path: Path, output_path: Path, dsn_output_path: Path | None = None) -> None:
    netlist = NetlistData(netlist_path)
    netlist.load()

    board = pcbnew.BOARD()
    add_outline(board)
    nets = create_nets(board, netlist.net_names)
    place_components(board, netlist, nets)
    annotate_board(board)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    pcbnew.SaveBoard(str(output_path), board)
    if dsn_output_path is not None:
        dsn_output_path.parent.mkdir(parents=True, exist_ok=True)
        if not pcbnew.ExportSpecctraDSN(board, str(dsn_output_path)):
            raise RuntimeError(f"Failed to export Specctra DSN to {dsn_output_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build the placed ADS131M08 KiCad PCB from the generated SKiDL netlist.")
    parser.add_argument("--netlist", type=Path, default=ADC_BOARD_NETLIST)
    parser.add_argument("--output", type=Path, default=ADC_BOARD_PCB)
    parser.add_argument("--dsn-output", type=Path, default=ADC_BOARD_DSN)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.netlist.exists():
        print(f"error: netlist does not exist: {args.netlist}", file=sys.stderr)
        return 66
    build_board(args.netlist, args.output, args.dsn_output)
    print(f"Wrote {args.output}")
    if args.dsn_output is not None:
        print(f"Wrote {args.dsn_output}")
    print("Placed U1 centrally, J1 on the left edge, J2/J3 on the right edge, and exported a Freerouting-ready DSN.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
