from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import pcbnew
from shapely import affinity
from shapely.geometry import GeometryCollection, LineString, MultiPolygon, Point, Polygon, box
from shapely.ops import nearest_points, unary_union

from curves.dragon_curve import dragon_curve_points
from curves.gosper_curve import gosper_curve_points
from curves.koch_snowflake_curve import koch_snowflake_points
from curves.sierpinski_arrowhead_curve import sierpinski_arrowhead_curve_points
from curves.selfavoiding_maze_path import self_avoiding_maze_path_points

CLEARANCE_MM = 0.25
EDGE_INSET_MM = 1.5
COURTYARD_MARGIN_MM = 0.1
TEXTURE_TILE_MM = 5.4
TEXTURE_SPACING_X_MM = 10.4
TEXTURE_SPACING_Y_MM = 9.0
TEXTURE_LINE_WIDTH_MM = 0.5
TEXTURE_EDGE_MARGIN_MM = 4.8
MIN_COMPONENT_AREA_MM2 = 3.0
ARC_RESOLUTION = 6
POLYGON_ERROR_MM = 0.02
ZONE_NAME = "adc-fractal-fill-texture-front"


@dataclass(frozen=True)
class TextureStats:
    gnd_net_id: int
    pads: int
    tracks: int
    vias: int
    footprints: int
    anchor_vias: int
    kept_components: int
    outer_rings: int
    holes: int
    exposed_area_mm2: float
    maze_motif: str


def _from_mm(value: float) -> int:
    return pcbnew.FromMM(value)


def _to_mm(value: int) -> float:
    return float(pcbnew.ToMM(value))


def _iter_polygons(geometry) -> Iterable[Polygon]:
    if geometry.is_empty:
        return
    if isinstance(geometry, Polygon):
        yield geometry
        return
    if isinstance(geometry, MultiPolygon):
        for polygon in geometry.geoms:
            if not polygon.is_empty:
                yield polygon
        return
    if isinstance(geometry, GeometryCollection):
        for item in geometry.geoms:
            yield from _iter_polygons(item)
        return
    raise TypeError(f"unsupported geometry type {geometry.geom_type}")


def _shape_line_to_coords(chain: pcbnew.SHAPE_LINE_CHAIN) -> list[tuple[float, float]]:
    coords: list[tuple[float, float]] = []
    for index in range(chain.PointCount()):
        point = chain.CPoint(index)
        coords.append((_to_mm(point.x), _to_mm(point.y)))
    if coords and coords[0] == coords[-1]:
        coords.pop()
    return coords


def _polyset_to_shapely(polyset: pcbnew.SHAPE_POLY_SET):
    polygons: list[Polygon] = []
    for outline_index in range(polyset.OutlineCount()):
        outer = _shape_line_to_coords(polyset.Outline(outline_index))
        if len(outer) < 3:
            continue
        holes = [
            _shape_line_to_coords(polyset.Hole(outline_index, hole_index))
            for hole_index in range(polyset.HoleCount(outline_index))
        ]
        polygon = Polygon(outer, [hole for hole in holes if len(hole) >= 3])
        if not polygon.is_valid:
            polygon = polygon.buffer(0)
        if not polygon.is_empty:
            polygons.append(polygon)
    if not polygons:
        return GeometryCollection()
    return unary_union(polygons)


def _chain_from_ring(coords: Iterable[tuple[float, float]]) -> pcbnew.SHAPE_LINE_CHAIN:
    chain = pcbnew.SHAPE_LINE_CHAIN()
    ring = list(coords)
    if ring and ring[0] == ring[-1]:
        ring = ring[:-1]
    for x, y in ring:
        chain.Append(_from_mm(x), _from_mm(y), False)
    chain.SetClosed(True)
    return chain


def _polyset_from_shapely(geometry) -> pcbnew.SHAPE_POLY_SET:
    polyset = pcbnew.SHAPE_POLY_SET()
    for polygon in _iter_polygons(geometry):
        if polygon.area < MIN_COMPONENT_AREA_MM2:
            continue
        outline_index = polyset.AddOutline(_chain_from_ring(polygon.exterior.coords))
        for hole in polygon.interiors:
            if Polygon(hole.coords).area < 0.05:
                continue
            polyset.AddHole(_chain_from_ring(hole.coords), outline_index)
    polyset.NormalizeAreaOutlines()
    return polyset


def _bbox_polygon(bbox: pcbnew.BOX2I) -> Polygon:
    return box(
        _to_mm(bbox.GetLeft()),
        _to_mm(bbox.GetTop()),
        _to_mm(bbox.GetRight()),
        _to_mm(bbox.GetBottom()),
    )


def _board_outline(board: pcbnew.BOARD):
    polyset = pcbnew.SHAPE_POLY_SET()
    if not board.GetBoardPolygonOutlines(polyset, None, True, False):
        raise ValueError("failed to read board Edge.Cuts outline")
    outline = _polyset_to_shapely(polyset).buffer(0)
    if outline.is_empty:
        raise ValueError("board outline geometry is empty")
    return outline


def _exact_pad_polygon(pad: pcbnew.PAD, clearance_mm: float):
    polyset = pcbnew.SHAPE_POLY_SET()
    pad.TransformShapeToPolygon(
        polyset,
        pcbnew.F_Cu,
        _from_mm(clearance_mm),
        _from_mm(POLYGON_ERROR_MM),
        pcbnew.ERROR_INSIDE,
        False,
    )
    return _polyset_to_shapely(polyset)


def _courtyard_or_body_polygon(footprint: pcbnew.FOOTPRINT):
    courtyard = footprint.GetCourtyard(pcbnew.F_CrtYd)
    if courtyard.OutlineCount() > 0:
        polygon = _polyset_to_shapely(courtyard)
        if not polygon.is_empty:
            return polygon.buffer(COURTYARD_MARGIN_MM, cap_style=1, join_style=1, resolution=ARC_RESOLUTION)
    return _bbox_polygon(footprint.GetBoundingBox(False, False)).buffer(COURTYARD_MARGIN_MM, cap_style=1, join_style=1, resolution=ARC_RESOLUTION)


def _track_polygon(track: pcbnew.PCB_TRACK, clearance_mm: float):
    start = (_to_mm(track.GetStartX()), _to_mm(track.GetStartY()))
    end = (_to_mm(track.GetEndX()), _to_mm(track.GetEndY()))
    width = _to_mm(track.GetWidth()) / 2.0 + clearance_mm
    return LineString([start, end]).buffer(width, cap_style=1, join_style=1, resolution=ARC_RESOLUTION)


def _via_polygon(via: pcbnew.PCB_VIA, clearance_mm: float):
    center = (_to_mm(via.GetX()), _to_mm(via.GetY()))
    radius = _to_mm(via.GetWidth(pcbnew.F_Cu)) / 2.0 + clearance_mm
    return Point(center).buffer(radius, resolution=ARC_RESOLUTION)


def _gnd_anchor_disks(board: pcbnew.BOARD, gnd_net_name: str):
    anchors: list[Polygon] = []
    for item in board.GetTracks():
        if not isinstance(item, pcbnew.PCB_VIA):
            continue
        if item.GetNetname() != gnd_net_name:
            continue
        center = (_to_mm(item.GetX()), _to_mm(item.GetY()))
        radius = _to_mm(item.GetWidth(pcbnew.F_Cu)) / 2.0 + 0.05
        anchors.append(Point(center).buffer(radius, resolution=ARC_RESOLUTION))
    return anchors


def _obstacle_union(board: pcbnew.BOARD, gnd_net_name: str, clearance_mm: float):
    geometries = []
    pads = 0
    tracks = 0
    vias = 0
    footprints = 0
    for footprint in board.GetFootprints():
        footprints += 1
        geometries.append(_courtyard_or_body_polygon(footprint))
        for pad in footprint.Pads():
            pads += 1
            if pad.GetNetname() == gnd_net_name:
                continue
            geometries.append(_exact_pad_polygon(pad, clearance_mm))
    for item in board.GetTracks():
        if isinstance(item, pcbnew.PCB_VIA):
            vias += 1
            if item.GetNetname() == gnd_net_name:
                continue
            geometries.append(_via_polygon(item, clearance_mm))
            continue
        tracks += 1
        if item.GetNetname() == gnd_net_name:
            continue
        geometries.append(_track_polygon(item, clearance_mm))
    return unary_union(geometries).buffer(0), pads, tracks, vias, footprints


def _centered_points(points: Iterable[tuple[float, float]]) -> list[tuple[float, float]]:
    normalized = list(points)
    if not normalized:
        raise ValueError("curve point list cannot be empty")
    return [(x - 0.5, y - 0.5) for x, y in normalized]


def _line_motif(points: Iterable[tuple[float, float]], *, tile_mm: float, width_mm: float, rotation_deg: float):
    line = LineString(_centered_points(points))
    geometry = line.buffer(width_mm / 2.0, cap_style=1, join_style=1, resolution=ARC_RESOLUTION)
    geometry = affinity.scale(geometry, xfact=tile_mm, yfact=tile_mm, origin=(0.0, 0.0))
    if rotation_deg:
        geometry = affinity.rotate(geometry, rotation_deg, origin=(0.0, 0.0))
    return geometry


def _filled_motif(points: Iterable[tuple[float, float]], *, tile_mm: float, rotation_deg: float):
    polygon = Polygon(_centered_points(points))
    geometry = affinity.scale(polygon, xfact=tile_mm, yfact=tile_mm, origin=(0.0, 0.0))
    if rotation_deg:
        geometry = affinity.rotate(geometry, rotation_deg, origin=(0.0, 0.0))
    return geometry


def _motif_geometry(row: int, column: int):
    variant = (row + (2 * column)) % 4
    if variant == 0:
        return _filled_motif(
            koch_snowflake_points(1, anti=True),
            tile_mm=TEXTURE_TILE_MM,
            rotation_deg=30.0 * ((row + column) % 6),
        )
    if variant == 1:
        return _line_motif(
            gosper_curve_points(2),
            tile_mm=TEXTURE_TILE_MM,
            width_mm=TEXTURE_LINE_WIDTH_MM,
            rotation_deg=60.0 * (column % 6),
        )
    if variant == 2:
        return _line_motif(
            dragon_curve_points(8),
            tile_mm=TEXTURE_TILE_MM * 0.94,
            width_mm=TEXTURE_LINE_WIDTH_MM * 0.9,
            rotation_deg=45.0 * ((row + column) % 4),
        )
    raw = self_avoiding_maze_path_points(4, 4, seed=(row + column) % 2)
    sampled = raw[::2]
    if sampled[-1] != raw[-1]:
        sampled.append(raw[-1])
    min_raw_x = min(x for x, _ in sampled)
    max_raw_x = max(x for x, _ in sampled)
    min_raw_y = min(y for _, y in sampled)
    max_raw_y = max(y for _, y in sampled)
    span_x = max_raw_x - min_raw_x
    span_y = max_raw_y - min_raw_y
    normalized = [((x - min_raw_x) / span_x, (y - min_raw_y) / span_y) for x, y in sampled]
    return _line_motif(
        normalized,
        tile_mm=TEXTURE_TILE_MM,
        width_mm=TEXTURE_LINE_WIDTH_MM * 0.85,
        rotation_deg=90.0 * ((row + column) % 4),
    )


def _decorative_cutouts(board_interior):
    min_x, min_y, max_x, max_y = board_interior.bounds
    safe_interior = board_interior.buffer(-TEXTURE_EDGE_MARGIN_MM)
    motif_parts = []
    row_step = TEXTURE_SPACING_Y_MM
    col_step = TEXTURE_SPACING_X_MM
    max_rows = int(((max_y - min_y) / row_step) + 3)
    max_cols = int(((max_x - min_x) / col_step) + 3)
    for row in range(max_rows):
        center_y = min_y + (row * row_step)
        if center_y > max_y:
            continue
        x_offset = (col_step / 2.0) if (row % 2) else 0.0
        for column in range(max_cols):
            center_x = min_x + x_offset + (column * col_step)
            if center_x > max_x:
                continue
            motif = affinity.translate(_motif_geometry(row, column), xoff=center_x, yoff=center_y)
            if safe_interior.is_empty or not motif.within(safe_interior):
                continue
            motif = motif.buffer(0)
            if motif.is_empty or motif.area < MIN_COMPONENT_AREA_MM2:
                continue
            motif_parts.append(motif)
    if not motif_parts:
        raise ValueError("decorative cutout placement produced no motifs")
    return unary_union(motif_parts).intersection(board_interior).buffer(0)


def _bridge_anchors(geometry, anchors: list[Polygon], obstacle_union, board_interior):
    del obstacle_union
    significant = [polygon for polygon in _iter_polygons(geometry) if polygon.area >= MIN_COMPONENT_AREA_MM2]
    if not significant:
        raise ValueError("continuous texture did not produce any usable polygons")
    major_geometry = unary_union(significant).buffer(0)
    bridge_parts = []
    anchored_major = False
    for anchor in anchors:
        if major_geometry.intersects(anchor):
            bridge_parts.append(anchor)
            anchored_major = True
            continue
        if major_geometry.distance(anchor) > 8.0:
            continue
        anchor_point, texture_point = nearest_points(anchor, major_geometry)
        bridge = LineString([anchor_point.coords[0], texture_point.coords[0]]).buffer(0.22, cap_style=1, join_style=1, resolution=ARC_RESOLUTION)
        if not bridge.within(board_interior):
            continue
        bridge_parts.extend((anchor, bridge))
        anchored_major = True
    if not anchored_major:
        raise ValueError("continuous texture did not connect to any existing GND vias")
    return unary_union([geometry, *bridge_parts]).buffer(0)


def _keep_anchored_components(geometry, anchors: list[Polygon]):
    kept: list[Polygon] = []
    for polygon in _iter_polygons(geometry):
        if polygon.area < MIN_COMPONENT_AREA_MM2:
            continue
        if any(polygon.intersects(anchor) for anchor in anchors):
            kept.append(polygon)
    if not kept:
        raise ValueError("continuous texture did not connect to any existing GND vias")
    return unary_union(kept).buffer(0)


def _remove_generated_art(board: pcbnew.BOARD) -> None:
    old_zones = [zone for zone in board.Zones() if zone.GetZoneName().startswith("adc-fractal-fill")]
    for zone in old_zones:
        board.Remove(zone)
    old_mask_polys = [
        drawing
        for drawing in board.GetDrawings()
        if isinstance(drawing, pcbnew.PCB_SHAPE)
        and drawing.GetShape() == pcbnew.S_POLYGON
        and drawing.GetLayerName() in {"F.Mask", "B.Mask"}
    ]
    for drawing in old_mask_polys:
        board.Remove(drawing)


def _add_zone(board: pcbnew.BOARD, geometry, *, gnd_net_id: int, clearance_mm: float, min_thickness_mm: float) -> pcbnew.ZONE:
    zone = pcbnew.ZONE(board)
    zone.SetLayer(pcbnew.F_Cu)
    zone.SetNetCode(gnd_net_id)
    zone.SetZoneName(ZONE_NAME)
    zone.SetLocalClearance(_from_mm(clearance_mm))
    zone.SetMinThickness(_from_mm(min_thickness_mm))
    zone.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
    zone.SetThermalReliefGap(_from_mm(0.5))
    zone.SetThermalReliefSpokeWidth(_from_mm(0.5))
    zone.SetAssignedPriority(1)
    zone.SetOutline(_polyset_from_shapely(geometry))
    board.Add(zone)
    return zone


def _count_holes(geometry) -> int:
    return sum(len(polygon.interiors) for polygon in _iter_polygons(geometry))


def _count_outer_rings(geometry) -> int:
    return sum(1 for _ in _iter_polygons(geometry))


def apply_continuous_texture_fill(
    board_path: Path,
    *,
    gnd_net_name: str,
    clearance_mm: float = CLEARANCE_MM,
    edge_inset_mm: float = EDGE_INSET_MM,
    min_thickness_mm: float = 0.1,
) -> TextureStats:
    board = pcbnew.LoadBoard(str(board_path))
    gnd_net = board.FindNet(gnd_net_name)
    if gnd_net is None:
        raise ValueError(f"board does not define net {gnd_net_name!r}")

    board_interior = _board_outline(board).buffer(-edge_inset_mm)
    if board_interior.is_empty:
        raise ValueError("board interior vanished after edge inset")

    obstacle_union, pads, tracks, vias, footprints = _obstacle_union(board, gnd_net_name, clearance_mm)
    anchors = [anchor.intersection(board_interior) for anchor in _gnd_anchor_disks(board, gnd_net_name)]
    anchors = [anchor for anchor in anchors if not anchor.is_empty]
    if not anchors:
        raise ValueError("no existing GND vias available to anchor decorative copper")

    decorative_cutouts = _decorative_cutouts(board_interior)
    carved = board_interior.difference(unary_union([obstacle_union, decorative_cutouts])).buffer(0)
    final_geometry = _bridge_anchors(carved, anchors, obstacle_union, board_interior).intersection(board_interior).buffer(0)
    if final_geometry.is_empty:
        raise ValueError("continuous texture fill produced no valid copper area")

    _remove_generated_art(board)
    _add_zone(board, final_geometry, gnd_net_id=gnd_net.GetNetCode(), clearance_mm=clearance_mm, min_thickness_mm=min_thickness_mm)

    filler = pcbnew.ZONE_FILLER(board)
    filler.Fill(board.Zones())
    pcbnew.SaveBoard(str(board_path), board)

    return TextureStats(
        gnd_net_id=gnd_net.GetNetCode(),
        pads=pads,
        tracks=tracks,
        vias=vias,
        footprints=footprints,
        anchor_vias=len(anchors),
        kept_components=_count_outer_rings(final_geometry),
        outer_rings=_count_outer_rings(final_geometry),
        holes=_count_holes(final_geometry),
        exposed_area_mm2=final_geometry.area,
        maze_motif=(
            "tiled local selfavoiding_maze_path motifs mixed with Koch, Gosper, Dragon, "
            "and Sierpinski cutouts inside one continuous copper background"
        ),
    )
