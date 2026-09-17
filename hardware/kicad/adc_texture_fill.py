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
from fractal_geometry import generate_peano_points

CLEARANCE_MM = 0.25
EDGE_INSET_MM = 1.5
COURTYARD_MARGIN_MM = 0.1
TEXTURE_TILE_MM = 6.4
TEXTURE_TILE_FINE_MM = 4.5
TEXTURE_SPACING_X_MM = 5.9
TEXTURE_SPACING_Y_MM = 5.4
TEXTURE_FINE_SPACING_X_MM = 7.0
TEXTURE_FINE_SPACING_Y_MM = 6.4
TEXTURE_LINE_WIDTH_MM = 0.24
TEXTURE_LINE_WIDTH_FINE_MM = 0.15
TEXTURE_BACKBONE_WIDTH_MM = 0.3
TEXTURE_EDGE_MARGIN_MM = 2.4
MIN_COMPONENT_AREA_MM2 = 0.8
MIN_LINE_COMPONENT_AREA_MM2 = 0.04
MIN_MOTIF_AREA_MM2 = 0.18
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
    available_area_mm2: float
    copper_coverage_ratio: float
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


def _polyset_from_shapely(geometry, *, min_area_mm2: float = MIN_LINE_COMPONENT_AREA_MM2, min_hole_mm2: float = 0.05) -> pcbnew.SHAPE_POLY_SET:
    polyset = pcbnew.SHAPE_POLY_SET()
    for polygon in _iter_polygons(geometry):
        if polygon.area < min_area_mm2:
            continue
        outline_index = polyset.AddOutline(_chain_from_ring(polygon.exterior.coords))
        for hole in polygon.interiors:
            if Polygon(hole.coords).area < min_hole_mm2:
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
    for footprint in board.GetFootprints():
        for pad in footprint.Pads():
            if pad.GetNetname() != gnd_net_name:
                continue
            pad_anchor = _exact_pad_polygon(pad, 0.05).buffer(0)
            if not pad_anchor.is_empty:
                anchors.append(pad_anchor)
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


def _normalize_points(points: Iterable[tuple[float, float]]) -> list[tuple[float, float]]:
    raw = list(points)
    if not raw:
        raise ValueError("curve point list cannot be empty")
    min_x = min(x for x, _ in raw)
    max_x = max(x for x, _ in raw)
    min_y = min(y for _, y in raw)
    max_y = max(y for _, y in raw)
    span_x = max(max_x - min_x, 1e-6)
    span_y = max(max_y - min_y, 1e-6)
    return [((x - min_x) / span_x, (y - min_y) / span_y) for x, y in raw]


def _line_motif(points: Iterable[tuple[float, float]], *, tile_mm: float, width_mm: float, rotation_deg: float):
    line = LineString(_centered_points(points))
    geometry = line.buffer(width_mm / 2.0, cap_style=1, join_style=1, resolution=ARC_RESOLUTION)
    geometry = affinity.scale(geometry, xfact=tile_mm, yfact=tile_mm, origin=(0.0, 0.0))
    if rotation_deg:
        geometry = affinity.rotate(geometry, rotation_deg, origin=(0.0, 0.0))
    return geometry


def _ring_motif(points: Iterable[tuple[float, float]], *, tile_mm: float, width_mm: float, rotation_deg: float):
    polygon = Polygon(_centered_points(points))
    geometry = LineString(list(polygon.exterior.coords)).buffer(
        width_mm / 2.0,
        cap_style=1,
        join_style=1,
        resolution=ARC_RESOLUTION,
    )
    geometry = affinity.scale(geometry, xfact=tile_mm, yfact=tile_mm, origin=(0.0, 0.0))
    if rotation_deg:
        geometry = affinity.rotate(geometry, rotation_deg, origin=(0.0, 0.0))
    return geometry


def _map_curve_points(
    points: Iterable[tuple[float, float]],
    *,
    start: tuple[float, float],
    end: tuple[float, float],
    spread_mm: float,
) -> list[tuple[float, float]]:
    normalized = _normalize_points(points)
    if len(normalized) < 2:
        raise ValueError("curve requires multiple points")
    if math.isclose(spread_mm, 0.0, abs_tol=1e-12):
        raise ValueError("spread must be non-zero")

    sx, sy = start
    ex, ey = end
    dx = ex - sx
    dy = ey - sy
    length = math.hypot(dx, dy)
    if length <= 0:
        raise ValueError("start and end must differ")

    first_x, first_y = normalized[0]
    last_x, last_y = normalized[-1]
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
    for x, y in normalized:
        rel_x = x - first_x
        rel_y = y - first_y
        u = (rel_x * base_tangent[0] + rel_y * base_tangent[1]) / base_length
        v = rel_x * base_normal[0] + rel_y * base_normal[1]
        relative_points.append((u, v))

    first_v = relative_points[0][1]
    last_v = relative_points[-1][1]
    adjusted_points = []
    for u, v in relative_points:
        baseline_v = ((1.0 - u) * first_v) + (u * last_v)
        adjusted_points.append((u, v - baseline_v))

    normal_values = [v for _, v in adjusted_points]
    min_v = min(normal_values)
    max_v = max(normal_values)
    scale_v = max(max_v - min_v, 1e-6)
    spread_sign = 1.0 if spread_mm > 0 else -1.0
    spread_abs = abs(spread_mm)

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


def _curve_band(
    points: Iterable[tuple[float, float]],
    *,
    start: tuple[float, float],
    end: tuple[float, float],
    spread_mm: float,
    width_mm: float,
) -> Polygon:
    line = LineString(_map_curve_points(points, start=start, end=end, spread_mm=spread_mm))
    return line.buffer(width_mm / 2.0, cap_style=1, join_style=1, resolution=ARC_RESOLUTION)


def _cell_noise(row: int, column: int, salt: int) -> float:
    value = ((row + 1) * 92_837) + ((column + 1) * 68_917) + (salt * 12_347)
    return float(value % 1000) / 1000.0


def _cell_rotation(row: int, column: int, salt: int, step_deg: float) -> float:
    return step_deg * int(_cell_noise(row, column, salt) * (360.0 / step_deg))


def _cell_scale(base_mm: float, row: int, column: int, salt: int, *, span: float = 0.22) -> float:
    return base_mm * (1.0 - (span / 2.0) + (_cell_noise(row, column, salt) * span))


def _motif_geometry(row: int, column: int):
    variant = ((row * 3) + (column * 5) + int(_cell_noise(row, column, 9) * 11.0)) % 10
    if variant == 0:
        raw = self_avoiding_maze_path_points(6, 6, seed=((row * 7) + column) % 17)
        return _line_motif(
            _normalize_points(raw),
            tile_mm=_cell_scale(TEXTURE_TILE_MM * 1.15, row, column, 1),
            width_mm=TEXTURE_LINE_WIDTH_MM,
            rotation_deg=_cell_rotation(row, column, 2, 15.0),
        )
    if variant == 1:
        return _line_motif(
            gosper_curve_points(3),
            tile_mm=_cell_scale(TEXTURE_TILE_MM * 1.02, row, column, 3),
            width_mm=TEXTURE_LINE_WIDTH_FINE_MM,
            rotation_deg=_cell_rotation(row, column, 4, 10.0),
        )
    if variant == 2:
        return _line_motif(
            dragon_curve_points(9),
            tile_mm=_cell_scale(TEXTURE_TILE_MM * 0.94, row, column, 5),
            width_mm=TEXTURE_LINE_WIDTH_FINE_MM,
            rotation_deg=_cell_rotation(row, column, 6, 7.5),
        )
    if variant == 3:
        return _line_motif(
            sierpinski_arrowhead_curve_points(6),
            tile_mm=_cell_scale(TEXTURE_TILE_MM * 0.98, row, column, 7),
            width_mm=TEXTURE_LINE_WIDTH_FINE_MM,
            rotation_deg=_cell_rotation(row, column, 8, 10.0),
        )
    if variant == 4:
        return _ring_motif(
            koch_snowflake_points(4, anti=False),
            tile_mm=_cell_scale(TEXTURE_TILE_MM * 1.02, row, column, 10),
            width_mm=TEXTURE_LINE_WIDTH_FINE_MM,
            rotation_deg=_cell_rotation(row, column, 11, 10.0),
        )
    if variant == 5:
        return _line_motif(
            _normalize_points(generate_peano_points(2)),
            tile_mm=_cell_scale(TEXTURE_TILE_FINE_MM * 1.3, row, column, 12),
            width_mm=TEXTURE_LINE_WIDTH_FINE_MM,
            rotation_deg=_cell_rotation(row, column, 13, 6.0),
        )
    if variant == 6:
        return _line_motif(
            dragon_curve_points(8),
            tile_mm=_cell_scale(TEXTURE_TILE_FINE_MM * 1.16, row, column, 14),
            width_mm=TEXTURE_LINE_WIDTH_FINE_MM * 0.92,
            rotation_deg=_cell_rotation(row, column, 15, 5.0),
        )
    if variant == 7:
        raw = self_avoiding_maze_path_points(5, 5, seed=((row * 11) + column) % 13)
        sampled = raw[::2]
        if sampled[-1] != raw[-1]:
            sampled.append(raw[-1])
        return _line_motif(
            _normalize_points(sampled),
            tile_mm=_cell_scale(TEXTURE_TILE_MM * 0.92, row, column, 16),
            width_mm=TEXTURE_LINE_WIDTH_FINE_MM,
            rotation_deg=_cell_rotation(row, column, 17, 7.5),
        )
    if variant == 8:
        return _ring_motif(
            koch_snowflake_points(4, anti=True),
            tile_mm=_cell_scale(TEXTURE_TILE_FINE_MM * 1.08, row, column, 18),
            width_mm=TEXTURE_LINE_WIDTH_FINE_MM * 0.94,
            rotation_deg=_cell_rotation(row, column, 19, 12.0),
        )
    peano = generate_peano_points(2)
    return _line_motif(
        _normalize_points(peano[::2] + [peano[-1]]),
        tile_mm=_cell_scale(TEXTURE_TILE_FINE_MM * 1.08, row, column, 20),
        width_mm=TEXTURE_LINE_WIDTH_FINE_MM * 0.9,
        rotation_deg=_cell_rotation(row, column, 21, 9.0),
    )


def _motif_layer(
    board_interior,
    *,
    spacing_x_mm: float,
    spacing_y_mm: float,
    row_phase: int,
    x_shift_mm: float = 0.0,
    y_shift_mm: float = 0.0,
    density: float = 1.0,
):
    min_x, min_y, max_x, max_y = board_interior.bounds
    safe_interior = board_interior.buffer(-TEXTURE_EDGE_MARGIN_MM)
    motif_parts = []
    row_step = spacing_y_mm
    col_step = spacing_x_mm
    max_rows = int(((max_y - min_y) / row_step) + 3)
    max_cols = int(((max_x - min_x) / col_step) + 3)
    for row in range(max_rows):
        center_y = min_y + y_shift_mm + (row * row_step)
        if center_y > max_y:
            continue
        x_offset = (col_step / 2.0) if (row % 2) else 0.0
        for column in range(max_cols):
            if _cell_noise(row + row_phase, column, 23) > density:
                continue
            jitter_x = (_cell_noise(row + row_phase, column, 21) - 0.5) * (col_step * 0.48)
            jitter_y = (_cell_noise(row + row_phase, column, 22) - 0.5) * (row_step * 0.48)
            center_x = min_x + x_shift_mm + x_offset + (column * col_step) + jitter_x
            center_y_jittered = center_y + jitter_y
            if center_x > max_x:
                continue
            motif = affinity.translate(
                _motif_geometry(row + row_phase, column + (row_phase * 3)),
                xoff=center_x,
                yoff=center_y_jittered,
            )
            if safe_interior.is_empty:
                continue
            motif = motif.intersection(safe_interior).buffer(0)
            if motif.is_empty or motif.area < MIN_MOTIF_AREA_MM2:
                continue
            motif_parts.append(motif)
    return motif_parts


def _weave_backbone_layer(board_interior):
    safe_interior = board_interior.buffer(-TEXTURE_EDGE_MARGIN_MM)
    if safe_interior.is_empty:
        return []
    min_x, min_y, max_x, max_y = safe_interior.bounds
    height = max_y - min_y
    width = max_x - min_x
    parts = []

    band_count = max(5, int(height / 9.8))
    y_step = height / float(band_count + 1)
    for band_index in range(band_count):
        center_y = min_y + ((band_index + 1) * y_step)
        sweep_tilt = (_cell_noise(band_index, 0, 41) - 0.5) * 1.1
        spread_mm = min(2.8, y_step * 0.56)
        maze = self_avoiding_maze_path_points(11 + (band_index % 3), 5 + (band_index % 2), seed=band_index + 23)
        parts.append(
            _curve_band(
                maze,
                start=(min_x + 0.8, center_y),
                end=(max_x - 0.8, center_y + sweep_tilt),
                spread_mm=spread_mm if band_index % 2 == 0 else -spread_mm,
                width_mm=TEXTURE_BACKBONE_WIDTH_MM,
            ).intersection(safe_interior)
        )

    diagonal_specs = (
        (gosper_curve_points(3), (min_x + 2.0, min_y + (height * 0.18)), (max_x - 4.0, min_y + (height * 0.56)), 2.8),
        (dragon_curve_points(9), (min_x + 5.0, min_y + (height * 0.76)), (max_x - 2.0, min_y + (height * 0.22)), -2.6),
        (_normalize_points(generate_peano_points(2)), (min_x + 5.0, min_y + (height * 0.48)), (max_x - 5.0, min_y + (height * 0.84)), 2.1),
        (sierpinski_arrowhead_curve_points(6), (min_x + (width * 0.1), max_y - 4.0), (min_x + (width * 0.58), min_y + 4.2), -1.9),
    )
    for points, start, end, spread in diagonal_specs:
        parts.append(
            _curve_band(
                points,
                start=start,
                end=end,
                spread_mm=spread,
                width_mm=TEXTURE_LINE_WIDTH_FINE_MM,
            ).intersection(safe_interior)
        )
    return [part.buffer(0) for part in parts if not part.is_empty]


def _line_art_geometry(board_interior, anchors: list[Polygon]):
    del anchors
    motif_parts = _weave_backbone_layer(board_interior)
    motif_parts.extend(
        _motif_layer(
            board_interior,
            spacing_x_mm=TEXTURE_SPACING_X_MM,
            spacing_y_mm=TEXTURE_SPACING_Y_MM,
            row_phase=0,
            density=0.74,
        )
    )
    motif_parts.extend(
        _motif_layer(
            board_interior,
            spacing_x_mm=TEXTURE_FINE_SPACING_X_MM,
            spacing_y_mm=TEXTURE_FINE_SPACING_Y_MM,
            row_phase=11,
            x_shift_mm=TEXTURE_FINE_SPACING_X_MM * 0.42,
            y_shift_mm=TEXTURE_FINE_SPACING_Y_MM * 0.36,
            density=0.34,
        )
    )
    if not motif_parts:
        raise ValueError("line-art placement produced no motifs")
    return unary_union(motif_parts).intersection(board_interior).buffer(0)


def _anchor_spokes(anchors: list[Polygon], board_interior):
    parts = []
    for anchor in anchors:
        center = anchor.centroid.coords[0]
        for dx, dy in ((-7.0, 0.0), (7.0, 0.0), (0.0, -7.0), (0.0, 7.0)):
            spoke = LineString([center, (center[0] + dx, center[1] + dy)]).buffer(
                TEXTURE_LINE_WIDTH_FINE_MM * 0.9,
                cap_style=1,
                join_style=1,
                resolution=ARC_RESOLUTION,
            )
            parts.append(spoke.intersection(board_interior))
    return [part.buffer(0) for part in parts if not part.is_empty]


def _bridge_anchors(geometry, anchors: list[Polygon], obstacle_union, board_interior):
    significant = [polygon for polygon in _iter_polygons(geometry) if polygon.area >= MIN_LINE_COMPONENT_AREA_MM2]
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
        if major_geometry.distance(anchor) > 30.0:
            continue
        anchor_point, texture_point = nearest_points(anchor, major_geometry)
        bridge = LineString([anchor_point.coords[0], texture_point.coords[0]]).buffer(0.22, cap_style=1, join_style=1, resolution=ARC_RESOLUTION)
        if not bridge.within(board_interior):
            continue
        if not obstacle_union.is_empty and bridge.intersects(obstacle_union):
            continue
        bridge_parts.extend((anchor, bridge))
        anchored_major = True
    if not anchored_major:
        raise ValueError("continuous texture did not connect to any existing GND vias")
    return unary_union([geometry, *bridge_parts]).buffer(0)


def _stitch_components(geometry, obstacle_union, board_interior):
    polygons = [polygon for polygon in _iter_polygons(geometry) if polygon.area >= MIN_LINE_COMPONENT_AREA_MM2]
    if len(polygons) < 2:
        return geometry

    stitched = max(polygons, key=lambda polygon: polygon.area)
    remaining = [polygon for polygon in polygons if polygon is not stitched]
    bridge_parts = []
    while remaining:
        candidate = min(remaining, key=lambda polygon: stitched.distance(polygon))
        distance_mm = stitched.distance(candidate)
        if distance_mm > 6.5:
            break
        stitched_point, candidate_point = nearest_points(stitched, candidate)
        bridge = LineString([stitched_point.coords[0], candidate_point.coords[0]]).buffer(
            TEXTURE_LINE_WIDTH_FINE_MM * 0.9,
            cap_style=1,
            join_style=1,
            resolution=ARC_RESOLUTION,
        )
        if bridge.within(board_interior) and (obstacle_union.is_empty or not bridge.intersects(obstacle_union)):
            bridge_parts.append(bridge)
            stitched = unary_union([stitched, candidate, bridge]).buffer(0)
        remaining.remove(candidate)
    if not bridge_parts:
        return geometry
    return unary_union([geometry, *bridge_parts]).buffer(0)


def _keep_anchored_components(geometry, anchors: list[Polygon]):
    kept: list[Polygon] = []
    connected = unary_union(anchors).buffer(0)
    pending = [polygon for polygon in _iter_polygons(geometry) if polygon.area >= MIN_LINE_COMPONENT_AREA_MM2]
    progress = True
    while progress and pending:
        progress = False
        for polygon in list(pending):
            if not polygon.intersects(connected):
                continue
            kept.append(polygon)
            connected = unary_union([connected, polygon]).buffer(0)
            pending.remove(polygon)
            progress = True
    if not kept:
        raise ValueError("continuous texture did not connect to any existing GND vias")
    return unary_union(kept).buffer(0)


def _remove_generated_art(board: pcbnew.BOARD) -> None:
    old_zones = [zone for zone in board.Zones() if zone.GetZoneName().startswith("adc-fractal-fill")]
    for zone in old_zones:
        board.Remove(zone)
    old_shapes = [
        drawing
        for drawing in board.GetDrawings()
        if isinstance(drawing, pcbnew.PCB_SHAPE)
        and drawing.GetLayerName() in {"F.Mask", "B.Mask", "F.SilkS"}
    ]
    for drawing in old_shapes:
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

    open_area = board_interior.difference(obstacle_union).buffer(0)
    if open_area.is_empty:
        raise ValueError("open board area vanished after obstacle subtraction")

    line_art = _line_art_geometry(board_interior, anchors)
    carved = line_art.difference(obstacle_union).intersection(board_interior).buffer(0)
    stitched = _stitch_components(carved, obstacle_union, board_interior)
    final_geometry = _bridge_anchors(stitched, anchors, obstacle_union, board_interior).intersection(board_interior).buffer(0)
    final_geometry = _keep_anchored_components(final_geometry, anchors)
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
        available_area_mm2=open_area.area,
        copper_coverage_ratio=(final_geometry.area / max(open_area.area, 1e-6)),
        maze_motif=(
            "single GND zone built from buffered line-art polygons: board-spanning self-avoiding "
            "maze backbones mixed with tiled self-avoiding maze, Koch, Gosper, Dragon, "
            "Peano, and Sierpinski curves"
        ),
    )
