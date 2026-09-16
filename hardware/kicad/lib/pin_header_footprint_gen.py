# Adapted from kicad-mcp-pro (oaslananka), MIT License.
# https://github.com/oaslananka/kicad-mcp-pro
# See hardware/kicad/lib/THIRD_PARTY_LICENSES.md for the license text and provenance.
"""Closed-form through-hole pin-header footprint generation for KiCad."""

from __future__ import annotations

GENERATED_SEXPR_DIALECT_VERSION = "20250316"
_HEADER_PITCH_MM = (1.27, 2.0, 2.54)
_LAYER_FAB = "F.Fab"
_LAYER_CU = "F.Cu"
_LAYER_SILK = "F.SilkS"
_LAYER_CYARD = "F.CrtYd"


def _sexpr_string(value: str) -> str:
    return '"' + value.replace("\\", "\\\\").replace('"', '\\"') + '"'


def _fp_header_tht(name: str, description: str, tags: str) -> list[str]:
    return [
        f"(footprint {_sexpr_string(name)}",
        f"\t(version {GENERATED_SEXPR_DIALECT_VERSION})",
        '\t(generator "mycomidi-pin-header-gen")',
        f"\t(layer {_sexpr_string(_LAYER_CU)})",
        f"\t(descr {_sexpr_string(description)})",
        f"\t(tags {_sexpr_string(tags)})",
    ]


def _ref_value(ref_y: float, val_y: float, fab_y: float | None = None) -> list[str]:
    lines = [
        f'\t(fp_text reference "REF**" (at 0 {ref_y:.4f})'
        f" (layer {_LAYER_SILK}) (effects (font (size 1 1) (thickness 0.15))))",
        f'\t(fp_text value "VAL**" (at 0 {val_y:.4f})'
        f" (layer {_LAYER_FAB}) (effects (font (size 1 1) (thickness 0.15))))",
    ]
    if fab_y is not None:
        lines.append(
            f'\t(fp_text user "${{REFERENCE}}" (at 0 {fab_y:.4f})'
            f" (layer {_LAYER_FAB}) (effects (font (size 0.8 0.8) (thickness 0.12))))"
        )
    return lines


def _pad_tht(number: int, x: float, y: float, drill: float, size: float) -> str:
    shape = "rect" if number == 1 else "circle"
    return (
        f'\t(pad "{number}" thru_hole {shape} (at {x:.4f} {y:.4f})'
        f" (size {size:.4f} {size:.4f}) (drill {drill:.4f})"
        f" (layers *.Cu *.Mask))"
    )


def _rect_line(layer: str, x1: float, y1: float, x2: float, y2: float, width: float = 0.1) -> list[str]:
    return [
        (
            f"\t(fp_line (start {x1:.4f} {y1:.4f}) (end {x2:.4f} {y1:.4f}) "
            f"(layer {layer}) (stroke (width {width})(type solid)))"
        ),
        (
            f"\t(fp_line (start {x2:.4f} {y1:.4f}) (end {x2:.4f} {y2:.4f}) "
            f"(layer {layer}) (stroke (width {width})(type solid)))"
        ),
        (
            f"\t(fp_line (start {x2:.4f} {y2:.4f}) (end {x1:.4f} {y2:.4f}) "
            f"(layer {layer}) (stroke (width {width})(type solid)))"
        ),
        (
            f"\t(fp_line (start {x1:.4f} {y2:.4f}) (end {x1:.4f} {y1:.4f}) "
            f"(layer {layer}) (stroke (width {width})(type solid)))"
        ),
    ]


def generate_pin_header_footprint(
    pin_count: int,
    *,
    rows: int = 1,
    pitch_mm: float = 2.54,
    name: str | None = None,
    description: str | None = None,
    tags: str = "pin-header connector electrode",
) -> str:
    """Generate a through-hole 1xN or 2xN pin-header footprint as KiCad S-expression text."""
    if pin_count < 1:
        raise ValueError("pin_count must be at least 1.")
    if pitch_mm not in _HEADER_PITCH_MM:
        raise ValueError(f"pitch_mm must be one of {list(_HEADER_PITCH_MM)}")
    if rows not in (1, 2):
        raise ValueError("rows must be 1 or 2.")

    drill = pitch_mm * 0.4
    pad_size = drill + 0.8
    name = name or f"PinHeader_{rows}x{pin_count:02d}_{pitch_mm:.2f}mm"
    description = description or f"Pin header {rows}×{pin_count} {pitch_mm:.2f}mm"
    lines = _fp_header_tht(name, description, tags)
    outline_y = (pin_count - 1) * pitch_mm / 2 + pitch_mm / 2
    lines += _ref_value(-(pin_count * pitch_mm / 2 + 0.5), pin_count * pitch_mm / 2 + 0.5, 0.0)

    for index in range(pin_count):
        for row in range(rows):
            number = index * rows + row + 1
            x = row * pitch_mm - (rows - 1) * pitch_mm / 2
            y = -((pin_count - 1) * pitch_mm / 2) + index * pitch_mm
            lines.append(_pad_tht(number, x, y, drill, pad_size))

    outline_x = (rows - 1) * pitch_mm / 2 + pitch_mm / 2
    lines += _rect_line(_LAYER_SILK, -outline_x, -outline_y, outline_x, outline_y)
    lines += _rect_line(
        _LAYER_CYARD,
        -outline_x - 0.25,
        -outline_y - 0.25,
        outline_x + 0.25,
        outline_y + 0.25,
        0.05,
    )
    lines.append(")")
    return "\n".join(lines)


def generate_electrode_connector_footprint(
    channel_count: int,
    *,
    rows: int = 1,
    pitch_mm: float = 2.54,
    name: str | None = None,
) -> str:
    """Generate a simple electrode connector footprint where one channel maps to one pin."""
    return generate_pin_header_footprint(
        channel_count,
        rows=rows,
        pitch_mm=pitch_mm,
        name=name or f"MycoMIDI_Electrode_{rows}x{channel_count:02d}_{pitch_mm:.2f}mm",
        description=f"MycoMIDI electrode connector {rows}×{channel_count} {pitch_mm:.2f}mm",
        tags="electrode connector mycomidi pin-header",
    )


__all__ = [
    "GENERATED_SEXPR_DIALECT_VERSION",
    "generate_electrode_connector_footprint",
    "generate_pin_header_footprint",
]
