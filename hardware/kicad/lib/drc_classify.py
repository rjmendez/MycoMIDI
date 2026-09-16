# Adapted from kicad-mcp-pro (oaslananka), MIT License.
# https://github.com/oaslananka/kicad-mcp-pro
# See hardware/kicad/lib/THIRD_PARTY_LICENSES.md for the license text and provenance.
"""Classify KiCad DRC/ERC JSON as unavailable, findings, clean, or malformed."""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Literal

ReportStatus = Literal["unavailable", "findings", "clean", "malformed"]
ReportKind = Literal["drc", "erc"]

_DRC_KEYS = ("violations", "unconnected_items", "items_not_passing_courtyard")


@dataclass(frozen=True, slots=True)
class ReportClassification:
    status: ReportStatus
    report_kind: ReportKind | None
    error: str | None = None
    report: dict[str, object] | None = None


def _load_json_object(report_input: object) -> tuple[dict[str, object] | None, ReportStatus, str | None]:
    if report_input is None:
        return None, "unavailable", "No report payload was provided."
    if isinstance(report_input, dict):
        return report_input, "clean", None
    if isinstance(report_input, bytes):
        try:
            report_input = report_input.decode("utf-8")
        except UnicodeDecodeError:
            return None, "malformed", "Report bytes are not valid UTF-8."
    if isinstance(report_input, str):
        try:
            parsed = json.loads(report_input)
        except json.JSONDecodeError as exc:
            return None, "malformed", f"Report is not valid JSON: line {exc.lineno}, column {exc.colno}."
        if not isinstance(parsed, dict):
            return None, "malformed", "Report root must be a JSON object."
        return parsed, "clean", None
    return None, "malformed", "Report input must be a dict, JSON string, or JSON bytes."


def _validate_report_list(report: dict[str, object], key: str, *, required: bool = False) -> str | None:
    if key not in report:
        if required:
            return f"Report is missing required field '{key}'."
        return None
    raw = report[key]
    if not isinstance(raw, list):
        return f"Report field '{key}' must be a list."
    if any(not isinstance(entry, dict) for entry in raw):
        return f"Report field '{key}' must contain only objects."
    return None


def _classify_drc(report: dict[str, object]) -> tuple[ReportStatus, str | None]:
    for key, required in (
        ("violations", True),
        ("unconnected_items", False),
        ("items_not_passing_courtyard", False),
    ):
        if error := _validate_report_list(report, key, required=required):
            return "malformed", error
    has_findings = any(bool(report.get(key, [])) for key in _DRC_KEYS)
    return ("findings" if has_findings else "clean"), None


def _classify_erc(report: dict[str, object]) -> tuple[ReportStatus, str | None]:
    sheets = report.get("sheets")
    if not isinstance(sheets, list):
        return "malformed", "Report field 'sheets' must be a list."
    for index, sheet in enumerate(sheets):
        if not isinstance(sheet, dict):
            return "malformed", f"Report field 'sheets' must contain only objects (bad entry at index {index})."
        violations = sheet.get("violations")
        if not isinstance(violations, list):
            return "malformed", f"ERC sheet at index {index} is missing a list-valued 'violations' field."
        if any(not isinstance(entry, dict) for entry in violations):
            return "malformed", f"ERC sheet at index {index} has a non-object violation entry."
    has_findings = any(bool(sheet.get("violations", [])) for sheet in sheets if isinstance(sheet, dict))
    return ("findings" if has_findings else "clean"), None


def classify_kicad_report(report_input: object) -> ReportClassification:
    """Classify a KiCad validation payload without conflating malformed data with clean data."""
    report, preload_status, preload_error = _load_json_object(report_input)
    if report is None:
        return ReportClassification(preload_status, None, preload_error, None)

    schema = report.get("$schema")
    schema_text = schema.lower() if isinstance(schema, str) else ""
    if "erc" in schema_text or "sheets" in report:
        status, error = _classify_erc(report)
        return ReportClassification(status, "erc", error, report)
    if "drc" in schema_text or "violations" in report:
        status, error = _classify_drc(report)
        return ReportClassification(status, "drc", error, report)
    return ReportClassification(
        "malformed",
        None,
        "Report does not look like a KiCad DRC or ERC JSON document.",
        report,
    )


def classify_report_status(report_input: object) -> tuple[ReportStatus, str | None]:
    result = classify_kicad_report(report_input)
    return result.status, result.error


__all__ = [
    "ReportClassification",
    "ReportKind",
    "ReportStatus",
    "classify_kicad_report",
    "classify_report_status",
]
