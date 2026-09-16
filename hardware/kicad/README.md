# KiCad tooling

This directory holds lightweight KiCad helpers intended to work without
`pcbnew`, a full KiCad install, or any non-stdlib Python dependencies.

## Lifted from open-source KiCad MCP tooling

MycoMIDI ports a few small, standalone artifacts from MIT-licensed KiCad MCP
servers so hardware automation can stay lightweight and auditable:

- `lib/kicad_sexpr_cst.py` — byte-preserving KiCad S-expression CST parser and serializer,
  adapted from ProductOfAmerica's `mcp-server-kicad`
- `lib/atomic_write.py` — same-directory atomic-write helper adapted from the same project
- `lib/drc_classify.py` — KiCad DRC/ERC JSON classification helper adapted from
  `oaslananka/kicad-mcp-pro`
- `lib/pin_header_footprint_gen.py` — closed-form through-hole pin-header /
  electrode-connector footprint generator adapted from `oaslananka/kicad-mcp-pro`

Why these were lifted: they are pure-Python, portable, and useful for safely
editing `.kicad_pcb` / `.kicad_sch` text and generating simple connector
footprints in environments where the full KiCad Python API is unavailable.

See [`lib/THIRD_PARTY_LICENSES.md`](lib/THIRD_PARTY_LICENSES.md) for upstream
provenance and license texts.
