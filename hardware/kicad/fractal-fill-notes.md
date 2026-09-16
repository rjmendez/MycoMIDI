# Fractal dead-space fill notes

This feature is **decorative PCB art**, not an electrical design technique.

It fills blank board area with the same Hilbert-curve geometry already used by
`hardware/kicad/fractal_trace_router.py`, but uses that geometry in three safer,
explicitly isolated variants.

## Variants

`hardware/kicad/fractal_fill.py` writes a board copy at
`hardware/kicad/demo/fractal_fill_demo.kicad_pcb` containing:

1. **Silkscreen fill** on `F.SilkS` and `B.SilkS`
   - decorative only
   - no copper
   - zero electrical interaction by construction

2. **Masked isolated copper fill** on `F.Cu` + `B.Cu`
   - dedicated isolated art net: `FRACTAL_FILL_MASKED`
   - top and bottom traces are connected only to each other through two vias
   - no pads, zones, or traces from the functional design touch that net
   - normal board soldermask still covers the copper, so the artwork is hidden after assembly

3. **Exposed isolated copper fill** on `F.Cu` + `B.Cu` with matching `F.Mask` + `B.Mask`
   - same isolated copper-net approach as the masked variant, but on its own dedicated net `FRACTAL_FILL_EXPOSED`
   - extra graphics on the soldermask layers open the mask over the decorative copper traces
   - the copper artwork is visible after fabrication/finish, but remains electrically floating relative to the real circuit

## Isolation strategy

The important safety property is not the fractal itself. It is the **net ownership**.

For the copper variants, the script adds exactly two dedicated board nets:

- net id `1` / net name `FRACTAL_FILL_MASKED`
- net id `2` / net name `FRACTAL_FILL_EXPOSED`

Only the decorative copper segments and their stitching vias use those nets.
On the demo board there are no other real functional nets at all, so the art nets
cannot accidentally short to any circuit net in this demo. On a real board, keep
that same rule: decorative copper must stay on reserved art-only nets, with DRC
clearance against every functional net.

## Why traces instead of a poured plane

This implementation uses dense Hilbert **track geometry** rather than a large poured
polygonal copper plane. That keeps the generated file simple, human-auditable, and
predictable in the text-based demo workflow while still exercising both copper layers,
mask openings, and via-connected continuity.

If you later want truly flood-filled fractal polygons, build that on top of the same
Hilbert geometry only after validating the polygon boolean/offset math carefully.

## Honest caveats

- This is decorative copper art, **not** a ground plane.
- It is **not** an ESD shield.
- It is **not** an RF structure.
- It is **not** a return-path optimization.
- It should not be counted as useful thermal copper.
- The via-connected copper art is intentionally floating in this demo; do not assume it improves EMC.
- Exposed copper art can tarnish, fingerprint, or cosmetically vary depending on board finish.
- Keep decorative copper well away from analog front ends, high-impedance nodes, clocks, and anything safety-critical.

Those are not legalistic warnings. They are the engineering reality.

## Example

From the repo root:

```bash
python3 hardware/kicad/fractal_fill.py \
  --input hardware/kicad/demo/demo.kicad_pcb \
  --output hardware/kicad/demo/fractal_fill_demo.kicad_pcb
```

Then re-run DRC:

```bash
./scripts/kicad-cli.sh pcb drc \
  --format json \
  --output hardware/kicad/demo/fractal_fill_demo-drc.json \
  --exit-code-violations \
  hardware/kicad/demo/fractal_fill_demo.kicad_pcb
```
