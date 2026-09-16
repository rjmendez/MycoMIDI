# Fractal dead-space fill notes

This feature is **decorative PCB art**, not an electrical design technique.

It fills blank board area with the same Hilbert-curve geometry already used by
`hardware/kicad/fractal_trace_router.py`, but uses that geometry in two
different deployment styles:

- the original isolated-net demo board
- the real ADS131M08 board, where the copper art is intentionally tied to the
  board's existing `GND` net

## Variants

`hardware/kicad/fractal_fill.py --profile demo` writes a board copy at
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

## Real ADS131M08 board strategy

The real-board profile no longer drops three hardcoded stamps into hand-picked
rectangles.

For `hardware/kicad/adc_board/adc_board_8ch.kicad_pcb`,
`fractal_fill.py --profile adc-board-gnd` now:

1. parses the actual `Edge.Cuts` loop from the routed KiCad board instead of
   trusting a fixed board-size tuple
2. reuses the board-text obstacle pass to enumerate pads, tracks, vias,
   footprint extents, and existing silk/text bounds
3. grid-scans the real board interior and marks cells free/blocked using the
   same clearance model as the decorative fill (`0.25 mm` local clearance,
   `0.45 mm` region clearance for copper)
4. greedily merges contiguous free cells into candidate rectangles
5. tiles the shared Hilbert geometry repeatedly across each useful rectangle
   instead of placing one fixed-size fractal in each area
6. routes the real-board copper variants only from anchor points already on the
   real `GND` network, then emits every new segment/via on that same KiCad net

The result is still approximate rather than polygon-perfect: the free-space
detector is grid/rectangle based, not a general polygon boolean engine. But it
reacts to the actual routed board instead of a few manually chosen decorative
patches.

That means the safety argument changes from **\"isolated art cannot short
anything\"** to **\"GND-connected art only touches GND, so it still cannot short
two different nets together\"**.

## Isolation strategy

The important safety property is not the fractal itself. It is the **net ownership**.

For the copper variants, the script adds exactly two dedicated board nets:

- net id `1` / net name `FRACTAL_FILL_MASKED`
- net id `2` / net name `FRACTAL_FILL_EXPOSED`

Only the decorative copper segments and their stitching vias use those nets.
On the demo board there are no other real functional nets at all, so the art nets
cannot accidentally short to any circuit net in this demo.

## Why thin traces instead of a poured plane

This implementation uses dense Hilbert **track geometry** rather than a large poured
polygonal copper plane. That keeps the generated file simple, human-auditable, and
predictable in the text-based workflow while still exercising both copper layers,
mask openings, and via-connected continuity.

On the real ADS131M08 board, the copper art width is intentionally smaller than
the functional routing width. The fractal is decorative GND augmentation, not a
high-current power feed and not a precision impedance-controlled signal, so
there is no reason to make it as fat as the real traces.

If you later want truly flood-filled fractal polygons, build that on top of the
same Hilbert geometry only after validating the polygon boolean/offset math
carefully.

## Honest caveats

- This is decorative copper art, **not** a substitute for a properly planned ground plane.
- It is **not** an ESD shield.
- It is **not** an RF structure.
- It is **not** a return-path optimization.
- It should not be counted as useful thermal copper.
- The demo-board copper art is intentionally floating; the ADS131M08-board
  copper art is intentionally GND-tied. In neither case should you assume
  meaningful EMC improvement without separate evidence.
- The real-board pass is a **grid-derived rectangle tiler**. It covers much more
  real empty area than the earlier three-rectangle version, but it can still
  leave awkward slivers or irregular leftover pockets unfilled.
- Exposed copper art can tarnish, fingerprint, or cosmetically vary depending on board finish.
- Keep decorative copper well away from analog front ends, high-impedance nodes, clocks, and anything safety-critical.

Those are not legalistic warnings. They are the engineering reality.

## Example

From the repo root:

```bash
python3 hardware/kicad/fractal_fill.py \
  --profile demo \
  --input hardware/kicad/demo/demo.kicad_pcb \
  --output hardware/kicad/demo/fractal_fill_demo.kicad_pcb
```

Real ADS131M08 board:

```bash
python3 hardware/kicad/fractal_fill.py \
  --profile adc-board-gnd \
  --input hardware/kicad/adc_board/adc_board_8ch.kicad_pcb \
  --output hardware/kicad/adc_board/adc_board_8ch.kicad_pcb
```

Then re-run DRC:

```bash
./scripts/kicad-cli.sh pcb drc \
  --format json \
  --output hardware/kicad/demo/fractal_fill_demo-drc.json \
  --exit-code-violations \
  hardware/kicad/demo/fractal_fill_demo.kicad_pcb
```
