# Fractal dead-space fill notes

This feature is **decorative PCB art**, not an electrical design technique.

It fills blank board area with procedural fractal linework. The demo board uses
the same Hilbert-curve geometry already used by
`hardware/kicad/fractal_trace_router.py`; the real ADC board now mixes multiple
curve families so the result stops looking like one repeated stamp.

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
2. builds a real `shapely` obstacle union per copper layer from that layer's
   routed tracks, shared vias, pad copper, and footprint courtyards/body bounds
3. fills the remaining interior with dense families of parallel wavy stripe
   centerlines rather than sparse anchor-driven maze trunks
4. clips each stripe family against the obstacle union, buffers the surviving
   segments into manufacturable copper widths, and keeps the segments isolated
   instead of forcing them to connect back to a `GND` anchor
5. writes the resulting decorative copper back as filled `gr_poly` graphics on
   both `F.Cu` and `B.Cu`, which avoids KiCad isolated-zone warnings while
   preserving real-copper clearance

The safety argument is therefore simpler: **the decorative copper never touches
real routed copper at all**. It is just dense, no-net copper artwork that stays
clear of pads, traces, vias, and courtyard keepouts on each layer.

## Isolation strategy

The important safety property is not the fractal itself. It is the **net ownership**.

For the copper variants, the script adds exactly two dedicated board nets:

- net id `1` / net name `FRACTAL_FILL_MASKED`
- net id `2` / net name `FRACTAL_FILL_EXPOSED`

Only the decorative copper segments and their stitching vias use those nets.
On the demo board there are no other real functional nets at all, so the art nets
cannot accidentally short to any circuit net in this demo.

## Why solid copper on the real board

The demo profile still uses dense fractal **track geometry** because it is a
standalone art sandbox with isolated dummy nets.

The real ADS131M08 profile is different: its copper fill is now emitted as
**solid exposed F.Cu GND zones/polygons**, not sparse decorative trace
skeletons. The fractal is used to shape the zone boundary, while the interior
remains filled like ordinary poured copper inside the detected free-space
rectangle.

Where a free-space island does not already touch an existing GND feature, the
script adds a short solid zone corridor back to an existing GND anchor so the
fill stays electrically continuous without turning the visible copper into a
hollow trace drawing.

Earlier renders looked wrong for two reasons:

- many of the visible decorative regions were still silkscreen-only, so they
  could only ever appear as hollow line art
- the one obvious exposed copper island near C5/J3 used only a subtle single
  boundary perturbation, so it read like a plain rectangle

The current real-board output instead exposes multiple front-layer GND regions,
uses closed Koch-derived silhouettes at modest order (`koch-anti` order 2,
`koch-anti-lite` order 1, and `koch-classic` order 1), and bakes the zone fill
before saving.

Those order caps are deliberate: higher orders do look more intricate, but they
also create much narrower necks and more chances for KiCad's zone filler to
drop disconnected slivers or for DRC to flag isolated copper in the thinner
side-strip regions.

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
