# Fractal signal routing notes

This file documents the **actual signal-trace** reroute experiment on
`hardware/kicad/adc_board/adc_board_8ch.kicad_pcb`.

## Scope

The goal was not to globally regenerate the entire board. The supported, DRC-
clean slice here is a **surgical post-autoroute rewrite**:

- keep the Freerouting fanout and all timing-sensitive traces intact
- replace only a few long, low-risk analog middle segments with procedural
  meanders
- avoid touching the already-added GND fractal-fill zones

That keeps the board electrically valid while still making some pad-to-ADC
paths materially more annoying to eyeball.

## Nets intentionally kept direct

These were left on their existing plain Freerouting paths:

- `CLKIN`
- `SCLK`
- `DRDY`
- `SYNC_RESET`
- `CS`
- `DIN`
- `DOUT`

Reason: even though the board is not high-speed RF, those nets still carry the
clock/edge-timed SPI/control relationship. The user explicitly called out the
clock-class signals, and keeping the whole SPI/control bundle direct avoids
gratuitous skew/stub/noise risk for a purely aesthetic goal.

The power/reference nets also stayed direct:

- `AVDD`
- `DVDD`
- `REFP`

Reason: obscuring analog rails/reference routing is less valuable than
preserving the existing short supply/reference paths and avoiding any change
near decoupling/current-return behavior.

## Nets rerouted procedurally

Only four AIN nets were changed in the checked-in board, because that subset
was the largest one proven clean without disturbing the existing GND art or the
timing-sensitive routes.

| Net | Original long segment replaced | Layer | Curve family | Parameters |
| --- | --- | --- | --- | --- |
| `AIN0N` | `(15.531, 12.0) -> (42.1075, 38.5765)` | `B.Cu` | self-avoiding maze | `5x3`, `seed=7`, `spread=0.55 mm` |
| `AIN1P` | `(14.728, 13.3655) -> (40.4106, 39.0481)` | `B.Cu` | Peano | `order=1`, `spread=-0.55 mm` |
| `AIN2P` | `(14.5129, 15.8754) -> (39.0382, 40.4007)` | `B.Cu` | Sierpinski arrowhead | `order=3`, `spread=0.45 mm` |
| `AIN5P` | `(13.3183, 28.3917) -> (31.5023, 46.5757)` | `B.Cu` | Gosper | `order=1`, `spread=-0.9 mm` |

The script leaves each net's short pad-entry / chip-fanout segments alone and
only swaps the long middle run, which preserves the proven endpoint geometry.

## How the script works

`hardware/kicad/fractal_signal_router.py`:

1. parses the KiCad board using the repo's byte-preserving CST helper
2. finds four exact existing `segment` records by:
   - net name
   - layer
   - start/end coordinates
3. removes only those four original straight-ish segments
4. generates a replacement polyline for each net from a different curve family
5. maps the curve onto the original segment baseline while forcing the first
   and last generated points back onto the exact original segment endpoints
6. appends the replacement KiCad `segment` chain for the same net id

Because it is text-surgical rather than a full pcbnew save, untouched areas of
the board — especially the GND fill zones — are not regenerated or normalized.

## Verification method

Verification was intentionally broader than "DRC passes":

1. **Fresh DRC**
   - command:
     ```bash
     ./scripts/kicad-cli.sh pcb drc \
       --format json \
       --output hardware/kicad/adc_board/adc_board_8ch-drc.json \
       --exit-code-violations \
       hardware/kicad/adc_board/adc_board_8ch.kicad_pcb
     ```
   - required result: `0 violations`, `0 unconnected items`

2. **Net-to-pad membership check**
   - compare the before/after pad sets for every interesting net
   - this proves the reroute did not relabel pads onto the wrong net

3. **Changed-net scope check**
   - diff the board's copper items by net name
   - expected result: only `AIN0N`, `AIN1P`, `AIN2P`, and `AIN5P` changed;
     `CLKIN`, `SCLK`, `DRDY`, `SYNC_RESET`, `CS`, `DIN`, `DOUT`, `AVDD`,
     `DVDD`, `REFP`, and all zone objects stayed untouched

4. **GND-fill integrity check**
   - compare the extracted top-level `(zone ...)` blocks before/after
   - expected result: byte-identical zone content

5. **Render review**
   - export a 2D SVG that includes copper layers
   - render 3D top and bottom views
   - visually confirm:
     - the rerouted AIN subset now looks less direct / more maze-like
     - timing/control nets still look like ordinary direct routes
     - the prior GND copper art is still present and undisturbed

## Honest limit

This is deliberately **not** all 16 AIN nets. Several larger attempts produced
clearance, edge, or GND-art collisions. The checked-in subset is the narrow
slice that stayed electrically clean and verifiable in the current layout
without rewriting the whole board again.
