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

Five AIN nets are procedurally rerouted in the checked-in branch. Three
upper/mid nets (`AIN0N`, `AIN1P`, `AIN2P`) keep the earlier lightweight
in-corridor perturbations. This acute-angle pass specifically strengthens the
two lower nets (`AIN3P`, `AIN5P`) with sharper empty-space-filling detours into
open `B.Cu` area so the bottom-side render reads as deliberate woven routing,
not a barely perturbed diagonal.

| Net | Original long segment replaced | Layer | Curve family | Parameters |
| --- | --- | --- | --- | --- |
| `AIN0N` | `(15.531, 12.0) -> (42.1075, 38.5765)` | `B.Cu` | self-avoiding maze | direct-endpoint mapping, `5x3`, `seed=7`, `spread=0.55 mm` |
| `AIN1P` | `(14.728, 13.3655) -> (40.4106, 39.0481)` | `B.Cu` | Peano | direct-endpoint mapping, `order=1`, `spread=-0.55 mm` |
| `AIN2P` | `(14.5129, 15.8754) -> (39.0382, 40.4007)` | `B.Cu` | Sierpinski arrowhead | direct-endpoint mapping, `order=3`, `spread=0.45 mm` |
| `AIN3P` | `(13.4683, 23.3117) -> (35.8618, 45.7052)` | `B.Cu` | Peano | staged detour via `(18.0,46.0) -> (29.0,46.0)`, `order=2`, `spread=4.8 mm`, then orthogonalized into sharp corners |
| `AIN5P` | `(13.3183, 28.3917) -> (31.5023, 46.5757)` | `B.Cu` | self-avoiding maze | staged detour via `(18.0,58.8) -> (29.8,58.8)`, `10x5`, `seed=21`, `spread=5.4 mm`, then orthogonalized into sharp corners |

The script leaves each net's short pad-entry / chip-fanout segments alone and
only swaps the long middle run. For the two stronger lower-net detours it uses
short lane/corridor segments to enter and exit the curve window, which keeps
the real net endpoints identical while making the visible route materially less
direct.

## How the script works

`hardware/kicad/fractal_signal_router.py`:

1. parses the KiCad board using the repo's byte-preserving CST helper
2. identifies the already-rerouted current-branch lower nets by net name and
   removes only the replaceable `B.Cu` segments, while preserving the short
   header/chip stubs that already proved clean
3. generates stronger replacement polylines for:
   - `AIN3P` via a widened Peano window
   - `AIN5P` via a widened self-avoiding maze window
4. converts each widened window polyline into Manhattan-style bends so the
   shipped traces show obvious sharp turns instead of gentle diagonal wiggles
5. appends the replacement KiCad `segment` chains for the same net ids

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
   - diff the board's copper items by net name against the previous branch head
   - expected result for this acute-angle pass: only `AIN3P` and `AIN5P`
     change;
     the earlier `AIN0N` / `AIN1P` / `AIN2P` procedural routes remain as-is,
     and `CLKIN`, `SCLK`, `DRDY`, `SYNC_RESET`, `CS`, `DIN`, `DOUT`, `AVDD`,
     `DVDD`, `REFP`, and all zone objects stay untouched

4. **GND-fill integrity check**
   - compare the extracted top-level `(zone ...)` blocks before/after after
     normalizing them through the KiCad CST serializer
   - expected result: identical zone content, plus no segment/via changes on
     net `GND`

5. **Render review**
   - export a 2D SVG that includes copper layers
   - render 3D top and bottom views
   - visually confirm:
     - the rerouted AIN subset now looks less direct / more maze-like
     - timing/control nets still look like ordinary direct routes
     - the prior GND copper art is still present and undisturbed

## Honest limit

This is deliberately **not** all 16 AIN nets. Larger attempts to push the upper
and mid-band AIN traces into equally dramatic detours produced clearance,
crossing, or GND-art-adjacent collisions on this compact board. The checked-in
branch therefore keeps the lighter upper reroutes and spends the extra acute-
angle chaos budget on the two lower nets where the open area could absorb it
cleanly.
