# Fractal signal routing notes

This file documents the **actual signal-trace** reroute experiments on
`hardware/kicad/adc_board/adc_board_8ch.kicad_pcb`.

## Scope

The goal was not to globally regenerate the entire board. The supported, DRC-
clean slice here is a **surgical post-route rewrite**:

- keep the Freerouting fanout and all timing-sensitive traces intact
- replace only a few long, low-risk analog middle segments with procedural
  meanders
- keep changes off the timing bundle and the ADC supply/reference cluster
- regenerate the decorative weave after rerouting so the isolated copper art
  keeps proper clearance around the new paths

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

Fourteen AIN nets are procedurally rerouted in the checked-in branch. A
follow-up maze-corridor pass upgrades three of the still-too-smooth runs
(`AIN0P`, `AIN2N`, `AIN3P`) to the same orthogonal corridor language already
used by `AIN5P`. The remaining rerouted nets keep their earlier lighter
procedural perturbations because the first clean single-net maze attempts for
them still produced DRC violations.

| Net | Original long segment replaced | Layer | Curve family | Parameters |
| --- | --- | --- | --- | --- |
| `AIN0N` | `(15.531, 12.0) -> (42.1075, 38.5765)` | `B.Cu` | self-avoiding maze | direct-endpoint mapping, `5x3`, `seed=7`, `spread=0.55 mm` |
| `AIN0P` | `(15.8093, 10.7948) -> (43.6, 38.5855)` | `B.Cu` | self-avoiding maze | orthogonalized direct corridor, `12x2`, `seed=3`, `spread=-0.2 mm` |
| `AIN1N` | `(14.5401, 14.54) -> (40.9188, 40.9187)` | `F.Cu` | Sierpinski arrowhead | direct-endpoint mapping, `order=2`, `spread=0.8 mm` |
| `AIN1P` | `(14.728, 13.3655) -> (40.4106, 39.0481)` | `B.Cu` | Peano | direct-endpoint mapping, `order=1`, `spread=-0.55 mm` |
| `AIN2N` | `(13.8995, 17.08) -> (38.5952, 41.7757)` | `B.Cu` | self-avoiding maze | orthogonalized direct corridor, `12x2`, `seed=3`, `spread=0.12 mm` |
| `AIN2P` | `(14.5129, 15.8754) -> (39.0382, 40.4007)` | `B.Cu` | Sierpinski arrowhead | direct-endpoint mapping, `order=3`, `spread=0.45 mm` |
| `AIN3N` | `(13.3762, 19.62) -> (36.3319, 42.5757)` | `B.Cu` | Gosper | direct-endpoint mapping, `order=2`, `spread=-0.4 mm` |
| `AIN3P` | `(13.4683, 23.3117) -> (35.8618, 45.7052)` | `B.Cu` | self-avoiding maze | staged detour via `(18.0,46.0) -> (29.0,46.0)`, `10x5`, `seed=21`, `spread=4.8 mm`, then orthogonalized into sharp corners |
| `AIN4P` | `(16.3297, 23.43) -> (37.2997, 44.4)` | `F.Cu` | Peano | direct-endpoint mapping, `order=1`, `spread=0.5 mm` |
| `AIN4N` | `(14.1785, 22.16) -> (37.0458, 45.0273)` | `B.Cu` | Sierpinski arrowhead | direct-endpoint mapping, `order=3`, `spread=0.45 mm` |
| `AIN5N` | `(36.4677, 45.7772) -> (15.3905, 24.7)` | `F.Cu` | Peano | direct-endpoint mapping from chip side back to header side, `order=1`, `spread=0.6 mm` |
| `AIN5P` | `(13.3183, 28.3917) -> (31.5023, 46.5757)` | `B.Cu` | self-avoiding maze | staged detour via `(18.0,58.8) -> (29.8,58.8)`, `10x5`, `seed=21`, `spread=5.4 mm`, then orthogonalized into sharp corners |
| `AIN6N` | `(36.3262, 46.3484) -> (17.2178, 27.24)` | `F.Cu` | Peano | direct-endpoint mapping from chip side back to header side, `order=1`, `spread=0.15 mm` |
| `AIN6P` | `(17.8923, 28.51) -> (37.5448, 48.1625)` | `F.Cu` | Sierpinski arrowhead | direct-endpoint mapping, `order=4`, `spread=2.8 mm` |

The script leaves each net's short pad-entry / chip-fanout segments alone and
only swaps the long middle run. For the two stronger lower-net detours it uses
short lane/corridor segments to enter and exit the curve window, which keeps
the real net endpoints identical while making the visible route materially less
direct.

## How the script works

`hardware/kicad/fractal_signal_router.py`:

1. parses the KiCad board using the repo's byte-preserving CST helper
2. identifies the targeted `B.Cu` or `F.Cu` nets by net name and removes only the
   replaceable long segments, while preserving the short header/chip stubs
   that already proved clean
3. generates replacement polylines for:
   - two orthogonalized direct corridors: `AIN0P`, `AIN2N`
   - two still-smoother direct-endpoint mapped curves: `AIN3N`, `AIN4N`
   - five additional direct-endpoint mapped front-layer curves: `AIN1N`,
     `AIN4P`, `AIN5N`, `AIN6N`, `AIN6P`
   - two widened curve windows: `AIN3P`, `AIN5P`
4. converts each corridor/window polyline into Manhattan-style bends so the
   shipped traces show obvious sharp turns instead of gentle diagonal wiggles
5. appends the replacement KiCad `segment` chains for the same net ids

After the route rewrite, `adc_texture_fill.py` must be re-run so the
disconnected decorative copper islands get fresh clearance cutouts around the
additional hidden traces.

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
   - diff the board's copper items by net name against the `733533b` baseline
   - expected result for the v3 pass: `AIN0P`, `AIN2N`, `AIN3N`, `AIN4N`,
     `AIN1N`, `AIN4P`, `AIN5N`, `AIN6N`, and `AIN6P` differ from `733533b`;
     the earlier `AIN0N` / `AIN1P` / `AIN2P` / `AIN3P` / `AIN5P` procedural
     routes remain as-is, `AIN7N` / `AIN7P` remain direct, and `CLKIN`,
     `SCLK`, `DRDY`, `SYNC_RESET`, `CS`, `DIN`, `DOUT`, `AVDD`, `DVDD`,
     `REFP`, and `CAP` stay untouched

4. **Decorative-fill regeneration check**
   - re-run `adc_texture_fill.py` after the route edit, then verify fresh DRC
   - expected result: zero shorts/clearance errors despite denser hidden-signal
     routing inside the decorative weave

5. **Render review**
   - export a 2D SVG that includes copper layers
   - render 3D top and bottom views
   - visually confirm:
     - the rerouted AIN subset now looks less direct / more maze-like
     - timing/control nets still look like ordinary direct routes
     - the prior GND copper art is still present and undisturbed

## Honest limit

This is still deliberately **not** all 16 AIN nets. The viable extra targets on
top of `733533b` were the four backside analog runs that already used their own
via transition into `B.Cu` plus five long front-layer diagonals that accepted
modest full-run procedural replacements without forcing any protected-net
changes. The remaining untouched pair (`AIN7N`, `AIN7P`) repeatedly produced
same-layer shorts or clearance failures against neighboring analog routes under
every aggressive family/spacing combination tested, so they were left direct
rather than ship a visually larger but electrically worse board.
