# Array geometry (shoebox block)

For one small fruiting block (~20x15x10cm).

See also:
- `pin-board.md` -- electrode materials, shared reference, cable rules
- `adc-module.md` -- 8/16/24/32ch shared-clock scaling across blocks

## Default
- 8 channels = 8 differential pairs + 1 shared reference pin
- Per-pair pin spacing: 1.5cm default
- Allowed per-pair range: 1-2cm
- Pair orientation: keep all pairs the same (default: vertical)
- Pair center spacing: ~5cm across the 20cm axis, ~7cm across the 15cm axis
- Layout: 4x2 grid on one broad face

## Why
- 1-2cm matches published fungal precedent:
  Adamatzky et al. used 1-2cm between electrodes, 8 electrode pairs,
  sampled once per second (`arXiv:2112.09907`; also surfaced in Loci)
- 1.5cm is a good middle default:
  close enough to stay comparable to published setups, not so close that
  minor placement error dominates
- Do not pack pair centers too tightly:
  for a ~20x15cm face, diversity between regions matters more than
  oversampling one wet/dry pocket
- Use a grid, not a tight cluster:
  clustered pairs are likely redundant
- Use a grid, not perimeter-only:
  perimeter-only overweights edge effects (drying, bag seams, slits,
  handling) and misses the center

## What is evidence vs extrapolation
- Directly cited:
  1-2cm intra-pair spacing from Adamatzky precedent
- Engineering extrapolation:
  the ~5-7cm spacing between channel centers on a shoebox block
- Reason for the extrapolation:
  no directly verified spatial-correlation-length result was available in
  the recovered notes, so inter-channel spacing is chosen to reduce obvious
  local redundancy while still fitting 8 pairs on one face

## Multi-block recommendation
- First bring-up:
  one 8ch board on one block is the simplest debug path
- After first clean recordings:
  prefer spreading added channels across 2+ blocks before making one block
  much denser
- Why:
  separate blocks should give more independent variation than shaving
  inter-channel spacing smaller on one block
- Practical rule:
  if adjacent channels on one block look near-identical for long runs,
  move some channels to a second block instead of packing tighter
- The synchronized multi-module setup in `adc-module.md` is already the
  right architecture for this:
  one clock, shared sync/reset, one CS per board, sample-aligned logs

## Suggested single-block layout

Front face, 20cm wide x 15cm tall.

Each `ChN` mark is the center of one differential pair.
Each pair = two gold pins spaced 1.5cm apart vertically.

```text
y=15cm
+--------------------------------------------------+
|                                                  |
|   Ch0       Ch1       Ch2       Ch3              |
|                                                  |
|                                                  |
|   Ch4       Ch5       Ch6       Ch7              |
|                                                  |
+--------------------------------------------------+
0cm                                              20cm
```

Suggested pair-center coordinates (origin = lower-left of front face):

| Channel | X (cm) | Y (cm) | Note |
|---------|--------|--------|------|
| Ch0 | 3 | 11 | upper-left |
| Ch1 | 8 | 11 | upper-mid-left |
| Ch2 | 13 | 11 | upper-mid-right |
| Ch3 | 18 | 11 | upper-right |
| Ch4 | 3 | 4 | lower-left |
| Ch5 | 8 | 4 | lower-mid-left |
| Ch6 | 13 | 4 | lower-mid-right |
| Ch7 | 18 | 4 | lower-right |

## Reference pin
- One shared Ag/AgCl reference pin per block, as in `pin-board.md`
- Put it off the main 4x2 grid:
  side/rear/lower area is better than the most active fruiting face
- Keep it a few cm away from the nearest recording pair
- Keep its position fixed and documented across runs

## Placement notes
- Avoid puncturing directly through fruit bodies; target colonized substrate
- Avoid bag seams, drainage corners, and obvious dry crust
- If the chosen face has a fruiting slit/opening, leave a keep-out zone of
  ~2-3cm around the slit and shift the grid to a cleaner face if needed
- Keep insertion depth consistent across all pairs

## If only four channels are used first
- Use Ch1, Ch2, Ch5, Ch6 first
- This keeps good spatial spread while avoiding only-edge sampling
