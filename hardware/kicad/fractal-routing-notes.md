# Fractal trace routing notes

This is a **novelty art script**, not a sane production routing strategy.

## What the script does

`hardware/kicad/fractal_trace_router.py` takes two XY coordinates, generates a Hilbert-curve space-filling path between them, emits native KiCad copper `segment` records, and writes the result to `hardware/kicad/demo/fractal_demo.kicad_pcb`.

Because the demo board has no pads, the script also adds a short return path below the baseline so the art is a closed copper loop and does not trip DRC with dangling-track warnings.

The default output intentionally looks excessive. That is the point.

## Engineering reality

Fractal or aggressive meander routing **drastically increases trace length**. That means more:

- resistance
- inductance
- propagation delay
- susceptibility to noise pickup

This is absolutely the wrong thing to do for any performance-sensitive signal. A working EE should hate it.

Use this only as decorative or joke routing for **non-critical single-ended low-speed nets**, for example:

- an LED indicator line
- a decorative grounded copper outline
- some obviously non-timing-critical GPIO experiment

Do **not** use this on:

- the ADC differential input pairs
- any clock line
- any SPI signal
- controlled-impedance traces
- differential pairs
- anything above a few kHz

That warning is not stylistic hedging. It is the actual electrical constraint.

## Practical limits

- Keep the fractal inside the board outline.
- Keep segment spacing comfortably larger than track width so DRC stays clean.
- Re-run KiCad DRC every time; decorative copper can still violate clearance or overlap rules if the geometry is pushed too hard.

## Example

From the repo root:

```bash
python3 hardware/kicad/fractal_trace_router.py \
  --input hardware/kicad/demo/demo.kicad_pcb \
  --output hardware/kicad/demo/fractal_demo.kicad_pcb \
  --start-x 1.0 --start-y 1.0 \
  --end-x 9.0 --end-y 1.0 \
  --spread 7.2 \
  --order 5 \
  --width 0.08 \
  --return-margin 0.4
```
