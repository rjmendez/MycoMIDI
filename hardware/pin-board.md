# Pin board (electrode header)

Passive only. No ICs, no power. Lives in the humid zone (in/on the bag).

## Parts
- N x gold-flash pin header (0.1" / 2.54mm), cut to pairs per channel
  (2 recording pins + 1 shared reference pin per block)
- Small perfboard or bare FR4 scrap, no copper pour needed
- Heat-shrink over each pin shaft, tip exposed only (~3-5mm)
- 1x shielded/twisted-pair multiconductor cable to ADC module
- 1x panel-mount connector (DB15/DB25 or M12-style) at the dry-zone
  boundary so the pin board itself has no active connector pins exposed
  to condensation

## Layout
- Fixed, documented spacing between each recording pair: 1-2cm
  (matches published fungal electrode spacing)
- One shared reference/ground pin per block (Ag/AgCl snap electrode,
  not gold) wired to the negative differential input (`AINxN`) shared
  across each channel's pair on the ADS131M08 module -- NOT to the
  ADC's `REFIN`/`REFOUT` pin. `REFIN`/`REFOUT` is the ADC's own precision
  voltage-reference pin (internal 1.2V reference or external reference
  input, per module, not per channel); wiring an electrode into it can
  overstress or corrupt the reference for every channel on that module.
  See hardware/adc-module.md for per-channel AIN pinout.
- Label channel 0..7 (or 0..31 across boards) directly on the board
  silkscreen/marker -- must match tokenizer channel-id config

## Electrode tip options (cost order)
1. Bare steel needle - avoid, corrodes in oxidizing substrate
2. Mechanical pencil graphite - free, short-lived, quick test only
3. Solid graphite/carbon rod - cheap, inert, ok for multi-day runs
4. Gold-flash pin header - best cost/quality, use for recording pins
5. Ag/AgCl snap pad - reserve for the single reference/ground pin

## Cable
- Twisted pair per channel, shield tied to ADC-module ground at one
  end only (avoid ground loops)
- Drip loop before entering the dry-zone enclosure
- Keep run length short (<1-2m) -- these are high-impedance sources
