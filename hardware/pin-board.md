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

## Recommended future active-probe + driven-shield + DRL topology
- Current pin board stays passive today: recording pins go to `AINxP`, the
  shared passive reference electrode goes to the corresponding `AINxN`, and
  no electrode is actively driven yet.
- The next ADS131M08-friendly noise-reduction step should combine three ideas
  instead of treating them as competing options:
  1. a **local active probe buffer** right at the recording electrode,
  2. a **driven guard / driven shield** held near that buffered signal's
     voltage,
  3. a separate **DRL / driven-ground** bias electrode for common-mode control.
- These roles are different:
  - the **active buffer** converts the recording electrode from a very
    high-impedance source into a low-impedance signal before the long cable run
  - the **driven guard** reduces leakage and effective capacitance by keeping
    nearby copper and cable conductors at nearly the same potential as the
    signal node
  - the **DRL electrode** feeds back an inverted common-mode estimate into the
    substrate/body so 50/60Hz pickup is reduced before digitization

### Recommended signal split
- **recording electrode** -> local unity-gain buffer input -> buffered output
  returned to `AINxP`
- **reference electrode** stays passive and returns separately to `AINxN`
- **bias/DRL electrode** is its own dedicated conductor and electrode; do not
  reuse the signal guard or the passive reference conductor for this path

### Why the active buffer and driven guard belong together
- A plain grounded shield around a high-impedance electrode lead reduces some
  radiated pickup, but it also adds capacitance from the signal to ground. On a
  weak biological source that extra capacitance worsens settling, motion
  sensitivity, and contamination leakage.
- A **driven** shield/guard works better here because it is held near the same
  potential as the electrode/buffered signal. The smaller voltage across the
  parasitic capacitance means less displacement current and less leakage error.
- In practice this means:
  - keep the raw electrode trace only a few millimeters long on the flex head
  - place a **guard ring / guard trace tied to the buffer output** around the
    input pad, input resistor, and op-amp non-inverting input
  - use the local buffer's low output impedance, not the raw electrode node, to
    drive any guarded cable conductor

### Recommended probe-head topology
- The active-probe design work already explored in this repo used a
  **TI LMP7701** unity-gain follower as the preferred first-pass buffer because
  its femtoamp-class input bias current suits the very high source impedance at
  the electrode/substrate interface.
- Keep the passive reference lead physically separate. Do **not** tie its guard
  or return to the recording buffer output.
- Treat the local PCB guard ring as baseline practice for the active probe, not
  as a later optimization.

### Cable recommendation for the flex + RJ45 plan
- If the probe head breaks out through a removable **flex + RJ45-style shielded
  twisted-pair harness**, reserve conductors explicitly for:
  - `BUF_OUT` - buffered recording signal
  - `BUF_GUARD` - driven guard conductor held near `BUF_OUT`
  - `REF_PASS` - passive reference returned to `AINxN`
  - `DRL_BIAS` - separate driven-bias electrode
  - probe power and analog ground
- Good first-pass 4-pair allocation for a single buffered channel:
  - pair 1: `VPROBE` + `AGND`
  - pair 2: `BUF_OUT` + `BUF_GUARD`
  - pair 3: `REF_PASS` + quiet return / spare
  - pair 4: `DRL_BIAS` + quiet return / spare
- Keep the distinction clear:
  - `BUF_GUARD` is a **driven** conductor intended to bootstrap the buffered
    signal path
  - the cable's **overall foil/drain shield** is still an EMI screen and should
    be bonded at the ADC enclosure / analog-ground entry point only, not driven
    and not tied at both ends
- If a chosen RJ45 cable has only an overall shield and no individually
  shielded pairs, use a spare adjacent conductor as the driven guard. If it has
  individually shielded pairs, that pair shield can fill the same role.

### DRL implementation guidance
- The ADS131M08 does **not** provide the integrated bias-drive amplifier found
  on some biopotential ADCs, so MycoMIDI needs a discrete low-noise op-amp
  stage in the dry zone.
- Purpose: estimate the channels' shared common-mode voltage, low-pass/filter
  it, buffer/invert it, and drive that correction back into the substrate
  through a dedicated bias electrode.
- Conservative starting references for the DRL output path are:
  - **470 kOhm to 1 MOhm** series resistance to the bias electrode
  - **1 nF to 10 nF** compensation / low-pass capacitance
  - per-channel RC filtering/protection retained at the ADC board
- Those values are in the same class as the OpenBCI/TI reference-style designs
  already reviewed for this repo, but the final stable values depend on cable
  capacitance, electrode impedance, and total loop gain, so bench validation is
  still required.

### Common pitfalls to avoid
- Do **not** tie the driven guard to ground; that restores the capacitance load
  you were trying to remove.
- Do **not** drive the entire cable shield directly from a high-impedance node.
  Drive guards only from a **low-impedance buffered node**, usually through a
  small isolation resistor or a dedicated guard-driver buffer if cable
  capacitance proves too large.
- Do **not** use the DRL electrode as the signal return path for the buffer or
  shield currents.
- Do **not** bond the cable shield at both wet and dry ends; that invites
  ground-loop current.
- Validate the combined active-buffer + guard + DRL loop on resistor/phantom
  loads first, then with cable-motion testing, before any biological
  experiment.

## Electrode tip options (cost order)
1. Bare steel needle - avoid, corrodes in oxidizing substrate
2. Mechanical pencil graphite - free, short-lived, quick test only
3. Solid graphite/carbon rod - cheap, inert, ok for multi-day runs
4. Gold-flash pin header - best cost/quality, use for recording pins
5. Ag/AgCl snap pad - reserve for the single reference/ground pin

## Cable
- Passive baseline: twisted pair per channel, shield tied to ADC-module ground
  at one end only (avoid ground loops)
- Active-probe upgrade path: reserve one adjacent conductor or pair shield for
  `BUF_GUARD` so the buffered signal can be bootstrapped over the cable instead
  of being surrounded only by grounded metal
- Drip loop before entering the dry-zone enclosure
- Keep run length short (<1-2m) in the passive case; a local active buffer
  helps substantially, but it does not make cable length irrelevant
