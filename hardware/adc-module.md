# ADC module (dry zone)

One module = one ADS131M08 (8ch, simultaneous, 24-bit, SPI, ~$5).
Lives away from humidity, connected to the pin board by cable.

## Per-module parts
- 1x ADS131M08 (TQFP-32, hand-solderable)
- 1.2V reference is internal -- no external ref needed
- Decoupling: 100nF + 1uF per supply pin, per datasheet
- Passive input protection per channel: series 1k-10k resistor +
  small (100pF-1nF) cap to ground for RF/ESD, before each AINxP/AINxN
- Screw terminal or header block matching the pin-board cable

## Shared bus (scales 8 -> 16 -> 24 -> 32ch)
- CLKIN: common to all modules (from one oscillator or MCU-generated
  clock), required for sample-synchronous capture across modules
- SCLK / MOSI / MISO: shared SPI bus across all modules
- CS: one unique line per module (CS1, CS2, CS3, CS4 ...)
- SYNC/RESET: shared, strobed once at startup to align all modules'
  sample periods
- DRDY: only one module's DRDY needs to be monitored -- all modules
  convert on the same clock edge once synced

## Scaling table
| Channels | Modules | New wires per step |
|----------|---------|---------------------|
| 8        | 1       | -                   |
| 16       | 2       | +1 CS               |
| 24       | 3       | +1 CS               |
| 32       | 4       | +1 CS               |

## Per-channel calibration (do this in software, not analog)
- Record a shorted/disconnected baseline per channel at startup
- Use ADS131M08's built-in offset/gain calibration registers
- Log gain=1 or gain=2 initially -- raise only if signal is too
  small to resolve above ADC noise floor at your data rate

## Recommended future active-probe front end

If the passive pin-board cable proves mains-hum-limited or motion/cable
capacitance limited, the best ADS131M08-compatible upgrade is:

- a **unity-gain active buffer** at the recording electrode,
- a **driven guard / driven shield** on the highest-impedance local copper and
  optionally on one cable conductor adjacent to the buffered signal,
- and a **separate DRL / driven-ground** amplifier on the dry-zone board.

These should be treated as one front-end strategy, not unrelated tweaks.

### Channel semantics stay the same

- `AINxP` receives the **buffered recording-electrode output**
- `AINxN` still receives the **passive reference electrode**
- the **DRL bias electrode** is a third electrode path and does **not** connect
  to `AINxP` or `AINxN`

That means the ADS131M08 board can keep the same differential measurement
concept even when the recording lead becomes active.

### Why the active buffer helps this board

The ADS131M08 input itself is high impedance, but a long cable ahead of it is
still a problem when the source electrode is also very high impedance. Putting
the buffer at the electrode means:

- the raw high-Z node is only millimeters long instead of meters long
- cable capacitance is driven from a low-impedance source instead of loading
  the electrode directly
- the existing ADC-side RC / ESD network can stay conservative because it is no
  longer trying to protect an ultra-fragile remote node

### Driven shield guidance at the ADC-module end

- Use a **local guard ring** on the probe head around the raw input node, tied
  to the buffer output.
- If the cable harness reserves a `BUF_GUARD` conductor, bring that conductor
  into the ADC enclosure as a **guard net**, not as analog ground.
- Do **not** terminate `BUF_GUARD` into the ADS131M08 input network. Its job is
  to bootstrap cable capacitance/leakage around the buffered signal path, not
  to become part of the measurement node.
- Do **not** confuse `BUF_GUARD` with the cable's foil/drain shield. The
  overall cable shield remains an EMI screen tied at the enclosure entry on one
  end only.

### Guard-driver topology recommendation

- First pass: allow the probe buffer output to drive both:
  - the measurement conductor to `AINxP`, and
  - the `BUF_GUARD` conductor
- Isolate each branch with a small **47 Ohm to 100 Ohm** series resistor placed
  near the driver so capacitive cable loading is less likely to destabilize the
  op-amp.
- If bench tests show peaking, ringing, or oscillation with the chosen cable,
  the next fix is **not** to abandon the guard concept; it is to add a separate
  unity-gain guard-driver buffer or raise the isolation resistor modestly.

### DRL / driven-ground stage recommendation

The ADS131M08 has no built-in bias-drive amplifier, so the DRL loop has to live
on this module or a small companion analog board in the dry zone.

Recommended behavior:

1. derive a shared common-mode estimate from the channel inputs or their analog
   front-end nodes,
2. band-limit / low-pass that estimate so the DRL loop stays slow and stable,
3. invert/buffer it with a low-noise op-amp,
4. drive a **dedicated bias electrode** through a large series resistor.

Good starting-value ranges from previously reviewed reference designs:

- DRL output resistor: **470 kOhm to 1 MOhm**
- DRL compensation / low-pass capacitor: **1 nF to 10 nF**
- keep the existing per-channel RC filtering at each ADS131M08 input

The DRL loop reduces whole-system common-mode interference. It does **not**
replace the active buffer, and it does **not** make the driven guard optional on
the raw high-impedance probe head.

### Suggested flex + RJ45 harness mapping for a single active channel

If using a shielded 4-pair RJ45-style harness between probe head and dry-zone
electronics, reserve pairs as:

- pair 1: `VPROBE` + `AGND`
- pair 2: `BUF_OUT` + `BUF_GUARD`
- pair 3: `REF_PASS` + quiet return / spare
- pair 4: `DRL_BIAS` + quiet return / spare

For multi-channel builds, scale this by grouping a small number of active
channels per cable or by moving to a denser connector; do not silently drop the
guard or DRL conductors just to preserve an 8-channel count on one cable.

### Pitfalls

- Do **not** ground the guard conductor.
- Do **not** drive the cable foil/drain shield as though it were the guard net.
- Do **not** reuse the passive reference electrode as the DRL electrode.
- Do **not** assume a unity-gain probe buffer is automatically stable with any
  cable capacitance; verify the actual harness.
