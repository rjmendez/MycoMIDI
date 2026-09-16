# Active electrode probe flex design

This pass delivers a **schematic-level concept and SKiDL netlist generator**, not
a finished KiCad flex board layout. The goal is a small active probe that sits
close to the **recording electrode** so the highest-impedance node stays short,
then drives a lower-impedance signal back to the dry-zone ADS131M08 module.

It follows the documented MycoMIDI channel convention:

- recording electrode -> `AINxP`
- reference electrode -> `AINxN`

and uses the previously recommended **TI LMP7701** as the preferred unity-gain
buffer op-amp. `TLV9061` remains a plausible runner-up if supply current or
availability becomes more important than ultra-low input bias current.

## Scope and assumptions

This is intentionally narrow:

- **One recording electrode is buffered on the flex head**
- **The reference electrode remains a separate passive lead** returned to
  `AINxN`
- The flex head is **not** a full instrumentation amplifier and does **not**
  buffer the reference lead
- The ADS131M08 module still performs the final differential measurement in the
  dry zone
- The preferred rail is **3.0 V to 3.3 V single-supply** from a small battery
  or a bus-derived rail, because that is easy to distribute and inside the
  LMP7701 operating range
- If a later bench setup shows large DC electrode offsets or poor reference-lead
  noise, the next step is a **two-buffer active electrode pair** or a dry-zone
  differential front-end, not more complexity on this first flex

## Why unity gain

Use **gain = 1** for the first flex revision.

Reasons:

- the main problem here is **high source impedance plus cable pickup**, not lack
  of amplitude
- the ADS131M08 already has programmable gain and a differential input stage
- extra local gain would also amplify cable-borne interference, op-amp offset,
  and startup/bias transients before the ADC sees them
- unity gain is the most stable and easiest to validate around a long-ish cable

If later logging shows the ADC noise floor still dominates after buffering,
moving to a small fixed gain such as 2x is reasonable, but it is not justified
yet for this pass.

## Proposed single-channel circuit

### High-level behavior

1. The **recording electrode pad** lands almost directly beside the op-amp input
2. The LMP7701 runs as a **voltage follower**
3. A small series resistor at the output helps isolate cable capacitance
4. A separate conductor carries the **reference electrode** back unchanged to
   `AINxN`
5. The flex therefore improves the vulnerable `AINxP` path without changing the
   existing channel semantics

### Schematic concept

```text
REC_PAD --- R1 ---+----> U1 IN+
                  |
                 R2
                  |
                VBIAS

U1 = LMP7701 unity follower
U1 OUT ----+---- U1 IN-
           |
          R6
           |
        BUF_OUT ------------------------------> ADS131M08 AINxP

REF_PAD --------------------------------------> ADS131M08 AINxN

VPROBE_IN -- R5 -- VPROBE_LOCAL ---> U1 V+
                  |
                 C2
                  |
                 GND
                  |
                 C3
                  |
                 GND

VPROBE_LOCAL -- R3 --+-- R4 -- GND
                     |
                    VBIAS
                     |
                    C1
                     |
                    GND
```

### Recommended populated values

| Ref | Value | Purpose |
|---|---:|---|
| U1 | LMP7701MF/NOPB | low-bias, low-noise SOT-23-5 buffer |
| R1 | 100 kΩ | input series resistor; limits fault/ESD current and slows hot-plug transients |
| R2 | 47 MΩ | weak bias return to mid-supply so the input does not float hard when electrode is disconnected |
| R3, R4 | 1 MΩ each | generate mid-supply `VBIAS` |
| C1 | 1 µF | low-frequency bypass for `VBIAS` |
| R5 | 22 Ω | simple supply filter from cable/battery rail into local op-amp rail |
| C2 | 100 nF | close high-frequency decoupler at U1 |
| C3 | 4.7 µF | local bulk decoupling for the probe head |
| R6 | 51 Ω | output isolation for capacitive cable loading |

### Optional, not-populated-first protection footprint

If handling damage or ESD proves to be a real problem, add footprints for two
**very low leakage clamp diodes** from the op-amp input node to the local rails,
but leave them **DNI by default**.

Why optional:

- ordinary TVS parts usually leak far more than the LMP7701 input current
- leakage on a very high-impedance bioelectric source can create more error than
  the protection is worth
- R1 already gives a simple first-line current limit that is hobbyist-friendly

For a follow-up revision, candidate parts to bench-check are low-leakage small
signal diodes such as **BAV199/BAS116-class** devices, but only after measuring
their actual room-temperature leakage in the intended assembly.

## Biasing and supply notes

### Preferred rail assumption

Assume a **3.3 V local rail** for the documented values above:

- derived from the dry-zone ADC/control enclosure and carried on the cable, or
- derived from a tiny local battery if you want the probe electrically quiet and
  can tolerate battery replacement

The LMP7701 also works above 3.3 V, but the ADC-side common-mode limits still
have to be respected.

### What `VBIAS` does here

`VBIAS` is **not** trying to force the live electrode signal to mid-supply under
normal operation. It is only a **weak return path** so the op-amp input does not
go completely undefined if:

- the electrode is unplugged
- the surface dries out
- the contact momentarily opens

That keeps the follower from railing unpredictably during open-circuit handling.

### Important common-mode caveat

This first-pass flex head assumes the recording/reference electrode common-mode
seen by the system stays inside a sensible single-supply operating window for
both:

- the **LMP7701 input/output**
- the **ADS131M08 absolute input limits**

If bench measurements show large DC offsets or negative-going excursions outside
that window, this simple follower is not enough by itself. In that case use one
of these next steps instead:

1. a buffered **recording + reference** electrode pair, or
2. a dry-zone differential amplifier / instrumentation stage with explicit
   common-mode control

## Output drive and cable return

The output is no longer the ultra-high-impedance electrode node, so it can be
treated more like a normal low-level analog signal.

Recommended cable behavior:

- `BUF_OUT` and ground/return should travel as a **twisted pair**
- the passive `REF_PAD` conductor should also return in the same harness so the
  ADS131M08 still sees `AINxP` versus `AINxN`
- keep the run modest, ideally still within the existing
  `hardware/pin-board.md` guidance of roughly **< 1-2 m**
- keep any overall cable shield tied at the **ADC end only**

`R6 = 51 Ω` should be placed **at the op-amp output pin**, not at the far end of
the cable. If later testing shows peaking or ringing with a specific harness,
raising R6 toward **100 Ω** is the first simple stability tweak.

The existing ADC-module-side RC/RFI network can remain in place; this flex stage
does not replace sensible board-side input filtering.

## Flex PCB layout concept

### Proposed physical partition

Use a **two-island flex**:

1. **electrode head island** near the recording contact
   - U1, R1, R2, R6, C2, C3, R3, R4, C1
   - exposed recording/reference pads
2. **cable/connector island**
   - wire-to-board connector or flex tail pads
   - strain relief

Connect the two islands with a narrow flex neck carrying only the few required
traces.

### Recommended stackup direction

Use a **simple 2-layer flex** with:

- polyimide core in the common **25 µm class**
- rolled-annealed copper in the **12-18 µm class**
- coverlay on both sides

Those are common flex-friendly ranges, but the exact stackup and minimum rules
must be confirmed with the chosen fabricator. This document does **not** claim a
fab-specific certified stackup.

### Bend-zone rules of thumb

Use conservative hobby-scale flex rules:

- keep **components, vias, and pad edges out of the bend**
- keep copper pours out of the bend unless the fab explicitly recommends
  otherwise
- prefer traces that run **along** the bend direction, not straight across it
- avoid sharp corners; use arcs/45° routing and teardrops at pads
- keep the dynamic bend radius roughly **>= 10x finished flex thickness**
  (static installation can sometimes tolerate less, but verify with the fab)
- keep component bodies at least about **1-2 mm away from the bend line**
  depending on package size and stiffness

These are intentionally approximate because actual limits depend on copper
weight, coverlay, stiffener thickness, and whether the bend is one-time or
repeated.

### High-impedance input placement

The most important layout rule is simple:

> keep the raw electrode path from `REC_PAD` to the LMP7701 input as short,
> clean, and contamination-resistant as possible

Practical implications:

- place `REC_PAD`, R1, and U1 so the raw high-Z trace is only a few millimeters
- do **not** route the raw input through the flex neck
- keep flux residue and exposed adhesive away from the input area
- if possible, add a **guard trace / guard ring tied to U1 output** around the
  non-inverting input node on the stiffened head island

Because the op-amp is in unity gain, the output is nearly the same potential as
the input, so a driven guard is a realistic way to reduce leakage across a damp
or dirty surface.

### Stiffeners

Add stiffeners under:

- the **SOT-23-5 op-amp and passives**
- any **JST-SH or FFC tail landing area**

Good first-pass choices:

- thin FR4 stiffener under the connector island
- thin FR4 or polyimide stiffener under the component island

This improves assembly yield and reduces solder-joint stress. Do **not** let the
stiffener extend into the intended bend zone.

### Trace/space guidance

For a hobby-ordered flex, design around **comfortable**, not heroic, rules:

- start around **6/6 mil** trace/space if the fab supports it well
- use wider traces for supply and ground where space allows
- keep the raw input trace narrow only if needed for geometry; electrical
  current here is tiny

If the chosen vendor easily supports 4/4 mil, that is useful margin, but this
concept does not depend on pushing below normal low-cost flex limits.

### Connector choice

#### Electrode side

Use **exposed ENIG pads** on the head island rather than a heavy connector:

- one pad for `REC_PAD`
- one pad for `REF_PAD`

That lets you solder or bond short electrode leads directly to the flex while
keeping the active circuit very close to the recording point. For a removable
electrode, use a separate tiny pigtail or stud bonded to those pads rather than
placing a bulky connector on the high-impedance head.

#### Cable side

For the first physical build, prefer a **4-pin JST-SH** on a stiffened island:

- **BM04B-SRSS-TB(LF)(SN)** on the flex
- mating cable harness to the dry-zone electronics

Suggested pinout:

1. `VPROBE_IN`
2. `GND`
3. `BUF_OUT`
4. `REF_PASS`

Why JST-SH here instead of a raw flex tail:

- easier hobby assembly
- easier cable replacement
- less dependency on matching FFC hardware during early experiments

If the design later grows into a multi-channel probe ribbon, an **FFC/FPC tail**
becomes more attractive.

## BOM

| Ref | Qty | Part | Example part number | Notes |
|---|---:|---|---|---|
| U1 | 1 | Precision op-amp | **LMP7701MF/NOPB** | preferred device, SOT-23-5 |
| R1 | 1 | 100 kΩ, 1%, 0603 | **Yageo RC0603FR-07100KL** | input series resistor |
| R2 | 1 | 47 MΩ, 1%, 0603 | **Vishay CRCW060347M0FKEA** | weak bias return |
| R3, R4 | 2 | 1 MΩ, 1%, 0603 | **Yageo RC0603FR-071ML** | divider for `VBIAS` |
| R5 | 1 | 22 Ω, 1%, 0603 | **Yageo RC0603FR-0722RL** | supply filtering |
| R6 | 1 | 51 Ω, 1%, 0603 | **Yageo RC0603FR-0751RL** | output isolation |
| C1 | 1 | 1 µF, X7R, 0603 | **Murata GRM188R71C105KA12D** | `VBIAS` bypass |
| C2 | 1 | 100 nF, X7R, 0603 | **Murata GRM188R71H104KA93D** | local decoupling |
| C3 | 1 | 4.7 µF, X5R/X7R, 0603 | **Murata GRM188R60J475KE19D** | local bulk cap |
| J1 | 1 | 4-pin JST-SH header | **BM04B-SRSS-TB(LF)(SN)** | cable-side connector |
| PAD_REC, PAD_REF | 2 | ENIG flex contact pads | board feature | electrode-side attachment |
| D1, D2 | 0-2 | low-leakage clamp diodes | optional footprint only | DNI until tested |

## What was actually produced in this pass

Produced:

- this design note
- a SKiDL schematic/netlist generator:
  `hardware/kicad/generate_active_probe_flex.py`

Not produced in this pass:

- a finished `.kicad_pcb` flex layout
- a fabricated cable harness
- measured stability/noise data

That is deliberate. A real flex layout is feasible later, but the narrowest
useful first deliverable here is the **documented circuit plus a machine-readable
netlist generator**.

## Recommended bench checks before layout freeze

1. Verify the electrode common-mode really stays inside the intended single-rail
   window
2. Compare:
   - passive lead straight to ADC
   - active buffered lead with same cable
3. Check for oscillation with the intended cable length and connector harness
4. Evaluate whether the passive reference lead now dominates noise pickup
5. Decide whether the optional input clamp footprint should remain DNI
