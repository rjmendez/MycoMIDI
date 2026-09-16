# Active electrode probe (unity-gain buffer at the flex-PCB tab)

Grounded in loci investigation `mycomidi-adc-project`, findings tagged
`active-probe-opamp` (op-amp datasheet comparison) and the existing
driven-shield/DRL bias-electrode design note in `hardware/pin-board.md`
("Future driven-ground / bias-electrode option").

## Why an active probe

`hardware/pin-board.md` documents the current passive pin board: recording
electrodes go straight to `AINxP` on the ADS131M08 module over 1-2m of
twisted-pair cable. A bare electrode/substrate contact is a very
high-impedance source (likely well over 1 MΩ, and highly variable with
substrate moisture). Combined with a long cable run, that high source
impedance turns the cable into an efficient antenna for 50/60Hz mains hum
and picks up microphonic/flex noise from cable movement, both of which show
up as noise directly in-band with the tiny mycelium bioelectric signal.

The standard fix (used in EEG/ECG front ends, and referenced in
`pin-board.md`'s note on OpenBCI Ganglion's reference/bias design) is to move
a **unity-gain buffer op-amp right to the electrode**, on a small flex PCB
tab. Converting the high-impedance electrode signal to a low-impedance
buffered signal *before* the long cable run means the cable no longer needs
to reject noise on behalf of a high-Z source -- it only has to carry a
low-Z buffered signal, which is far less susceptible to mains hum and
microphonic pickup. This is a separate, complementary idea to the
driven-ground/DRL bias-electrode note already in `pin-board.md`: DRL drives
an inverted common-mode estimate back into the substrate; the active probe
buffers the recording electrode itself. Both can coexist in the same
front-end revision.

This document is a design note only: not yet built, not yet tested, and
should be validated on resistor/phantom loads before any biological
experiment, same caveat as the existing bias-electrode note.

## Requirements recap

- Low input bias current (JFET/CMOS input) -- source impedance from the
  electrode/substrate contact is very high, so any material `Ib` creates a
  DC error and/or an extra noise-current term across `Zsource`.
- Low input-referred voltage noise -- the buffer sits ahead of any
  filtering, right where hum/microphonic pickup is worst.
- Single-supply friendly at 3.3V or 5V -- the system is Raspberry-Pi/battery
  powered, no split/dual supply available.
- Small single-channel package for a tiny flex PCB tab -- SOT-23-5, SC70-5,
  or similar.
- Reasonably low quiescent current -- the buffer is always-on and the
  system may be battery-powered.

## Candidates compared (real datasheet numbers)

All figures are typical @25°C unless noted, pulled directly from
manufacturer datasheet PDFs (TI datasheets via `pdftotext -layout` on
`ti.com/lit/ds/symlink/*.pdf`; ADI datasheets via an archive.org mirror of
`analog.com/media/.../*.pdf` after direct fetches to analog.com timed out
from this environment).

| Part | Input bias current | Voltage noise density | GBW | Quiescent current (per amp) | Supply range | Package (single-ch) |
|---|---|---|---|---|---|---|
| **TI LMP7701** | ±200 fA typ (femtoamp-scale) | 9 nV/√Hz @ 1kHz | 2.5 MHz | 715 µA typ | 2.7V–12V | 5-pin SOT-23 |
| TI TLV9061 | ±0.5 pA typ | 10 nV/√Hz @ 10kHz (headline spec) | 10 MHz | 538 µA typ | 1.8V–5.5V | SOT-23-5 / SC70-5 / X2SON-5 (0.8×0.8mm) |
| ADI AD8605 | ±0.2 pA typ / 1 pA max | 8 nV/√Hz @ 1kHz | 10 MHz | 1.0 mA typ / 1.2 mA max | 2.7V–5.5V | 5-lead SOT-23 / 5-ball WLCSP |
| ADI ADA4505-1 | 0.5 pA typ / 2 pA max | 65 nV/√Hz @ 1kHz | 50 kHz (low) | 9–11.5 µA typ/max | 1.8V–5.5V | 5-lead SOT-23 / 6-ball WLCSP |
| TI OPA333 | ±70 pA typ / ±200 pA max | not explicitly specified; noise-density plot shows a flat, no-1/f broadband floor visually in the tens-of-nV/√Hz range (chopper-stabilized) | 350 kHz (low) | 17 µA typ / 25–28 µA max | 1.8V–5.5V | 5-pin SOT-23 |

Sources: TI SBOS351E (`ti.com/lit/ds/symlink/opa333.pdf`), TI TLV9061
datasheet (`ti.com/lit/ds/symlink/tlv9061.pdf`), TI LMP7701/7702/7704
datasheet (`ti.com/lit/ds/symlink/lmp7702.pdf` -- the electrical
characteristics table is shared across the single/dual/quad family; only
`ISY` differs by part), ADI ADA4505-1/-2/-4 Rev E datasheet, ADI
AD8605/8606/8608 Rev O datasheet (both fetched via an archive.org mirror of
`analog.com/media/en/technical-documentation/data-sheets/...pdf` after
direct analog.com fetches timed out repeatedly from this environment).

**Gap:** Microchip MCP6001 was in scope for this comparison but could not be
independently verified this session -- both `ww1.microchip.com` and
`microchip.com` return HTTP 403 (Akamai bot-detection) from this sandbox's
network, via both direct `curl` and the `web_fetch` tool, and the
archive.org copy of the datasheet resolved to the Internet Archive's
donation-appeal wrapper page instead of the actual PDF. MCP6001 is a
reasonable, very-low-cost CMOS single-supply candidate by general
reputation, but it is intentionally left out of the numeric table above
rather than citing remembered/unverified figures.

**Gap:** Live distributor unit pricing (Digikey/Mouser/LCSC) could not be
independently re-verified this session for any of the five parts above --
Digikey product-detail pages returned wrong/generic cached content or 403
for these exact part numbers, Mouser timed out, and LCSC search pages
returned only the client-side-rendered search shell with no visible results
to the fetch tool. This mirrors the same pricing-fetch gap already recorded
in this investigation for the ADS1299/ADS131M08 comparison. All five parts
are, by general market reputation, inexpensive single op-amps in the
roughly $0.30-$1.50/unit range in small quantities from Digikey/Mouser/LCSC
(LCSC/JLCPCB in particular tends to stock TI SOT-23-5 single op-amps like
TLV9061 as basic/extended parts) -- but treat that price range as
unverified this session, not a sourced datasheet fact.

## Recommendation

**Primary recommendation: TI LMP7701 (single, 5-pin SOT-23).**

Rationale:
- Its ±200 fA input bias current is essentially tied for best-in-class with
  AD8605, and far better than TLV9061 (0.5 pA) or ADA4505-1/OPA333
  (tens-to-hundreds of pA class). This matters most here specifically
  because the electrode/substrate source impedance is extremely high and
  moisture-dependent -- any material `Ib` turns directly into a DC offset
  and/or extra noise-current term across `Zsource` that can swamp the tiny
  mycelium bioelectric signal.
- Its 9 nV/√Hz voltage noise is close to AD8605's 8 nV/√Hz (the lowest of
  the group), clearly better than TLV9061's 10 nV/√Hz, and dramatically
  better than ADA4505-1's 65 nV/√Hz or OPA333's higher (chopper) broadband
  floor.
- It has no chopper-stabilization switching artifacts (unlike OPA333),
  which matters specifically at the electrode tip, ahead of any
  downstream filtering -- injected switching noise here couples straight
  into the raw signal before anything can remove it.
- 2.5 MHz GBW is comfortably above what a DC-to-few-Hz mycelium signal
  needs, with margin for common-mode-rejection loop stability up through
  the 50/60Hz mains band and its harmonics.
- 715 µA quiescent current is not the lowest of the group, but it is
  negligible next to the Raspberry Pi + ADS131M08 system's own power draw,
  so it's an acceptable tradeoff for the bias-current and noise gains.
- Ships in a 5-pin SOT-23, fitting the flex-PCB-tab footprint requirement,
  and its 2.7V-12V supply range comfortably covers both the 3.3V and 5V
  single-supply rails called out in the requirements (it is simply not
  usable below 2.7V, e.g. if a 2xAA/2.4V rail were ever substituted).

**Runner-up / lower shortage-risk alternative: TI TLV9061.** A mainstream
CMOS (not chopper, not femtoamp-exotic) part: 0.5 pA bias current is still
very good for this application, 10 nV/√Hz noise trails LMP7701 by only
1 nV/√Hz, 10 MHz GBW and rail-to-rail I/O give the most design margin of
the group, and it is offered in the smallest packages compared here
(SOT-23-5, SC70-5, and X2SON-5 at 0.8×0.8mm), plus a shutdown-capable
"S" variant (TLV9061S) if the design ever wants to power the buffer down
between readings to save battery. Tradeoff: highest always-on quiescent
current of the low-noise group (538 µA -- close to LMP7701's 715 µA; the
real distinguishing tradeoff is that TLV9061 is a general-purpose
cost/availability play rather than a bias-current specialist).

**Ultra-low-power alternative if IQ dominates the power budget:**
ADA4505-1 -- quiescent current is only 9-11.5 µA, roughly 50-70x lower than
LMP7701/TLV9061 -- at the cost of much higher noise (65 nV/√Hz) and far
lower bandwidth (50 kHz GBP). Worth considering only if the design is
willing to trade noise floor for battery runtime, and would likely need an
extra downstream low-pass/gain stage to recover SNR.

**Avoid for this spot in the chain:** TI OPA333, despite its excellent DC
offset/drift spec. Its auto-zero/chopper architecture's switching artifacts
are a poor fit for a buffer placed directly at the electrode, ahead of any
filtering, in a system that is already fighting mains hum and
microphonic/flex noise pickup -- exactly the class of interference
chopping artifacts would compound rather than help with.

## Open follow-ups (not yet done)

- Verify MCP6001 numbers directly once a reachable datasheet mirror is
  found (Microchip's own domains 403 from this environment).
- Get real distributor pricing/stock (Digikey/Mouser/LCSC) once those
  sites are reachable from wherever this design work continues.
- Prototype LMP7701 on a resistor/phantom-load bench rig per the same
  validate-before-biological-experiment discipline already used for the
  DRL bias-electrode note in `hardware/pin-board.md`.
