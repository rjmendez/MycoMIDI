# ADS1299 migration plan

## 1. Executive summary

**Recommendation: later, not now.**

Migrating MycoMIDI from the current ADS131M08 direction to ADS1299 is only justified if real measurements show the present front end is **noise-floor-limited or mains-hum-limited**, and that a driven-bias electrode is likely to recover biologically meaningful signal that the current architecture is missing. The strongest ADS1299 advantages are:

- lower input-referred noise in the findings reviewed (roughly **1.08 uV RMS** vs **~1.5 uV RMS** for ADS131M08 in the compared operating points), and
- a built-in **bias-drive / DRL-style amplifier**, which the ADS131M08 does not provide.

What does **not** justify switching by itself:

- channel count alone,
- "lab-grade" aesthetics,
- or the TQFP-64 package alone.

Both families can support MycoMIDI's practical scale, but ADS131M08 matches the current repo's architecture far better: it is already documented, dramatically cheaper, simpler to scale with one CS per module, and does not force a bias-drive redesign before the first serious data-logging campaign.

So the honest call is:

- **Stay on ADS131M08 now** for the first real logging hardware.
- **Switch to ADS1299 later** only if bench data proves the biology is being hidden by front-end noise/common-mode pickup rather than by weak or inconsistent biological signals.
- **Do not switch at all** if the present setup turns out to be biology-limited, not ADC-limited.

## 2. Comparison table

| Dimension | ADS1299 | ADS131M08 (current direction) | What it means for MycoMIDI |
|---|---|---|---|
| Noise floor | Findings reviewed put ADS1299 around **1.08 uV RMS** input-referred noise in the compared low-rate/high-gain context | Findings reviewed put ADS131M08 around **~1.5 uV RMS** in the compared context | ADS1299 is attractive only if that delta matters in real mushroom recordings |
| Bias-drive / DRL | **Built in** | **Not built in**; would need external bias-drive circuitry | This is the main reason to consider ADS1299 |
| Channel scaling mechanism | 8 ch/chip, common 2-chip daisy precedent for 16 ch; beyond that gets less standard | 8 ch/chip, shared clock + shared bus + one CS/module already documented for 8/16/24/32 ch | This is a signal-integrity decision first, not a channel-count decision first |
| Cost | Findings captured **roughly $75-106/unit** ADS1299 listings; re-check before BOM lock | Repo hardware docs currently say **~$5/unit** | ADS1299 is an order-of-magnitude cost jump |
| SPI / firmware complexity | More stateful (`START`, `STOP`, `RDATAC`, `SDATAC`, daisy-read framing) | Simpler continuous-frame style for the planned logger path | ADS1299 is feasible, but is more driver work |
| Package / hand soldering | **TQFP-64, 0.5 mm pitch** | **TQFP-32, 0.8 mm pitch** | The IC is still hand-solderable for this project owner; passive ecosystem is the bigger assembly concern |
| Power | Findings synthesis favored ADS131M08 on power/flexibility; ADS1299 is not the low-power winner | Better fit for long-running lower-power capture | Matters more for always-on logging than for a short bench prototype |

## 3. Concrete numbers

### Noise

- ADS1299 finding set: about **1.08 uV RMS** input-referred noise in the compared operating point.
- ADS131M08 finding set: about **1.5 uV RMS** in the compared operating point.
- Treat that as a meaningful but not magical improvement: it helps only if the experiment is actually ADC-noise/common-mode limited.

### Pricing

- ADS1299 research findings captured **stocked distributor listings** in the rough range of **$75-106 per unit**.
- ADS131M08 is documented in this repo's current `hardware/adc-module.md` as about **$5 per unit**.
- Honest caveat: one later pricing pass also hit 403/timeout / JS-gated distributor pages, so these numbers should be treated as **planning-grade**, not procurement-grade. Re-check live quotes before locking a BOM.

### Package

- **ADS1299:** **TQFP-64, 0.5 mm pitch**.
- **ADS131M08:** **TQFP-32, 0.8 mm pitch**.

## 4. Sourcing / availability

Research findings support the following sourcing picture:

- **No official KiCad symbol/footprint** for ADS1299 was found in the standard libraries reviewed.
- **No confirmed LCSC/JLCPCB catalog listing** was found.
- **DigiKey and Newark stock** were observed in the research findings.
- TI's product pages/datasheet availability confirm ADS1299 is a real, current family, but that is not the same thing as having cheap hobbyist-source procurement or turnkey KiCad/JLC library support.

Practical implication: if MycoMIDI migrates, expect to make or vet a **custom symbol + footprint + sourcing path** rather than relying on the same low-friction path used for commodity parts.

## 5. Assembly reality check

The package itself should **not** be framed as the blocker here.

The project owner has already noted that **drag-soldering fine-pitch ICs under a microscope is manageable**. In that context, the ADS1299's **TQFP-64 / 0.5 mm** package is more work than the ADS131M08, but not an automatic show-stopper.

The real assembly-risk question is the **supporting passive network**, especially if the migration also triggers a fuller bias-drive / reference / protection redesign. The reviewed reference designs include parts like:

- **1 MOhm / 1-1.5 nF** bias feedback RCs,
- **2.2 kOhm + 1 nF** per-channel RC arrays,
- protection arrays such as **TPD4E1B06DCKR**.

So the thing to watch is not "can this person solder a TQFP-64?" but rather:

- how many extra passives appear,
- how small they are,
- and whether the layout pushes toward a lot of sub-0402 hand placement.

For this project, **tiny passive count/size is the real assembly cost driver**, not the IC package by itself. A board house may still be convenient, but this migration plan should **not** assume PCBA is mandatory just because ADS1299 is involved.

## 6. Driver rewrite scope

The good news from the `ads1299-spi-driver` findings is that the software impact is fairly contained.

- `tools/logger.py` can stay **unchanged** as long as the new driver still presents the same high-level shape (`read_all_volts()` returning calibrated channel data).
- The expected rewrite is concentrated in a new **ADS1299 driver class**.
- Findings estimated that at roughly **100-200 new lines** of driver code in the current project style.
- The complexity increase vs ADS131M08 is in the ADC protocol model, not in the logger pipeline: startup/config sequencing, `START`/`STOP`, `RDATAC` vs `SDATAC`, DRDY handling, and daisy-read framing.

Scaling caveat:

- ADS131M08's current plan scales cleanly by sharing clock/SPI and adding **one CS per module**.
- ADS1299's common real-world precedent is the **2-chip daisy arrangement** used to reach **16 channels**.
- If MycoMIDI later insists on going past 16 channels in a simple single-chain topology, ADS1299 becomes less elegant and likely needs extra bus partitioning or other glue logic.

So the driver rewrite is **not** the scary part; the topology gets awkward first.

## 7. FPGA

The reviewed findings do **not** justify an FPGA for this migration.

Reasons:

- MycoMIDI's actual sample-rate / throughput needs are modest.
- ADS1299 already handles the synchronization problem that matters most here in silicon.
- The more realistic host-side failure mode is Linux/Python acquisition jitter, which is better addressed by an interrupt/DMA-capable **MCU front-end** than by jumping straight to FPGA.

Conclusion: **no FPGA for ADS1299 capture right now**. If host timing ever becomes a real problem, use an MCU front-end first.

## 8. Reference design guidance

If a future migration proceeds, the reviewed designs already provide concrete starting values for a first-pass bias/reference redesign:

- **TI ADS1299 evaluation board:** **R8 = 392 kOhm, C20 = 10 nF**.
- **OpenBCI Cyton:** **R5 = 1 MOhm, C16 = 1 nF** in the bias path; plus **2.2 kOhm / 1 nF** per-channel RC arrays; protection parts including **TPD4E1B06DCKR** TVS arrays.
- **Low-Cost-EEG design:** **R9 = 1 MOhm, C14 = 1.5 nF**.

These should be treated as **starting references**, not copied blindly. Electrode geometry, cable length, shielding, common-mode environment, and the actual mushroom/electrode impedance will still need bench validation.

## 9. Recommendation / trigger condition

### Stay on ADS131M08 now if:

- the immediate goal is to get the first serious multi-channel logger built,
- cost matters,
- the current no-bias-drive hardware direction has not been empirically falsified,
- and 8/16/24/32-channel scaling simplicity is still valuable.

### Pull the trigger on ADS1299 only if bench evidence shows all of the following:

1. the current signal chain is **noise-floor-limited or common-mode-limited**, not biology-limited,
2. the missing information appears recoverable by **better common-mode rejection / driven-bias behavior**, not just by more software filtering,
3. the extra BOM cost and passive complexity are acceptable,
4. and the likely channel plan is still compatible with ADS1299's awkwardness beyond the common 2-chip / 16-channel story.

That means the best migration trigger is something like:

> **Measured evidence that the present ADS131M08-based front end is leaving biologically real signal on the table because mains/common-mode noise dominates, and that a driven-bias electrode strategy is the most plausible fix.**

Absent that evidence, the honest engineering recommendation is:

> **Stay with ADS131M08 for now.**

## 10. Other offload note

A stronger low-risk hardware win right now is **not** an ADC swap.

The broader hardware-offload survey found that a dedicated **hardware MIDI synth** path (for example **Dream SAM2695** or **VLSI VS1053b**) is a cleaner near-term upgrade:

- `tools/musicgen.py` already emits raw **General MIDI** byte streams / events in a form that maps naturally to external synth hardware,
- so this offload is much closer to **"plug into existing software output"** than redoing the analog front end,
- and it changes the part of the system that is already musically productive, rather than reopening the whole bioelectric acquisition stack.

If choosing where to spend hardware effort first, the evidence supports:

1. **build and measure the current ADS131M08 logger path**,
2. **consider hardware MIDI synthesis before an ADC-family swap**,
3. migrate to ADS1299 **only after** measurements show the analog front end is the real bottleneck.
