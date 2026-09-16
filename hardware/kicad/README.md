# KiCad CLI via Docker

This directory uses the official `kicad/kicad:9.0` Docker image so KiCad CLI
commands can run without installing KiCad on the host.


## Lifted from open-source KiCad MCP tooling

MycoMIDI ports a few small, standalone artifacts from MIT-licensed KiCad MCP
servers so hardware automation can stay lightweight and auditable:

- `lib/kicad_sexpr_cst.py` — byte-preserving KiCad S-expression CST parser and serializer,
  adapted from ProductOfAmerica's `mcp-server-kicad`
- `lib/atomic_write.py` — same-directory atomic-write helper adapted from the same project
- `lib/drc_classify.py` — KiCad DRC/ERC JSON classification helper adapted from
  `oaslananka/kicad-mcp-pro`
- `lib/pin_header_footprint_gen.py` — closed-form through-hole pin-header /
  electrode-connector footprint generator adapted from `oaslananka/kicad-mcp-pro`

Why these were lifted: they are pure-Python, portable, and useful for safely
editing `.kicad_pcb` / `.kicad_sch` text and generating simple connector
footprints in environments where the full KiCad Python API is unavailable.

See [`lib/THIRD_PARTY_LICENSES.md`](lib/THIRD_PARTY_LICENSES.md) for upstream
provenance and license texts.

## Prerequisites

- Docker installed and working for the current user
- The repo checked out locally
- Python 3 for the SKiDL netlist generator

## Wrapper

Run KiCad CLI through `scripts/kicad-cli.sh` from the repo root:

```bash
./scripts/kicad-cli.sh version
./scripts/kicad-cli.sh sch erc hardware/kicad/demo/demo.kicad_sch --format json
./scripts/kicad-cli.sh pcb drc hardware/kicad/demo/demo.kicad_pcb --format json --exit-code-violations
```

The wrapper:

- pins KiCad to `kicad/kicad:9.0`
- mounts the current directory at `/work`
- runs the container as the current UID/GID to avoid root-owned outputs
- forwards all remaining arguments to `kicad-cli`

If you want to mount a different directory, use `--workdir`:

```bash
./scripts/kicad-cli.sh --workdir hardware/kicad/demo sch erc demo.kicad_sch --format json
```

## Demo project

`hardware/kicad/demo/` contains a tiny schematic and PCB used to prove the
Docker wrapper works end-to-end.

## ADS131M08 board generator

The repository now includes a real ADS131M08-based 8-channel board flow:

- `hardware/kicad/ads131m08_skidl.py` — shared SKiDL part/templates
- `hardware/kicad/generate_channel.py` — single-channel prototype using the real ADS131M08 pinout
- `hardware/kicad/generate_board.py` — 8-channel ADS131M08 netlist generator
- `hardware/kicad/build_adc_board_layout.py` — KiCad/`pcbnew` script that imports the generated netlist, places the board, and exports a Freerouting-ready DSN
- `scripts/autoroute-adc-board.sh` — reproducible KiCad → Freerouting → KiCad pipeline that writes the final routed PCB
- `hardware/kicad/adc_board/adc_board_8ch.net` — generated SKiDL netlist
- `hardware/kicad/adc_board/adc_board_8ch.kicad_pcb` — generated PCB layout

### What is modeled

- one real `ADS131M08` TQFP-32 (`Package_QFP:TQFP-32_7x7mm_P0.8mm`)
- all eight differential input pairs: `AIN0P/N` through `AIN7P/N`
- SPI header: `CS`, `SCLK`, `DIN`, `DOUT`, `DRDY`
- power/control header: `AVDD`, `DVDD`, `GND`, `CLKIN`, `SYNC_RESET`
- supply support caps: `100n + 1u` on `AVDD` and `DVDD`
- `REFIN` capacitor and `CAP` LDO capacitor
- one 2x8 electrode header following the documented MycoMIDI convention:
  - `CHx_REC -> AINxP`
  - `CHx_REF -> AINxN`

### Pinout note

`generate_board.py` uses the ADS131M08 TQFP-32 pin numbers from TI datasheet
SBAS950B Rev. B, cross-checked against the Figure 5-1 top-view pin diagram as
well as Table 5-1. The actual package is a real four-side TQFP-32 footprint
(`Package_QFP:TQFP-32_7x7mm_P0.8mm`), not a two-row/DIP-style abstraction. It
exposes `REFIN` and `CAP`
(not separate `REFP` / `REFN` pins), so the SKiDL netlist uses a readability
alias where `REFP` lands on `REFIN` and the return side is the common ground
node.

## Fractal routing experiment

This repo also includes `hardware/kicad/fractal_trace_router.py`, which writes a
deliberately absurd Hilbert-curve copper route into
`hardware/kicad/demo/fractal_demo.kicad_pcb`.

On the padless demo board it closes the shape with a short return path so KiCad
does not flag the decorative trace as dangling copper.

Generate the board copy:

```bash
python3 hardware/kicad/fractal_trace_router.py
```

Then validate and preview it:

```bash
./scripts/kicad-cli.sh pcb drc \
  --format json \
  --output hardware/kicad/demo/fractal_demo-drc.json \
  hardware/kicad/demo/fractal_demo.kicad_pcb

./scripts/kicad-cli.sh pcb export svg \
  --output hardware/kicad/demo/fractal_demo.svg \
  hardware/kicad/demo/fractal_demo.kicad_pcb
```

See `hardware/kicad/fractal-routing-notes.md` for the blunt electrical caveats.

## Known limitations

- KiCad is intentionally pinned to `9.0`; update the wrapper if the project
  should move to a newer KiCad release.
- The wrapper only mounts one directory. Input paths must live under the
  mounted tree.
- Generated report files follow KiCad CLI behavior and default to the mounted
  working directory rather than the source file's directory.

## Generated electrode footprints

`hardware/kicad/footprints/` now contains generated 1x04, 1x08, and 1x16
through-hole connector footprints for candidate electrode headers.

These files were generated with
[`generate_pin_header_footprint`](https://github.com/rjmendez/kicad-mcp-tools)
from `kicad-mcp-tools`, then copied back into MycoMIDI as a real dogfooding
loop: MycoMIDI requirements informed the tool, the tool produced reusable
KiCad assets, and those assets are now tracked here for the hardware design.

They are candidates for 4-channel, 8-channel, and 16-channel electrode
connector breakouts as the ADS131M08 hardware scales beyond the first module.

### Setup

From the repo root:

```bash
python3 -m venv hardware/kicad/.venv
. hardware/kicad/.venv/bin/activate
pip install skidl kiutils kicad-skip
python -c "import skidl, kiutils, skip"
```

### Generate the netlist

```bash
. hardware/kicad/.venv/bin/activate
python hardware/kicad/generate_board.py
```

### Generate the placed PCB + DSN

`build_adc_board_layout.py` needs KiCad's `pcbnew` Python module, so run it in
the pinned KiCad container. It writes both a placed KiCad board and a Specctra
DSN for Freerouting:

```bash
docker run --rm -v "$PWD:/work" -w /work \
  kicad/kicad:9.0 \
  python3 hardware/kicad/build_adc_board_layout.py \
    --output hardware/kicad/adc_board/adc_board_8ch_unrouted.kicad_pcb \
    --dsn-output hardware/kicad/adc_board/adc_board_8ch_unrouted.dsn
```

### Autoroute with Freerouting

The clean routed board is now produced with Freerouting rather than the prior
handwritten point-to-point router:

```bash
./scripts/autoroute-adc-board.sh
```

### Validate with DRC

```bash
./scripts/kicad-cli.sh pcb drc \
  --format json \
  --output hardware/kicad/adc_board/adc_board_8ch-drc.json \
  --exit-code-violations \
  hardware/kicad/adc_board/adc_board_8ch.kicad_pcb
```

### Fractal dead-space fill on the real ADC board

The repository also includes `hardware/kicad/fractal_fill.py`, which reuses the
shared Hilbert geometry generator to place decorative dead-space fill on the
real ADS131M08 board while staying on the board's actual `GND` net.

Run it in-place on the checked-in routed board:

```bash
python3 hardware/kicad/fractal_fill.py \
  --profile adc-board-gnd \
  --input hardware/kicad/adc_board/adc_board_8ch.kicad_pcb \
  --output hardware/kicad/adc_board/adc_board_8ch.kicad_pcb
```

The real-board profile:

- confirms the routed board already defines `GND` in the KiCad net table and
  uses that real net id for every decorative copper segment/via
- keeps the decorative copper intentionally narrow at `0.1 mm`, which is much
  smaller than the board's functional `0.2 mm` routes because this art is only
  augmenting ground copper, not carrying a dedicated signal or power path
- validates the chosen empty rectangles against existing pads, vias, traces, and
  footprint bodies before writing the board
- leaves the masked/tented and exposed/unmasked copper variants electrically
  safe because both only connect `GND` to `GND`

### Current status

- The board file is a real KiCad PCB with outline, placed footprints, and a
  Freerouting-generated two-layer route.
- The ADS131M08 pinout is datasheet-sourced for the TQFP-32 package.
- The previous handwritten routing pass created many same-layer crossings and
  shorts; the flow now exports DSN and imports a Freerouting `.ses`, which
  produces a clean DRC on this board.
- The real ADS131M08 board now also carries decorative Hilbert fill on
  `F.Cu`/`B.Cu`/silkscreen; the copper art is tied to the existing `GND` net,
  not isolated on dummy art-only nets.
- C5/C6 silkscreen reference labels are placed explicitly in the generator so
  they stay readable and off the capacitor pads/traces after autorouting.
- `scripts/autoroute-adc-board.sh` is the supported regeneration path for the
  checked-in `adc_board_8ch.kicad_pcb`.
