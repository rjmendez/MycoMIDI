# KiCad CLI via Docker

This directory uses the official `kicad/kicad:9.0` Docker image so KiCad CLI
commands can run without installing KiCad on the host.

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
SBAS950B Rev. B, Table 5-1. The actual package exposes `REFIN` and `CAP`
(not separate `REFP` / `REFN` pins), so the SKiDL netlist uses a readability
alias where `REFP` lands on `REFIN` and the return side is the common ground
node.

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

### Current status

- The board file is a real KiCad PCB with outline, placed footprints, and a
  Freerouting-generated two-layer route.
- The ADS131M08 pinout is datasheet-sourced for the TQFP-32 package.
- The previous handwritten routing pass created many same-layer crossings and
  shorts; the flow now exports DSN and imports a Freerouting `.ses`, which
  produces a clean DRC on this board.
- `scripts/autoroute-adc-board.sh` is the supported regeneration path for the
  checked-in `adc_board_8ch.kicad_pcb`.
