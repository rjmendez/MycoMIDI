# KiCad CLI via Docker

This directory uses the official `kicad/kicad:9.0` Docker image so KiCad CLI
commands can run without installing KiCad on the host.

## Prerequisites

- Docker installed and working for the current user
- The repo checked out locally

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

Expected behavior from the repo root:

```bash
./scripts/kicad-cli.sh sch erc hardware/kicad/demo/demo.kicad_sch --format json
./scripts/kicad-cli.sh pcb drc hardware/kicad/demo/demo.kicad_pcb --format json --exit-code-violations
```

On a clean checkout with Docker available, both commands should complete and
write JSON reports in the mounted working directory. If you run them from the
repo root as shown above, KiCad writes `demo-erc.json` and `demo-drc.json` to
the repo root unless you pass `--output` or use `--workdir hardware/kicad/demo`.

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

## SKiDL netlist generators

This directory also includes inline SKiDL scripts for electrode-side schematic
generation without requiring KiCad symbol libraries:

- `generate_channel.py`: one-channel proof of concept
- `generate_board.py`: 8-channel electrode-board base schematic

Both follow the documented ADS131M08 input semantics from
`hardware/adc-module.md` and the connector conventions from
`hardware/pin-board.md`:

- recording electrode -> `AINxP`
- reference electrode -> `AINxN`
- never wire an electrode to the ADC `REFIN`/`REFOUT` reference pins

### Setup

From the repo root:

```bash
python3 -m venv hardware/kicad/.venv
. hardware/kicad/.venv/bin/activate
pip install skidl kiutils kicad-skip
python -c "import skidl, kiutils, skip"
```

### Run

```bash
. hardware/kicad/.venv/bin/activate
python hardware/kicad/generate_channel.py
python hardware/kicad/generate_board.py
```

### Outputs

- `hardware/kicad/single_channel_prototype.net`
- `hardware/kicad/electrode_board_8ch.net`

`generate_board.py` maps all eight `AINxP/AINxN` differential pairs to one
16-pin `CONN_02X08`-style header footprint and leaves the future driven-ground
/ bias path as a clearly labeled `BIAS_DRIVE_TODO` placeholder net stubbed to
`TP1`.
