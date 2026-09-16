# SKiDL channel prototype

`generate_channel.py` is a runnable proof of concept for one ADS131M08 electrode channel drawn at the schematic/netlist level with inline SKiDL parts.

It models the documented wiring convention from `hardware/adc-module.md` and `hardware/pin-board.md`:

- recording electrode -> `AINxP`
- shared reference electrode -> `AINxN`
- **not** the ADC's `REFIN`/`REFOUT` pin

## Setup

From the repo root:

```bash
python3 -m venv hardware/kicad/.venv
. hardware/kicad/.venv/bin/activate
pip install skidl kiutils kicad-skip
python -c "import skidl, kiutils, skip"
```

## Run

```bash
. hardware/kicad/.venv/bin/activate
python hardware/kicad/generate_channel.py
```

## Output

The script writes:

- `hardware/kicad/single_channel_prototype.net`

Today `channel_count = 1`, but the script uses a loop so the same pattern scales directly to more ADS131M08 differential channels.

## Notes

This prototype defines the ADC-channel symbol and electrode connector inline, so it does not depend on KiCad symbol libraries being installed on the machine.
SKiDL may still warn about missing `KICAD*_SYMBOL_DIR` or `fp-lib-table` settings on machines without KiCad installed, but the script should still complete and write the netlist because both symbols are defined inline.
