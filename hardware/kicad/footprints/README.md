# Generated electrode connector footprints

This directory holds generated KiCad footprint candidates for MycoMIDI
recording-electrode headers:

- `MycoMIDI_Electrode_1x04_2.54mm.kicad_mod`
- `MycoMIDI_Electrode_1x08_2.54mm.kicad_mod`
- `MycoMIDI_Electrode_1x16_2.54mm.kicad_mod`

They were generated with the
[`generate_pin_header_footprint`](https://github.com/rjmendez/kicad-mcp-tools)
function from `kicad-mcp-tools`, then copied back into this repository as a
concrete MycoMIDI -> kicad-mcp-tools -> MycoMIDI feedback loop.

Treat them as candidate footprints for future 4-channel, 8-channel, and
16-channel electrode connectors while the hardware interface is still being
refined.
