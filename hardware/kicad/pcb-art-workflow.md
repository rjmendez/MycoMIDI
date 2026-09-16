# PCB art workflow

This proof of concept keeps the **art-generation** step automated and honest, while acknowledging that the **KiCad import** step is still a manual GUI action today.

## What is included

- `hardware/kicad/fractal_art_generator.py` - original NumPy/Matplotlib implementation of a Koch snowflake generator
- `hardware/kicad/koch_snowflake_demo.svg` - example vector output generated from that script

The implementation was inspired by general fractal-art exploration, but it was written here from scratch and was **not copied** from the unlicensed `FarrelAD/Fractals-Design-Python` notebook.

## Generate the source artwork

Authoritative source output should remain vector SVG:

```bash
python3 hardware/kicad/fractal_art_generator.py \
  --depth 3 \
  --size 40 \
  --output hardware/kicad/koch_snowflake_demo.svg
```

If you want the same geometry as a raster derivative for KiCad's Image Converter, rerun the same generator with a PNG filename:

```bash
python3 hardware/kicad/fractal_art_generator.py \
  --depth 3 \
  --size 40 \
  --dpi 1200 \
  --output hardware/kicad/koch_snowflake_demo.png
```

## Recommended KiCad import path: Image Converter

For the current MycoMIDI workflow, the recommended maintained path is **KiCad's native Image Converter**. `svg2shenzhen` is not the primary recommendation here because it is abandoned, while KiCad's own tooling is maintained.

Why this is the primary recommendation:

- it is maintained by KiCad, unlike abandoned `svg2shenzhen`
- it produces a placeable footprint that is easy to move, rotate, and reuse
- it targets PCB layers directly, including `F.SilkS`

Current limitation: **there is no `kicad-cli` import subcommand for this step**, so the import remains GUI-only for now.

### Manual import steps

1. Generate the authoritative SVG with `fractal_art_generator.py`.
2. Generate a high-resolution PNG derivative from the same script if you plan to use Image Converter.
3. Open the board in KiCad PCB Editor.
4. Launch **Tools -> Image Converter**.
5. Load `hardware/kicad/koch_snowflake_demo.png`.
6. Adjust threshold/negative mode as needed for clean black-on-transparent tracing.
7. Export the result as a footprint on **`F.SilkS`**.
8. Add that footprint to the PCB and place it inside the board outline.
9. Save the board.
10. Re-run DRC.

## Alternative KiCad path: direct SVG import

If you want to keep the vector SVG directly, KiCad also supports:

- **File -> Import -> Graphics**

That path can import SVG graphics straight onto a chosen layer such as `F.SilkS`, but it is also a manual GUI flow. Typical steps are:

1. Open the board in KiCad PCB Editor.
2. Choose **File -> Import -> Graphics**.
3. Select `hardware/kicad/koch_snowflake_demo.svg`.
4. Set the destination layer to **`F.SilkS`**.
5. Adjust scale/line handling until the preview fits the intended board area.
6. Save the board and re-run DRC.

As of this proof of concept, KiCad exposes export commands via `kicad-cli`, not a board-graphics import command.

## Future option: Gerbolyze

If MycoMIDI later wants copper-layer art or more advanced Gerber-level compositing, `Gerbolyze` is the more capable modern option to evaluate. It is not needed for this initial silkscreen proof of concept.

## Demo board DRC baseline

Using the existing demo board and the Docker-backed wrapper:

```bash
scripts/kicad-cli.sh pcb drc \
  --format json \
  --output hardware/kicad/demo/demo-drc.json \
  hardware/kicad/demo/demo.kicad_pcb
```

Observed result for the unmodified demo board in this repository:

- `Found 0 violations`
- `Found 0 unconnected items`

That confirms the starting point is clean before any manual art import.

## Re-running DRC after manual import

After placing the artwork in KiCad and saving the board, re-run:

```bash
scripts/kicad-cli.sh pcb drc \
  --format json \
  --output hardware/kicad/demo/demo-drc.json \
  hardware/kicad/demo/demo.kicad_pcb
```

In this headless environment, the board-import step itself could not be automated, so post-import DRC still requires a human to complete the GUI import/save cycle first.

If DRC reports silkscreen-specific issues such as overlap with mask openings or the board edge, prefer:

- moving or scaling the artwork first
- using a **scoped** custom silkscreen-clearance exception for the art object/footprint only when the overlap is intentional

Do **not** disable silkscreen checks board-wide just to accommodate decorative art.

## Open-design requirement

For OSHWA-style open hardware, do not publish only final Gerbers or a traced footprint. Publish:

- the KiCad board files
- the exported art
- the actual generator source (`hardware/kicad/fractal_art_generator.py`)
- the parameters used to regenerate the artwork

That keeps the decorative PCB art modifiable in its true source form, not only as a downstream manufacturing artifact.
