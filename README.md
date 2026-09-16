# MycoMIDI

MycoMIDI is an experimental bioelectric-music project for recording slow electrical activity from mushroom / mycelium substrate, detecting spike patterns across multiple channels, turning those patterns into discrete "words," and rendering the resulting sequences as MIDI.

The current direction is:

1. record raw calibrated voltage data from a multi-channel ADC,
2. detect spikes with an adaptive offline tokenizer,
3. group spikes into words,
4. train a simple Markov model over those words,
5. render the word stream as generative music.

## Project purpose

The repository is organized around a recording-and-analysis pipeline rather than a direct "plug mushroom into synth" toy. The goal is to preserve raw data first, then derive music from actual detected events.

- **Input**: bioelectric signals from colonized substrate, captured with differential electrodes.
- **Acquisition**: multi-channel ADC logging, calibrated per channel.
- **Analysis**: spike detection using local baselines and the current MAD / robust-z detector.
- **Tokenization**: inter-spike timing is clustered into word-like units inspired by Adamatzky's fungal spiking-language work.
- **Generation**: per-channel word streams can train simple order-1 Markov models and be rendered to MIDI.

This is intentionally a **slow** instrument: fungal spikes happen on the scale of seconds to minutes to hours, not keyboard latency.

## Repository layout

### Current pipeline (`tools/`)

- `tools/logger.py`  
  Continuous raw calibrated CSV logger. It is designed around an ADS131M08-style interface (`adc.read_all_volts()`), writes one timestamped CSV row per sample, calibrates a startup baseline, and auto-detects channel count from the ADC instead of assuming a fixed 8-channel layout.

- `tools/tokenizer.py`  
  Offline spike detector and tokenizer. It loads logger CSVs, detects spikes with either:
  - the original local-average detector, or
  - the newer adaptive **median / MAD robust-z** detector,  
  then groups spikes into "words" and trains / samples an order-1 Markov model over the word symbols.

- `tools/musicgen.py`  
  Converts tokenized word streams into MIDI note events and standard MIDI files. It also includes optional live playback helpers via `python-rtmidi`, but the main path is offline rendering from logged CSV data.

- `tools/test_tokenizer.py`, `tools/test_musicgen.py`  
  Focused regression tests for the tokenizer and MIDI renderer.

### Legacy pipeline (top-level files)

These files predate the current `tools/` workflow and represent the older live single-ADC path:

- `mycomidi.py`
- `ADS1256.py`
- `config.py`

They are **not** marked as legacy in their current file headers/docstrings yet, but they clearly belong to the earlier design:

- `mycomidi.py` reads an ADS1256 continuously, publishes MQTT messages, and sends MIDI directly from per-sample threshold crossings.
- `ADS1256.py` is the older ADC driver layer for that pipeline.
- `config.py` contains Raspberry Pi GPIO / SPI setup helpers used by `ADS1256.py`.

For new work, prefer the `tools/` pipeline.

### Other directories

- `hardware/` — hardware notes, board concepts, layout guidance, and a realtime architecture proposal.

## Hardware overview

The hardware documentation is collected under [`hardware/`](hardware/):

- [`hardware/README.md`](hardware/README.md)  
  High-level revival plan for the 2026 direction. It summarizes the move to an ADS131M08-based multi-channel logger, notes why the old live ADS1256 path is no longer the preferred baseline, and outlines the intended order of operations: log first, tune detection second, reconnect generative output after real spikes are confirmed.

- [`hardware/adc-module.md`](hardware/adc-module.md)  
  Describes the **dry-zone ADC module**: one ADS131M08 per 8 channels, simultaneous 24-bit sampling, shared SPI bus, shared clock and sync/reset, and one chip-select per module. This is the scaling plan for 8/16/24/32 channel capture.

- [`hardware/pin-board.md`](hardware/pin-board.md)  
  Describes the **humid-zone passive electrode board**. It uses passive gold-pin recording electrodes plus a shared Ag/AgCl reference, keeps active electronics out of the wet area, and documents cable / shielding / labeling guidance.

- [`hardware/array-geometry.md`](hardware/array-geometry.md)  
  Suggests an initial electrode placement geometry for a shoebox-sized fruiting block: an 8-channel, 4x2 grid, ~1.5 cm intra-pair spacing, and practical placement guidance for avoiding seams, slits, and obvious dry zones.

- [`hardware/realtime-architecture.md`](hardware/realtime-architecture.md)  
  A design proposal for future live streaming. It explicitly says streaming is **not implemented yet** in the current tools, explains why fungal signals are inherently slow, and sketches a causal detector / queue-based architecture for a future realtime system.

## Setup / installation

There is currently **no `requirements.txt` or `pyproject.toml` in the repo**. If one is added later, prefer that source of truth.

Based on the imports in the current codebase:

### Core current pipeline dependencies

The current `tools/` pipeline uses only the Python standard library for offline logging / tokenization / MIDI-file generation.

- Python 3.10+ is recommended (the code uses modern type syntax such as `list[float] | None`).

### Optional dependency

- `python-rtmidi` — only needed for optional live MIDI playback helpers in `tools/musicgen.py`.

### Legacy pipeline dependencies

The older top-level live pipeline additionally imports:

- `numpy`
- `paho-mqtt`
- `python-rtmidi`
- `RPi.GPIO`
- `spidev`

If a `requirements.txt` is added later, use that instead of this handwritten list.

## Running the pipeline end-to-end

The current workflow is **logger -> tokenizer -> musicgen**.

### 1) Log calibrated raw CSV data

`tools/logger.py` is a real logger, but its `__main__` block is still a placeholder until a concrete ADS131M08 driver instance is wired in:

```python
# adc = ADS131M08Driver(...)  # wire up once hardware exists
raise SystemExit(
    "Wire an ADS131M08 driver instance into `adc` above, then "
    "call run(adc, session_name=sys.argv[1])"
)
```

So the intended invocation shape is:

```bash
python3 tools/logger.py SESSION_NAME
```

Example:

```bash
python3 tools/logger.py oyster-block-01
```

Today, that command will exit with the reminder above until the ADC driver is connected. Once wired, it will create:

```text
data/oyster-block-01.csv
```

with columns like:

```text
t_unix,ch0_v,ch1_v,...
```

### 2) Tokenize a recorded session

`tools/tokenizer.py` currently exposes a minimal command-line interface: it accepts a single CSV path and prints a short per-channel summary plus generated Markov output.

```bash
python3 tools/tokenizer.py data/oyster-block-01.csv
```

That script:

- loads the logger CSV,
- detects spikes per channel,
- groups spikes into words,
- converts words to symbols,
- fits a minimal Markov model per channel,
- prints a generated example sequence when symbols exist.

### 3) Render MIDI from the tokenized stream

`tools/musicgen.py` currently exposes its main workflow as Python functions rather than a standalone CLI parser. The supported path is the one shown in the module docstring and tests: load CSV data, train per-channel models, then call `render_csv_to_midi(...)`.

Example:

```bash
python3 - <<'PY'
from tools.musicgen import render_csv_to_midi
from tools.tokenizer import MarkovModel, load_csv, tokenize_channel

csv_path = "data/oyster-block-01.csv"
detect_kwargs = {
    "window": 30,
    "min_gap": 40,
    "method": "mad",
    "z_threshold": 4.0,
    "max_sample_delta": 2.0,
}

models = {}
for channel, samples in load_csv(csv_path).items():
    symbols = tokenize_channel(samples, **detect_kwargs)
    model = MarkovModel()
    if len(symbols) > 1:
        model.fit(symbols)
    models[channel] = model

summary = render_csv_to_midi(
    csv_path,
    models,
    "data/oyster-block-01.mid",
    detect_kwargs=detect_kwargs,
)

print(summary)
PY
```

This writes a standard MIDI file such as:

```text
data/oyster-block-01.mid
```

### Optional live playback of rendered note events

If `python-rtmidi` is installed, `tools/musicgen.py` also provides playback helpers for already-rendered note events. This is optional and separate from the offline MIDI-file path.

## Tests

From the repo root, run:

```bash
python3 -m unittest tools.test_musicgen tools.test_tokenizer -v
```

That command exercises the tokenizer and MIDI renderer on synthetic data.

## Current status and direction

- The **current supported architecture** is: log raw data, analyze offline, then render music.
- The **legacy ADS1256 / MQTT / immediate-MIDI path** remains in the repo for historical reference but is not the preferred direction for new development.
- The **future realtime architecture** is documented, but not implemented in the current tools yet.

If you are starting fresh, begin with `hardware/`, then `tools/logger.py`, `tools/tokenizer.py`, and `tools/musicgen.py`.
