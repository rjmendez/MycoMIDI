# Revival plan (2026)

## Hardware
- `hardware/pin-board.md` -- passive gold-pin electrode header, humid zone
- `hardware/adc-module.md` -- ADS131M08 module, dry zone, scales 8/16/24/32ch

## Species reference (Adamatzky, arXiv:2112.09907, verified)
| Species        | Avg interval | Avg amplitude |
|----------------|--------------|----------------|
| C. militaris   | 116 min      | 0.2 mV         |
| F. velutipes   | 102 min      | 0.3 mV         |
| S. commune     | 41 min       | 0.03 mV        |
| O. nidiformis  | 92 min       | 0.007 mV       |
Oyster (P. djamor, earlier Adamatzky work): ~2.6min and ~14min regimes.
Not yet measured for our actual Oyster/Lion's Mane blocks -- treat as
a starting point for detector tuning, not ground truth.

## Software
- `tools/logger.py` -- continuous raw calibrated CSV logger, no MIDI/MQTT
- `tools/tokenizer.py` -- spike detection -> word segmentation -> Markov
  model, offline, per channel

## Known issues in existing mycomidi.py / ADS1256.py
- Sign-extension mask likely wrong (`0xF000000` vs `0xFF000000`)
- RPi.GPIO unmaintained, breaks on newer kernels
- Fixed +/-1V threshold too coarse for mV-scale multi-minute spikes
- No per-channel calibration, no raw-data retention

## Order of operations
1. Build one pin board + one ADC module, run `tools/logger.py` for a
   few days on a live block
2. Run `tools/tokenizer.py` against the log, eyeball spike detection
   params (window/delta/min_gap) per channel/species
3. Only after real spikes are confirmed: reconnect MIDI/generative
   output driven by the Markov model instead of raw per-sample
   threshold triggers
