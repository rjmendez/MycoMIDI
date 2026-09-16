"""
Continuous raw logger. Decoupled from MIDI/MQTT entirely -- just
acquisition, calibration, and storage. Run this for days/weeks.

Replace the ADS1256-specific read calls with the ADS131M08 driver
once wired; interface kept minimal so the swap is a few lines.
"""
import csv
import time
from pathlib import Path

SAMPLE_HZ = 1.0          # matches published fungal recording rates
OUT_DIR = Path("data")
BASELINE_SAMPLES = 60     # seconds of quiet data to zero each channel


def read_raw_volts(adc) -> list[float]:
    """Return one simultaneous sample across all channels, in volts."""
    return adc.read_all_volts()


def calibrate(adc, seconds=BASELINE_SAMPLES) -> list[float]:
    """Average `seconds` of samples per channel as a zero offset.

    Channel count is taken from the ADC's own reading on each pass, not a
    hardcoded constant -- this keeps calibrate()/run() correct whether the
    ADC is one ADS131M08 module (8ch) or several chained together
    (16/24/32ch, see hardware/adc-module.md). A fixed N_CHANNELS silently
    truncated or index-crashed as soon as the module count changed.
    """
    sums: list[float] | None = None
    for _ in range(seconds):
        vals = read_raw_volts(adc)
        if sums is None:
            sums = [0.0] * len(vals)
        elif len(vals) != len(sums):
            raise ValueError(
                f"ADC channel count changed mid-calibration: "
                f"{len(sums)} -> {len(vals)}"
            )
        for i, v in enumerate(vals):
            sums[i] += v
        time.sleep(1.0 / SAMPLE_HZ)
    return [s / seconds for s in (sums or [])]


def run(adc, session_name: str):
    OUT_DIR.mkdir(exist_ok=True)
    offsets = calibrate(adc)
    n_channels = len(offsets)
    path = OUT_DIR / f"{session_name}.csv"
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_unix"] + [f"ch{i}_v" for i in range(n_channels)])
        while True:
            t0 = time.time()
            raw = read_raw_volts(adc)
            if len(raw) != n_channels:
                raise ValueError(
                    f"ADC channel count changed after calibration: "
                    f"{n_channels} -> {len(raw)}"
                )
            calibrated = [r - o for r, o in zip(raw, offsets)]
            w.writerow([t0] + calibrated)
            f.flush()
            sleep_for = (1.0 / SAMPLE_HZ) - (time.time() - t0)
            if sleep_for > 0:
                time.sleep(sleep_for)


if __name__ == "__main__":
    import sys
    # adc = ADS131M08Driver(...)  # wire up once hardware exists
    raise SystemExit(
        "Wire an ADS131M08 driver instance into `adc` above, then "
        "call run(adc, session_name=sys.argv[1])"
    )
