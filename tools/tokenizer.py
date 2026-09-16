"""
Offline spike tokenizer + word/Markov model, following the spike ->
word grouping method in Adamatzky, "Language of fungi derived from
electrical spiking activity" (arXiv:2112.09907).

Pipeline: raw csv (from logger.py) -> per-channel spike detection ->
inter-spike-interval clustering into "words" -> n-gram/Markov model
over the word sequence -> generate new sequences for sonification.

This replaces the old per-sample threshold-vs-moving-average trigger
in mycomidi.py, which fires on every sample crossing a fixed +/-1V
band -- too coarse for mV-scale, multi-minute spikes and gives no
structure above the single-spike level.
"""
import csv
import random
from collections import defaultdict, Counter
from dataclasses import dataclass
from statistics import median


@dataclass
class Spike:
    t: float
    channel: int
    amplitude: float


def load_csv(path: str) -> dict[int, list[tuple[float, float]]]:
    """channel -> list of (t_unix, volts)"""
    series: dict[int, list[tuple[float, float]]] = defaultdict(list)
    with open(path) as f:
        r = csv.reader(f)
        header = next(r)
        n_ch = len(header) - 1
        for row in r:
            t = float(row[0])
            for ch in range(n_ch):
                series[ch].append((t, float(row[ch + 1])))
    return series


def _is_single_sample_artifact(
    vals,
    idx: int,
    max_sample_delta: float | None,
    *,
    causal: bool = False,
) -> bool:
    if max_sample_delta is None or idx <= 0:
        return False
    if causal:
        return abs(vals[idx] - vals[idx - 1]) > max_sample_delta
    if idx >= len(vals) - 1:
        return False
    return (
        abs(vals[idx] - vals[idx - 1]) > max_sample_delta
        and abs(vals[idx] - vals[idx + 1]) > max_sample_delta
    )


def _record_spike_candidate(spikes: list[Spike], samples, idx: int, min_gap: int, last_idx: int) -> int:
    candidate = Spike(t=samples[idx][0], channel=-1, amplitude=samples[idx][1])
    if not spikes or (idx - last_idx) >= min_gap:
        spikes.append(candidate)
        return idx
    if abs(candidate.amplitude) > abs(spikes[-1].amplitude):
        spikes[-1] = candidate
        return idx
    return last_idx


def detect_spikes_local_avg(
    samples,
    window=100,
    delta=0.05,
    min_gap=100,
    max_sample_delta: float | None = 2.0,
) -> list[Spike]:
    """
    Original Adamatzky-style detector: compare each sample to the
    average of its +/- 2*window neighborhood, then apply a refractory
    min_gap. Optional max_sample_delta rejects isolated one-sample
    transients that are implausibly fast for multi-minute fungal spikes.
    """
    n = len(samples)
    vals = [v for _, v in samples]
    spikes = []
    last_idx = -min_gap
    for i in range(2 * window, n - 2 * window):
        lo, hi = i - 2 * window, i + 2 * window
        local_avg = sum(vals[lo:hi]) / (hi - lo)
        if (
            abs(vals[i]) - abs(local_avg) > delta
            and not _is_single_sample_artifact(vals, i, max_sample_delta)
        ):
            last_idx = _record_spike_candidate(spikes, samples, i, min_gap, last_idx)
    return spikes


def detect_spikes_mad(
    samples,
    window=100,
    min_gap=100,
    z_threshold: float = 6.0,
    mad_window: int | None = None,
    mad_guard: int | None = None,
    causal: bool = False,
    min_amplitude: float = 0.0,
    max_sample_delta: float | None = 2.0,
) -> list[Spike]:
    """
    Drift-tolerant adaptive detector based on robust z-scores computed
    from a local median and median absolute deviation (MAD).

    A sample is considered a spike when its absolute deviation from the
    local median exceeds both min_amplitude and z_threshold-scaled MAD.
    A guard band keeps the candidate's immediate neighborhood out of the
    baseline so a slow spike does not suppress its own score. Set causal=True
    to use only samples preceding that guard band for streaming-compatible
    detection.
    """
    n = len(samples)
    if n == 0:
        return []
    if mad_window is None:
        mad_window = max(5, 2 * window + 1)
    if mad_window < 3:
        mad_window = 3
    if mad_window % 2 == 0:
        mad_window += 1
    if mad_guard is None:
        mad_guard = max(1, min(window // 4, (mad_window - 3) // 4))
    if mad_guard < 0:
        raise ValueError("mad_guard must be non-negative")

    vals = [v for _, v in samples]
    half_window = mad_window // 2
    spikes = []
    last_idx = -min_gap

    if causal:
        indices = range(mad_window + mad_guard, n)
    else:
        indices = range(half_window, n - half_window)

    for i in indices:
        if causal:
            baseline_end = i - mad_guard
            segment = vals[baseline_end - mad_window : baseline_end]
        else:
            left = vals[i - half_window : i - mad_guard]
            right = vals[i + mad_guard + 1 : i + half_window + 1]
            segment = left + right
        if len(segment) < 3:
            continue
        local_median = median(segment)
        abs_deviation = abs(vals[i] - local_median)
        if abs_deviation <= min_amplitude:
            continue
        mad = median(abs(v - local_median) for v in segment)
        robust_z = float("inf") if mad == 0 else 0.6745 * abs_deviation / mad
        if (
            robust_z >= z_threshold
            and not _is_single_sample_artifact(
                vals,
                i,
                max_sample_delta,
                causal=causal,
            )
        ):
            last_idx = _record_spike_candidate(spikes, samples, i, min_gap, last_idx)
    return spikes


def detect_spikes(
    samples,
    window=100,
    delta=0.05,
    min_gap=100,
    method: str = "mad",
    z_threshold: float = 6.0,
    mad_window: int | None = None,
    mad_guard: int | None = None,
    causal: bool = False,
    min_amplitude: float = 0.0,
    max_sample_delta: float | None = 2.0,
) -> list[Spike]:
    """
    Semi-automatic spike detection with selectable methods:

    - method="mad" (default): adaptive robust-z detector using local
      median/MAD, tolerant to drift and cheap to compute. mad_guard
      excludes the candidate neighborhood from its baseline; causal=True
      uses a trailing-only baseline. Use min_amplitude for an absolute floor.
    - method="local_avg": original Adamatzky-style fixed delta against
      a local moving average.

    The public call shape stays stable; existing callers can continue
    to pass window/delta/min_gap and optionally select the legacy
    detector with method="local_avg".
    """
    if method == "mad":
        return detect_spikes_mad(
            samples,
            window=window,
            min_gap=min_gap,
            z_threshold=z_threshold,
            mad_window=mad_window,
            mad_guard=mad_guard,
            causal=causal,
            min_amplitude=min_amplitude,
            max_sample_delta=max_sample_delta,
        )
    if method == "local_avg":
        return detect_spikes_local_avg(
            samples,
            window=window,
            delta=delta,
            min_gap=min_gap,
            max_sample_delta=max_sample_delta,
        )
    raise ValueError(f"Unknown detection method: {method}")


def spikes_to_words(spikes: list[Spike], gap_threshold_s: float = 600) -> list[list[Spike]]:
    """
    Group consecutive spikes into a 'word' when the inter-spike gap is
    below gap_threshold_s. Adamatzky's word-length distributions
    (avg 3.3-4.7 spikes/word) came from a similar clustering step --
    tune gap_threshold_s against your own inter-spike interval
    histogram rather than assuming this default is right for your
    species/substrate.
    """
    if not spikes:
        return []
    words: list[list[Spike]] = [[spikes[0]]]
    for s in spikes[1:]:
        if s.t - words[-1][-1].t <= gap_threshold_s:
            words[-1].append(s)
        else:
            words.append([s])
    return words


def word_to_symbol(word: list[Spike], n_bins: int = 8) -> str:
    """Quantize a word's (length, avg amplitude) into a discrete symbol."""
    length_bin = min(len(word), n_bins - 1)
    avg_amp = sum(abs(s.amplitude) for s in word) / len(word)
    amp_bin = min(int(avg_amp * n_bins), n_bins - 1)
    return f"L{length_bin}A{amp_bin}"


class MarkovModel:
    """Order-1 Markov chain over word symbols. Minimal on purpose --
    swap for an RNN later once you have weeks of tokenized data."""

    def __init__(self):
        self.transitions: dict[str, Counter] = defaultdict(Counter)

    def fit(self, symbols: list[str]):
        for a, b in zip(symbols, symbols[1:]):
            self.transitions[a][b] += 1

    def generate(self, start: str, length: int = 20) -> list[str]:
        out = [start]
        cur = start
        for _ in range(length - 1):
            choices = self.transitions.get(cur)
            if choices:
                cur = random.choice(list(choices.elements()))
            elif self.transitions:
                cur = random.choice(list(self.transitions.keys()))
            # else: no transitions learned at all (e.g. a single-word
            # session) -- repeat the start symbol instead of crashing.
            out.append(cur)
        return out


def tokenize_channel(samples, **detect_kwargs) -> list[str]:
    spikes = detect_spikes(samples, **detect_kwargs)
    words = spikes_to_words(spikes)
    return [word_to_symbol(w) for w in words]


if __name__ == "__main__":
    import sys

    series = load_csv(sys.argv[1])
    for ch, samples in series.items():
        symbols = tokenize_channel(samples)
        print(f"channel {ch}: {len(symbols)} words")
        model = MarkovModel()
        model.fit(symbols)
        if symbols:
            print("  generated:", model.generate(symbols[0], length=10))
