# Real-time streaming architecture (design proposal)

Goal: let a person "play" with live mushroom signals instead of only
logging now / tokenizing+generating later. This document proposes an
architecture to get there. It does **not** modify `tools/logger.py` or
`tools/tokenizer.py` — those stay as-is (another workstream owns
adapting them); the sketches below are illustrative interfaces only.

## 0. Set expectations first: this is not a "real" instrument

Read this before designing anything else, because it constrains every
decision below.

- Published fungal spike durations and inter-spike intervals are on the
  order of **minutes to hours** (see `hardware/README.md` species
  table: e.g. C. militaris ~116 min avg interval, S. commune ~41 min).
  A single spike itself typically takes on the order of a minute or
  more to rise and fall.
- There is **no way** to get millisecond keyboard-like responsiveness
  out of the underlying biology. "Real-time" here means: the system
  reacts within roughly a second or two of a rising spike crossing a
  detection threshold *mid-event*, not that the mushroom is generating
  fast musical gestures.
- Practically, live sessions will sound like a slow, sparse, ambient
  generative instrument — periods of silence/drone punctuated by
  occasional triggered notes/chords as spikes cross threshold, plus
  continuous low-rate modulation (e.g. mapping smoothed amplitude or
  band energy to a drone pitch/filter/CC) to give the ear something
  between events. Anyone expecting a responsive synth playable like a
  keyboard will be disappointed; anyone expecting a slow biofeedback
  drone instrument will get a faithful one.
- We *can* legitimately compress the "boring" wait: crossfade in a
  short pre-rendered/generated phrase from the offline Markov model
  as soon as a live spike is confirmed, so the human perceives a
  responsive "hit" even though the underlying biological event is
  still unfolding. That's a UX trick, not faster biology — call it
  that when presenting it to the user.

## 1. Acquisition: continuous multi-channel sampling on the Pi

Reuse the ADS131M08 shared-clock design already documented in
`hardware/adc-module.md`:

- Single shared `CLKIN` across all modules → simultaneous sampling
  across up to 32 channels, not per-channel round-robin like the old
  ADS1256 single-ADC mux (`ADS1256_GetAll()` currently loops over 8
  channels serially with a `SYNC`/`WAKEUP` per channel — that alone
  introduces tens of ms of inter-channel skew per scan, which is fine
  for logging but sloppy for anything claiming "simultaneous").
- Only one `DRDY` line needs to be watched (any module, since they're
  clock-synced) to know a new simultaneous frame is ready.
- Keep the shared SPI bus + per-module `CS` wiring from the doc; add
  a single background reader thread/process that owns the SPI bus
  and does nothing else (see below).

### Sample rate

Given spike timescales of minutes, we do **not** need kHz sampling for
musicality — `logger.py`'s `SAMPLE_HZ = 1.0` already matches published
recording rates. For a live system, oversample modestly (e.g. 4-10 Hz)
relative to that so the *onset* of a spike (the crossing moment) is
detected with sub-second-to-low-second precision instead of being
quantized to whole seconds, while staying far below anything that
would stress SPI/Pi CPU. There is no benefit to sampling faster than
that — the signal itself has no useful energy up there, only noise and
mains hum.

### Buffering / latency budget on a Pi

Three stages, each with a bounded buffer, so latency is capped
end-to-end and never *unbounded* even under scheduling hiccups on a
non-RT Linux (standard Raspberry Pi OS, no PREEMPT_RT assumed):

```
[ADC ISR/poll thread]  -->  ring buffer (per channel, seconds of depth)
        |
        v
[detector process/thread]  -->  event queue (spike/word events only)
        |
        v
[MIDI/sonification process]  -->  MIDI out (rtmidi / ALSA seq)
```

| Stage                      | Budget (typical Pi 4/5, non-RT)      |
|-----------------------------|---------------------------------------|
| SPI read + DRDY wait per frame | ~1-5 ms per simultaneous frame at 4-10Hz sampling (SPI itself is fast; DRDY wait dominates and is bounded by the sample period) |
| Ring buffer write + hand-off (thread-safe queue, not a lock around SPI) | <1 ms |
| Online detector update (incremental MAD/z-score, O(1) per sample, see below) | <1 ms per channel per sample |
| Debounce/refractory hold before firing (mirrors `min_gap` in the offline detector) | intentional — 100ms-several sec, tunable |
| MIDI message send (`rtmidi`, local ALSA) | <5 ms typically |
| **Total added latency budget (excluding intentional debounce)** | **roughly 10-50 ms** from "sample crosses threshold" to "MIDI note fires" |

The dominant "latency" a listener will actually perceive is **not**
this pipeline — it's waiting for the mushroom to produce a spike large
enough to cross threshold in the first place (minutes). The above
budget matters for a different reason: it bounds *jitter/artifact*
(e.g. avoiding the old `mycomidi.py` pattern of a blocking
`time.sleep(spike/30)` between note-on and note-off inside the
per-sample read loop, which stalls the *entire* acquisition loop for
all 8 channels while one note rings out — see "known issues" below).

Use separate OS threads/processes for acquisition vs. detection vs.
MIDI output, connected by bounded queues (e.g. Python
`multiprocessing.Queue` or a plain `collections.deque` behind a lock if
staying single-process), specifically so a slow MIDI/synth backend or
a note's artificial hold time never blocks the ADC read loop the way
`mycomidi.py`'s `note_on`/`time.sleep`/`note_off` sequence currently
does inline in the single acquisition loop.

## 2. Making spike detection causal (online, incremental)

### Why the current `detect_spikes()` cannot be used live unmodified

`tools/tokenizer.py::detect_spikes()` (read, not modified, for this
design) computes, for sample `i`:

```python
lo, hi = i - 2*window, i + 2*window
local_avg = sum(vals[lo:hi]) / (hi - lo)
```

That average spans `i - 2*window` **through** `i + 2*window` — it
needs `2*window` samples *after* the current one before it can decide
whether sample `i` was a spike. That's fine offline (the whole CSV
already exists) but is structurally incompatible with live use: at
sample `i`, the future 2*window samples haven't happened yet. There is
no way to "unmodify" this into causal use; a live detector needs a
different (but analogous) computation that only looks backward.

### Recommended online approach: incremental MAD-based z-score

This is the natural causal analogue of the existing
mean/threshold idea, and it's simple enough to implement in O(1)
memory/time per sample, robust to slow baseline drift (which the raw
±1V fixed threshold in `mycomidi.py` completely ignores), and robust
to outliers (which a plain rolling mean/stdev is not, and which matters
since spikes themselves are outliers you don't want polluting your own
baseline estimate — this is precisely why a plain moving average, like
`mycomidi.py`'s `moving_average(buffer)`, drifts toward/gets dragged by
the very spikes you're trying to detect).

Core idea, per channel, maintained causally:

1. Keep a trailing buffer of the last `N` samples (`N` corresponds
   to the offline `window` scale, e.g. equivalent time span, not
   forward+backward like today).
2. Maintain a running **median** and **MAD** (median absolute
   deviation) over that trailing buffer — update incrementally (e.g.
   via a small sorted structure / two-heap median tracker, or just
   recompute over a modest `N` since Pi CPU is not the bottleneck at
   4-10 Hz).
3. Compute a robust z-score for the new sample:
   `z = 0.6745 * (x - median) / MAD` (0.6745 is the standard
   MAD→stdev scaling constant for a normal distribution).
4. Flag a spike onset when `|z| > z_threshold` (e.g. start at 3.5-5,
   tune per channel/species per the existing offline `delta`
   equivalent) **and** the debounce/refractory timer (`min_gap`
   equivalent) has elapsed since the last flagged spike.
5. Only the trailing buffer is needed — no forward-looking window,
   so this can run exactly one sample behind real time.

This is deliberately the same *shape* of idea as the offline detector
(deviation-from-local-baseline beyond a delta, with a minimum gap) —
just computed causally and made robust via median/MAD instead of a
plain mean, so it degrades better under a live/noisy Pi + long cable
run than a naive port of `local_avg` restricted to only-past-samples
would.

Note: I searched loci investigation `mycomidi-adc-project` for prior
findings from a `myco-spike-detect-algos` workstream and found none
stored yet (only cultivation/hardware findings present at time of
writing) — this MAD-z-score recommendation is this document's own
proposal, not a synthesis of separately-vetted findings. If/when that
other research lands in loci, reconcile against it before final
implementation.

### Sketch: causal online detector interface

```python
# Illustrative only -- NOT applied to tools/tokenizer.py.
# A live-safe counterpart to detect_spikes(), same Spike dataclass.

from collections import deque

class OnlineSpikeDetector:
    """
    Causal, incremental spike-onset detector for one channel.
    O(1)-ish per sample; only ever looks backward.
    """

    def __init__(self, window: int = 200, z_threshold: float = 4.0,
                 min_gap_s: float = 5.0):
        self.window = window
        self.z_threshold = z_threshold
        self.min_gap_s = min_gap_s
        self._buf: deque[float] = deque(maxlen=window)
        self._last_spike_t: float | None = None

    def update(self, t: float, value: float) -> "Spike | None":
        """Feed one new sample; return a Spike if this sample is a
        detected onset, else None. Never inspects samples after t."""
        spike = None
        if len(self._buf) >= max(20, self.window // 4):
            median = _median(self._buf)
            mad = _mad(self._buf, median) or 1e-9
            z = 0.6745 * (value - median) / mad
            gap_ok = (
                self._last_spike_t is None
                or (t - self._last_spike_t) >= self.min_gap_s
            )
            if abs(z) > self.z_threshold and gap_ok:
                spike = Spike(t=t, channel=-1, amplitude=value)
                self._last_spike_t = t
        self._buf.append(value)
        return spike
```

`_median`/`_mad` can start as simple `sorted(buf)`-based helpers at
these buffer sizes (hundreds of samples at a few Hz — trivially cheap
on a Pi) and only need a fancier incremental structure if profiling
ever shows otherwise.

## 3. Adapting word/Markov generation to run online

The offline pipeline is: whole CSV → `detect_spikes()` (needs the
whole file) → `spikes_to_words()` (groups by gap) → `word_to_symbol()`
→ fit `MarkovModel` over the *entire* symbol sequence → `generate()`.
For live use we want the same conceptual stages, but each running
incrementally off a live spike stream, with a **pretrained** Markov
model doing generation from the start (fit once, offline, from
accumulated logs) while a live per-session model optionally keeps
adapting.

Two things are naturally already streaming-friendly and need no
conceptual change, only a live driver:
- `word_to_symbol()` is pure/stateless given a finished word — reuse
  as-is (imported, not copied) once a live word is finalized.
- `MarkovModel.generate()` is already a stateful walk and works fine
  fed by a live "current symbol" instead of `symbols[0]`.

What needs an incremental counterpart:

```python
# Illustrative only -- NOT applied to tools/tokenizer.py.
# Live counterparts to spikes_to_words()/tokenize_channel(), designed
# to import and reuse word_to_symbol()/MarkovModel from tokenizer.py
# rather than duplicate their logic.

from tools.tokenizer import Spike, word_to_symbol, MarkovModel

class OnlineWordAssembler:
    """
    Causal counterpart to spikes_to_words(): folds one live Spike at
    a time into an in-progress 'word', and flushes a finished word
    once gap_threshold_s has elapsed with no new spike -- decided by
    a periodic tick() call rather than needing a lookahead spike.
    """

    def __init__(self, gap_threshold_s: float = 600):
        self.gap_threshold_s = gap_threshold_s
        self._current: list[Spike] = []

    def on_spike(self, spike: Spike) -> list[Spike] | None:
        """Call once per detected spike. Returns a finished word
        (list[Spike]) if this spike's gap from the prior one closed
        out the previous word, else None (word still open)."""
        finished = None
        if self._current and (spike.t - self._current[-1].t) > self.gap_threshold_s:
            finished = self._current
            self._current = []
        self._current.append(spike)
        return finished

    def tick(self, now_t: float) -> list[Spike] | None:
        """Call periodically (e.g. every detector loop iteration) so
        a word can be flushed by elapsed wall-clock time even with no
        new spike yet (needed because unlike the offline version, a
        live word can't wait for 'the next spike' to know it's over --
        it may never come, or not for a long time)."""
        if self._current and (now_t - self._current[-1].t) > self.gap_threshold_s:
            finished, self._current = self._current, []
            return finished
        return None


class OnlineTokenizer:
    """
    Ties together per-channel OnlineSpikeDetector + OnlineWordAssembler
    + word_to_symbol(), and feeds a MarkovModel two ways:
      - live_model: updated incrementally as new words complete, so the
        instrument's vocabulary/transitions keep evolving this session
      - generate_model: a separately pretrained model (fit offline from
        prior logs via tools/tokenizer.py's existing fit()) used for
        immediate generation from note 1, since a fresh live_model has
        no transitions yet at session start
    """

    def __init__(self, channel: int, generate_model: MarkovModel,
                 detector: OnlineSpikeDetector | None = None,
                 assembler: OnlineWordAssembler | None = None):
        self.channel = channel
        self.detector = detector or OnlineSpikeDetector()
        self.assembler = assembler or OnlineWordAssembler()
        self.generate_model = generate_model   # pretrained, read-mostly
        self.live_model = MarkovModel()          # grows this session
        self.last_symbol: str | None = None

    def on_sample(self, t: float, value: float) -> str | None:
        """Feed one new calibrated sample. Returns a new word symbol
        if a word just completed (spike event -> possible MIDI trigger
        point), else None."""
        symbol = None
        spike = self.detector.update(t, value)
        finished_word = self.assembler.on_spike(spike) if spike else self.assembler.tick(t)
        if finished_word:
            symbol = word_to_symbol(finished_word)
            if self.last_symbol is not None:
                self.live_model.transitions[self.last_symbol][symbol] += 1
            self.last_symbol = symbol
        return symbol

    def next_notes(self, length: int = 4) -> list[str]:
        """Ask the pretrained model to continue from the current live
        symbol -- this is the 'compress the wait' trick from section 0:
        as soon as a live word/symbol lands, immediately generate a
        short continuation phrase to sonify, rather than waiting for
        more live spikes to arrive before anything plays."""
        start = self.last_symbol or next(iter(self.generate_model.transitions), "L0A0")
        return self.generate_model.generate(start, length=length)
```

Notes on this sketch:
- `tokenize_channel()` and `MarkovModel.fit()` in the existing offline
  `tools/tokenizer.py` remain exactly as-is and are the recommended way
  to *produce* `generate_model` — run them nightly/periodically over
  whatever `tools/logger.py` has accumulated, pickle/save the resulting
  `MarkovModel.transitions`, and load that into `OnlineTokenizer` at
  live-session start. This is a clean batch/online split: batch
  process improves the "vocabulary," online process only has to do
  cheap incremental work per sample.
- `OnlineWordAssembler.tick()` is the one truly new concept versus the
  offline code — offline, "a word is over" is discovered retroactively
  by seeing the *next* spike is far away; live, you don't get that
  luxury (the next spike might not come for a long time, or ever), so
  something has to periodically ask "has enough silence passed to
  consider the current word done," driven by wall-clock polling
  instead of the next data point.
- All of the above is written as free functions/classes that *import*
  `Spike`, `word_to_symbol`, and `MarkovModel` from the real
  `tools/tokenizer.py` rather than reimplementing them, so when that
  file's owning workstream evolves those definitions, this online path
  picks up the change for free instead of drifting out of sync.

## 4. Putting it together: process/thread layout on the Pi

```
Process A: acquisition
  - owns the SPI bus / ADS131M08 modules exclusively
  - one shared CLKIN + one DRDY watched, per adc-module.md
  - per DRDY event: read all channels' simultaneous frame, apply the
    same per-channel calibration offsets as tools/logger.py's
    calibrate(), push (t, [values]) onto a bounded queue
  - ALSO writes the same continuous calibrated CSV that
    tools/logger.py writes today (unchanged) -- live playing and
    offline logging are not mutually exclusive; keep logging always on
    so tonight's live session is tomorrow's better-trained Markov model

Process B: detector (one OnlineTokenizer per channel)
  - consumes the acquisition queue
  - per channel: OnlineSpikeDetector.update() -> OnlineWordAssembler
    -> word_to_symbol() -> live_model update
  - on a finished word: push a "word event" (channel, symbol, word
    stats) onto a small event queue for the MIDI stage
  - runs a lightweight periodic tick() (e.g. every acquisition frame)
    so words can flush on elapsed silence, not just on new spikes

Process C: sonification / MIDI out
  - consumes word events only (sparse -- one every so many
    seconds/minutes per channel, not per-sample)
  - maps (channel, symbol, amplitude/length bins already encoded in
    the symbol) to note/velocity/CC, asks generate_model.generate()
    for a short continuation phrase, sends MIDI via rtmidi/ALSA seq
  - never blocks on time.sleep() inside the same loop that reads the
    ADC -- this was the concretely broken part of mycomidi.py: note
    on/off timing there is a blocking sleep() *inside* the per-sample
    acquisition loop for all 8 channels, so one long "spike" note
    stalls sampling of every other channel for its duration. Keeping
    this in a separate process/thread with only queue hand-offs fixes
    that class of bug independent of anything else in this doc.
```

Optionally, a continuous secondary mapping (smoothed amplitude/energy
per channel → a slowly-modulating CC/drone parameter, updated every
acquisition frame rather than only on discrete word events) can run
in Process C alongside the event-triggered notes, so there is always
*something* audible between spikes rather than long silences — this
is the practical answer to the "not a real instrument" honesty note
in section 0: give the ear a continuous ambient layer, and treat
discrete spike/word events as accents on top of it.

## 5. Known bugs in the legacy reactive design this replaces

Documented here for context on *why* the above shape was chosen, not
as a todo against `mycomidi.py` (owned elsewhere):

- Per-sample fixed ±1V threshold vs `moving_average()` fires on any
  crossing regardless of species/channel noise floor — no calibration,
  no debounce beyond the fixed `buffer=100` sample deque, and a plain
  (non-robust) moving average that the spikes themselves drag around.
- `note_on()`/`note_off()` with an inline `time.sleep(spike/30)`
  between them, executed serially inside the single 8-channel
  acquisition `while True` loop — blocks acquisition of channels 1-7
  while channel 0's note rings out, and vice versa.
- `ADS1256_Read_ADC_Data()`'s sign-extension mask (`0xF000000`, 7
  hex digits) looks like a typo for `0xFF000000` (8 hex digits) per
  `hardware/README.md`'s known-issues note — affects negative-value
  decoding, independent of but compounding the above.
- `RPi.GPIO`-based `config.py`/`ADS1256.py` is unmaintained and known
  to break on newer Raspberry Pi OS kernels — any real build-out
  should target the ADS131M08 driver path already scoped in
  `tools/logger.py`'s docstring, not extend the ADS1256/RPi.GPIO path.

## 6. Summary / recommendation

1. Sample continuously via the documented ADS131M08 shared-clock
   design at a modest few-Hz rate (matches signal bandwidth; no need
   for kHz).
2. Replace `detect_spikes()`'s centered window with a causal,
   incremental **MAD-based robust z-score** detector run one sample
   behind real time — the natural online analogue of the existing
   approach, but robust to drift/outliers and requiring no future
   data.
3. Keep acquisition, detection, and MIDI output in separate
   threads/processes connected by bounded queues so a note's
   artificial hold time can never block ADC sampling (the concrete
   bug in current `mycomidi.py`).
4. Split the offline `tools/tokenizer.py` Markov model into a
   periodically-retrained "vocabulary" model (batch, from
   `tools/logger.py`'s accumulating CSV, unchanged) plus a lightweight
   online driver (sketched above) that reuses `Spike`,
   `word_to_symbol()`, and `MarkovModel` as-is and only adds the
   causal spike/word-boundary logic that genuinely cannot be offline.
5. Be upfront with the user: this yields near-real-time reaction to
   slowly-forming biological events (order of seconds of added system
   latency once a real spike starts crossing threshold), not a
   millisecond-latency instrument — and optionally layer a continuous
   ambient mapping plus generated continuation phrases so a live
   session still feels musically engaging between genuine spike
   events.
