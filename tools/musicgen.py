"""
Turn tokenized fungal spike words into expressive MIDI.

Example:
    from tools.musicgen import render_csv_to_midi
    from tools.tokenizer import MarkovModel, load_csv, tokenize_channel

    detect_kwargs = {"window": 30, "min_gap": 40, "z_threshold": 4.0}
    models = {}
    for channel, samples in load_csv("data/session.csv").items():
        symbols = tokenize_channel(samples, **detect_kwargs)
        model = MarkovModel()
        if len(symbols) > 1:
            model.fit(symbols)
        models[channel] = model

    summary = render_csv_to_midi(
        "data/session.csv",
        models,
        "data/session.mid",
        detect_kwargs=detect_kwargs,
    )
    print(summary.note_count, summary.pitch_range)

Optional live playback helpers use `python-rtmidi`; install it with:
    pip install python-rtmidi
"""

from __future__ import annotations

import math
import re
import struct
import time
from dataclasses import dataclass
from pathlib import Path
from statistics import median

from tools import tokenizer as tokenizer_module
from tools.tokenizer import MarkovModel, Spike, load_csv, tokenize_channel, word_to_symbol

PENTATONIC_SCALE = (0, 2, 4, 7, 9)
TICKS_PER_BEAT = 480
DEFAULT_TEMPO_BPM = 84
WORD_DURATION_BEATS = (0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0)


@dataclass(frozen=True)
class ChannelRole:
    name: str
    midi_channel: int
    base_note: int
    scale_degree_offset: int
    chord_degrees: tuple[int, ...]
    program: int


@dataclass(frozen=True)
class NoteEvent:
    start_beat: float
    duration_beats: float
    midi_channel: int
    pitches: tuple[int, ...]
    velocity: int
    source_channel: int
    source_symbol: str


@dataclass(frozen=True)
class RenderSummary:
    output_path: Path
    word_count: int
    note_count: int
    channels_used: tuple[int, ...]
    pitch_range: tuple[int | None, int | None]
    duration_beats: float
    tempo_bpm: int


CHANNEL_ROLES = (
    ChannelRole("root-bass", 0, 36, 0, (0,), 32),
    ChannelRole("lower-harmony", 1, 43, 1, (0, 2), 42),
    ChannelRole("mid-harmony", 2, 50, 2, (0, 2), 48),
    ChannelRole("upper-color", 3, 57, 3, (0, 4), 11),
    ChannelRole("lead", 4, 60, 1, (0,), 71),
    ChannelRole("answer", 5, 64, 2, (0, 2), 74),
    ChannelRole("shimmer", 6, 69, 3, (2,), 89),
    ChannelRole("accent", 7, 72, 4, (0, 4), 14),
)


def _require_rtmidi():
    import rtmidi

    return rtmidi


def _channel_role(channel: int) -> ChannelRole:
    return CHANNEL_ROLES[channel % len(CHANNEL_ROLES)]


def _parse_symbol(symbol: str) -> tuple[int, int]:
    match = re.fullmatch(r"L(\d+)A(\d+)", symbol)
    if not match:
        raise ValueError(f"Unsupported symbol format: {symbol!r}")
    return int(match.group(1)), int(match.group(2))


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _quantize(value: float, grid: float = 0.25) -> float:
    return round(value / grid) * grid


def _scale_degree_to_semitones(degree: int, scale: tuple[int, ...] = PENTATONIC_SCALE) -> int:
    octave, index = divmod(degree, len(scale))
    return octave * 12 + scale[index]


def _transpose_scale(root_note: int, root_degree: int, interval_degree: int) -> int:
    return root_note + (
        _scale_degree_to_semitones(root_degree + interval_degree)
        - _scale_degree_to_semitones(root_degree)
    )


def _compress_gap_to_beats(seconds: float, reference_seconds: float, floor: float, ceil: float) -> float:
    if seconds <= 0:
        return floor
    reference = max(reference_seconds, 1e-6)
    beats = 0.5 + 2.0 * math.log1p(seconds / reference)
    return _quantize(_clamp(beats, floor, ceil))


def _word_peak_amplitude(word: list[Spike]) -> float:
    return max(abs(spike.amplitude) for spike in word)


def _word_internal_intervals(word: list[Spike]) -> list[float]:
    return [right.t - left.t for left, right in zip(word, word[1:]) if right.t > left.t]


def _duration_from_word(word: list[Spike], reference_isi: float) -> float:
    del reference_isi
    duration_index = min(max(len(word), 1) - 1, len(WORD_DURATION_BEATS) - 1)
    return WORD_DURATION_BEATS[duration_index]


def _velocity_from_word(word: list[Spike], global_peak: float) -> int:
    ratio = _word_peak_amplitude(word) / max(global_peak, 1e-6)
    velocity = 40 + round(_clamp(ratio, 0.0, 1.0) * 87)
    return int(_clamp(velocity, 40, 127))


def _phrase_symbols(model: MarkovModel | None, start_symbol: str, word_length: int) -> list[str]:
    phrase_length = max(1, min(3, word_length))
    if model and model.transitions:
        generated = model.generate(start_symbol, length=phrase_length)
        if generated:
            return generated
    return [start_symbol] * phrase_length


def _phrase_schedule(word: list[Spike], phrase_length: int, total_duration: float, reference_isi: float) -> list[tuple[float, float]]:
    if phrase_length <= 1:
        return [(0.0, total_duration)]

    intervals = _word_internal_intervals(word)[: phrase_length - 1]
    while len(intervals) < phrase_length - 1:
        intervals.append(reference_isi)

    offsets = [0.0]
    for interval in intervals:
        offsets.append(
            offsets[-1]
            + _compress_gap_to_beats(interval, reference_isi, floor=0.25, ceil=1.5)
        )

    natural_span = offsets[-1]
    max_span = max(0.25, total_duration - 0.25)
    scale = min(1.0, max_span / natural_span) if natural_span > 0 else 1.0
    offsets = [_quantize(offset * scale) for offset in offsets]

    schedule = []
    for index, offset in enumerate(offsets):
        if index + 1 < len(offsets):
            next_offset = offsets[index + 1]
            duration = max(0.25, (next_offset - offset) * 0.9)
        else:
            duration = max(0.25, total_duration - offset)
        schedule.append((offset, _quantize(min(duration, total_duration - offset))))
    return schedule


def _pitches_for_symbol(
    symbol: str,
    role: ChannelRole,
    previous_amp_bin: int | None,
    melodic_degree: int | None,
) -> tuple[tuple[int, ...], int, int]:
    length_bin, amp_bin = _parse_symbol(symbol)
    if melodic_degree is None or previous_amp_bin is None:
        root_degree = role.scale_degree_offset + (amp_bin % len(PENTATONIC_SCALE))
    else:
        interval = int(_clamp(amp_bin - previous_amp_bin, -len(PENTATONIC_SCALE), len(PENTATONIC_SCALE)))
        root_degree = melodic_degree + interval
    register_shift = 12 * min(2, length_bin // 2)
    root_note = role.base_note + register_shift + _scale_degree_to_semitones(root_degree)
    pitches = {
        int(_clamp(_transpose_scale(root_note, root_degree, degree), 24, 108))
        for degree in role.chord_degrees
    }
    return tuple(sorted(pitches)), root_degree, amp_bin


def _varlen(value: int) -> bytes:
    if value < 0:
        raise ValueError("MIDI delta times must be non-negative")
    buffer = value & 0x7F
    output = bytearray([buffer])
    value >>= 7
    while value:
        output.insert(0, 0x80 | (value & 0x7F))
        value >>= 7
    return bytes(output)


def _event_bytes(kind: str, midi_channel: int, *, note: int | None = None, velocity: int = 0, program: int | None = None) -> bytes:
    if kind == "note_on" and note is not None:
        return bytes([0x90 | midi_channel, note, velocity])
    if kind == "note_off" and note is not None:
        return bytes([0x80 | midi_channel, note, 0])
    if kind == "program_change" and program is not None:
        return bytes([0xC0 | midi_channel, program])
    raise ValueError(f"Unsupported MIDI event: {kind}")


def _write_standard_midi_file(path: Path, note_events: list[NoteEvent], tempo_bpm: int) -> None:
    events: list[tuple[int, int, bytes]] = []
    microseconds_per_quarter = round(60_000_000 / tempo_bpm)
    tempo_bytes = b"\xFF\x51\x03" + microseconds_per_quarter.to_bytes(3, "big")
    time_signature_bytes = b"\xFF\x58\x04\x04\x02\x18\x08"
    track_name_bytes = b"\xFF\x03\x09MycoMIDI2"

    events.append((0, -20, track_name_bytes))
    events.append((0, -10, tempo_bytes))
    events.append((0, -9, time_signature_bytes))

    used_roles = {_channel_role(event.source_channel) for event in note_events}
    for role in sorted(used_roles, key=lambda item: item.midi_channel):
        events.append(
            (
                0,
                -5 + role.midi_channel,
                _event_bytes("program_change", role.midi_channel, program=role.program),
            )
        )

    for event in note_events:
        start_tick = max(0, round(event.start_beat * TICKS_PER_BEAT))
        end_tick = max(start_tick + 1, round((event.start_beat + event.duration_beats) * TICKS_PER_BEAT))
        for pitch in event.pitches:
            events.append(
                (
                    start_tick,
                    10,
                    _event_bytes("note_on", event.midi_channel, note=pitch, velocity=event.velocity),
                )
            )
            events.append(
                (
                    end_tick,
                    5,
                    _event_bytes("note_off", event.midi_channel, note=pitch),
                )
            )

    events.sort(key=lambda item: (item[0], item[1]))
    track = bytearray()
    previous_tick = 0
    for tick, _, payload in events:
        track.extend(_varlen(tick - previous_tick))
        track.extend(payload)
        previous_tick = tick
    track.extend(b"\x00\xFF\x2F\x00")

    header = b"MThd" + struct.pack(">LHHH", 6, 0, 1, TICKS_PER_BEAT)
    chunk = b"MTrk" + struct.pack(">L", len(track)) + bytes(track)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(header + chunk)


def _extract_words(csv_path: str | Path, detect_kwargs: dict | None) -> list[tuple[int, list[Spike], str]]:
    series = load_csv(str(csv_path))
    extracted = []
    for channel, samples in series.items():
        symbols = tokenize_channel(samples, **(detect_kwargs or {}))
        spikes = tokenizer_module.detect_spikes(samples, **(detect_kwargs or {}))
        words = tokenizer_module.spikes_to_words(spikes)
        derived_symbols = [word_to_symbol(word) for word in words]
        if symbols != derived_symbols:
            raise ValueError(
                f"tokenize_channel drifted from detect_spikes/spikes_to_words on channel {channel}: "
                f"{symbols!r} != {derived_symbols!r}"
            )
        extracted.extend((channel, word, symbol) for word, symbol in zip(words, symbols))
    return extracted


def render_csv_to_note_events(
    csv_path: str | Path,
    channel_models: dict[int, MarkovModel],
    *,
    detect_kwargs: dict | None = None,
) -> list[NoteEvent]:
    word_rows = _extract_words(csv_path, detect_kwargs)
    if not word_rows:
        return []

    all_intervals = [
        interval
        for _, word, _ in word_rows
        for interval in _word_internal_intervals(word)
    ]
    global_peak = max(_word_peak_amplitude(word) for _, word, _ in word_rows)
    reference_isi = median(all_intervals) if all_intervals else 60.0

    sorted_rows = sorted(word_rows, key=lambda item: item[1][0].t)
    word_start_gaps = [
        right[1][0].t - left[1][0].t
        for left, right in zip(sorted_rows, sorted_rows[1:])
        if right[1][0].t > left[1][0].t
    ]
    reference_gap = median(word_start_gaps) if word_start_gaps else max(reference_isi * 2, 120.0)

    note_events: list[NoteEvent] = []
    current_beat = 0.0
    previous_start = None
    channel_pitch_state: dict[int, tuple[int, int]] = {}

    for channel, word, symbol in sorted_rows:
        start_time = word[0].t
        if previous_start is not None:
            current_beat += _compress_gap_to_beats(
                start_time - previous_start,
                reference_gap,
                floor=0.5,
                ceil=8.0,
            )
        previous_start = start_time

        role = _channel_role(channel)
        duration = _duration_from_word(word, reference_isi)
        velocity = _velocity_from_word(word, global_peak)
        phrase_symbols = _phrase_symbols(channel_models.get(channel), symbol, len(word))
        schedule = _phrase_schedule(word, len(phrase_symbols), duration, reference_isi)

        for phrase_symbol, (offset, note_duration) in zip(phrase_symbols, schedule):
            previous_amp_bin, melodic_degree = channel_pitch_state.get(channel, (None, None))
            pitches, melodic_degree, amp_bin = _pitches_for_symbol(
                phrase_symbol,
                role,
                previous_amp_bin,
                melodic_degree,
            )
            channel_pitch_state[channel] = (amp_bin, melodic_degree)
            note_events.append(
                NoteEvent(
                    start_beat=current_beat + offset,
                    duration_beats=note_duration,
                    midi_channel=role.midi_channel,
                    pitches=pitches,
                    velocity=velocity,
                    source_channel=channel,
                    source_symbol=phrase_symbol,
                )
            )

    return note_events


def render_csv_to_midi(
    csv_path: str | Path,
    channel_models: dict[int, MarkovModel],
    output_path: str | Path,
    *,
    detect_kwargs: dict | None = None,
    tempo_bpm: int = DEFAULT_TEMPO_BPM,
) -> RenderSummary:
    word_rows = _extract_words(csv_path, detect_kwargs)
    note_events = render_csv_to_note_events(csv_path, channel_models, detect_kwargs=detect_kwargs)
    output = Path(output_path)
    _write_standard_midi_file(output, note_events, tempo_bpm)

    note_count = sum(len(event.pitches) for event in note_events)
    all_pitches = [pitch for event in note_events for pitch in event.pitches]
    channels_used = tuple(sorted({event.source_channel for event in note_events}))
    duration_beats = max(
        (event.start_beat + event.duration_beats for event in note_events),
        default=0.0,
    )
    return RenderSummary(
        output_path=output,
        word_count=len(word_rows),
        note_count=note_count,
        channels_used=channels_used,
        pitch_range=(min(all_pitches), max(all_pitches)) if all_pitches else (None, None),
        duration_beats=duration_beats,
        tempo_bpm=tempo_bpm,
    )


def play_note_events(
    note_events: list[NoteEvent],
    *,
    tempo_bpm: int = DEFAULT_TEMPO_BPM,
    port_index: int = 0,
    open_virtual: bool = True,
) -> None:
    """
    Optional live playback for a rendered phrase via python-rtmidi.
    """
    rtmidi = _require_rtmidi()
    midi_out = rtmidi.MidiOut()
    ports = midi_out.get_ports()
    if ports:
        midi_out.open_port(min(port_index, len(ports) - 1))
    elif open_virtual:
        midi_out.open_virtual_port("MycoMIDI")
    else:
        raise RuntimeError("No MIDI output ports available")

    for role in CHANNEL_ROLES:
        midi_out.send_message([0xC0 | role.midi_channel, role.program])

    seconds_per_beat = 60.0 / tempo_bpm
    previous_time = 0.0
    timeline = []
    for event in note_events:
        for pitch in event.pitches:
            timeline.append((event.start_beat, "on", event.midi_channel, pitch, event.velocity))
            timeline.append((event.start_beat + event.duration_beats, "off", event.midi_channel, pitch, 0))
    timeline.sort(key=lambda item: (item[0], 0 if item[1] == "off" else 1))

    for beat_time, kind, midi_channel, pitch, velocity in timeline:
        sleep_for = (beat_time - previous_time) * seconds_per_beat
        if sleep_for > 0:
            time.sleep(sleep_for)
        status = 0x90 if kind == "on" else 0x80
        midi_out.send_message([status | midi_channel, pitch, velocity])
        previous_time = beat_time


class LiveMusicGenerator:
    """
    Future streaming extension point.

    Real-time processing is intentionally not implemented here. See
    `hardware/realtime-architecture.md` for the planned causal detector
    and queue/process split.
    """

    def __init__(self, channel_models: dict[int, MarkovModel]):
        self.channel_models = channel_models

    def process_new_sample(self, timestamp: float, channel: int, value: float) -> None:
        raise NotImplementedError(
            "Streaming mode is not implemented yet; see hardware/realtime-architecture.md."
        )
