import csv
import math
import os
import unittest
from pathlib import Path

from tools.musicgen import (
    _channel_role,
    _compress_gap_to_beats,
    _pitches_for_symbol,
    render_csv_to_midi,
    render_csv_to_note_events,
)
from tools.tokenizer import MarkovModel, load_csv, tokenize_channel


REPO_ROOT = Path(__file__).resolve().parent.parent


def _inject_spike(value: float, sample_index: int, centers: list[int], amplitude: float) -> float:
    shape = [0.0, 0.08, 0.22, 0.7, 1.15, 0.7, 0.22, 0.08, 0.0]
    for center in centers:
        start = center - len(shape) // 2
        rel = sample_index - start
        if 0 <= rel < len(shape):
            value += shape[rel] * amplitude
    return value


def create_synthetic_logger_csv(path: Path, *, channels: int = 3, samples: int = 2400) -> None:
    per_channel_centers = {
        0: [120, 180, 240, 980, 1040, 1710, 1775],
        1: [160, 220, 1000, 1620, 1680],
        2: [140, 210, 270, 1080, 1750],
    }
    amplitudes = {0: 1.0, 1: 0.8, 2: 1.2}
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t_unix"] + [f"ch{i}_v" for i in range(channels)])
        for sample_index in range(samples):
            row = [float(sample_index)]
            for channel in range(channels):
                value = 0.018 * math.sin(sample_index / 33.0 + channel)
                value += 0.012 * math.cos(sample_index / 57.0 + channel * 0.5)
                value = _inject_spike(
                    value,
                    sample_index,
                    per_channel_centers[channel],
                    amplitudes[channel],
                )
                row.append(round(value, 6))
            writer.writerow(row)


class MusicGenerationTests(unittest.TestCase):
    def setUp(self):
        self.csv_path = REPO_ROOT / "tools" / f"_synthetic_musicgen_{os.getpid()}.csv"
        self.midi_path = REPO_ROOT / "tools" / f"_synthetic_musicgen_{os.getpid()}.mid"
        create_synthetic_logger_csv(self.csv_path)
        self.addCleanup(lambda: self.csv_path.unlink(missing_ok=True))
        self.addCleanup(lambda: self.midi_path.unlink(missing_ok=True))
        self.detect_kwargs = {
            "window": 30,
            "min_gap": 40,
            "method": "mad",
            "z_threshold": 4.0,
            "max_sample_delta": 2.0,
        }

    def _train_models(self) -> dict[int, MarkovModel]:
        models = {}
        for channel, samples in load_csv(str(self.csv_path)).items():
            symbols = tokenize_channel(samples, **self.detect_kwargs)
            model = MarkovModel()
            if len(symbols) > 1:
                model.fit(symbols)
            models[channel] = model
        return models

    def test_render_pipeline_writes_midi_and_events(self):
        models = self._train_models()

        note_events = render_csv_to_note_events(
            self.csv_path,
            models,
            detect_kwargs=self.detect_kwargs,
        )
        self.assertGreaterEqual(len(note_events), 6)
        self.assertGreaterEqual(len({event.source_channel for event in note_events}), 3)
        self.assertTrue(all(24 <= pitch <= 108 for event in note_events for pitch in event.pitches))

        summary = render_csv_to_midi(
            self.csv_path,
            models,
            self.midi_path,
            detect_kwargs=self.detect_kwargs,
        )
        self.assertTrue(self.midi_path.exists())
        self.assertEqual(self.midi_path.read_bytes()[:4], b"MThd")
        self.assertGreater(summary.note_count, 0)
        self.assertGreater(summary.duration_beats, 0.0)
        self.assertIsNotNone(summary.pitch_range[0])
        self.assertIsNotNone(summary.pitch_range[1])
        print(
            "musicgen summary:",
            {
                "words": summary.word_count,
                "notes": summary.note_count,
                "pitch_range": summary.pitch_range,
                "channels": summary.channels_used,
                "duration_beats": summary.duration_beats,
            },
        )

    def test_gap_compression_is_logarithmic_and_bounded(self):
        short = _compress_gap_to_beats(60.0, 60.0, 0.5, 8.0)
        medium = _compress_gap_to_beats(600.0, 60.0, 0.5, 8.0)
        very_long = _compress_gap_to_beats(60_000.0, 60.0, 0.5, 8.0)

        self.assertLess(short, medium)
        self.assertLessEqual(medium, very_long)
        self.assertEqual(very_long, 8.0)

    def test_amplitude_delta_controls_scale_locked_interval(self):
        role = _channel_role(4)
        first, degree, amp_bin = _pitches_for_symbol("L1A2", role, None, None)
        rising, next_degree, next_amp_bin = _pitches_for_symbol(
            "L1A4",
            role,
            amp_bin,
            degree,
        )
        falling, _, _ = _pitches_for_symbol(
            "L1A1",
            role,
            next_amp_bin,
            next_degree,
        )

        self.assertGreater(rising[0], first[0])
        self.assertLess(falling[0], rising[0])
        allowed_pitch_classes = {
            (role.base_note + semitones) % 12 for semitones in (0, 2, 4, 7, 9)
        }
        self.assertTrue(
            all(
                pitch % 12 in allowed_pitch_classes
                for pitches in (first, rising, falling)
                for pitch in pitches
            )
        )


if __name__ == "__main__":
    unittest.main()
