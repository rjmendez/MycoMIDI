import math
import random
import unittest

from tools.tokenizer import MarkovModel, Spike, detect_spikes, spikes_to_words, tokenize_channel


def synthetic_samples():
    centers = [120, 320, 610, 860]
    shape = [0.0, 0.05, 0.2, 0.7, 1.4, 0.7, 0.2, 0.05, 0.0]
    samples = []
    for i in range(1000):
        value = 0.02 * math.sin(i / 45.0) + 0.01 * math.cos(i / 19.0)
        for center in centers:
            start = center - len(shape) // 2
            rel = i - start
            if 0 <= rel < len(shape):
                value += shape[rel]
        if i == 500:
            value += 6.0
        samples.append((float(i), value))
    return samples, centers


class DetectSpikesTests(unittest.TestCase):
    def assert_spike_locations(self, spikes, centers):
        self.assertEqual(len(spikes), len(centers))
        self.assertEqual([round(spike.t) for spike in spikes], centers)

    def test_detect_spikes_local_avg_finds_expected_events(self):
        samples, centers = synthetic_samples()
        spikes = detect_spikes(
            samples,
            window=30,
            delta=0.25,
            min_gap=80,
            method="local_avg",
            max_sample_delta=2.0,
        )
        self.assert_spike_locations(spikes, centers)

    def test_detect_spikes_mad_finds_expected_events(self):
        samples, centers = synthetic_samples()
        spikes = detect_spikes(
            samples,
            window=30,
            delta=0.25,
            min_gap=80,
            method="mad",
            z_threshold=5.0,
            max_sample_delta=2.0,
        )
        self.assert_spike_locations(spikes, centers)

    def test_detect_spikes_mad_causal_does_not_use_future_samples(self):
        samples, _ = synthetic_samples()
        prefix = samples[:700]
        altered = list(samples)
        altered[700:] = [(t, value + 20.0) for t, value in altered[700:]]
        kwargs = {
            "window": 30,
            "min_gap": 80,
            "method": "mad",
            "z_threshold": 5.0,
            "mad_guard": 8,
            "causal": True,
            "max_sample_delta": 2.0,
        }

        prefix_spikes = detect_spikes(prefix, **kwargs)
        altered_spikes = [
            spike for spike in detect_spikes(altered, **kwargs) if spike.t < 700
        ]

        self.assertEqual(
            [round(spike.t) for spike in prefix_spikes],
            [round(spike.t) for spike in altered_spikes],
        )

    def test_detect_spikes_mad_causal_rejects_fast_jump_without_lookahead(self):
        samples = [
            (float(i), 6.0 if i == 250 else 0.01 * math.sin(i / 13.0))
            for i in range(400)
        ]

        spikes = detect_spikes(
            samples,
            window=30,
            min_gap=80,
            method="mad",
            z_threshold=5.0,
            causal=True,
            max_sample_delta=2.0,
        )

        self.assertNotIn(250, [round(spike.t) for spike in spikes])

    def test_detect_spikes_mad_guard_preserves_slow_peak(self):
        samples = []
        for i in range(500):
            distance = abs(i - 250)
            slow_peak = max(0.0, 1.0 - distance / 45.0)
            noise = 0.01 * math.sin(i / 11.0)
            samples.append((float(i), noise + slow_peak))

        spikes = detect_spikes(
            samples,
            window=60,
            mad_window=121,
            mad_guard=30,
            min_gap=100,
            method="mad",
            z_threshold=5.0,
            min_amplitude=0.2,
            max_sample_delta=2.0,
        )

        self.assertEqual(len(spikes), 1)
        self.assertLessEqual(abs(spikes[0].t - 250), 5)

    def test_detect_spikes_rejects_single_sample_artifact(self):
        samples, _ = synthetic_samples()
        for method in ("local_avg", "mad"):
            unfiltered = detect_spikes(
                samples,
                window=30,
                delta=0.25,
                min_gap=80,
                method=method,
                z_threshold=5.0,
                max_sample_delta=None,
            )
            filtered = detect_spikes(
                samples,
                window=30,
                delta=0.25,
                min_gap=80,
                method=method,
                z_threshold=5.0,
                max_sample_delta=2.0,
            )
            self.assertIn(500, [round(spike.t) for spike in unfiltered])
            self.assertNotIn(500, [round(spike.t) for spike in filtered])

    def test_tokenize_channel_uses_default_detector_without_error(self):
        samples, _ = synthetic_samples()
        symbols = tokenize_channel(
            samples,
            window=30,
            delta=0.25,
            min_gap=80,
            z_threshold=5.0,
            max_sample_delta=2.0,
        )
        self.assertEqual(symbols, ["L4A7"])


class WordAndMarkovTests(unittest.TestCase):
    def test_spikes_to_words_groups_by_gap(self):
        spikes = [
            Spike(t=10.0, channel=0, amplitude=0.6),
            Spike(t=40.0, channel=0, amplitude=0.8),
            Spike(t=120.0, channel=0, amplitude=0.7),
            Spike(t=400.0, channel=0, amplitude=0.9),
        ]
        words = spikes_to_words(spikes, gap_threshold_s=90.0)
        self.assertEqual([[round(spike.t) for spike in word] for word in words], [[10, 40, 120], [400]])

    def test_markov_model_fit_and_generate(self):
        random.seed(0)
        model = MarkovModel()
        symbols = ["L2A7", "L1A5", "L2A7", "L3A6"]
        model.fit(symbols)
        generated = model.generate("L2A7", length=6)
        self.assertEqual(len(generated), 6)
        self.assertEqual(generated[0], "L2A7")
        self.assertTrue(set(generated).issubset(set(symbols)))

    def test_markov_model_generate_with_no_transitions_does_not_crash(self):
        # A single-word session never populates any transitions (fit()
        # zips symbols with symbols[1:], which is empty for length 1).
        model = MarkovModel()
        model.fit(["L4A7"])
        generated = model.generate("L4A7", length=5)
        self.assertEqual(generated, ["L4A7"] * 5)

    def test_markov_model_generate_with_untrained_model_does_not_crash(self):
        model = MarkovModel()
        generated = model.generate("L0A0", length=3)
        self.assertEqual(generated, ["L0A0"] * 3)


if __name__ == "__main__":
    unittest.main()
