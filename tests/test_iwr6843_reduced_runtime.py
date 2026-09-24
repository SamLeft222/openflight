"""Reduced transfer through the runtime: same measurement, fewer bytes.

plans/iwr6843-on-chip-reduction.md, Phase 0: with the overview handed over
unquantised, ``measure_reduced`` must reproduce the full-capture measurement
-- on the baseline track and through the OPS-guided search -- while the radar
sends a fraction of the capture.
"""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest
from test_iwr6843_pipeline import _cal, synth_shot  # noqa: F401
from test_iwr6843_reduced import N_FRAMES, PERIOD_US, _timed_iq8

from openflight.iwr6843.dump import pack_dump, parse_capture_metadata, parse_dump
from openflight.iwr6843.reduced import CaptureReducedSource, ball_gate_for
from openflight.iwr6843.runtime import IWR6843Runtime

NET_M = 4.0
BALL_MS = 45.0
BALL_MPH = BALL_MS * 2.23694


def _shot_adc(**overrides):
    params = dict(
        speed_ms=BALL_MS,
        launch_deg=18.0,
        n_frames=N_FRAMES,
        n_loops=12,
        n_tx=3,
        frame_period_us=PERIOD_US,
        trigger_frame=0,
    )
    params.update(overrides)
    return synth_shot(**params)


def _shot(**overrides):
    return _timed_iq8(_shot_adc(**overrides))


def _two_movers(slow_ms: float) -> bytes:
    """The ball plus a slower, louder second mover the OPS speed can point at.

    Loud enough to win range-gate detections, not enough to steal the
    tracker's fastest-credible choice from the ball.
    """
    meta, fast = parse_dump(_shot_adc())
    _, slow = parse_dump(_shot_adc(speed_ms=slow_ms, launch_deg=8.0, amp=500.0, seed=1))
    return _timed_iq8(
        pack_dump(
            fast + slow,
            n_tx=meta["n_tx"],
            trigger_frame=0,
            version=meta["version"],
            frame_period_us=meta["frame_period_us"],
        )
    )


@pytest.fixture(name="capture", scope="module")
def _capture():
    return _shot()


@pytest.fixture(name="runtime")
def _runtime(cal):
    return IWR6843Runtime(
        capture_monitor=SimpleNamespace(),
        calibration=cal,
        net_range_m=NET_M,
        tx_order="normal",
        tdm_sign_policy="positive",
    )


def _source(capture, *, quantise=False):
    gate = ball_gate_for(parse_capture_metadata(capture), NET_M)
    return CaptureReducedSource(capture, gate=gate, quantise=quantise)


def _full(runtime, capture, ball_speed_mph=BALL_MPH):
    return runtime.measure_capture(capture, ball_speed_mph=ball_speed_mph, club="driver")


def _reduced(runtime, source, ball_speed_mph=BALL_MPH):
    return runtime.measure_reduced(source, ball_speed_mph=ball_speed_mph, club="driver")


def _same(a, b):
    """The same shot: identical statuses and counts; floats agree to 1e-9.

    The overview's integer MTI maths can differ from the float full-capture
    path in the last ulp, which reaches the track fit's floats at that level.
    """
    left, right = a.to_dict(), b.to_dict()
    assert left.keys() == right.keys()
    for key, value in left.items():
        if isinstance(value, float) and isinstance(right[key], float):
            assert value == pytest.approx(right[key], rel=1e-9, abs=1e-12), key
        else:
            assert value == right[key], key


def test_baseline_track_is_reproduced_exactly(runtime, capture):
    full = _full(runtime, capture)
    source = _source(capture)

    reduced = _reduced(runtime, source)

    assert full.accepted
    _same(reduced, full)
    assert len(source.strip_nbytes) == 1


def test_ops_guided_search_is_reproduced_exactly(runtime):
    # The tracker prefers the faster mover; OPS reports the slower one, so the
    # baseline fails the speed check and the OPS-guided search evaluates
    # candidates on strips fetched for them.
    slow_ms = 34.0
    capture = _two_movers(slow_ms)
    ops_mph = slow_ms * 2.23694
    full = _full(runtime, capture, ops_mph)
    source = _source(capture)

    reduced = _reduced(runtime, source, ops_mph)

    _same(reduced, full)
    assert full.status.startswith("accepted_ops_guided")
    assert len(source.strip_nbytes) == 2


def test_no_candidates_means_no_candidate_strips(runtime, capture):
    # OPS 25 % off and no mover at that speed: nothing to fetch for the search.
    ops_mph = BALL_MPH * 1.25
    source = _source(capture)

    reduced = _reduced(runtime, source, ops_mph)

    _same(reduced, _full(runtime, capture, ops_mph))
    assert reduced.status == "accepted_track_speed_warning"
    assert len(source.strip_nbytes) == 1


def test_reduced_transfer_is_a_fraction_of_the_capture(runtime, capture):
    source = _source(capture, quantise=True)

    _reduced(runtime, source)

    assert source.total_nbytes < 0.5 * len(capture)


def test_quantised_overview_gives_the_same_shot(runtime, capture):
    full = _full(runtime, capture)

    reduced = _reduced(runtime, _source(capture, quantise=True))

    assert reduced.accepted
    assert reduced.angle_deg == pytest.approx(full.angle_deg, abs=0.5)


def test_no_ball_requests_no_baseline_strip(runtime):
    empty = _shot(amp=0.0)
    source = _source(empty)

    reduced = _reduced(runtime, source)

    _same(reduced, _full(runtime, empty))
    assert not reduced.accepted
    assert source.strip_nbytes == []


def test_overview_gate_must_match_the_tracker(runtime, capture):
    source = CaptureReducedSource(capture, gate=(40, 90), quantise=False)

    with pytest.raises(ValueError, match="does not match the tracker gate"):
        _reduced(runtime, source)


def test_full_capture_path_is_unchanged_by_the_refactor(runtime, capture):
    """process_shot still measures the whole capture the way it always did."""
    runtime.capture_monitor = SimpleNamespace(
        capture_for_shot=lambda *_a, **_k: SimpleNamespace(
            valid=True, raw=capture, trigger_timestamp=0.0, path=None, sequence=1
        )
    )

    result = runtime.process_shot(impact_timestamp=0.0, ball_speed_mph=BALL_MPH, club="driver")

    _same(result.measurement, _full(runtime, capture))
    assert np.isfinite(result.measurement.angle_deg)
