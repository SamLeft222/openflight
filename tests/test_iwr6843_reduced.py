"""Reduced-transfer wire format and firmware reference model.

plans/iwr6843-on-chip-reduction.md: the radar sends a power overview and then
strips of complex data along the Pi-selected track(s). ``build_overview`` and
``serve_strips`` are the reference the firmware must reproduce byte-for-byte;
``pack_overview``/``parse_overview`` are the one executable definition of the
overview layout.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from test_iwr6843_pipeline import synth_shot

from openflight.iwr6843 import reduced, tracking
from openflight.iwr6843.dump import (
    SAMPLE_RANGE_FFT_IQ8_VARIABLE_TIMED,
    SAMPLE_RANGE_FFT_IQ16_WINDOWED,
    TEMP_REPORT_KEYS,
    pack_dump,
    parse_dump,
    project_tx_pair,
)
from openflight.iwr6843.shot import prepare_shot_dump
from openflight.iwr6843.tracking import BallTrack

N_FRAMES = 12
N_BINS = 53
STARTS = (20,) * 4 + (32,) * 4 + (47,) * 4
PERIOD_US = 4000
GATE = (48, 98)


def _timed_iq8(raw_adc: bytes, starts=STARTS) -> bytes:
    """Raw ADC -> the v7 timed IQ8 layout the current firmware streams."""
    meta, cube = parse_dump(raw_adc)
    rfft = np.fft.fft(cube, axis=-1)
    windows = np.stack([rfft[frame, ..., s : s + N_BINS] for frame, s in enumerate(starts)])
    return pack_dump(
        windows,
        n_tx=meta["n_tx"],
        version=7,
        frame_period_us=PERIOD_US,
        sample_fmt=SAMPLE_RANGE_FFT_IQ8_VARIABLE_TIMED,
        range_bin_starts=starts,
        range_bin_counts=(N_BINS,) * len(starts),
        frame_time_offsets_us=tuple(PERIOD_US * f for f in range(len(starts))),
        temperature_report={key: 40 for key in TEMP_REPORT_KEYS},
    )


@pytest.fixture(name="capture", scope="module")
def _capture():
    return _timed_iq8(
        synth_shot(
            speed_ms=45.0,
            launch_deg=18.0,
            n_frames=N_FRAMES,
            n_loops=12,
            n_tx=3,
            frame_period_us=PERIOD_US,
            trigger_frame=0,
        )
    )


@pytest.fixture(name="overview", scope="module")
def _overview(capture):
    return reduced.build_overview(capture, gate=GATE)


def _vertical(capture):
    return prepare_shot_dump(project_tx_pair(capture, reduced.VERTICAL_TX_PAIR))


# -- log power encoding ----------------------------------------------------------


def test_log_power_round_trip_precision():
    power = np.array([1e-3, 0.5, 1.0, 3.7, 1e6, 4.2e11])

    decoded = reduced.decode_log_power(reduced.encode_log_power(power))

    np.testing.assert_allclose(decoded, power, rtol=2 ** (1 / 512) - 1)


def test_exact_zero_power_has_its_own_code():
    codes = reduced.encode_log_power(np.array([0.0, 1.0]))

    assert codes[0] == reduced.ZERO_POWER_CODE
    assert reduced.decode_log_power(codes)[0] == 0.0


def test_extreme_powers_clip_instead_of_wrapping():
    codes = reduced.encode_log_power(np.array([1e-300, 1e300]))

    assert codes[0] == reduced.ZERO_POWER_CODE + 1
    assert codes[1] == np.iinfo(np.int16).max


@pytest.mark.parametrize("bad", [-1.0, math.nan, math.inf])
def test_unrepresentable_power_is_rejected(bad):
    with pytest.raises(ValueError, match="finite and non-negative"):
        reduced.encode_log_power(np.array([1.0, bad]))


# -- the radar-side overview ----------------------------------------------------


def test_overview_holds_gate_power_noise_and_means(capture, overview):
    vertical = _vertical(capture)
    loops = 12

    for scope in tracking.MTI_SCOPES:
        full = vertical.loop_power(scope)
        for frame, start in enumerate(STARTS):
            a, b = max(GATE[0], start) - start, min(GATE[1], start + N_BINS) - start
            rows = slice(frame * loops, (frame + 1) * loops)
            np.testing.assert_array_equal(overview.power[scope][rows, a:b], full[rows, a:b])
            assert not overview.power[scope][rows, :a].any()
            assert not overview.power[scope][rows, b:].any()
        assert overview.noise[scope] == vertical.noise_power(scope)
    np.testing.assert_array_equal(
        overview.window_means,
        tracking.compute_window_means(vertical.cube, vertical.geometry, range_domain=True),
    )


def test_overview_needs_a_timed_capture():
    cube = np.zeros((2, 6, 4, 8), dtype=complex)
    windowed = pack_dump(
        cube,
        n_tx=3,
        version=4,
        sample_fmt=SAMPLE_RANGE_FFT_IQ16_WINDOWED,
        range_bin_starts=(20, 20),
    )
    with pytest.raises(ValueError, match="timed"):
        reduced.build_overview(windowed, gate=GATE)


@pytest.mark.parametrize("gate", [(98, 48), (48, 48), (-1, 98)])
def test_overview_rejects_an_invalid_gate(capture, gate):
    with pytest.raises(ValueError, match="gate"):
        reduced.build_overview(capture, gate=gate)


# -- overview wire format ---------------------------------------------------------


def test_overview_round_trip(overview):
    parsed = reduced.parse_overview(reduced.pack_overview(overview))

    assert parsed.capture_prefix == overview.capture_prefix
    assert parsed.gate == overview.gate
    for scope in tracking.MTI_SCOPES:
        assert parsed.noise[scope] == pytest.approx(overview.noise[scope], rel=1e-7)
        np.testing.assert_allclose(
            parsed.power[scope], overview.power[scope], rtol=2 ** (1 / 512) - 1
        )
        assert np.array_equal(parsed.power[scope] == 0, overview.power[scope] == 0)
    np.testing.assert_allclose(parsed.window_means, overview.window_means, rtol=1e-6, atol=1e-3)


def test_overview_is_a_fraction_of_the_capture(capture, overview):
    packed = reduced.pack_overview(overview)
    gate_bins = sum(min(GATE[1], s + N_BINS) - max(GATE[0], s) for s in STARTS if s < GATE[1])
    means_bins = max(STARTS) + N_BINS - min(STARTS)
    expected = (
        len(overview.capture_prefix)
        + reduced.OVERVIEW_HEADER.size
        + 2 * gate_bins * 12 * 2
        + 2 * 4 * means_bins * 8
    )

    assert len(packed) == expected
    assert len(packed) < 0.5 * len(capture)


def _corrupt(packed: bytes, overview, offset: int, value: bytes) -> bytes:
    at = len(overview.capture_prefix) + offset
    return packed[:at] + value + packed[at + len(value) :]


@pytest.mark.parametrize(
    "mutate, message",
    [
        (lambda p, o: _corrupt(p, o, 0, b"XXXX"), "magic"),
        (lambda p, o: _corrupt(p, o, 4, (2).to_bytes(2, "little")), "version"),
        (lambda p, o: _corrupt(p, o, 10, (11).to_bytes(2, "little")), "loops"),
        (lambda p, o: _corrupt(p, o, 6, (99).to_bytes(2, "little")), "gate"),
        (lambda p, o: _corrupt(p, o, 20, (19).to_bytes(2, "little")), "window means"),
        (lambda p, o: p[:-1], "bytes, expected"),
        (lambda p, o: p + b"\0", "bytes, expected"),
        (lambda p, o: p[: len(o.capture_prefix) + 5], "short overview header"),
    ],
)
def test_malformed_overview_is_rejected(overview, mutate, message):
    packed = reduced.pack_overview(overview)

    with pytest.raises(ValueError, match=message):
        reduced.parse_overview(mutate(packed, overview))


@pytest.mark.parametrize("noise", [0.0, -1.0, math.nan])
def test_overview_with_unusable_noise_is_not_packed(overview, noise):
    bad = reduced.Overview(
        capture_prefix=overview.capture_prefix,
        gate=overview.gate,
        power=overview.power,
        noise={"burst": noise, "window": 1.0},
        window_means=overview.window_means,
    )
    with pytest.raises(ValueError, match="noise"):
        reduced.pack_overview(bad)


# -- strips ------------------------------------------------------------------------


def _request():
    return tuple((s + 30, 5) if f % 3 else None for f, s in enumerate(STARTS))


def test_assembled_capture_holds_exactly_the_strips(capture, overview):
    request = _request()

    assembled, present = reduced.assemble_capture(overview, reduced.serve_strips(capture, request))

    meta, full = parse_dump(capture)
    assembled_meta, cube = parse_dump(assembled)
    assert assembled_meta["range_bin_starts"] == meta["range_bin_starts"]
    assert assembled_meta["iq8_scales"] == meta["iq8_scales"]
    for frame, window in enumerate(request):
        start, count = window if window is not None else (STARTS[frame], 1)
        local = start - STARTS[frame]
        expected = np.zeros(N_BINS, dtype=bool)
        expected[local : local + count] = True
        np.testing.assert_array_equal(present[frame], expected)
        np.testing.assert_array_equal(cube[frame][..., expected], full[frame][..., expected])
        assert not cube[frame][..., ~expected].any()


def test_strips_are_smaller_than_the_capture(capture):
    assert len(reduced.serve_strips(capture, _request())) < 0.2 * len(capture)


def test_assembling_without_strips_gives_an_empty_capture(capture, overview):
    assembled, present = reduced.assemble_capture(overview)

    assert not present.any()
    assert not parse_dump(assembled)[1].any()
    assert len(assembled) == len(capture)


@pytest.mark.parametrize(
    "request_, message",
    [
        (((20, 5),) * (N_FRAMES - 1), "12"),
        (((19, 5),) + ((20, 5),) * (N_FRAMES - 1), "outside its window"),
        (((70, 5),) + ((20, 5),) * (N_FRAMES - 1), "outside its window"),
        (((20, 0),) + ((20, 5),) * (N_FRAMES - 1), "outside its window"),
    ],
)
def test_invalid_strip_requests_are_rejected(capture, request_, message):
    with pytest.raises(ValueError, match=message):
        reduced.serve_strips(capture, request_)


def test_strips_from_another_capture_are_rejected(capture, overview):
    other = _timed_iq8(
        synth_shot(
            speed_ms=45.0,
            launch_deg=18.0,
            n_frames=N_FRAMES + 1,
            n_loops=12,
            n_tx=3,
            frame_period_us=PERIOD_US,
            trigger_frame=0,
        ),
        starts=STARTS + (47,),
    )
    strips = reduced.serve_strips(other, (None,) * (N_FRAMES + 1))

    with pytest.raises(ValueError, match="n_frames"):
        reduced.assemble_capture(overview, strips)


# -- corridor requests -------------------------------------------------------------


def _track(t_first=0.010, t_last=0.030, slope_bins=900.0, intercept_bins=40.0):
    return BallTrack(
        speed_ms=slope_bins * tracking.RANGE_SPAN_M / 128,
        slope_bins=slope_bins,
        intercept_bins=intercept_bins,
        rms_bins=0.3,
        n_inliers=40,
        t_first=t_first,
        t_last=t_last,
        low_confidence=False,
    )


def _snapshot_bins(meta, track):
    """Every (frame, bin) LCMF/horizontal can read for ``track``."""
    geometry = reduced.capture_geometry(meta)
    wanted = set()
    for frame in range(geometry.n_frames):
        for loop in range(geometry.n_loops):
            t = geometry.loop_time(frame, loop)
            margin = reduced.TRACK_TIME_MARGIN_S
            if track.t_first - margin <= t <= track.t_last + margin:
                wanted.add((frame, int(round(track.bin_at(t)))))
    return wanted


def _covered(request, frame, absolute_bin, width=1):
    window = request[frame]
    return window is not None and window[0] + width <= absolute_bin < window[0] + window[1] - width


def test_corridor_covers_every_snapshot_with_a_margin(overview):
    meta = overview.metadata
    track = _track()

    request = reduced.corridor_request([track], meta)

    wanted = _snapshot_bins(meta, track)
    assert wanted
    for frame, absolute_bin in wanted:
        if STARTS[frame] + 1 <= absolute_bin < STARTS[frame] + N_BINS - 1:
            assert _covered(request, frame, absolute_bin), (frame, absolute_bin)


def test_corridor_skips_frames_outside_the_track(overview):
    request = reduced.corridor_request([_track(t_first=0.010, t_last=0.020)], overview.metadata)

    assert request[0] is None
    assert request[-1] is None
    assert any(window is not None for window in request)


def test_corridor_is_the_union_of_tracks(overview):
    meta = overview.metadata
    a, b = _track(intercept_bins=40.0), _track(intercept_bins=46.0)

    union = reduced.corridor_request([a, b], meta)

    for track in (a, b):
        single = reduced.corridor_request([track], meta)
        for frame, window in enumerate(single):
            if window is not None:
                assert union[frame][0] <= window[0]
                assert union[frame][0] + union[frame][1] >= window[0] + window[1]


def test_corridor_is_clipped_to_each_frame_window(overview):
    request = reduced.corridor_request([_track(slope_bins=3000.0)], overview.metadata)

    for frame, window in enumerate(request):
        if window is not None:
            assert STARTS[frame] <= window[0]
            assert window[0] + window[1] <= STARTS[frame] + N_BINS


def test_corridor_needs_a_margin(overview):
    with pytest.raises(ValueError, match="at least one bin"):
        reduced.corridor_request([_track()], overview.metadata, width_bins=0)


# -- wire helpers for the driver -------------------------------------------------------


def test_strip_request_encoding():
    assert reduced.encode_strip_request((None, (47, 1), (255, 255))) == "00002f01ffff"


@pytest.mark.parametrize(
    "request_, message",
    [
        ((None,) * (reduced.MAX_STRIP_REQUEST_FRAMES + 1), "at most"),
        (((47, 0),), "does not fit"),
        (((256, 1),), "does not fit"),
        (((-1, 1),), "does not fit"),
        (((47, 256),), "does not fit"),
    ],
)
def test_unencodable_strip_requests_are_rejected(request_, message):
    with pytest.raises(ValueError, match=message):
        reduced.encode_strip_request(request_)


def test_overview_size_is_known_once_its_header_arrives(overview):
    packed = reduced.pack_overview(overview)
    header_end = len(overview.capture_prefix) + reduced.OVERVIEW_HEADER.size

    for cut in (0, 20, 44, len(overview.capture_prefix), header_end - 1):
        assert reduced.overview_nbytes(packed[:cut]) is None, cut
    for cut in (header_end, header_end + 100, len(packed)):
        assert reduced.overview_nbytes(packed[:cut]) == len(packed)


def test_overview_size_rejects_a_non_overview(overview):
    packed = bytearray(reduced.pack_overview(overview))
    packed[len(overview.capture_prefix)] = ord("X")

    with pytest.raises(ValueError, match="magic"):
        reduced.overview_nbytes(bytes(packed))
