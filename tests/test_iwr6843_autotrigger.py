"""The IWR6843 on-radar shot detector, reference model (autotrigger.py)."""

from __future__ import annotations

import numpy as np
import pytest

from openflight.iwr6843.autotrigger import (
    AutoTrigger,
    AutoTriggerConfig,
    frame_power,
    median,
    normalise,
)

START, BINS = 20, 53
TEE = 39


def _config(**overrides) -> AutoTriggerConfig:
    return AutoTriggerConfig(
        **{"end_lo": TEE - 1, "end_hi": TEE + 1, "delay_frames": 0, **overrides}
    )


def _row(targets: dict[float, float] | None = None, floor: float = 1.0) -> list[float]:
    """A normalised frame: ``floor`` everywhere, plus z at (fractional) absolute bins."""
    row = [floor] * BINS
    for position, z in (targets or {}).items():
        row[int(round(position)) - START] = z
    return row


def _track(end: float, slope: float, n: int, z: float = 20.0) -> list[list[float]]:
    """n frames of a target moving ``slope`` bins/frame, reaching ``end`` last."""
    return [_row({end - slope * (n - 1 - k): z}) for k in range(n)]


def _run(trigger: AutoTrigger, rows) -> list[bool]:
    return [trigger.push(START, row) for row in rows]


def test_outward_club_track_reaching_the_tee_fires():
    trigger = AutoTrigger(_config())

    fired = _run(trigger, [_row()] * 3 + _track(TEE, 1.5, 5))

    assert fired == [False] * 7 + [True]
    assert trigger.detection.end_bin == TEE
    assert trigger.detection.slope_q == 6
    assert trigger.detection.frame == 7


@pytest.mark.parametrize(
    ("end", "slope", "why"),
    [
        (TEE, 0.0, "stationary (the golfer)"),
        (TEE, 0.1, "walking pace"),
        (TEE, 4.0, "faster than any club"),
        (TEE - 6, 1.5, "still short of the tee"),
        (TEE + 6, 1.5, "already past the tee"),
    ],
)
def test_tracks_that_are_not_a_club_at_the_tee_do_not_fire(end, slope, why):
    trigger = AutoTrigger(_config())

    assert not any(_run(trigger, _track(end, slope, 8))), why


def test_every_frame_must_reach_the_threshold():
    trigger = AutoTrigger(_config(z_min_x10=80))
    rows = _track(TEE, 1.5, 5, z=20.0)
    rows[2] = _row()  # one frame of the track missing

    assert not any(_run(trigger, rows))
    assert not any(_run(AutoTrigger(_config(z_min_x10=80)), _track(TEE, 1.5, 5, z=7.9)))
    assert _run(AutoTrigger(_config(z_min_x10=80)), _track(TEE, 1.5, 5, z=8.0))[-1]


def test_fractional_positions_use_either_neighbouring_bin():
    trigger = AutoTrigger(_config())
    # 1.25 bins/frame lands between bins; a hit in either neighbour counts.
    rows = [_row({np.floor(TEE - 1.25 * (4 - k)): 20.0}) for k in range(5)]

    assert _run(trigger, rows)[-1]


def test_delay_requests_the_freeze_later_and_ignores_new_tracks_meanwhile():
    trigger = AutoTrigger(_config(delay_frames=3))

    fired = _run(trigger, _track(TEE, 1.5, 5) + _track(TEE, 1.5, 5))

    # Detected at frame 4, freeze requested 3 frames later, then quiet.
    assert fired == [False] * 7 + [True] + [False] * 2
    assert trigger.detection.frame == 4


def test_fires_once_until_reset():
    trigger = AutoTrigger(_config())

    fired = _run(trigger, _track(TEE, 1.5, 5) + _track(TEE, 1.5, 5))
    trigger.reset()
    again = _run(trigger, _track(TEE, 1.5, 5))

    assert fired.count(True) == 1
    assert again[-1]


def test_needs_a_full_history_before_searching():
    trigger = AutoTrigger(_config(n_frames=5))

    assert not any(_run(trigger, _track(TEE, 1.5, 5)[1:]))


def test_reset_forgets_history_and_a_pending_freeze():
    trigger = AutoTrigger(_config(delay_frames=2))
    _run(trigger, _track(TEE, 1.5, 5))

    trigger.reset()

    assert trigger.detection is None
    assert not any(_run(trigger, [_row()] * 4))


def test_best_track_prefers_the_higher_score():
    trigger = AutoTrigger(_config())
    rows = [_row({TEE - 1.5 * (4 - k): 20.0, TEE + 1 - 1.0 * (4 - k): 45.0}) for k in range(5)]

    assert _run(trigger, rows)[-1]
    assert (trigger.detection.end_bin, trigger.detection.slope_q) == (TEE + 1, 4)


def test_frames_with_different_windows_are_compared_in_absolute_bins():
    trigger = AutoTrigger(_config())
    rows = _track(TEE, 1.5, 5)
    shifted = [0.0] * 12 + rows[-1][: BINS - 12]  # same targets, window starts 12 bins lower

    fired = [trigger.push(START, row) for row in rows[:-1]] + [trigger.push(START - 12, shifted)]

    assert fired[-1]


def test_frame_power_is_the_loop_mti_power_of_the_first_tx():
    rng = np.random.default_rng(0)
    codes = rng.integers(-128, 128, size=(36, 4, 7, 2))
    x = codes[0::3, ..., 1] + 1j * codes[0::3, ..., 0]
    expected = (np.abs(x - x.mean(axis=0)) ** 2).sum(axis=(0, 1))

    power = frame_power(codes, n_tx=3)

    assert power == pytest.approx(expected.tolist(), rel=1e-12)


def test_frame_power_is_zero_for_a_static_scene():
    codes = np.broadcast_to(np.array([3, -7]), (36, 4, 5, 2))

    assert frame_power(codes, n_tx=3) == [0.0] * 5


def test_median_and_normalise():
    assert median([3.0, 1.0, 2.0]) == 2.0
    assert median([4.0, 1.0, 3.0, 2.0]) == 2.5
    assert median([]) == 0.0
    assert normalise([1.0, 2.0, 400.0], 50.0) == [0.5, 1.0, 50.0]
    assert normalise([0.0, 0.0, 5.0], 50.0) == [0.0, 0.0, 0.0]


def test_rejects_windows_outside_the_bin_space():
    trigger = AutoTrigger(_config())

    with pytest.raises(ValueError):
        trigger.push(250, [1.0] * 10)
    with pytest.raises(ValueError):
        trigger.push(0, [1.0] * 65)


@pytest.mark.parametrize(
    "overrides",
    [
        {"end_lo": 40, "end_hi": 39},
        {"end_hi": 256},
        {"n_frames": 1},
        {"n_frames": 9},
        {"slope_min_q": 0},
        {"slope_min_q": 5, "slope_max_q": 4},
        {"z_min_x10": 0},
        {"z_min_x10": 600, "z_cap_x10": 500},
        {"delay_frames": 33},
    ],
)
def test_config_validation(overrides):
    with pytest.raises(ValueError):
        _config(**overrides)


def test_config_command_line():
    assert _config(delay_frames=3).command() == "autoTrigCfg 1 38 40 5 80 500 2 12 3"
