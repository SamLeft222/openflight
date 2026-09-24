"""Supplied signal products: the Pi side of the on-chip reduction plan.

In reduced-transfer mode (plans/iwr6843-on-chip-reduction.md) the radar sends
the per-loop power maps, noise powers, and window-scope static means instead
of the whole capture. The tracker, the OPS-guided search, and LCMF must then
use those supplied products. Supplying exactly what the Pi would have
computed must change nothing; supplying something else must be what is used.
"""

from __future__ import annotations

import numpy as np
import pytest
from test_iwr6843_pipeline import _cal, synth_shot, windowed_range_snapshot_dump  # noqa: F401

from openflight.iwr6843 import process_dump, tracking
from openflight.iwr6843.recovery import find_recovery_candidates
from openflight.iwr6843.shot import prepare_shot_dump

STARTS = (20,) * 4 + (32,) * 4 + (47,) * 4


@pytest.fixture(name="raw")
def _raw():
    return windowed_range_snapshot_dump(
        synth_shot(speed_ms=45.0, launch_deg=18.0, n_loops=10, trigger_frame=3),
        starts_chronological=STARTS,
    )


def _computed(raw):
    prepared = prepare_shot_dump(raw)
    return prepared, {
        "power": {s: tracking.loop_power(prepared.mti(s)) for s in ("burst", "window")},
        "noise": {s: prepared.noise_power(s) for s in ("burst", "window")},
        "means": tracking.compute_window_means(
            prepared.cube, prepared.geometry, range_domain=prepared.range_domain
        ),
    }


def _supplied(raw, *, power=None, noise=None, means=None):
    return prepare_shot_dump(
        raw, supplied_power=power, supplied_noise=noise, supplied_window_means=means
    )


# -- window-scope static means -------------------------------------------------


def test_window_mti_from_supplied_means_is_identical(raw):
    prepared, products = _computed(raw)

    supplied = tracking.mti_filter(
        prepared.cube,
        scope="window",
        range_domain=True,
        geometry=prepared.geometry,
        window_means=products["means"],
    )

    np.testing.assert_array_equal(supplied, prepared.mti("window"))


def test_window_means_are_used_not_recomputed(raw):
    prepared, products = _computed(raw)
    shifted = products["means"] + (1.0 + 2.0j)

    supplied = _supplied(raw, means=shifted).mti("window")

    computed = prepared.mti("window")
    for frame in range(prepared.geometry.n_frames):
        count = prepared.geometry.frame_bin_count(frame)
        np.testing.assert_allclose(
            supplied[frame, ..., :count], computed[frame, ..., :count] - (1.0 + 2.0j)
        )


def test_window_means_need_per_frame_windows():
    cube = np.zeros((2, 4, 4, 8), dtype=complex)
    with pytest.raises(ValueError, match="per-frame range windows"):
        tracking.mti_filter(
            cube, scope="window", range_domain=True, window_means=np.zeros((2, 4, 128))
        )


def test_window_means_are_not_used_for_burst_scope(raw):
    prepared, products = _computed(raw)

    burst = _supplied(raw, means=products["means"] + 5.0).mti("burst")

    np.testing.assert_array_equal(burst, prepared.mti("burst"))


# -- power and noise -----------------------------------------------------------


def test_supplying_the_computed_products_changes_nothing(raw, cal):
    prepared, products = _computed(raw)
    supplied = _supplied(raw, **products)

    for scope in ("burst", "window"):
        assert supplied.noise_power(scope) == prepared.noise_power(scope)
        np.testing.assert_array_equal(supplied.loop_power(scope), products["power"][scope])
    full = process_dump(raw, cal, coherent_loops=1, two_ray=False, prepared=prepared)
    reduced = process_dump(raw, cal, coherent_loops=1, two_ray=False, prepared=supplied)
    assert reduced.track == full.track


def test_supplied_power_drives_the_tracker(raw, cal):
    _prepared, products = _computed(raw)
    silent = {scope: np.zeros_like(power) for scope, power in products["power"].items()}

    shot = process_dump(
        raw, cal, coherent_loops=1, two_ray=False, prepared=_supplied(raw, power=silent)
    )

    assert shot.track is None


def test_supplied_noise_is_returned(raw):
    supplied = _supplied(raw, noise={"burst": 1.5, "window": 2.5})

    assert supplied.noise_power("burst") == 1.5
    assert supplied.noise_power("window") == 2.5


def test_find_ball_accepts_a_power_map(raw):
    prepared, products = _computed(raw)

    from_mti = tracking.find_ball(prepared.mti(), prepared.geometry)
    from_power = tracking.find_ball(None, prepared.geometry, power=products["power"]["burst"])

    assert from_power == from_mti


def test_find_ball_needs_mti_or_power(raw):
    prepared = prepare_shot_dump(raw)
    with pytest.raises(ValueError, match="mti or power"):
        tracking.find_ball(None, prepared.geometry)


def test_recovery_candidates_use_supplied_power(raw, cal):
    prepared, products = _computed(raw)
    kwargs = dict(ball_speed_mph=45.0 * 2.23694, net_range_m=None)

    full = find_recovery_candidates(raw, cal, prepared=prepared, **kwargs)
    same = find_recovery_candidates(raw, cal, prepared=_supplied(raw, **products), **kwargs)
    silent = {scope: np.zeros_like(power) for scope, power in products["power"].items()}
    none = find_recovery_candidates(raw, cal, prepared=_supplied(raw, power=silent), **kwargs)

    assert full
    assert same == full
    assert none == []


@pytest.mark.parametrize(
    "kwargs, message",
    [
        ({"supplied_power": {"burst": np.zeros((1, 1))}}, "both MTI scopes"),
        ({"supplied_noise": {"window": 1.0}}, "both MTI scopes"),
        ({"supplied_noise": {"burst": 0.0, "window": 1.0}}, "positive"),
        ({"supplied_noise": {"burst": float("nan"), "window": 1.0}}, "positive"),
    ],
)
def test_malformed_supplied_products_are_rejected(raw, kwargs, message):
    with pytest.raises(ValueError, match=message):
        prepare_shot_dump(raw, **kwargs)


def test_supplied_power_must_match_the_capture_shape(raw):
    _prepared, products = _computed(raw)
    wrong = {scope: power[:-1] for scope, power in products["power"].items()}

    with pytest.raises(ValueError, match="shape"):
        prepare_shot_dump(raw, supplied_power=wrong)
