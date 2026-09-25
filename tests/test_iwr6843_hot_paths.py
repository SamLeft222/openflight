"""The batched LCMF grid and recovery search give bit-identical results.

Both are hot paths on the Pi (the OPS-guided recovery took ~6-7 s per hard
shot). The optimised forms must reproduce the original one-angle-at-a-time
and one-pair-at-a-time arithmetic exactly, so these tests compare with
``==`` rather than a tolerance: any drift could move a grid argmin or a
RANSAC bucket and change a shot.
"""

from __future__ import annotations

from unittest.mock import patch

import numpy as np
import pytest

from openflight.iwr6843 import lcmf
from openflight.iwr6843.calibration import Calibration
from openflight.iwr6843.multipath import leave_one_channel_out_error
from openflight.iwr6843.recovery import RecoveryCandidate, _candidate_tracks_for_scope, _fit_line
from openflight.iwr6843.tracking import BallTrack, Geometry

GEOMETRY = {
    "speed_ms": 45.0,
    "tee_x_m": 1.8,
    "ball_height_m": 0.05,
    "radar_height_m": 0.16,
    "tilt_rad": np.radians(11.5),
    "tx_order": "normal",
    "tdm_tau_s": 1.0e-4,
}
GRID_DEG = np.arange(-5.0, 45.0 + 0.25, 0.5)


def _snapshots(seed: int, n: int = 60):
    rng = np.random.default_rng(seed)
    range_m = np.sort(rng.uniform(1.9, 4.5, n))
    vectors = rng.normal(size=(n, 8)) + 1j * rng.normal(size=(n, 8))
    frames = np.sort(rng.integers(0, 12, n))
    return range_m, vectors, frames


@pytest.mark.parametrize("model", ["two8", "four4", "four4_path_tdm"])
def test_batched_dictionary_equals_one_angle_at_a_time(model):
    range_m, _vectors, _frames = _snapshots(1)
    batched = lcmf._spatial_dictionary(
        model, np.radians(GRID_DEG)[:, None], range_m, GEOMETRY, GEOMETRY["tdm_tau_s"]
    )
    single = np.stack(
        [
            lcmf._spatial_dictionary(
                model, np.radians(angle), range_m, GEOMETRY, GEOMETRY["tdm_tau_s"]
            )
            for angle in GRID_DEG
        ]
    )

    assert batched.shape == single.shape
    assert np.array_equal(batched, single)


@pytest.mark.parametrize("ceiling", [1.0, 1e3])
def test_batched_frame_objective_equals_one_row_at_a_time(ceiling):
    rng = np.random.default_rng(2)
    errors = rng.lognormal(sigma=2.0, size=(40, 70))
    errors[3, 5] = 0.0  # clipped up to the floor
    errors[7, 9] = 1e9  # clipped down to the ceiling
    frames = np.sort(rng.integers(0, 9, 70))

    batched = lcmf._frame_objectives(errors, frames, ceiling)

    assert batched.tolist() == [lcmf._frame_objective(row, frames, ceiling) for row in errors]


@pytest.mark.parametrize("seed", [3, 4, 5])
def test_channel_estimates_equal_the_per_angle_search(seed):
    range_m, vectors, frames = _snapshots(seed)
    cache = {"frame": frames, "vec": vectors, "r": range_m}
    indices = np.arange(len(range_m))

    estimates, evidence = lcmf._channel_estimates(cache, indices, GEOMETRY, GRID_DEG)

    for model in lcmf.CHANNEL_MODELS:
        objective = np.asarray(
            [
                lcmf._frame_objective(
                    leave_one_channel_out_error(
                        vectors,
                        lcmf._spatial_dictionary(
                            model, np.radians(angle), range_m, GEOMETRY, GEOMETRY["tdm_tau_s"]
                        ),
                    ),
                    frames,
                    1e3,
                )
                for angle in GRID_DEG
            ]
        )
        name = f"channel_{model}_deg"
        assert estimates[name] == lcmf._refine_grid(GRID_DEG, objective)
        assert evidence[name] == lcmf.grid_curvature(objective)


def _reference_candidates(times, bins, geometry, calibration, *, scope, ball_speed_mph):
    """The original pair-at-a-time recovery search, kept verbatim as the oracle."""
    resolution_m = geometry.range_res_m
    candidates: dict[tuple[int, int], RecoveryCandidate] = {}
    apparent_tee_m = calibration.tee_range_m + calibration.range_bias_m
    min_slope = 20.0 / resolution_m
    max_slope = 90.0 / resolution_m
    for first in range(times.size - 1):
        later_times = times[first + 1 :]
        delta_times = times[first] - later_times
        valid_time = np.abs(delta_times) >= 0.003
        slopes = np.divide(
            bins[first] - bins[first + 1 :],
            delta_times,
            out=np.zeros_like(delta_times),
            where=valid_time,
        )
        valid_seconds = np.flatnonzero(valid_time & (slopes >= min_slope) & (slopes <= max_slope))
        for second_offset in valid_seconds:
            slope = float(slopes[second_offset])
            intercept = bins[first] - slope * times[first]
            inliers = np.abs(bins - (slope * times + intercept)) < 0.8
            count = int(inliers.sum())
            if count < 10:
                continue
            inlier_times = times[inliers]
            inlier_bins = bins[inliers]
            fit = _fit_line(inlier_times, inlier_bins)
            if fit is None:
                continue
            fitted_slope, fitted_intercept = fit
            speed_ms = float(fitted_slope * resolution_m)
            if not 20.0 <= speed_ms <= 90.0:
                continue
            residuals = inlier_bins - (fitted_slope * inlier_times + fitted_intercept)
            rms = float(np.sqrt(np.mean(residuals**2)))
            first_time = float(inlier_times.min())
            last_time = float(inlier_times.max())
            track = BallTrack(
                speed_ms=speed_ms,
                slope_bins=float(fitted_slope),
                intercept_bins=float(fitted_intercept),
                rms_bins=rms,
                n_inliers=count,
                t_first=first_time,
                t_last=last_time,
                low_confidence=rms >= 0.45 or last_time - first_time < 0.012,
            )
            impact_s = (apparent_tee_m / resolution_m - fitted_intercept) / fitted_slope
            if not 0.0 <= impact_s <= geometry.capture_duration_s:
                continue
            candidate = RecoveryCandidate(
                track=track,
                scope=scope,
                impact_s=float(impact_s),
                speed_ratio=track.speed_mph / ball_speed_mph,
            )
            key = (round(speed_ms / 0.6), round(impact_s / 0.0008))
            incumbent = candidates.get(key)
            quality = (count, -rms, last_time - first_time)
            if incumbent is None:
                candidates[key] = candidate
                continue
            old = incumbent.track
            old_quality = (old.n_inliers, -old.rms_bins, old.t_last - old.t_first)
            if quality > old_quality:
                candidates[key] = candidate
    return list(candidates.values())


def _detections(seed: int, geometry: Geometry):
    """Two receding walks (one ball, one ghost) plus clutter, several per loop."""
    rng = np.random.default_rng(seed)
    loops = np.arange(0, geometry.n_frames * geometry.n_loops, 3)
    times = np.asarray(
        [geometry.loop_time(i // geometry.n_loops, i % geometry.n_loops) for i in loops]
    )
    walks = []
    for start_bin, speed_ms in ((30.0, 40.0), (34.0, 52.0)):
        walk = start_bin + speed_ms / geometry.range_res_m * times
        walks.append(np.rint(walk + rng.normal(scale=0.3, size=walk.size)))
    clutter = rng.integers(20, 90, size=times.size)
    all_loops = np.concatenate([loops, loops, loops])
    all_bins = np.concatenate([*walks, clutter]).astype(int)
    keep = all_bins < geometry.n_samples + 40
    order = np.argsort(all_loops[keep], kind="stable")
    return all_loops[keep][order], all_bins[keep][order]


@pytest.mark.parametrize("seed", [6, 7, 8])
@pytest.mark.parametrize("scope", ["burst", "window"])
def test_candidate_search_equals_the_pair_at_a_time_search(seed, scope):
    geometry = Geometry(24, 36, 3, 4, 53, 0.003, 0, range_fft_size=128)
    loop_indices, bins = _detections(seed, geometry)
    times = np.asarray(
        [geometry.loop_time(i // geometry.n_loops, i % geometry.n_loops) for i in loop_indices]
    )
    calibration = Calibration.identity()
    calibration.tee_range_m = 1.5

    with (
        patch("openflight.iwr6843.recovery.tracking.loop_power", return_value=np.zeros(1)),
        patch(
            "openflight.iwr6843.recovery.tracking._detections",
            return_value=(loop_indices, bins),
        ),
    ):
        candidates = _candidate_tracks_for_scope(
            np.zeros((1, 1)),
            geometry,
            calibration,
            scope=scope,
            ball_speed_mph=95.0,
            max_range_m=None,
            mti=np.zeros((1, 1)),
        )
    expected = _reference_candidates(
        times, bins, geometry, calibration, scope=scope, ball_speed_mph=95.0
    )

    assert len(expected) > 3  # the fixture must exercise bucketing and replacement
    assert candidates == expected
