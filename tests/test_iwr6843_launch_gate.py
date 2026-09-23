"""User-facing gate for IWR6843 LCMF launch angles.

Field report (2026-09-23, 42 shots paired with a Garmin R10 via GSPro): the
estimator "accepted" launch angles that were off by 6-38 degrees, producing
+/-20 yd carry swings and 0 yd carries. Three accepted-result shapes were
wrong almost every time and are withheld from the displayed/carry launch
angle (the shot then falls back to the labelled estimate):

* single-channel results (the other LCMF channel pinned at its search-grid
  edge or disagreeing by >8 deg): 11 of 12 wrong by 6-34 deg;
* ``accepted_track_speed_warning`` (TI range walk disagrees with OPS ball
  speed and no OPS-compatible track exists): 2 of 2 wrong by 31-38 deg;
* non-positive launch (<= 0 deg): 10.9 deg (R10) read as -2.1 deg, and
  launch <= 0 makes the ballistic carry exactly 0 yd.

The measurement values below are copied from those logged shots.
"""

from datetime import datetime
from pathlib import Path
from types import SimpleNamespace

import pytest

from openflight import server as server_module
from openflight.clubs import ClubType
from openflight.launch_monitor import Shot


def _measurement(*, angle, status="accepted", single_channel=False, horizontal=None):
    return SimpleNamespace(
        accepted=True,
        status=status,
        angle_deg=angle,
        single_channel=single_channel,
        horizontal_deg=horizontal,
        horizontal_confidence=0.9 if horizontal is not None else None,
        horizontal_status="hlcmf_v1_accepted" if horizontal is not None else None,
        n_snapshots=40,
        n_frames=12,
        component_std_deg=0.0,
        to_dict=lambda: {"estimator": "lcmf_v1", "status": status, "launch_angle_deg": angle},
    )


def _run(monkeypatch, measurement):
    emitted = []
    capture = SimpleNamespace(
        trigger_timestamp=100.01,
        path=Path("/tmp/test.l3dump"),
        raw=b"raw",
        dump_duration_s=5.3,
        error=None,
        valid=True,
        sequence=1,
    )
    runtime = SimpleNamespace(
        process_shot=lambda **kwargs: SimpleNamespace(capture=capture, measurement=measurement)
    )
    monkeypatch.setattr(server_module, "iwr6843_runtime", runtime)
    monkeypatch.setattr(server_module, "get_session_logger", lambda: None)
    monkeypatch.setattr(
        server_module.socketio, "emit", lambda event, payload: emitted.append((event, payload))
    )
    shot = Shot(
        ball_speed_mph=110.0,
        club_speed_mph=76.0,
        timestamp=datetime.now(),
        impact_timestamp=100.0,
        club=ClubType.IRON_9,
    )
    server_module._process_iwr6843_angle(shot)
    return shot, emitted


def _iwr_status(emitted):
    updates = [p["iwr6843"] for e, p in emitted if e == "trigger_diagnostic_update"]
    assert updates, emitted
    return updates[-1]


@pytest.mark.parametrize(
    "measurement, reason",
    [
        # R10 9.3 deg; two8 pinned at -5.0, four4 43.0 -> single channel.
        (
            _measurement(
                angle=43.0, status="accepted_ops_guided_single_channel", single_channel=True
            ),
            "withheld_single_channel",
        ),
        # R10 2.9 deg; channels 8.4 / 19.0 disagree -> single channel, plain status.
        (
            _measurement(angle=19.0, status="accepted", single_channel=True),
            "withheld_single_channel",
        ),
        # R10 36.4 deg; OPS-incompatible track.
        (
            _measurement(angle=-1.1, status="accepted_track_speed_warning"),
            "withheld_track_speed_warning",
        ),
        # R10 41.0 deg; OPS-incompatible track reported at a plausible angle.
        (
            _measurement(angle=9.8, status="accepted_track_speed_warning"),
            "withheld_track_speed_warning",
        ),
        # R10 10.9 deg; both channels agreed on a negative angle.
        (_measurement(angle=-2.1, status="accepted"), "withheld_non_positive_launch"),
        (_measurement(angle=0.0, status="accepted_ops_guided"), "withheld_non_positive_launch"),
    ],
)
def test_unreliable_accepted_launch_is_withheld(monkeypatch, measurement, reason):
    shot, emitted = _run(monkeypatch, measurement)

    assert shot.launch_angle_vertical is None
    assert shot.launch_angle_vertical_source != "radar"
    assert _iwr_status(emitted) == {"state": "rejected", "reason": reason}


def test_withheld_launch_does_not_publish_its_horizontal(monkeypatch):
    """A wrong track makes the TX2 horizontal from the same track wrong too."""
    shot, _ = _run(
        monkeypatch,
        _measurement(
            angle=30.9,
            status="accepted_ops_guided_single_channel",
            single_channel=True,
            horizontal=21.4,
        ),
    )

    assert shot.launch_angle_horizontal is None
    assert shot.launch_angle_horizontal_source is None


def test_withheld_launch_falls_back_to_labelled_estimate(monkeypatch):
    monkeypatch.setattr(server_module, "camera_capture_config", {"enabled": False})
    shot, _ = _run(monkeypatch, _measurement(angle=-2.1, status="accepted"))

    server_module._ensure_user_facing_launch_angles(shot)

    assert shot.launch_angle_vertical_source == "estimated"
    assert shot.launch_angle_vertical > 0.0


@pytest.mark.parametrize(
    "measurement",
    [
        # Two-channel results that matched the R10 within 0.3-1.0 deg.
        _measurement(angle=10.5, status="accepted_ops_guided"),
        _measurement(angle=3.6, status="accepted_ops_guided"),
        _measurement(angle=7.3, status="accepted"),
        _measurement(angle=0.1, status="accepted"),
        _measurement(angle=10.5, status="accepted_low_confidence_recovery"),
    ],
)
def test_reliable_accepted_launch_is_still_applied(monkeypatch, measurement):
    shot, emitted = _run(monkeypatch, measurement)

    assert shot.launch_angle_vertical == pytest.approx(measurement.angle_deg)
    assert shot.launch_angle_vertical_source == "radar"
    assert _iwr_status(emitted)["state"] == "accepted"


def test_measurement_without_gate_fields_is_applied(monkeypatch):
    """Older/duck-typed results without status/single_channel keep working."""
    measurement = SimpleNamespace(
        accepted=True,
        angle_deg=17.4,
        n_snapshots=20,
        n_frames=6,
        component_std_deg=1.1,
        to_dict=lambda: {},
    )
    shot, _ = _run(monkeypatch, measurement)

    assert shot.launch_angle_vertical == pytest.approx(17.4)
    assert shot.launch_angle_vertical_source == "radar"
