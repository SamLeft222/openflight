"""Weak-echo IWR readings and the player's own launch history.

Field report (2026-09-23, 70 shots paired with a Garmin R10): when the ball
echo is near the noise floor the estimator keeps only a few frames (<= 8 of
~24), and those readings were typically 3.7 deg off with +/-13 deg misses.
Withholding them only helps if the fallback is better than they are -- the
club-table estimate guessed 22-28 deg for ~11 deg 9-irons. The player's own
median launch per club (from good-echo radar shots) is: it removed the
+/-13 deg misses, and cut withheld-shot launch error from ~15 to ~6 deg.
"""

import pytest
from test_iwr6843_launch_gate import _iwr_status, _measurement, _run

from openflight import server as server_module
from openflight.clubs import ClubType
from openflight.launch_history import MIN_SHOTS_FOR_TYPICAL, LaunchHistory

PROFILE = "profile-sam"
CLUB = ClubType.IRON_9
TYPICAL_9_IRON = 11.0


@pytest.fixture
def history():
    return LaunchHistory()


def _seed(history, value=TYPICAL_9_IRON, *, count=MIN_SHOTS_FOR_TYPICAL, club=CLUB):
    for _ in range(count):
        history.record(PROFILE, club.value, value)


# -- which readings are withheld -------------------------------------------


@pytest.mark.parametrize("n_frames", [5, 6, 8])
def test_weak_echo_is_withheld_when_a_personal_launch_exists(monkeypatch, history, n_frames):
    _seed(history)
    # 6-iron, R10 7.1 deg, read as 19.5 deg from 6 frames.
    shot, emitted = _run(
        monkeypatch, _measurement(angle=19.5, n_frames=n_frames), profile_id=PROFILE
    )

    assert shot.launch_angle_vertical is None
    assert _iwr_status(emitted) == {"state": "rejected", "reason": "withheld_weak_echo"}


def test_weak_echo_is_kept_without_a_personal_launch(monkeypatch, history):
    """The club table is worse than a weak reading, so never swap to it."""
    _seed(history, count=MIN_SHOTS_FOR_TYPICAL - 1)
    # 4-iron, R10 5.5 deg, read as 7.1 deg; the table would have said 22.5.
    shot, emitted = _run(monkeypatch, _measurement(angle=7.1, n_frames=5), profile_id=PROFILE)

    assert shot.launch_angle_vertical == pytest.approx(7.1)
    assert shot.launch_angle_vertical_source == "radar"
    assert _iwr_status(emitted)["reason"] == "accepted"


def test_weak_echo_uses_the_history_of_the_shot_club(monkeypatch, history):
    _seed(history, club=ClubType.DRIVER)
    shot, _ = _run(monkeypatch, _measurement(angle=7.1, n_frames=5), profile_id=PROFILE)

    assert shot.launch_angle_vertical == pytest.approx(7.1)


def test_weak_echo_uses_the_history_of_the_shot_profile(monkeypatch, history):
    _seed(history)
    shot, _ = _run(monkeypatch, _measurement(angle=7.1, n_frames=5), profile_id="someone-else")

    assert shot.launch_angle_vertical == pytest.approx(7.1)


def test_good_echo_is_applied_even_with_a_personal_launch(monkeypatch, history):
    _seed(history)
    shot, emitted = _run(monkeypatch, _measurement(angle=14.1, n_frames=9), profile_id=PROFILE)

    assert shot.launch_angle_vertical == pytest.approx(14.1)
    assert _iwr_status(emitted)["reason"] == "accepted"


def test_existing_gate_reasons_take_precedence(monkeypatch, history):
    _seed(history)
    shot, emitted = _run(
        monkeypatch,
        _measurement(angle=43.0, single_channel=True, n_frames=5),
        profile_id=PROFILE,
    )

    assert shot.launch_angle_vertical is None
    assert _iwr_status(emitted)["reason"] == "withheld_single_channel"


@pytest.mark.parametrize(
    "measurement, available, reason",
    [
        (_measurement(angle=10.0, n_frames=8), True, "withheld_weak_echo"),
        (_measurement(angle=10.0, n_frames=8), False, None),
        (_measurement(angle=10.0, n_frames=9), True, None),
        (_measurement(angle=-1.0, n_frames=8), True, "withheld_non_positive_launch"),
    ],
)
def test_withheld_reason_is_a_pure_function(measurement, available, reason):
    assert (
        server_module.iwr6843_launch_withheld_reason(
            measurement, personal_launch_available=available
        )
        == reason
    )


def test_measurement_without_frame_count_is_not_weak():
    """Older/duck-typed results without n_frames keep working."""
    measurement = _measurement(angle=10.0)
    del measurement.n_frames

    assert (
        server_module.iwr6843_launch_withheld_reason(measurement, personal_launch_available=True)
        is None
    )


# -- what gets recorded ---------------------------------------------------


def test_good_echo_launch_is_recorded(monkeypatch, history):
    _run(monkeypatch, _measurement(angle=10.4, n_frames=17), profile_id=PROFILE)

    assert LaunchHistory().count(PROFILE, CLUB.value) == 1


@pytest.mark.parametrize(
    "measurement",
    [
        _measurement(angle=7.1, n_frames=5),  # weak echo, applied (no history yet)
        _measurement(angle=43.0, single_channel=True),  # withheld
        _measurement(angle=-2.1),  # withheld
    ],
)
def test_weak_or_withheld_launch_is_not_recorded(monkeypatch, history, measurement):
    _run(monkeypatch, measurement, profile_id=PROFILE)

    assert LaunchHistory().count(PROFILE, CLUB.value) == 0


def test_shot_without_profile_is_not_recorded(monkeypatch, history):
    _run(monkeypatch, _measurement(angle=10.4, n_frames=17), profile_id="")

    assert LaunchHistory().count("", CLUB.value) == 0
    assert not history.path.exists()


# -- the fallback ------------------------------------------------------------


def _fallback(monkeypatch, shot):
    monkeypatch.setattr(server_module, "camera_capture_config", {"enabled": False})
    server_module._ensure_user_facing_launch_angles(shot)
    return shot


def test_withheld_shot_falls_back_to_the_personal_launch(monkeypatch, history):
    _seed(history, 10.0, count=3)
    _seed(history, 12.0, count=3)
    shot, _ = _run(monkeypatch, _measurement(angle=19.5, n_frames=6), profile_id=PROFILE)

    _fallback(monkeypatch, shot)

    assert shot.launch_angle_vertical == pytest.approx(11.0)
    assert shot.launch_angle_vertical_source == "estimated"
    assert shot.launch_angle_vertical_confidence == pytest.approx(
        server_module.PERSONAL_LAUNCH_CONFIDENCE
    )


def test_fallback_uses_the_club_table_without_history(monkeypatch, history):
    shot, _ = _run(monkeypatch, _measurement(angle=43.0, single_channel=True), profile_id=PROFILE)
    table_angle, table_confidence = server_module.estimate_launch_angle(
        shot.club, shot.ball_speed_mph, club_speed_mph=shot.club_speed_mph
    )

    _fallback(monkeypatch, shot)

    assert shot.launch_angle_vertical == pytest.approx(table_angle)
    assert shot.launch_angle_vertical_confidence == pytest.approx(table_confidence)
    assert shot.launch_angle_vertical_source == "estimated"


def test_fallback_does_not_overwrite_a_measured_launch(monkeypatch, history):
    _seed(history)
    shot, _ = _run(monkeypatch, _measurement(angle=14.1, n_frames=17), profile_id=PROFILE)

    _fallback(monkeypatch, shot)

    assert shot.launch_angle_vertical == pytest.approx(14.1)
    assert shot.launch_angle_vertical_source == "radar"
