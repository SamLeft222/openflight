"""Per-profile launch history: the player's own measured launch per club.

Field report (2026-09-23, 70 shots paired with a Garmin R10): the club-table
fallback assumes a tour-average strike and guessed 22-28 deg for 9-irons this
player launches at ~11 deg. The median of the player's own good-echo radar
launches for that club cut withheld-shot launch error from ~15 to ~6 deg.
"""

import json
import math

import pytest

from openflight.launch_history import (
    DEFAULT_LAUNCH_HISTORY_PATH,
    LAUNCH_HISTORY_PATH_ENV,
    MAX_SHOTS_PER_CLUB,
    MIN_SHOTS_FOR_TYPICAL,
    LaunchHistory,
    resolve_launch_history_path,
)


@pytest.fixture
def history(tmp_path):
    return LaunchHistory(tmp_path / "launch_history.json")


def _record(history, values, *, profile="p1", club="driver"):
    for value in values:
        history.record(profile, club, value)


def test_no_typical_launch_until_enough_shots(history):
    _record(history, [10.0] * (MIN_SHOTS_FOR_TYPICAL - 1))

    assert history.typical_launch("p1", "driver") is None
    assert history.count("p1", "driver") == MIN_SHOTS_FOR_TYPICAL - 1


def test_typical_launch_is_the_median(history):
    # One skied drive must not drag the typical launch up the way a mean would.
    _record(history, [9.0, 10.0, 11.0, 12.0, 34.0])

    assert history.typical_launch("p1", "driver") == pytest.approx(11.0)


def test_clubs_and_profiles_are_kept_apart(history):
    _record(history, [11.0] * 5, profile="p1", club="9-iron")
    _record(history, [15.0] * 5, profile="p1", club="6-iron")
    _record(history, [20.0] * 5, profile="p2", club="9-iron")

    assert history.typical_launch("p1", "9-iron") == pytest.approx(11.0)
    assert history.typical_launch("p1", "6-iron") == pytest.approx(15.0)
    assert history.typical_launch("p2", "9-iron") == pytest.approx(20.0)
    assert history.typical_launch("p2", "6-iron") is None


def test_keeps_only_the_most_recent_shots(history):
    # A setup or swing change must age out rather than bias the median forever.
    _record(history, [30.0] * MAX_SHOTS_PER_CLUB + [10.0] * MAX_SHOTS_PER_CLUB)

    assert history.count("p1", "driver") == MAX_SHOTS_PER_CLUB
    assert history.typical_launch("p1", "driver") == pytest.approx(10.0)


def test_history_survives_a_restart(tmp_path):
    path = tmp_path / "launch_history.json"
    _record(LaunchHistory(path), [8.0, 9.0, 10.0, 11.0, 12.0])

    assert LaunchHistory(path).typical_launch("p1", "driver") == pytest.approx(10.0)


@pytest.mark.parametrize(
    "profile, club, value",
    [
        ("", "driver", 10.0),
        ("p1", "", 10.0),
        ("p1", "driver", math.nan),
        ("p1", "driver", math.inf),
        ("p1", "driver", 0.0),
        ("p1", "driver", -3.0),
        ("p1", "driver", 90.0),
        ("p1", "driver", None),
    ],
)
def test_unusable_values_are_not_recorded(history, profile, club, value):
    history.record(profile, club, value)

    assert history.count(profile or "p1", club or "driver") == 0
    assert not history.path.exists()


def test_corrupt_file_starts_empty_and_is_repaired(tmp_path):
    path = tmp_path / "launch_history.json"
    path.write_text("{not json", encoding="utf-8")

    history = LaunchHistory(path)
    assert history.typical_launch("p1", "driver") is None

    _record(history, [10.0] * 5)
    assert LaunchHistory(path).typical_launch("p1", "driver") == pytest.approx(10.0)


def test_malformed_entries_are_ignored(tmp_path):
    path = tmp_path / "launch_history.json"
    path.write_text(
        json.dumps(
            {
                "profiles": {
                    "p1": {
                        "driver": [10.0, "high", None, 11.0, 12.0, 13.0, 14.0, -5.0, 1e9],
                        "9-iron": "not a list",
                    },
                    "p2": ["not", "a", "dict"],
                }
            }
        ),
        encoding="utf-8",
    )

    history = LaunchHistory(path)

    assert history.count("p1", "driver") == 5
    assert history.typical_launch("p1", "driver") == pytest.approx(12.0)
    assert history.count("p1", "9-iron") == 0
    assert history.count("p2", "driver") == 0


def test_save_failure_never_raises(tmp_path):
    blocker = tmp_path / "blocker"
    blocker.write_text("a file where a directory must go", encoding="utf-8")
    history = LaunchHistory(blocker / "launch_history.json")

    _record(history, [10.0] * 5)  # must not raise

    # The in-memory history still serves this run.
    assert history.typical_launch("p1", "driver") == pytest.approx(10.0)


def test_path_comes_from_argument_then_env_then_default(tmp_path, monkeypatch):
    monkeypatch.setenv(LAUNCH_HISTORY_PATH_ENV, str(tmp_path / "from-env.json"))
    assert resolve_launch_history_path(tmp_path / "arg.json") == tmp_path / "arg.json"
    assert resolve_launch_history_path() == tmp_path / "from-env.json"

    monkeypatch.setenv(LAUNCH_HISTORY_PATH_ENV, "   ")
    assert resolve_launch_history_path() == DEFAULT_LAUNCH_HISTORY_PATH


def test_tests_never_touch_the_real_history():
    """conftest points the store at a temp file for every test."""
    assert resolve_launch_history_path() != DEFAULT_LAUNCH_HISTORY_PATH
