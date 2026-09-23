"""Per-profile launch history: the player's own measured launch per club.

The club-table launch estimate assumes a tour-average strike. Paired Garmin
R10 sessions (2026-09-23) showed it guessing 22-28 deg for 9-irons the player
launched at ~11 deg. Every IWR6843 launch the server applies with a strong
ball echo is recorded here, and the median of the recent ones for a club is
the player's typical launch -- a far better stand-in when a shot's own
reading is withheld or too weak to trust.

History is keyed by profile id (a profile may be a person or a place, see
``profiles``), then by club, and holds the most recent
``MAX_SHOTS_PER_CLUB`` launches so a setup or swing change ages out. It is
kept out of the profile roster because the roster is broadcast to the UI on
every change.
"""

from __future__ import annotations

import json
import logging
import math
import os
import statistics
import threading
import uuid
from pathlib import Path
from typing import Any, Dict, List, Union

logger = logging.getLogger(__name__)

DEFAULT_LAUNCH_HISTORY_PATH = Path.home() / ".config" / "openflight" / "launch_history.json"
LAUNCH_HISTORY_PATH_ENV = "OPENFLIGHT_LAUNCH_HISTORY_PATH"
MAX_SHOTS_PER_CLUB = 30
MIN_SHOTS_FOR_TYPICAL = 5
# Recorded launches must be physical ball flights. Non-positive launches are
# already withheld upstream; this bound only rejects corrupt values.
MAX_LAUNCH_DEG = 60.0


def resolve_launch_history_path(path: Union[str, Path, None] = None) -> Path:
    """Constructor argument, then ``OPENFLIGHT_LAUNCH_HISTORY_PATH``, then the user default."""
    if path is not None and str(path).strip():
        return Path(path).expanduser()
    env_path = (os.environ.get(LAUNCH_HISTORY_PATH_ENV) or "").strip()
    if env_path:
        return Path(env_path).expanduser()
    return DEFAULT_LAUNCH_HISTORY_PATH


def _usable_launch(value: Any) -> float | None:
    """The value as a launch angle, or None when it is not a plausible one."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    launch = float(value)
    if not math.isfinite(launch) or not 0.0 < launch < MAX_LAUNCH_DEG:
        return None
    return launch


class LaunchHistory:
    """Load, extend, and atomically persist per-profile, per-club launches.

    Reads and writes never raise into a caller: an unreadable file starts an
    empty history, and a failed save is logged while the in-memory history
    keeps serving this process.
    """

    def __init__(self, path: Union[str, Path, None] = None):
        self.path = resolve_launch_history_path(path)
        self._lock = threading.Lock()
        self._launches: Dict[str, Dict[str, List[float]]] = {}
        self._load()

    def record(self, profile_id: str, club: str, launch_deg: Any) -> None:
        """Append one measured launch. Unusable input changes nothing."""
        launch = _usable_launch(launch_deg)
        if not profile_id or not club or launch is None:
            return
        with self._lock:
            launches = self._launches.setdefault(profile_id, {}).setdefault(club, [])
            launches.append(launch)
            del launches[:-MAX_SHOTS_PER_CLUB]
        self._save()

    def count(self, profile_id: str, club: str) -> int:
        """How many launches are held for this profile and club."""
        return len(self._launches.get(profile_id, {}).get(club, []))

    def typical_launch(self, profile_id: str, club: str) -> float | None:
        """Median recent launch, or None below ``MIN_SHOTS_FOR_TYPICAL`` shots."""
        launches = self._launches.get(profile_id, {}).get(club, [])
        if len(launches) < MIN_SHOTS_FOR_TYPICAL:
            return None
        return float(statistics.median(launches))

    def _save(self) -> None:
        with self._lock:
            payload = {"profiles": {pid: dict(clubs) for pid, clubs in self._launches.items()}}
            text = json.dumps(payload, indent=2)
        temp_path = self.path.with_name(f"{self.path.name}.{uuid.uuid4().hex}.tmp")
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with open(temp_path, "w", encoding="utf-8") as handle:
                handle.write(text)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temp_path, self.path)
        except OSError as error:
            logger.error("[launch_history] could not save %s: %s", self.path, error)
            try:
                temp_path.unlink()
            except OSError:
                pass

    def _load(self) -> None:
        try:
            with open(self.path, "r", encoding="utf-8") as handle:
                raw = json.load(handle)
        except FileNotFoundError:
            return
        except (OSError, json.JSONDecodeError) as error:
            logger.warning("[launch_history] could not read %s: %s", self.path, error)
            return

        profiles = raw.get("profiles") if isinstance(raw, dict) else None
        if not isinstance(profiles, dict):
            return
        for profile_id, clubs in profiles.items():
            if not isinstance(clubs, dict):
                continue
            for club, values in clubs.items():
                if not isinstance(values, list):
                    continue
                launches = [v for v in map(_usable_launch, values) if v is not None]
                if launches:
                    self._launches.setdefault(str(profile_id), {})[str(club)] = launches[
                        -MAX_SHOTS_PER_CLUB:
                    ]
