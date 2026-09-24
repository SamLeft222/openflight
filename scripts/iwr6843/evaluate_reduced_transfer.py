#!/usr/bin/env python3
"""Replay saved IWR6843 captures through the reduced transfer (Phase 0).

plans/iwr6843-on-chip-reduction.md. For every capture in the given sessions,
measure it three ways and report how they compare and how many bytes the
radar would have sent:

* ``full``      -- the whole capture, as the server measures it today;
* ``exact``     -- overview (unquantised) + strips; must equal ``full``;
* ``reduced``   -- overview as packed on the wire (log16 power, float32
                   noise and means) + strips: what the radar would send.

Example (dumps copied off the Pi into one folder):

    uv run python scripts/iwr6843/evaluate_reduced_transfer.py \\
        ~/openflight_logs/session_*_range.jsonl --dump-dir ~/openflight_logs/iwr6843 \\
        --tee-m 1.829 --net-m 4.877 --tilt-deg 11.5 --radar-height-m 0.157 \\
        --ball-height-m 0.059 --cfg config/iwr6843_l3dump_dense_36f2ms_53bin_iq8.cfg \\
        --out /tmp/reduced.jsonl
"""

from __future__ import annotations

import argparse
import json
import statistics
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path

from openflight.iwr6843.dump import parse_capture_metadata
from openflight.iwr6843.monitor import tx_order_from_config
from openflight.iwr6843.reduced import CaptureReducedSource, ball_gate_for
from openflight.iwr6843.replay import ReplayInput, build_replay_calibration, inputs_from_session
from openflight.iwr6843.runtime import IWR6843Runtime

# Measured on the Pi: a 549,764-byte dense capture streamed in 5.34 s.
DEFAULT_RATE_BYTES_PER_S = 549_764 / 5.34


@dataclass(frozen=True)
class Settings:
    """Geometry and firmware configuration shared by every replay."""

    calibration_path: str
    cfg: str
    tee_m: float
    net_m: float | None
    tilt_deg: float | None
    radar_height_m: float | None
    ball_height_m: float


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("sessions", nargs="+", type=Path, help="Session JSONL files")
    parser.add_argument(
        "--dump-dir",
        type=Path,
        default=None,
        help="Look captures up by file name here (for dumps copied off the Pi)",
    )
    parser.add_argument("--tee-m", required=True, type=float)
    parser.add_argument("--net-m", type=float, default=None)
    parser.add_argument("--tilt-deg", type=float, default=None)
    parser.add_argument("--radar-height-m", type=float, default=None)
    parser.add_argument("--ball-height-m", type=float, default=0.040)
    parser.add_argument("--cal", default="config/iwr6843_calibration_reference.json")
    parser.add_argument("--cfg", default="config/iwr6843_l3dump_dense_36f2ms_53bin_iq8.cfg")
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE_BYTES_PER_S)
    parser.add_argument("--jobs", type=int, default=None)
    parser.add_argument("--out", type=Path, default=None, help="Per-capture JSONL")
    return parser


def _clubs_by_shot(session: Path) -> dict[int, str]:
    clubs = {}
    with session.open(encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            entry = json.loads(line)
            if entry.get("type") == "shot_detected" and entry.get("shot_number") is not None:
                clubs[entry["shot_number"]] = entry.get("club")
    return clubs


def collect_inputs(sessions: list[Path], dump_dir: Path | None) -> list[ReplayInput]:
    """Replay inputs with each shot's club and, optionally, relocated dumps."""
    inputs = []
    for session in sessions:
        clubs = _clubs_by_shot(session)
        for item in inputs_from_session(session):
            path = dump_dir / item.capture_path.name if dump_dir else item.capture_path
            if not path.is_file():
                continue
            inputs.append(
                ReplayInput(
                    source=item.source,
                    shot_number=item.shot_number,
                    capture_path=path,
                    ball_speed_mph=item.ball_speed_mph,
                    club=clubs.get(item.shot_number),
                )
            )
    return inputs


def _runtime(settings: Settings) -> IWR6843Runtime:
    return IWR6843Runtime(
        capture_monitor=None,
        calibration=build_replay_calibration(
            settings.calibration_path,
            tee_range_m=settings.tee_m,
            tilt_deg=settings.tilt_deg,
            radar_height_m=settings.radar_height_m,
            ball_height_m=settings.ball_height_m,
        ),
        net_range_m=settings.net_m,
        tx_order=tx_order_from_config(settings.cfg),
        tdm_sign_policy="positive",
    )


def evaluate_capture(item: ReplayInput, settings: Settings) -> dict:
    """Measure one capture fully, exactly-reduced, and reduced; count bytes."""
    runtime = _runtime(settings)
    raw = item.capture_path.read_bytes()
    gate = ball_gate_for(parse_capture_metadata(raw), settings.net_m)
    kwargs = dict(ball_speed_mph=item.ball_speed_mph, club=item.club)
    full = runtime.measure_capture(raw, **kwargs)
    exact = runtime.measure_reduced(CaptureReducedSource(raw, gate=gate, quantise=False), **kwargs)
    source = CaptureReducedSource(raw, gate=gate, quantise=True)
    reduced = runtime.measure_reduced(source, **kwargs)
    return {
        "capture": item.capture_path.name,
        "shot_number": item.shot_number,
        "club": item.club,
        "ball_speed_mph": item.ball_speed_mph,
        "full_status": full.status,
        "full_launch_deg": full.angle_deg,
        "exact_match": exact.to_dict() == full.to_dict(),
        "reduced_status": reduced.status,
        "reduced_launch_deg": reduced.angle_deg,
        "capture_bytes": len(raw),
        "overview_bytes": source.overview_nbytes,
        "strip_bytes": source.strip_nbytes,
        "reduced_bytes": source.total_nbytes,
    }


def summarise(rows: list[dict], rate: float) -> dict:
    """Headline numbers for the plan's Phase 0 acceptance criteria."""

    def seconds(values):
        return round(statistics.median(values) / rate, 2) if values else None

    both = [
        row
        for row in rows
        if row["full_launch_deg"] is not None and row["reduced_launch_deg"] is not None
    ]
    shifts = sorted(abs(row["reduced_launch_deg"] - row["full_launch_deg"]) for row in both)
    by_requests: dict[int, list[int]] = {}
    for row in rows:
        by_requests.setdefault(len(row["strip_bytes"]), []).append(row["reduced_bytes"])
    return {
        "captures": len(rows),
        "exact_matches": sum(row["exact_match"] for row in rows),
        "status_changes": sum(row["full_status"] != row["reduced_status"] for row in rows),
        "launch_shift_deg": {
            "median": statistics.median(shifts) if shifts else None,
            "p90": shifts[int(0.9 * (len(shifts) - 1))] if shifts else None,
            "max": shifts[-1] if shifts else None,
        },
        "capture_seconds_median": seconds([row["capture_bytes"] for row in rows]),
        "reduced_seconds_median": seconds([row["reduced_bytes"] for row in rows]),
        "reduced_bytes_by_strip_requests": {
            str(count): {"captures": len(values), "median_bytes": statistics.median(values)}
            for count, values in sorted(by_requests.items())
        },
    }


def main(argv: list[str] | None = None) -> dict:
    args = _parser().parse_args(argv)
    settings = Settings(
        calibration_path=args.cal,
        cfg=args.cfg,
        tee_m=args.tee_m,
        net_m=args.net_m,
        tilt_deg=args.tilt_deg,
        radar_height_m=args.radar_height_m,
        ball_height_m=args.ball_height_m,
    )
    inputs = collect_inputs(args.sessions, args.dump_dir)
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        rows = list(pool.map(evaluate_capture, inputs, [settings] * len(inputs)))
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with args.out.open("w", encoding="utf-8") as handle:
            for row in rows:
                handle.write(json.dumps(row, sort_keys=True) + "\n")
    summary = summarise(rows, args.rate)
    print(json.dumps(summary, indent=2))
    return summary


if __name__ == "__main__":
    main()
