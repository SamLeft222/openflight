#!/usr/bin/env python3
"""Validate the reduced-transfer firmware on a real IWR6843 (plan Phase 2).

plans/iwr6843-on-chip-reduction.md. Stop OpenFlight first -- it owns the
radar's serial port:

    sudo systemctl stop openflight
    uv run python scripts/hardware-test/test_iwr6843_reduced_transfer.py

No ball is needed: every check compares bytes from one freeze. Each cycle
freezes the ring and reads it every way -- l3overview, several l3strip
requests, then l3dump (which streams the held ring and resumes) -- and checks
that the overview and strips equal the Python reference built from that same
dump. It also checks l3release, the error paths, and the 10 s timeout that
resumes an abandoned hold. Exit status is non-zero if any check fails.
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from pathlib import Path

from openflight.iwr6843 import reduced
from openflight.iwr6843.driver import IWR6843Radar
from openflight.iwr6843.dump import parse_capture_metadata

DEFAULT_CFG = "config/iwr6843_l3dump_dense_36f2ms_53bin_iq8.cfg"
HOLD_TIMEOUT_S = 10.0


class Checks:
    """Collects PASS/FAIL lines; any failure fails the run."""

    def __init__(self):
        self.failures = 0

    def check(self, ok: bool, label: str, detail: str = "") -> bool:
        print(
            f"  [{'PASS' if ok else 'FAIL'}] {label}{f' -- {detail}' if detail and not ok else ''}"
        )
        self.failures += 0 if ok else 1
        return ok


def _stat(stats: str, name: str) -> int | None:
    match = re.search(rf"\b{name}=(\d+)", stats)
    return int(match.group(1)) if match else None


def _requests(meta: dict) -> dict[str, reduced.StripRequest]:
    """A track-like diagonal, whole windows for a few frames, and nothing."""
    starts, counts = meta["range_bin_starts"], meta["range_bin_counts"]
    n = meta["n_frames"]
    diagonal = []
    for frame, (start, count) in enumerate(zip(starts, counts)):
        if not n // 3 <= frame < n - 2:
            diagonal.append(None)
            continue
        offset = min(count - 4, (frame - n // 3) * 2)
        diagonal.append((start + offset, 4))
    whole = tuple(
        (s, c) if f in (0, n // 2, n - 1) else None for f, (s, c) in enumerate(zip(starts, counts))
    )
    return {"diagonal": tuple(diagonal), "whole windows": whole, "none": (None,) * n}


def _compare_overview(checks: Checks, packed: bytes, full: bytes, gate) -> None:
    expected = reduced.pack_overview(reduced.build_overview(full, gate=gate))
    if checks.check(packed == expected, "overview equals the reference built from the same dump"):
        return
    got, want = reduced.parse_overview(packed), reduced.parse_overview(expected)
    print(f"      prefix equal: {got.capture_prefix == want.capture_prefix}")
    print(f"      noise got {got.noise} want {want.noise}")
    for scope in ("burst", "window"):
        codes = reduced.encode_log_power(got.power[scope]) != reduced.encode_log_power(
            want.power[scope]
        )
        print(f"      {scope} power codes differing: {int(codes.sum())}")


def run_cycle(
    checks: Checks, radar: IWR6843Radar, gate, out: Path | None, index: int, timeout_s: float
) -> dict:
    timings = {}
    start = time.monotonic()
    packed = radar.read_overview(timeout_s=timeout_s)
    timings["overview_s"] = time.monotonic() - start
    overview = reduced.parse_overview(packed)
    meta = overview.metadata
    strips = {}
    for name, request in _requests(meta).items():
        start = time.monotonic()
        strips[name] = (request, radar.read_strips(request))
        timings[f"strips_{name}_s"] = time.monotonic() - start
    start = time.monotonic()
    full = radar.read_dump()
    timings["full_dump_s"] = time.monotonic() - start
    stats = radar.stats()
    timings["overview_firmware_ms"] = _stat(stats, "overview_ms")
    timings["overview_compute_ms"] = _stat(stats, "prepare_ms")

    _compare_overview(checks, packed, full, gate)
    for name, (request, data) in strips.items():
        checks.check(
            data == reduced.serve_strips(full, request),
            f"strips '{name}' equal the reference from the same dump",
        )
    checks.check(
        _stat(stats, "active") == 1 and _stat(stats, "held") == 0,
        "l3dump of the held ring resumed capture",
        stats.strip(),
    )
    timings["overview_bytes"] = len(packed)
    timings["full_bytes"] = len(full)
    if out is not None:
        (out / f"cycle{index}_overview.bin").write_bytes(packed)
        (out / f"cycle{index}_full.l3dump").write_bytes(full)
        for name, (_request, data) in strips.items():
            (out / f"cycle{index}_strips_{name.replace(' ', '_')}.l3dump").write_bytes(data)
    return timings


def check_release_and_errors(checks: Checks, radar: IWR6843Radar, n_frames: int) -> None:
    before = _stat(radar.stats(), "releases") or 0
    try:
        radar.read_strips((None,) * n_frames)
        checks.check(False, "l3strip without a hold is refused")
    except RuntimeError as error:
        checks.check(
            "no held capture" in str(error), "l3strip without a hold is refused", str(error)
        )
    radar.read_overview()
    try:
        radar.read_overview()
        checks.check(False, "a second l3overview while held is refused")
    except RuntimeError as error:
        checks.check(
            "already held" in str(error), "a second l3overview while held is refused", str(error)
        )
    radar.release()
    stats = radar.stats()
    checks.check(
        _stat(stats, "held") == 0 and _stat(stats, "active") == 1,
        "l3release resumes capture",
        stats.strip(),
    )
    checks.check(_stat(stats, "releases") == before + 1, "l3release is counted", stats.strip())
    radar.release()
    checks.check(
        _stat(radar.stats(), "releases") == before + 1, "l3release without a hold changes nothing"
    )


def check_timeout(checks: Checks, radar: IWR6843Radar) -> None:
    before = _stat(radar.stats(), "timeouts") or 0
    radar.read_overview()
    print(f"  waiting {HOLD_TIMEOUT_S + 1:.0f} s for the abandoned-hold timeout...")
    time.sleep(HOLD_TIMEOUT_S + 1.0)
    stats = radar.stats()
    checks.check(
        _stat(stats, "timeouts") == before + 1 and _stat(stats, "held") == 0,
        "an abandoned hold times out",
        stats.strip(),
    )
    checks.check(_stat(stats, "active") == 1, "capture resumed after the timeout", stats.strip())
    checks.check(len(radar.read_dump()) > 0, "l3dump works after the timeout")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", default=None, help="Radar CLI port (default: autodetect)")
    parser.add_argument("--cfg", default=DEFAULT_CFG)
    parser.add_argument("--net-m", type=float, default=4.877)
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--out", type=Path, default=None, help="Save every transfer here")
    parser.add_argument("--skip-timeout", action="store_true")
    parser.add_argument(
        "--overview-timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for an overview (generous, so a slow build reports its timing)",
    )
    args = parser.parse_args()

    checks = Checks()
    if args.out is not None:
        args.out.mkdir(parents=True, exist_ok=True)
    radar = IWR6843Radar(args.port)
    try:
        print(f"Configuring {radar.port} with {args.cfg}")
        radar.send_config(args.cfg)
        print("Reading one ordinary l3dump for the capture layout")
        reference = radar.read_dump()
        meta = parse_capture_metadata(reference)
        gate = reduced.ball_gate_for(meta, args.net_m)
        radar.configure_overview_gate(gate)
        print(f"Overview gate bins {gate[0]}-{gate[1]} (net {args.net_m} m)")

        all_timings = []
        for index in range(args.cycles):
            print(f"Cycle {index + 1}/{args.cycles}")
            time.sleep(0.2)
            all_timings.append(
                run_cycle(checks, radar, gate, args.out, index, args.overview_timeout)
            )
        print("Release and error paths")
        check_release_and_errors(checks, radar, meta["n_frames"])
        if not args.skip_timeout:
            print("Abandoned-hold timeout")
            check_timeout(checks, radar)

        print("\nTimings (median over cycles):")
        for key in all_timings[0]:
            values = sorted(t[key] for t in all_timings if t[key] is not None)
            if values:
                print(f"  {key:28} {values[len(values) // 2]:.3f}")
    finally:
        try:
            radar.release()
        finally:
            radar.close()
    print(
        f"\n{'ALL CHECKS PASSED' if not checks.failures else f'{checks.failures} CHECK(S) FAILED'}"
    )
    return 1 if checks.failures else 0


if __name__ == "__main__":
    sys.exit(main())
