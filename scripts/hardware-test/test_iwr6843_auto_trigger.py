#!/usr/bin/env python3
"""Check the IWR6843's own shot detector on the radar (auto-trigger firmware).

Stop OpenFlight first -- it owns the radar's serial port:

    sudo systemctl stop openflight
    ~/openflight/.venv/bin/python scripts/hardware-test/test_iwr6843_auto_trigger.py

1. Configures the capture, the overview gate and the detector (autoTrigCfg).
2. Quiet period: stand still; any detection here is a false trigger.
3. Swings: make ``--swings`` full swings (with or without a ball). Each
   detection's notice is printed, its held capture fetched as an overview
   and checked, and the radar released. ``--save DIR`` instead streams each
   detected ring whole (l3dump) and saves it, for tuning the detector.
4. An unrequested detection must be released by the firmware's 10 s timeout.

Exit status is non-zero if any check fails.
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from pathlib import Path

from openflight.iwr6843 import reduced
from openflight.iwr6843.autotrigger import AutoTriggerConfig
from openflight.iwr6843.driver import IWR6843Radar
from openflight.iwr6843.reduced import RANGE_FFT_SIZE
from openflight.iwr6843.tracking import RANGE_SPAN_M

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


def _config(args) -> AutoTriggerConfig:
    tee_bin = round(args.tee_m / (RANGE_SPAN_M / RANGE_FFT_SIZE))
    low, high = (int(part) for part in args.end_bins.split(":"))
    return AutoTriggerConfig(
        end_lo=tee_bin + low,
        end_hi=tee_bin + high,
        delay_frames=args.delay_frames,
        z_min_x10=round(args.z_min * 10),
    )


def quiet_period(checks: Checks, radar: IWR6843Radar, seconds: float) -> None:
    print(f"Quiet period: stand still for {seconds:.0f} s...")
    deadline = time.monotonic() + seconds
    false_triggers = 0
    while time.monotonic() < deadline:
        notice = radar.read_auto_trigger_notice(timeout_s=0.2)
        if notice is not None:
            false_triggers += 1
            print(f"    false trigger: {notice}")
            radar.release()
    checks.check(false_triggers == 0, "no detections while standing still", f"{false_triggers}")


def swings(checks: Checks, radar: IWR6843Radar, count: int, save: Path | None, timeout_s: float):
    print(f"Swings: make {count} full swings, a few seconds apart...")
    seen = 0
    ages = []
    for index in range(count):
        notice = radar.read_auto_trigger_notice(timeout_s=timeout_s)
        if notice is None:
            print(f"    swing {index + 1}: no detection within {timeout_s:.0f} s")
            continue
        seen += 1
        ages.append(notice.age_ms)
        print(
            f"    swing {index + 1}: detected (#{notice.seq}) club at bin {notice.end_bin}, "
            f"{notice.slope_q / 4:.2f} bins/frame, notice {notice.age_ms} ms after the freeze "
            f"request, {notice.n_frames} frames"
        )
        if save is not None:
            raw = radar.read_dump()  # streams the held ring whole, then resumes
            path = save / f"auto_{time.strftime('%Y%m%d_%H%M%S')}_{notice.seq:03d}.l3dump"
            path.write_bytes(raw)
            checks.check(len(raw) > 0, f"swing {index + 1}: held ring saved to {path.name}")
        else:
            overview = reduced.parse_overview(radar.read_overview())
            checks.check(
                overview.metadata["n_frames"] == notice.n_frames,
                f"swing {index + 1}: l3overview serves the detected capture",
                f"{overview.metadata['n_frames']} frames vs notice {notice.n_frames}",
            )
            radar.release()
    checks.check(seen == count, f"every swing detected ({seen}/{count})")
    if ages:
        print(f"    notice delay after the freeze request: {min(ages)}-{max(ages)} ms")


def unrequested_hold_times_out(checks: Checks, radar: IWR6843Radar, timeout_s: float) -> None:
    print("Timeout: make one more swing and wait (the Pi will not fetch it)...")
    notice = radar.read_auto_trigger_notice(timeout_s=timeout_s)
    if not checks.check(notice is not None, "swing detected"):
        return
    before = _stat(radar.stats(), "timeouts") or 0
    time.sleep(HOLD_TIMEOUT_S + 1.0)
    stats = radar.stats()
    checks.check(
        _stat(stats, "timeouts") == before + 1 and _stat(stats, "held") == 0,
        "the firmware released the unrequested hold",
        stats.strip(),
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", default=None, help="Radar CLI port (default: autodetect)")
    parser.add_argument("--cfg", default=DEFAULT_CFG)
    parser.add_argument("--net-m", type=float, default=4.877)
    parser.add_argument("--tee-m", type=float, default=1.829)
    parser.add_argument("--end-bins", default="-1:1", help="LO:HI bins from the tee")
    parser.add_argument("--delay-frames", type=int, default=3)
    parser.add_argument("--z-min", type=float, default=8.0)
    parser.add_argument("--quiet-s", type=float, default=30.0)
    parser.add_argument("--swings", type=int, default=5)
    parser.add_argument("--swing-timeout", type=float, default=60.0)
    parser.add_argument("--save", type=Path, default=None, help="Save each detected ring here")
    parser.add_argument("--skip-timeout", action="store_true")
    args = parser.parse_args()

    checks = Checks()
    config = _config(args)
    if args.save is not None:
        args.save.mkdir(parents=True, exist_ok=True)
    radar = IWR6843Radar(args.port)
    try:
        print(f"Configuring {radar.port} with {args.cfg}")
        radar.send_config(args.cfg)
        radar.configure_overview_gate(reduced.default_ball_gate(args.net_m))
        print(f"Detector: {config.command()}")
        radar.configure_auto_trigger(config)
        time.sleep(0.2)
        checks.check(_stat(radar.stats(), "enabled") == 1, "firmware enabled the detector")

        quiet_period(checks, radar, args.quiet_s)
        swings(checks, radar, args.swings, args.save, args.swing_timeout)
        if not args.skip_timeout:
            unrequested_hold_times_out(checks, radar, args.swing_timeout)
        stats = radar.stats()
        print(f"\nFirmware counters: {stats.strip()}")
    finally:
        try:
            radar.configure_auto_trigger(None)
            radar.release()
        finally:
            radar.close()
    print(
        f"\n{'ALL CHECKS PASSED' if not checks.failures else f'{checks.failures} CHECK(S) FAILED'}"
    )
    return 1 if checks.failures else 0


if __name__ == "__main__":
    sys.exit(main())
