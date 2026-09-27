"""The firmware's shot detector (auto_trigger.c) against autotrigger.py.

auto_trigger.c is portable C: the same file links into the radar image and,
here, into a host harness. Frame by frame it must make exactly the reference
model's decisions, down to the bits of the track score.
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from openflight.iwr6843.autotrigger import AutoTrigger, AutoTriggerConfig, capture_frames
from openflight.iwr6843.dump import (
    SAMPLE_RANGE_FFT_IQ8_VARIABLE_TIMED,
    SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED,
    TEMP_REPORT_KEYS,
    pack_dump,
)

FIRMWARE = Path(__file__).parents[1] / "firmware" / "iwr6843"
FIELD_CAPTURES = Path.home() / "openflight_logs" / "iwr6843"
CC = shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")
TEE = 39
N_TX, N_RX, LOOPS, BINS = 3, 4, 12, 53

pytestmark = pytest.mark.skipif(CC is None, reason="no C compiler for the host harness")

CONFIGS = [
    AutoTriggerConfig(end_lo=TEE - 1, end_hi=TEE + 1, delay_frames=3),
    AutoTriggerConfig(end_lo=TEE - 1, end_hi=TEE + 1, delay_frames=0),
    AutoTriggerConfig(end_lo=TEE - 4, end_hi=TEE + 2, n_frames=3, slope_min_q=3),
    AutoTriggerConfig(end_lo=TEE, end_hi=TEE, n_frames=8, z_min_x10=35, z_cap_x10=35),
]


@pytest.fixture(name="harness", scope="module")
def _harness(tmp_path_factory):
    exe = tmp_path_factory.mktemp("auto_trigger_host") / "auto_trigger_host"
    subprocess.run(
        [
            CC,
            "-std=c89",
            "-pedantic",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-Wno-long-long",
            # A fused multiply-add rounds differently from the radar's VFPv3.
            "-ffp-contract=off",
            "-O2",
            "-o",
            str(exe),
            str(FIRMWARE / "test" / "auto_trigger_host.c"),
            str(FIRMWARE / "auto_trigger.c"),
        ],
        check=True,
    )
    return exe


def _club_capture(*, iq8: bool, seed: int, arrive: int = 12, slope: float = 1.5) -> bytes:
    """A timed capture: noise, a static golfer, and a club reaching the tee."""
    rng = np.random.default_rng(seed)
    n_frames = 20
    starts = (20,) * 14 + (32,) * 6
    cube = (
        rng.normal(size=(n_frames, LOOPS * N_TX, N_RX, BINS))
        + 1j * rng.normal(size=(n_frames, LOOPS * N_TX, N_RX, BINS))
    ) * 40.0
    for frame, start in enumerate(starts):
        golfer = 47 - start
        cube[frame, :, :, golfer] += 900.0 * np.exp(1j * rng.uniform(0, 2 * np.pi))
        position = TEE - slope * (arrive - frame)
        if frame <= arrive and 0 <= round(position) - start < BINS:
            phases = np.exp(1j * rng.uniform(0, 2 * np.pi, size=(LOOPS * N_TX, 1)))
            cube[frame, :, :, round(position) - start] += 2500.0 * phases
    fmt = SAMPLE_RANGE_FFT_IQ8_VARIABLE_TIMED if iq8 else SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED
    return pack_dump(
        cube,
        n_tx=N_TX,
        version=7,
        frame_period_us=2000,
        sample_fmt=fmt,
        range_bin_starts=starts,
        range_bin_counts=(BINS,) * n_frames,
        frame_time_offsets_us=tuple(2000 * f for f in range(n_frames)),
        temperature_report={key: 40 for key in TEMP_REPORT_KEYS},
    )


def _reference(raw: bytes, config: AutoTriggerConfig, n_tx: int = N_TX) -> list[str]:
    trigger = AutoTrigger(config)
    lines = []
    for frame, (start, codes) in enumerate(capture_frames(raw)):
        fired = trigger.push_codes(start, codes, n_tx=n_tx)
        d = trigger.detection
        lines.append(
            f"{frame} {int(fired)} {int(d is not None)} {d.frame if d else 0} "
            f"{d.end_bin if d else 0} {d.slope_q if d else 0} "
            f"{format(d.score if d else 0.0, '.17g')}"
        )
    return lines


def _firmware(harness, tmp_path, raw: bytes, config: AutoTriggerConfig) -> list[str]:
    dump = tmp_path / "capture.l3dump"
    dump.write_bytes(raw)
    args = config.command().split()[2:]  # drop "autoTrigCfg 1"
    result = subprocess.run(
        [str(harness), str(dump), *args], capture_output=True, text=True, check=True
    )
    return result.stdout.splitlines()


@pytest.mark.parametrize("iq8", [True, False], ids=["iq8", "iq16"])
@pytest.mark.parametrize("config", CONFIGS, ids=lambda c: c.command().replace(" ", "_"))
def test_firmware_matches_reference_on_club_captures(harness, tmp_path, iq8, config):
    for seed in range(3):
        raw = _club_capture(iq8=iq8, seed=seed)

        assert _firmware(harness, tmp_path, raw, config) == _reference(raw, config)


def test_the_club_capture_fires_near_the_tee_arrival(harness, tmp_path):
    config = CONFIGS[1]  # no delay
    lines = _firmware(harness, tmp_path, _club_capture(iq8=True, seed=0), config)
    fired = [int(line.split()[0]) for line in lines if line.split()[1] == "1"]

    assert len(fired) == 1
    assert 11 <= fired[0] <= 12


def test_rejects_a_bad_configuration(harness, tmp_path):
    dump = tmp_path / "capture.l3dump"
    dump.write_bytes(_club_capture(iq8=True, seed=0))

    result = subprocess.run(
        [str(harness), str(dump), "40", "39", "5", "80", "500", "2", "12", "3"],
        capture_output=True,
        check=False,
    )

    assert result.returncode == 2


@pytest.mark.skipif(not FIELD_CAPTURES.is_dir(), reason="no saved field captures here")
def test_firmware_matches_reference_on_field_captures(harness, tmp_path):
    dumps = sorted(FIELD_CAPTURES.glob("*.l3dump"))[:25]
    for path in dumps:
        raw = path.read_bytes()

        assert _firmware(harness, tmp_path, raw, CONFIGS[0]) == _reference(raw, CONFIGS[0]), (
            path.name
        )
