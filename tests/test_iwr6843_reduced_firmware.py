"""The firmware's reduced-transfer core against the Python reference.

firmware/iwr6843/reduced_overview.c is portable C: the same file links into
the radar image and, here, into a host harness. Its overview and strips must
equal reduced.py's byte-for-byte (plans/iwr6843-on-chip-reduction.md,
Phase 2). On 2026-09-24 they did on all 80 saved field captures; these tests
keep that true on synthetic captures of every supported shape.
"""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
from test_iwr6843_pipeline import synth_shot
from test_iwr6843_reduced import GATE, N_BINS, N_FRAMES, PERIOD_US, STARTS, _timed_iq8

from openflight.iwr6843 import reduced
from openflight.iwr6843.dump import (
    SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED,
    TEMP_REPORT_KEYS,
    pack_dump,
    parse_capture_metadata,
    parse_dump,
)

FIRMWARE = Path(__file__).parents[1] / "firmware" / "iwr6843"
CC = shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")

pytestmark = pytest.mark.skipif(CC is None, reason="no C compiler for the host harness")


@pytest.fixture(name="harness", scope="module")
def _harness(tmp_path_factory):
    exe = tmp_path_factory.mktemp("reduced_host") / "reduced_host"
    subprocess.run(
        [
            CC,
            "-std=c89",
            "-pedantic",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-Wno-long-long",
            "-O2",
            "-o",
            str(exe),
            str(FIRMWARE / "test" / "reduced_host.c"),
            str(FIRMWARE / "reduced_overview.c"),
        ],
        check=True,
    )
    return exe


def _run(harness, tmp_path, raw: bytes, *args: str) -> subprocess.CompletedProcess:
    dump = tmp_path / "capture.l3dump"
    dump.write_bytes(raw)
    return subprocess.run(
        [str(harness), args[0], str(dump), *args[1:]], capture_output=True, check=False
    )


def _synthetic(n_tx=3, iq8=True, seed=0) -> bytes:
    adc = synth_shot(
        speed_ms=45.0,
        launch_deg=18.0,
        n_frames=N_FRAMES,
        n_loops=12,
        n_tx=n_tx,
        frame_period_us=PERIOD_US,
        trigger_frame=0,
        seed=seed,
    )
    if iq8:
        return _timed_iq8(adc)
    meta, cube = parse_dump(adc)
    rfft = np.fft.fft(cube, axis=-1)
    windows = np.stack([rfft[f, ..., s : s + N_BINS] for f, s in enumerate(STARTS)])
    return pack_dump(
        windows / 64.0,  # keep IQ16 samples inside int16
        n_tx=meta["n_tx"],
        version=7,
        frame_period_us=PERIOD_US,
        sample_fmt=SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED,
        range_bin_starts=STARTS,
        range_bin_counts=(N_BINS,) * N_FRAMES,
        frame_time_offsets_us=tuple(PERIOD_US * f for f in range(N_FRAMES)),
        temperature_report={key: 40 for key in TEMP_REPORT_KEYS},
    )


CAPTURES = {
    "iq8_3tx": dict(n_tx=3, iq8=True),
    "iq16_3tx": dict(n_tx=3, iq8=False),
    "iq8_2tx": dict(n_tx=2, iq8=True),
    "iq8_3tx_seed5": dict(n_tx=3, iq8=True, seed=5),
}


# -- overview -----------------------------------------------------------------------


@pytest.mark.parametrize("shape", CAPTURES)
@pytest.mark.parametrize("gate", [GATE, (20, 116), (60, 61)])
def test_overview_equals_the_python_reference(harness, tmp_path, shape, gate):
    raw = _synthetic(**CAPTURES[shape])

    result = _run(harness, tmp_path, raw, "overview", str(gate[0]), str(gate[1]))

    assert result.returncode == 0, result.stderr
    assert result.stdout == reduced.pack_overview(reduced.build_overview(raw, gate=gate))


@pytest.mark.parametrize("gate", [(98, 48), (48, 48), (0, 257)])
def test_invalid_gate_is_refused(harness, tmp_path, gate):
    result = _run(harness, tmp_path, _synthetic(), "overview", str(gate[0]), str(gate[1]))

    assert result.returncode != 0
    assert b"error -2" in result.stderr


def test_capture_without_noise_is_refused_like_the_reference(harness, tmp_path):
    raw = _timed_iq8(
        synth_shot(
            n_frames=N_FRAMES,
            n_loops=12,
            n_tx=3,
            frame_period_us=PERIOD_US,
            trigger_frame=0,
            amp=0.0,
            noise=0.0,
        )
    )

    result = _run(harness, tmp_path, raw, "overview", *map(str, GATE))

    assert result.returncode != 0
    assert b"error -4" in result.stderr
    with pytest.raises(ValueError, match="noise"):
        reduced.pack_overview(reduced.build_overview(raw, gate=GATE))


# -- strips ---------------------------------------------------------------------------


def _requests(raw):
    meta = parse_capture_metadata(raw)
    starts, counts = meta["range_bin_starts"], meta["range_bin_counts"]
    n = meta["n_frames"]
    return {
        "none": (None,) * n,
        "whole windows": tuple(zip(starts, counts)),
        "first bin": tuple((s, 1) for s in starts),
        "last bin": tuple((s + c - 1, 1) for s, c in zip(starts, counts)),
        "mixed": tuple(None if f % 3 == 0 else (s + 10, 7) for f, s in enumerate(starts)),
    }


@pytest.mark.parametrize("shape", CAPTURES)
@pytest.mark.parametrize("which", ["none", "whole windows", "first bin", "last bin", "mixed"])
def test_strips_equal_the_python_reference(harness, tmp_path, shape, which):
    raw = _synthetic(**CAPTURES[shape])
    request = _requests(raw)[which]

    result = _run(harness, tmp_path, raw, "strips", reduced.encode_strip_request(request))

    assert result.returncode == 0, result.stderr
    assert result.stdout == reduced.serve_strips(raw, request)


def test_uppercase_hex_is_accepted(harness, tmp_path):
    raw = _synthetic()
    request = _requests(raw)["mixed"]

    result = _run(harness, tmp_path, raw, "strips", reduced.encode_strip_request(request).upper())

    assert result.stdout == reduced.serve_strips(raw, request)


@pytest.mark.parametrize(
    "hex_request, error",
    [
        ("00" * (2 * N_FRAMES - 1), b"error -5"),  # one byte short
        ("0000" * N_FRAMES + "00", b"error -5"),  # too long
        ("zz00" + "0000" * (N_FRAMES - 1), b"error -5"),  # not hex
        ("1305" + "0000" * (N_FRAMES - 1), b"error -3"),  # starts before the window
        (f"{STARTS[0] + 50:02x}05" + "0000" * (N_FRAMES - 1), b"error -3"),  # past it
    ],
)
def test_bad_strip_requests_are_refused(harness, tmp_path, hex_request, error):
    result = _run(harness, tmp_path, _synthetic(), "strips", hex_request)

    assert result.returncode != 0
    assert error in result.stderr


# -- generated table ---------------------------------------------------------------------


def test_log_table_header_matches_its_generator():
    result = subprocess.run(
        [sys.executable, str(FIRMWARE / "gen_reduced_log_table.py"), "--check"], check=False
    )

    assert result.returncode == 0, "run firmware/iwr6843/gen_reduced_log_table.py"
