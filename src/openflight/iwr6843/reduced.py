"""Reduced-transfer capture: a power overview plus complex strips.

Reference implementation of plans/iwr6843-on-chip-reduction.md. Instead of
streaming the whole 537 KB ring, the radar sends

1. an **overview**: the capture's own header and per-frame tables, then the
   vertical-pair per-loop MTI power maps (both scopes, ball-gate bins only,
   log-encoded int16), both noise powers, and the window-scope static means;
2. **strips**: an ordinary timed dump holding only the requested range bins
   of each frame, as stored (IQ8 keeps its scales).

The Pi runs its unchanged tracker and OPS-guided search on the overview,
requests strips along the chosen track(s), and ``assemble_capture`` rebuilds a
dump in the full capture's layout (zeros outside the strips) so the rest of
the pipeline runs unchanged.

``build_overview`` and ``serve_strips`` model the firmware: the firmware must
reproduce their bytes exactly from the same ring.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass

import numpy as np

from openflight.iwr6843 import tracking
from openflight.iwr6843.dump import (
    SAMPLE_RANGE_FFT_IQ8_VARIABLE_TIMED,
    SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED,
    capture_metadata_prefix,
    pack_dump,
    parse_capture_metadata,
    parse_dump,
    project_tx_pair,
)
from openflight.iwr6843.lcmf import TRACK_TIME_MARGIN_S
from openflight.iwr6843.shot import TX2_LOOP_PERIOD_S, geometry_from_header, prepare_shot_dump
from openflight.iwr6843.tracking import MTI_SCOPES, BallTrack, Geometry

OVERVIEW_MAGIC = b"ILOV"
OVERVIEW_VERSION = 1
# magic, version, gate lo bin, gate hi bin, loops per frame,
# burst noise, window noise, means lo bin, means hi bin
OVERVIEW_HEADER = struct.Struct("<4sHHHHffHH")
LOG_POWER_STEPS_PER_OCTAVE = 256
ZERO_POWER_CODE = -32768
VERTICAL_TX_PAIR = (0, 2)
DEFAULT_STRIP_WIDTH_BINS = 1

# One (absolute start bin, bin count) per frame in dump order, or None for a
# frame the Pi does not need.
StripRequest = tuple[tuple[int, int] | None, ...]

_TIMED_FORMATS = (SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED, SAMPLE_RANGE_FFT_IQ8_VARIABLE_TIMED)


@dataclass(frozen=True)
class Overview:
    """Everything the Pi needs to choose ball tracks without the complex data."""

    capture_prefix: bytes
    gate: tuple[int, int]
    power: dict[str, np.ndarray]
    noise: dict[str, float]
    window_means: np.ndarray

    @property
    def metadata(self) -> dict:
        """Header and per-frame tables of the full capture."""
        return parse_capture_metadata(self.capture_prefix)


# -- shared helpers ------------------------------------------------------------


def _require_timed(meta: dict) -> None:
    if meta["sample_fmt"] not in _TIMED_FORMATS:
        raise ValueError("reduced transfer needs a timed variable-width range capture")
    if meta["n_tx"] not in (2, 3):
        raise ValueError(f"reduced transfer supports 2 or 3 TX, got {meta['n_tx']}")


def _loops(meta: dict) -> int:
    return meta["chirps_per_frame"] // meta["n_tx"]


def capture_geometry(meta: dict) -> Geometry:
    """The loop timing LCMF and the horizontal proxy use for this capture."""
    loop_period_s = TX2_LOOP_PERIOD_S if meta["n_tx"] == 3 else tracking.LOOP_PRI_S
    return geometry_from_header(meta, loop_period_s=loop_period_s)


def _gate_columns(meta: dict, gate: tuple[int, int]) -> list[tuple[int, int]]:
    """Per frame, the local columns [a, b) of the gate inside the frame window."""
    lo, hi = gate
    columns = []
    for start, count in zip(meta["range_bin_starts"], meta["range_bin_counts"]):
        a = max(lo, start) - start
        b = min(hi, start + count) - start
        columns.append((a, max(a, b)))
    return columns


def _means_span(meta: dict) -> tuple[int, int]:
    """Absolute bins [lo, hi) held by any frame window."""
    starts, counts = meta["range_bin_starts"], meta["range_bin_counts"]
    return min(starts), max(start + count for start, count in zip(starts, counts))


def encode_log_power(power: np.ndarray) -> np.ndarray:
    """Power -> int16 codes of ``round(log2(p) * 256)``; exact zero is its own code."""
    values = np.asarray(power, dtype=float)
    if not np.all(np.isfinite(values)) or np.any(values < 0.0):
        raise ValueError("power must be finite and non-negative")
    codes = np.full(values.shape, ZERO_POWER_CODE, dtype=np.int16)
    positive = values > 0.0
    codes[positive] = np.clip(
        np.round(np.log2(values[positive]) * LOG_POWER_STEPS_PER_OCTAVE),
        ZERO_POWER_CODE + 1,
        np.iinfo(np.int16).max,
    ).astype(np.int16)
    return codes


def decode_log_power(codes: np.ndarray) -> np.ndarray:
    """Inverse of ``encode_log_power`` (to 1/256-octave precision)."""
    codes = np.asarray(codes, dtype=np.int16)
    power = np.exp2(codes.astype(float) / LOG_POWER_STEPS_PER_OCTAVE)
    power[codes == ZERO_POWER_CODE] = 0.0
    return power


# -- the radar side (firmware reference) -------------------------------------


def build_overview(raw: bytes, *, gate: tuple[int, int]) -> Overview:
    """What the radar computes from its full ring, before quantisation."""
    meta = parse_capture_metadata(raw)
    _require_timed(meta)
    lo, hi = gate
    if not 0 <= lo < hi <= 0xFFFF:
        raise ValueError(f"invalid gate {gate}")
    vertical_raw = project_tx_pair(raw, VERTICAL_TX_PAIR) if meta["n_tx"] == 3 else raw
    prepared = prepare_shot_dump(vertical_raw)
    loops = _loops(meta)
    power = {}
    for scope in MTI_SCOPES:
        full = prepared.loop_power(scope)
        kept = np.zeros_like(full)
        for frame, (a, b) in enumerate(_gate_columns(meta, gate)):
            rows = slice(frame * loops, (frame + 1) * loops)
            kept[rows, a:b] = full[rows, a:b]
        power[scope] = kept
    return Overview(
        capture_prefix=capture_metadata_prefix(raw),
        gate=(lo, hi),
        power=power,
        noise={scope: prepared.noise_power(scope) for scope in MTI_SCOPES},
        window_means=tracking.compute_window_means(
            prepared.cube, prepared.geometry, range_domain=prepared.range_domain
        ),
    )


def serve_strips(raw: bytes, request: StripRequest) -> bytes:
    """The requested bins of each frame as a timed dump, samples as stored.

    The wire format needs at least one bin per frame, so a frame the Pi did
    not request carries the first bin of its window.
    """
    meta, cube = parse_dump(raw)
    _require_timed(meta)
    if len(request) != meta["n_frames"]:
        raise ValueError(f"strip request has {len(request)} frames, capture has {meta['n_frames']}")
    strips = np.zeros_like(cube)
    starts: list[int] = []
    counts: list[int] = []
    for frame, window in enumerate(request):
        frame_start = meta["range_bin_starts"][frame]
        frame_count = meta["range_bin_counts"][frame]
        start, count = window if window is not None else (frame_start, 1)
        if count < 1 or start < frame_start or start + count > frame_start + frame_count:
            raise ValueError(
                f"frame {frame} strip {(start, count)} is outside its window "
                f"{(frame_start, frame_count)}"
            )
        local = start - frame_start
        strips[frame, ..., :count] = cube[frame, ..., local : local + count]
        starts.append(start)
        counts.append(count)
    return _pack_like(meta, strips, starts, counts)


def _pack_like(meta: dict, cube: np.ndarray, starts, counts) -> bytes:
    return pack_dump(
        cube,
        n_tx=meta["n_tx"],
        trigger_frame=meta["trigger_frame"],
        version=meta["version"],
        frame_period_us=meta["frame_period_us"],
        sample_fmt=meta["sample_fmt"],
        range_bin_starts=starts,
        range_bin_counts=counts,
        frame_time_offsets_us=meta["frame_time_offsets_us"],
        temperature_report=meta.get("temperature_report"),
        iq8_scales=meta.get("iq8_scales"),
    )


# -- the wire format -------------------------------------------------------------


def pack_overview(overview: Overview) -> bytes:
    """Overview -> bytes: the exact layout the firmware must emit."""
    meta = overview.metadata
    _require_timed(meta)
    noise = [float(overview.noise[scope]) for scope in MTI_SCOPES]
    if not all(math.isfinite(value) and value > 0.0 for value in noise):
        raise ValueError("noise power must be finite and positive")
    loops = _loops(meta)
    means_lo, means_hi = _means_span(meta)
    head = OVERVIEW_HEADER.pack(
        OVERVIEW_MAGIC,
        OVERVIEW_VERSION,
        overview.gate[0],
        overview.gate[1],
        loops,
        *noise,
        means_lo,
        means_hi,
    )
    body = []
    for scope in MTI_SCOPES:
        for frame, (a, b) in enumerate(_gate_columns(meta, overview.gate)):
            block = overview.power[scope][frame * loops : (frame + 1) * loops, a:b]
            body.append(encode_log_power(block).astype("<i2").tobytes())
    means = overview.window_means[:, :, means_lo:means_hi].astype("<c8")
    return overview.capture_prefix + head + b"".join(body) + means.tobytes()


def parse_overview(raw: bytes) -> Overview:
    """Bytes -> Overview, with the power maps back in capture-local bins."""
    meta = parse_capture_metadata(raw)
    _require_timed(meta)
    offset = meta["header_nbytes"] + meta.get("frame_metadata_nbytes", 0)
    if len(raw) < offset + OVERVIEW_HEADER.size:
        raise ValueError("short overview header")
    magic, version, lo, hi, loops, noise_burst, noise_window, means_lo, means_hi = (
        OVERVIEW_HEADER.unpack_from(raw, offset)
    )
    if magic != OVERVIEW_MAGIC:
        raise ValueError(f"bad overview magic {magic!r}")
    if version != OVERVIEW_VERSION:
        raise ValueError(f"unsupported overview version {version}")
    if loops != _loops(meta):
        raise ValueError(f"overview has {loops} loops per frame, capture has {_loops(meta)}")
    if not lo < hi:
        raise ValueError(f"invalid overview gate {(lo, hi)}")
    if (means_lo, means_hi) != _means_span(meta):
        raise ValueError("overview window means do not cover the capture's range windows")

    columns = _gate_columns(meta, (lo, hi))
    power_values = sum(b - a for a, b in columns) * loops
    n_rx = meta["n_rx"]
    means_values = 2 * n_rx * (means_hi - means_lo)
    body = offset + OVERVIEW_HEADER.size
    expected = body + len(MTI_SCOPES) * power_values * 2 + means_values * 8
    if len(raw) != expected:
        raise ValueError(f"overview is {len(raw)} bytes, expected {expected}")

    n_rows = meta["n_frames"] * loops
    power = {}
    for scope in MTI_SCOPES:
        values = np.zeros((n_rows, meta["n_samples"]))
        for frame, (a, b) in enumerate(columns):
            n = loops * (b - a)
            codes = np.frombuffer(raw, dtype="<i2", offset=body, count=n).reshape(loops, b - a)
            values[frame * loops : (frame + 1) * loops, a:b] = decode_log_power(codes)
            body += 2 * n
        power[scope] = values
    fft_size = tracking.window_fft_size(geometry_from_header(meta))
    window_means = np.zeros((2, n_rx, fft_size), dtype=complex)
    window_means[:, :, means_lo:means_hi] = np.frombuffer(
        raw, dtype="<c8", offset=body, count=means_values
    ).reshape(2, n_rx, means_hi - means_lo)
    return Overview(
        capture_prefix=raw[:offset],
        gate=(lo, hi),
        power=power,
        noise={"burst": float(noise_burst), "window": float(noise_window)},
        window_means=window_means,
    )


# -- the Pi side -----------------------------------------------------------------


def corridor_request(
    tracks: list[BallTrack],
    meta: dict,
    *,
    width_bins: int = DEFAULT_STRIP_WIDTH_BINS,
) -> StripRequest:
    """Strips covering every snapshot LCMF and the horizontal proxy read.

    Both read ``round(track.bin_at(t))`` for every loop within
    ``TRACK_TIME_MARGIN_S`` of the track; the union over ``tracks`` plus
    ``width_bins`` either side is requested, clipped to each frame's window.
    """
    if width_bins < 1:
        raise ValueError("strips need at least one bin either side of the track")
    geometry = capture_geometry(meta)
    request: list[tuple[int, int] | None] = []
    for frame in range(geometry.n_frames):
        first = geometry.loop_time(frame, 0)
        last = geometry.loop_time(frame, geometry.n_loops - 1)
        lo = hi = None
        for track in tracks:
            a = max(first, track.t_first - TRACK_TIME_MARGIN_S)
            b = min(last, track.t_last + TRACK_TIME_MARGIN_S)
            if a > b:
                continue
            bins = (track.bin_at(a), track.bin_at(b))
            track_lo = math.floor(min(bins)) - width_bins
            track_hi = math.ceil(max(bins)) + width_bins
            lo = track_lo if lo is None else min(lo, track_lo)
            hi = track_hi if hi is None else max(hi, track_hi)
        frame_start = geometry.frame_bin_start(frame)
        frame_end = frame_start + geometry.frame_bin_count(frame) - 1
        if lo is None or max(lo, frame_start) > min(hi, frame_end):
            request.append(None)
            continue
        lo, hi = max(lo, frame_start), min(hi, frame_end)
        request.append((lo, hi - lo + 1))
    return tuple(request)


def assemble_capture(overview: Overview, strips: bytes | None = None) -> tuple[bytes, np.ndarray]:
    """A dump in the full capture's layout holding only the strips.

    Returns the dump and a [n_frames, n_samples] mask of the local bins that
    hold real samples. Every other sample is zero; only the supplied overview
    products may be used for them.
    """
    meta = overview.metadata
    _require_timed(meta)
    shape = (meta["n_frames"], meta["chirps_per_frame"], meta["n_rx"], meta["n_samples"])
    cube = np.zeros(shape, dtype=complex)
    present = np.zeros((meta["n_frames"], meta["n_samples"]), dtype=bool)
    if strips is not None:
        strip_meta, strip_cube = parse_dump(strips)
        for key in (
            "n_frames",
            "chirps_per_frame",
            "n_tx",
            "n_rx",
            "sample_fmt",
            "trigger_frame",
            "frame_time_offsets_us",
        ):
            if strip_meta[key] != meta[key]:
                raise ValueError(f"strips do not match the capture: {key}")
        if strip_meta.get("iq8_scales") != meta.get("iq8_scales"):
            raise ValueError("strips do not match the capture: iq8_scales")
        for frame in range(meta["n_frames"]):
            start = strip_meta["range_bin_starts"][frame]
            count = strip_meta["range_bin_counts"][frame]
            local = start - meta["range_bin_starts"][frame]
            if local < 0 or local + count > meta["range_bin_counts"][frame]:
                raise ValueError(f"frame {frame} strip lies outside the capture window")
            cube[frame, ..., local : local + count] = strip_cube[frame, ..., :count]
            present[frame, local : local + count] = True
    return _pack_like(meta, cube, meta["range_bin_starts"], meta["range_bin_counts"]), present
