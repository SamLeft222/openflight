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
from dataclasses import dataclass, field
from typing import Protocol

import numpy as np

from openflight.iwr6843 import tracking
from openflight.iwr6843.dump import (
    SAMPLE_RANGE_FFT_IQ8_VARIABLE_TIMED,
    SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED,
    capture_metadata_prefix,
    pack_dump,
    parse_capture_metadata,
    parse_dump,
)
from openflight.iwr6843.lcmf import TRACK_TIME_MARGIN_S, PreparedLCMFCapture, prepare_lcmf_capture
from openflight.iwr6843.shot import RANGE_FFT_SIZE, TX2_LOOP_PERIOD_S, geometry_from_header
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
# l3strip carries 4 hex digits per frame on the firmware's 255-character CLI line.
MAX_STRIP_REQUEST_FRAMES = 61

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


def _vertical_codes(meta: dict, cube: np.ndarray) -> list[tuple[np.ndarray, np.ndarray, int]]:
    """Per frame: integer (re, im) codes [pair, loop, rx, bin] of the vertical TX pair, scale."""
    loops = _loops(meta)
    pair = VERTICAL_TX_PAIR if meta["n_tx"] == 3 else (0, 1)
    scales = meta.get("iq8_scales") or (1,) * meta["n_frames"]
    frames = []
    for frame, count in enumerate(meta["range_bin_counts"]):
        tdm = cube[frame, :, :, :count].reshape(loops, meta["n_tx"], meta["n_rx"], count)
        stored = tdm[:, list(pair)].transpose(1, 0, 2, 3) / scales[frame]
        frames.append(
            (
                np.rint(stored.real).astype(np.int64),
                np.rint(stored.imag).astype(np.int64),
                int(scales[frame]),
            )
        )
    return frames


def build_overview(raw: bytes, *, gate: tuple[int, int]) -> Overview:
    """What the radar computes from its full ring, before quantisation.

    Integer formulation shared bit-for-bit with firmware/iwr6843/
    reduced_overview.c. For stored codes c (sample = c * scale s) over L
    loops, burst-scope MTI power is exactly s^2 |L c - sum c|^2 / L^2 and
    window-scope power is |n s c - T|^2 / n^2 with T the bin's scaled code
    sum over its n loops. Each is rounded once, so it can differ from the
    float MTI of the full-capture path in the last ulp.
    """
    meta, cube = parse_dump(raw)
    _require_timed(meta)
    lo, hi = gate
    if not 0 <= lo < hi <= 0xFFFF:
        raise ValueError(f"invalid gate {gate}")
    loops = _loops(meta)
    n_rx = meta["n_rx"]
    frames = _vertical_codes(meta, cube)
    fft_size = tracking.window_fft_size(geometry_from_header(meta))

    # Window-scope sums: T per (pair, rx, bin) and loop count n per bin.
    total_re = np.zeros((2, n_rx, fft_size), dtype=np.int64)
    total_im = np.zeros((2, n_rx, fft_size), dtype=np.int64)
    counts = np.zeros(fft_size, dtype=np.int64)
    for (re, im, scale), start, count in zip(
        frames, meta["range_bin_starts"], meta["range_bin_counts"]
    ):
        total_re[:, :, start : start + count] += scale * re.sum(axis=1)
        total_im[:, :, start : start + count] += scale * im.sum(axis=1)
        counts[start : start + count] += loops

    n_rows = meta["n_frames"] * loops
    power = {scope: np.zeros((n_rows, meta["n_samples"])) for scope in MTI_SCOPES}
    values: dict[str, list[np.ndarray]] = {scope: [] for scope in MTI_SCOPES}
    columns = _gate_columns(meta, (lo, hi))
    for frame, ((re, im, scale), start, count) in enumerate(
        zip(frames, meta["range_bin_starts"], meta["range_bin_counts"])
    ):
        rows = slice(frame * loops, (frame + 1) * loops)
        a, b = columns[frame]

        # burst: N = |L c - sum c|^2 exactly; power = N * s^2 / L^2
        k = (scale * scale) / (loops * loops)
        e_re = loops * re - re.sum(axis=1, keepdims=True)
        e_im = loops * im - im.sum(axis=1, keepdims=True)
        n_exact = e_re * e_re + e_im * e_im  # [pair, loop, rx, bin]
        values["burst"].append(n_exact.astype(np.float64).reshape(-1) * k)
        power["burst"][rows, a:b] = n_exact.sum(axis=(0, 2))[:, a:b].astype(np.float64) * k

        # window: M = |n s c - T|^2 (float), power = M / n^2
        n_bin = counts[start : start + count]
        inv_nn = 1.0 / (n_bin * n_bin).astype(np.float64)
        w_re = (n_bin * scale * re - total_re[:, None, :, start : start + count]).astype(np.float64)
        w_im = (n_bin * scale * im - total_im[:, None, :, start : start + count]).astype(np.float64)
        m = w_re * w_re + w_im * w_im  # [pair, loop, rx, bin]
        values["window"].append((m * inv_nn).reshape(-1))
        summed = np.zeros((loops, count))
        for pair_index in range(2):  # the firmware's summation order
            for rx in range(n_rx):
                summed = summed + m[pair_index, :, rx, :]
        power["window"][rows, a:b] = (summed * inv_nn)[:, a:b]

    means = np.zeros((2, n_rx, fft_size), dtype=complex)
    held = counts > 0
    means.real[:, :, held] = total_re[:, :, held] / counts[held]
    means.imag[:, :, held] = total_im[:, :, held] / counts[held]
    return Overview(
        capture_prefix=capture_metadata_prefix(raw),
        gate=(lo, hi),
        power=power,
        noise={scope: float(np.median(np.concatenate(values[scope]))) for scope in MTI_SCOPES},
        window_means=means,
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


@dataclass(frozen=True)
class _OverviewLayout:
    meta: dict
    offset: int
    gate: tuple[int, int]
    loops: int
    noise: dict[str, float]
    means_span: tuple[int, int]
    columns: list[tuple[int, int]]
    nbytes: int


def _overview_layout(raw: bytes) -> _OverviewLayout:
    """Validated overview header and total size; needs only the header bytes."""
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
    means_values = 2 * meta["n_rx"] * (means_hi - means_lo)
    return _OverviewLayout(
        meta=meta,
        offset=offset,
        gate=(lo, hi),
        loops=loops,
        noise={"burst": float(noise_burst), "window": float(noise_window)},
        means_span=(means_lo, means_hi),
        columns=columns,
        nbytes=offset
        + OVERVIEW_HEADER.size
        + len(MTI_SCOPES) * power_values * 2
        + means_values * 8,
    )


def overview_nbytes(raw: bytes) -> int | None:
    """Total size of a streamed overview, or None until its header has arrived.

    Raises ValueError when the header that has arrived is not an overview.
    """
    try:
        return _overview_layout(raw).nbytes
    except ValueError as error:
        if str(error).startswith("short"):
            return None
        raise


def parse_overview(raw: bytes) -> Overview:
    """Bytes -> Overview, with the power maps back in capture-local bins."""
    layout = _overview_layout(raw)
    if len(raw) != layout.nbytes:
        raise ValueError(f"overview is {len(raw)} bytes, expected {layout.nbytes}")
    meta, offset, loops, columns = layout.meta, layout.offset, layout.loops, layout.columns
    means_lo, means_hi = layout.means_span
    n_rx = meta["n_rx"]
    means_values = 2 * n_rx * (means_hi - means_lo)
    body = offset + OVERVIEW_HEADER.size

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
        gate=layout.gate,
        power=power,
        noise=layout.noise,
        window_means=window_means,
    )


# -- the Pi side -----------------------------------------------------------------


class ReducedSource(Protocol):
    """Where a reduced capture comes from: the radar, or a model of it."""

    def overview(self) -> Overview:
        """Freeze the ring and return its overview."""

    def strips(self, request: StripRequest) -> bytes:
        """Return the requested strips of the frozen ring."""


@dataclass
class CaptureReducedSource:
    """The radar's reduced transfer, modelled from a full capture.

    ``quantise=False`` hands over the overview before it is packed, so the
    reduced path can be checked bit-for-bit against the full-capture path.
    Byte counts are what the radar would send.
    """

    raw: bytes
    gate: tuple[int, int]
    quantise: bool = True
    overview_nbytes: int = 0
    strip_nbytes: list[int] = field(default_factory=list)

    def overview(self) -> Overview:
        """The overview as the radar would send it (or unquantised)."""
        built = build_overview(self.raw, gate=self.gate)
        packed = pack_overview(built)
        self.overview_nbytes = len(packed)
        return parse_overview(packed) if self.quantise else built

    def strips(self, request: StripRequest) -> bytes:
        """The requested strips, counting their bytes."""
        response = serve_strips(self.raw, request)
        self.strip_nbytes.append(len(response))
        return response

    @property
    def total_nbytes(self) -> int:
        """Everything sent for this shot."""
        return self.overview_nbytes + sum(self.strip_nbytes)


def default_ball_gate(net_range_m: float | None) -> tuple[int, int]:
    """The overview gate for range-snapshot captures, known before any capture."""
    return tracking.ball_gate_bins(
        tracking.RANGE_SPAN_M / RANGE_FFT_SIZE,
        max_range_m=tracking.max_ball_range_m(net_range_m),
    )


def ball_gate_for(meta: dict, net_range_m: float | None) -> tuple[int, int]:
    """The overview gate the ball tracker needs for this capture and net."""
    return tracking.ball_gate_bins(
        geometry_from_header(meta).range_res_m,
        max_range_m=tracking.max_ball_range_m(net_range_m),
    )


def prepare_reduced(
    overview: Overview, strips: bytes | None = None
) -> tuple[bytes, PreparedLCMFCapture]:
    """The rebuilt capture and its LCMF preparation with the overview products."""
    raw, _present = assemble_capture(overview, strips)
    return raw, prepare_lcmf_capture(
        raw,
        supplied_power=overview.power,
        supplied_noise=overview.noise,
        supplied_window_means=overview.window_means,
    )


def encode_strip_request(request: StripRequest) -> str:
    """The l3strip argument: start and count per frame as hex, 0000 for none."""
    if len(request) > MAX_STRIP_REQUEST_FRAMES:
        raise ValueError(
            f"strip requests carry at most {MAX_STRIP_REQUEST_FRAMES} frames, got {len(request)}"
        )
    parts = []
    for frame, window in enumerate(request):
        if window is None:
            parts.append("0000")
            continue
        start, count = window
        if not 0 <= start <= 255 or not 1 <= count <= 255:
            raise ValueError(f"frame {frame} strip {window} does not fit the request format")
        parts.append(f"{start:02x}{count:02x}")
    return "".join(parts)


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
