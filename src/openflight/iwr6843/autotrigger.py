"""On-radar shot detection for the IWR6843 (reference model of auto_trigger.c).

With the OPS243 on its internal trigger there is no sound edge, and the Pi
hears about a shot ~110 ms after impact -- later than the 72 ms IWR ring can
wait. So the radar detects the swing itself, from the frames it already
records before impact.

The ball's echo is too weak to trigger on (the golfer moves in the same
range bins), but the club head's approach is a clean outward track at club
speed that reaches the tee at impact. Per frame, for the first TX:

1. MTI power per range bin: over the frame's loops, sum of |x - mean|^2 over
   the RX channels.
2. Normalise by the frame's median power and cap it (``z``).
3. Keep the last ``n_frames`` rows. Fire when some straight outward track --
   slope ``slope_min_q``..``slope_max_q`` quarter bins per frame, ending at a
   bin in ``[end_lo, end_hi]`` -- has ``z >= z_min`` in every frame.
4. Request the freeze ``delay_frames`` frames later.

Everything is exact integer arithmetic until one division per value, and the
line search runs in quarter bins, so firmware/iwr6843/auto_trigger.c computes
bit-identical results (tests/test_iwr6843_auto_trigger_firmware.py).
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass, field

import numpy as np

from openflight.iwr6843.dump import (
    SAMPLE_RANGE_FFT_IQ8_VARIABLE_TIMED,
    SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED,
    _parse_frame_metadata,
    parse_header,
)

MAX_FRAMES = 8
MAX_WINDOW_BINS = 64
BIN_SPACE = 256


@dataclass(frozen=True)
class AutoTriggerConfig:
    """Detector settings; the firmware takes ``z_min``/``z_cap`` in tenths."""

    end_lo: int
    end_hi: int
    n_frames: int = 5
    z_min_x10: int = 80
    z_cap_x10: int = 500
    slope_min_q: int = 2
    slope_max_q: int = 12
    delay_frames: int = 3

    def __post_init__(self):
        if not 0 <= self.end_lo <= self.end_hi < BIN_SPACE:
            raise ValueError("need 0 <= end_lo <= end_hi < 256")
        if not 2 <= self.n_frames <= MAX_FRAMES:
            raise ValueError(f"n_frames must be 2..{MAX_FRAMES}")
        if not 1 <= self.slope_min_q <= self.slope_max_q <= 64:
            raise ValueError("need 1 <= slope_min_q <= slope_max_q <= 64 (quarter bins)")
        if not 1 <= self.z_min_x10 <= self.z_cap_x10 <= 100_000:
            raise ValueError("need 1 <= z_min_x10 <= z_cap_x10 <= 100000")
        if not 0 <= self.delay_frames <= 32:
            raise ValueError("delay_frames must be 0..32")

    @property
    def z_min(self) -> float:
        """Threshold every frame of the track must reach."""
        return self.z_min_x10 / 10.0

    @property
    def z_cap(self) -> float:
        """Ceiling on one bin's normalised power, so one glint can't carry a track."""
        return self.z_cap_x10 / 10.0

    def command(self) -> str:
        """The firmware's ``autoTrigCfg`` line that enables this configuration."""
        return (
            f"autoTrigCfg 1 {self.end_lo} {self.end_hi} {self.n_frames} {self.z_min_x10} "
            f"{self.z_cap_x10} {self.slope_min_q} {self.slope_max_q} {self.delay_frames}"
        )


@dataclass(frozen=True)
class Detection:
    """The best track at the frame it was found."""

    frame: int
    end_bin: int
    slope_q: int
    score: float


def capture_frames(raw: bytes) -> Iterator[tuple[int, np.ndarray]]:
    """A timed capture's frames as stored: ``(bin_start, codes)`` in dump order.

    ``codes`` is integer ``(chirps, rx, bins, 2)`` (imag, real): IQ8 codes
    without their frame scale, which normalisation cancels, or IQ16 values.
    """
    meta = parse_header(raw)
    if meta["sample_fmt"] not in (
        SAMPLE_RANGE_FFT_IQ8_VARIABLE_TIMED,
        SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED,
    ):
        raise ValueError("auto-trigger replay needs a timed capture")
    _parse_frame_metadata(raw, meta)
    iq8 = meta["sample_fmt"] == SAMPLE_RANGE_FFT_IQ8_VARIABLE_TIMED
    dtype = np.int8 if iq8 else np.dtype("<i2")
    offset = meta["header_nbytes"] + meta.get("frame_metadata_nbytes", 0)
    chirps, n_rx = meta["chirps_per_frame"], meta["n_rx"]
    for start, count in zip(meta["range_bin_starts"], meta["range_bin_counts"]):
        values = 2 * chirps * n_rx * count
        body = np.frombuffer(raw, dtype=dtype, offset=offset, count=values)
        offset += body.nbytes
        yield start, body.astype(np.int16).reshape(chirps, n_rx, count, 2)


def frame_power(codes: np.ndarray, *, n_tx: int, tx: int = 0) -> list[float]:
    """MTI power per bin of one frame.

    ``codes`` is the frame as stored, integer ``(chirps, rx, bins, 2)`` with
    the last axis (imag, real). Per bin: ``sum_rx (L*sum|x|^2 - |sum x|^2)``,
    exact in integers, divided once by the loop count ``L``.
    """
    x = np.asarray(codes, dtype=np.int64)[tx::n_tx]
    loops = x.shape[0]
    sum_sq = (x * x).sum(axis=(0, 3))  # rx, bins
    sums = x.sum(axis=0)  # rx, bins, 2
    numerator = (loops * sum_sq - (sums * sums).sum(axis=2)).sum(axis=0)
    return [int(value) / loops for value in numerator]


def median(values: list[float]) -> float:
    """Median by sorting: the middle value, or the mean of the middle two."""
    ordered = sorted(values)
    n = len(ordered)
    if n == 0:
        return 0.0
    if n % 2:
        return ordered[n // 2]
    return (ordered[n // 2 - 1] + ordered[n // 2]) * 0.5


def normalise(power: list[float], z_cap: float) -> list[float]:
    """Power over the frame median, capped; all zero if the median is zero."""
    level = median(power)
    if level <= 0.0:
        return [0.0] * len(power)
    return [min(value / level, z_cap) for value in power]


def _floor_div4(value: int) -> int:
    return value // 4  # Python floors toward -inf, as the C helper does


@dataclass
class AutoTrigger:
    """Streaming detector: push one frame at a time, in capture order."""

    config: AutoTriggerConfig
    _rows: list[tuple[int, list[float]]] = field(default_factory=list)
    _frames_seen: int = 0
    _countdown: int = -1
    _fired: bool = False
    detection: Detection | None = None

    def reset(self) -> None:
        """Forget history (the ring restarted, so frames are no longer consecutive)."""
        self._rows.clear()
        self._frames_seen = 0
        self._countdown = -1
        self._fired = False
        self.detection = None

    def push_codes(self, bin_start: int, codes: np.ndarray, *, n_tx: int) -> bool:
        """Push one stored frame; True when the freeze should be requested now."""
        z = normalise(frame_power(codes, n_tx=n_tx), self.config.z_cap)
        return self.push(bin_start, z)

    def push(self, bin_start: int, z: list[float]) -> bool:
        """Push one frame's normalised row; True when the freeze should be requested.

        Fires once; later frames return False until ``reset``.
        """
        if len(z) > MAX_WINDOW_BINS or not 0 <= bin_start <= BIN_SPACE - len(z):
            raise ValueError("frame window outside the supported bins")
        frame = self._frames_seen
        self._frames_seen += 1
        self._rows.append((bin_start, list(z)))
        if len(self._rows) > self.config.n_frames:
            self._rows.pop(0)
        if self._fired:
            return False
        if self._countdown > 0:
            self._countdown -= 1
            self._fired = self._countdown == 0
            return self._fired
        if len(self._rows) < self.config.n_frames:
            return False
        best = self._best_track(frame)
        if best is None:
            return False
        self.detection = best
        self._countdown = self.config.delay_frames
        self._fired = self._countdown == 0
        return self._fired

    def _z(self, row: int, absolute_bin: int) -> float:
        start, values = self._rows[row]
        offset = absolute_bin - start
        return values[offset] if 0 <= offset < len(values) else 0.0

    def _best_track(self, frame: int) -> Detection | None:
        cfg = self.config
        n = cfg.n_frames
        z_min = cfg.z_min
        best = None
        for slope_q in range(cfg.slope_min_q, cfg.slope_max_q + 1):
            for end in range(cfg.end_lo, cfg.end_hi + 1):
                lit = 0
                score = 0.0
                for k in range(n):
                    position_q = 4 * end - slope_q * (n - 1 - k)
                    lo = _floor_div4(position_q)
                    hi = -_floor_div4(-position_q)
                    value = max(self._z(k, lo), self._z(k, hi))
                    score += value
                    if value >= z_min:
                        lit += 1
                if lit == n and (best is None or score > best.score):
                    best = Detection(frame=frame, end_bin=end, slope_q=slope_q, score=score)
        return best


__all__ = [
    "AutoTrigger",
    "capture_frames",
    "AutoTriggerConfig",
    "Detection",
    "frame_power",
    "median",
    "normalise",
]
