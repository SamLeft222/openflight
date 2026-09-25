"""GPIO-triggered IWR6843 L3 capture and OPS-shot correlation."""

from __future__ import annotations

import logging
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable

from openflight.gpio_factory import ensure_lgpio_pin_factory
from openflight.iwr6843.driver import IWR6843Radar
from openflight.iwr6843.dump import HEADER, parse_header, payload_nbytes
from openflight.iwr6843.reduced import StripRequest, parse_overview

logger = logging.getLogger(__name__)

_GRACEFUL_DUMP_SHUTDOWN_S = 12.0
# A held reduced-transfer capture nobody finishes is released after this long:
# under the firmware's own 10 s timeout, so the Pi's state never disagrees.
DEFAULT_HOLD_BUDGET_S = 8.0


def tx_order_from_config(config_path: str | Path) -> str:
    """Infer the vertical physical TX order from chirp masks in a cfg."""
    masks = []
    with Path(config_path).open(encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if line.startswith("chirpCfg"):
                masks.append(line.rsplit(maxsplit=1)[-1])
    if masks == ["1", "4"]:
        return "normal"
    if masks == ["4", "1"]:
        return "reversed"
    if masks == ["1", "2", "4"]:
        return "normal"
    raise ValueError(f"IWR6843 config must contain chirp TX masks 1/4, 4/1, or 1/2/4, got {masks}")


@dataclass(frozen=True)
class IWR6843Capture:
    """One GPIO edge and its completed L3 dump, or its reduced-transfer overview.

    ``transfer`` is "full" (``raw`` holds the dump) or "reduced" (``overview``
    holds the overview, and the radar holds the frozen ring until
    ``IWR6843CaptureMonitor.finish_capture``). ``fallback_reason`` says why a
    reduced-mode monitor fell back to a full dump.
    """

    sequence: int
    trigger_timestamp: float
    completed_timestamp: float
    dump_duration_s: float
    raw: bytes | None
    path: Path | None
    error: str | None = None
    temperature_report: dict[str, int] | None = None
    overview: bytes | None = None
    transfer: str = "full"
    fallback_reason: str | None = None

    @property
    def valid(self) -> bool:
        """Whether a complete dump or overview was captured."""
        return (self.raw is not None or self.overview is not None) and self.error is None


@dataclass
class _Hold:
    """The capture whose frozen ring the radar is holding.

    ``done`` wakes the capture thread: set when the hold is finished, or when
    the OPS rejects the sound (``rejected``) so the thread releases it now.
    A capture ``claimed`` by a shot is never released by a rejection.
    """

    sequence: int
    edge_timestamp: float
    done: threading.Event
    finished: bool = False
    rejected: bool = False
    claimed: bool = False


class IWR6843CaptureMonitor:
    """Capture TI rolling-buffer dumps on the same sound edge used by OPS.

    The GPIO callback only timestamps and queues the edge. Serial transfer is
    handled on a dedicated thread because one 768 KiB dump takes several
    seconds at the firmware UART rate.
    """

    def __init__(
        self,
        *,
        config_path: str | Path,
        output_dir: str | Path,
        port: str | None = None,
        gpio_pin: int = 17,
        radar: IWR6843Radar | None = None,
        button_factory: Callable | None = None,
        match_tolerance_s: float = 0.75,
        save_dumps: bool = False,
        trigger_observers: list[Callable[[float], None]] | None = None,
        reduced_gate: tuple[int, int] | None = None,
        hold_budget_s: float = DEFAULT_HOLD_BUDGET_S,
    ):
        self.config_path = Path(config_path)
        self.output_dir = Path(output_dir).expanduser()
        self.gpio_pin = gpio_pin
        self.match_tolerance_s = match_tolerance_s
        self.save_dumps = save_dumps
        self.radar = radar or IWR6843Radar(port=port)
        self._button_factory = button_factory
        self._button = None
        self._running = False
        self._armed = False
        self._capture_active = False
        self._sequence = 0
        self._last_edge_timestamp = 0.0
        self._events: queue.Queue[float | None] = queue.Queue(maxsize=1)
        self._captures: deque[IWR6843Capture] = deque()
        self._condition = threading.Condition()
        self._worker: threading.Thread | None = None
        self._trigger_observers = list(trigger_observers or [])
        # Reduced transfer (plans/iwr6843-on-chip-reduction.md): None keeps the
        # classic full dump. All radar I/O while a ring is held goes through
        # _serial_lock; _hold names the held capture.
        self.reduced_gate = reduced_gate
        self.hold_budget_s = hold_budget_s
        self._serial_lock = threading.Lock()
        self._hold: _Hold | None = None
        # Edges the OPS rejected before their capture completed.
        self._rejected_edges: deque[float] = deque(maxlen=16)

    @property
    def port(self) -> str:
        """Connected TI serial port."""
        return self.radar.port

    def start(self, *, armed: bool = True) -> None:
        """Configure the radar and GPIO, optionally arming trigger capture."""
        if self._running:
            return
        if not self.config_path.is_file():
            raise FileNotFoundError(f"IWR6843 config not found: {self.config_path}")
        if self.save_dumps:
            self.output_dir.mkdir(parents=True, exist_ok=True)
        configured = False
        try:
            self.radar.send_config(str(self.config_path))
            configured = True
            if self.reduced_gate is not None:
                try:
                    self.radar.configure_overview_gate(self.reduced_gate)
                except RuntimeError as exc:
                    # Released firmware has no overviewCfg: keep the radar
                    # working with full dumps rather than failing the IWR.
                    logger.warning(
                        "[IWR6843] Firmware lacks the reduced transfer (%s); using full dumps",
                        exc,
                    )
                    self.reduced_gate = None

            button_factory = self._button_factory
            if button_factory is None:
                # Must precede the first gpiozero device: on a Pi 5 gpiozero's
                # own auto-detection fails outright. See gpio_factory.
                ensure_lgpio_pin_factory()

                from gpiozero import Button  # pylint: disable=import-error,import-outside-toplevel

                button_factory = Button
            # No gpiozero debounce: lgpio delays delivery by the debounce interval,
            # which previously cost the first 50 ms of ball flight.
            self._button = button_factory(self.gpio_pin, pull_up=False, bounce_time=None)
            self._running = True
            self._worker = threading.Thread(
                target=self._capture_loop,
                name="iwr6843-capture",
                daemon=True,
            )
            self._worker.start()
            if armed:
                self.arm()
        except Exception:
            self._running = False
            if self._button is not None:
                self._button.close()
                self._button = None
            if configured:
                self._stop_sensor_and_close()
            else:
                self.radar.close()
            raise
        logger.info(
            "[IWR6843] Configured on BCM%d using %s (%s%s)",
            self.gpio_pin,
            self.port,
            self.config_path.name,
            ", armed" if self._armed else ", waiting for OPS",
        )

    def arm(self) -> None:
        """Accept GPIO edges after the OPS trigger path is fully initialized."""
        if not self._running:
            raise RuntimeError("cannot arm an IWR6843 monitor that is not running")
        if self._armed:
            return
        # Attach while logically disarmed so a line already high from OPS
        # startup cannot synchronously create a false capture.
        self._button.when_pressed = self.notify_trigger
        self._armed = True
        logger.info("[IWR6843] Armed on BCM%d", self.gpio_pin)

    def notify_trigger(self, timestamp: float | None = None) -> bool:
        """Queue a GPIO edge without doing serial work in the callback."""
        if not self._running or not self._armed:
            return False
        edge_timestamp = time.time() if timestamp is None else float(timestamp)
        with self._condition:
            # Reject acoustic ringing and any second edge while the seven-second
            # UART dump is in flight. The OPS side makes the same shot wait.
            if (
                self._capture_active
                or not self._events.empty()
                or edge_timestamp - self._last_edge_timestamp < 0.1
            ):
                logger.debug("[IWR6843] Ignoring duplicate/busy trigger edge")
                return False
            self._last_edge_timestamp = edge_timestamp
            self._events.put_nowait(edge_timestamp)
            self._condition.notify_all()
        for observer in self._trigger_observers:
            try:
                observer(edge_timestamp)
            except Exception:  # pylint: disable=broad-exception-caught
                logger.warning("[IWR6843] Trigger observer failed", exc_info=True)
        return True

    def _validate_dump(self, raw: bytes) -> dict:
        if len(raw) < HEADER.size:
            raise ValueError(f"short IWR6843 dump: {len(raw)} bytes")
        metadata = parse_header(raw)
        expected = metadata["header_nbytes"] + payload_nbytes(metadata, raw)
        if len(raw) != expected:
            raise ValueError(f"short IWR6843 dump: {len(raw)} bytes, expected {expected}")
        return metadata

    def _capture_path(
        self, sequence: int, trigger_timestamp: float, suffix: str = ".l3dump"
    ) -> Path:
        timestamp = datetime.fromtimestamp(trigger_timestamp).strftime("%Y%m%d_%H%M%S_%f")[:-3]
        return self.output_dir / f"iwr6843_{timestamp}_{sequence:03d}{suffix}"

    def _read_full(self, sequence: int, edge_timestamp: float) -> tuple[bytes, Path | None, dict]:
        """One ordinary dump (or the held ring streamed whole), validated and saved."""
        raw = self.radar.read_dump()
        metadata = self._validate_dump(raw)
        path = None
        if self.save_dumps:
            path = self._capture_path(sequence, edge_timestamp)
            path.write_bytes(raw)
        return raw, path, metadata

    def _read_reduced(self, sequence: int, edge_timestamp: float) -> dict:
        """The overview, holding the ring -- or an ordinary dump if that fails."""
        try:
            overview = self.radar.read_overview()
            metadata = parse_overview(overview).metadata
        except Exception as exc:  # pylint: disable=broad-exception-caught
            logger.warning(
                "[IWR6843] Overview #%d failed (%s); falling back to a full dump",
                sequence,
                exc,
            )
            try:
                self.radar.release()
            except Exception:  # pylint: disable=broad-exception-caught
                logger.warning("[IWR6843] Release before fallback failed", exc_info=True)
            raw, path, metadata = self._read_full(sequence, edge_timestamp)
            return {"raw": raw, "path": path, "metadata": metadata, "fallback_reason": str(exc)}
        path = None
        if self.save_dumps:
            path = self._capture_path(sequence, edge_timestamp, ".overview")
            path.write_bytes(overview)
        with self._serial_lock:
            self._hold = _Hold(
                sequence=sequence, edge_timestamp=edge_timestamp, done=threading.Event()
            )
        return {"overview": overview, "path": path, "metadata": metadata, "transfer": "reduced"}

    def read_strips(self, capture: IWR6843Capture, request: StripRequest) -> bytes:
        """Strips of a held capture's frozen ring."""
        with self._serial_lock:
            if not self._holding(capture):
                raise RuntimeError(f"IWR6843 capture #{capture.sequence} is no longer held")
            return self.radar.read_strips(request)

    def finish_capture(self, capture: IWR6843Capture, *, full_dump: bool) -> bytes | None:
        """Let a reduced-transfer capture go; returns the full dump when asked for.

        ``full_dump`` streams the held ring whole (and the firmware resumes);
        otherwise the ring is released. A full-transfer capture returns its
        dump. Finishing a capture that is no longer held returns None.
        """
        if capture.transfer != "reduced":
            return capture.raw
        with self._serial_lock:
            if not self._holding(capture):
                return None
            hold = self._hold
            try:
                if full_dump:
                    raw, _path, _metadata = self._read_full(
                        capture.sequence, capture.trigger_timestamp
                    )
                    return raw
                self.radar.release()
                return None
            finally:
                self._end_hold(hold)

    def _holding(self, capture: IWR6843Capture) -> bool:
        hold = self._hold
        return hold is not None and hold.sequence == capture.sequence and not hold.finished

    def _end_hold(self, hold: _Hold) -> None:
        """Mark a hold over; the caller holds _serial_lock."""
        hold.finished = True
        if self._hold is hold:
            self._hold = None
        hold.done.set()

    def _await_finish(self, sequence: int) -> None:
        """Block new edges until the held capture is finished or its budget runs out."""
        hold = self._hold
        if hold is None or hold.sequence != sequence:
            return
        hold.done.wait(self.hold_budget_s)
        with self._serial_lock:
            if hold.finished:
                return
            if hold.rejected:
                logger.info(
                    "[IWR6843] Capture #%d: the OPS found no shot; releasing the radar", sequence
                )
            else:
                logger.warning(
                    "[IWR6843] Capture #%d was not finished within %.1fs; releasing the radar",
                    sequence,
                    self.hold_budget_s,
                )
            try:
                self.radar.release()
            except Exception:  # pylint: disable=broad-exception-caught
                logger.warning("[IWR6843] Release of abandoned capture failed", exc_info=True)
            finally:
                self._end_hold(hold)

    def _capture_loop(self) -> None:
        while self._running:
            edge_timestamp = self._events.get()
            if edge_timestamp is None or not self._running:
                break
            with self._condition:
                self._capture_active = True
                self._sequence += 1
                sequence = self._sequence
            start = time.time()
            raw = None
            path = None
            error = None
            metadata = None
            result: dict = {}
            try:
                logger.info(
                    "[IWR6843] Trigger #%d: %s",
                    sequence,
                    "fetching the overview of the frozen L3 ring"
                    if self.reduced_gate is not None
                    else "dumping firmware-frozen L3 ring",
                )
                if self.reduced_gate is not None:
                    result = self._read_reduced(sequence, edge_timestamp)
                else:
                    raw, path, metadata = self._read_full(sequence, edge_timestamp)
                    result = {"raw": raw, "path": path, "metadata": metadata}
            except Exception as exc:  # pylint: disable=broad-exception-caught
                error = str(exc)
                logger.warning("[IWR6843] Capture #%d failed: %s", sequence, exc, exc_info=True)
            raw = result.get("raw")
            path = result.get("path")
            metadata = result.get("metadata")
            completed = time.time()
            capture = IWR6843Capture(
                sequence=sequence,
                trigger_timestamp=edge_timestamp,
                completed_timestamp=completed,
                dump_duration_s=completed - start,
                raw=raw,
                path=path,
                error=error,
                temperature_report=(
                    metadata.get("temperature_report") if metadata is not None else None
                ),
                overview=result.get("overview"),
                transfer=result.get("transfer", "full"),
                fallback_reason=result.get("fallback_reason"),
            )
            with self._condition:
                if self._take_rejection(edge_timestamp):
                    self._reject_hold(sequence)
                else:
                    self._captures.append(capture)
                self._condition.notify_all()
            # A held ring cannot capture another shot: keep new edges out until
            # the runtime finishes this capture (or its budget runs out).
            self._await_finish(sequence)
            with self._condition:
                self._capture_active = False
                self._condition.notify_all()
            logger.info(
                "[IWR6843] Capture #%d complete: %s in %.2fs",
                sequence,
                f"{len(raw)} bytes"
                if raw is not None
                else f"overview {len(capture.overview)} bytes"
                if capture.overview is not None
                else error,
                capture.dump_duration_s,
            )

    def discard_trigger(self, timestamp: float | None) -> bool:
        """The OPS found no shot for the sound at ``timestamp``: free its capture.

        The matching capture is dropped and, if the radar is holding its ring,
        the capture thread releases it at once instead of after the hold
        budget. A capture still being read is released when it arrives. A
        capture already claimed by a shot is left alone. Never touches the
        radar itself, so it is safe to call from the OPS thread. Returns
        whether a completed capture matched.
        """
        if timestamp is None:
            return False
        timestamp = float(timestamp)
        with self._condition:
            matched = [
                capture
                for capture in self._captures
                if abs(capture.trigger_timestamp - timestamp) <= self.match_tolerance_s
            ]
            for capture in matched:
                self._captures.remove(capture)
                logger.info(
                    "[IWR6843] Capture #%d: the OPS found no shot for this sound; discarding it",
                    capture.sequence,
                )
                self._reject_hold(capture.sequence)
            if not matched:
                self._rejected_edges.append(timestamp)
            return bool(matched)

    def _claim(self, capture: IWR6843Capture) -> IWR6843Capture:
        """Hand a capture to a shot; caller holds _condition."""
        hold = self._hold
        if hold is not None and hold.sequence == capture.sequence:
            hold.claimed = True
        return capture

    def _take_rejection(self, edge_timestamp: float) -> bool:
        """Consume a pending OPS rejection of this edge; caller holds _condition."""
        for rejected in self._rejected_edges:
            if abs(rejected - edge_timestamp) <= self.match_tolerance_s:
                self._rejected_edges.remove(rejected)
                return True
        return False

    def _reject_hold(self, sequence: int) -> None:
        """Wake the capture thread to release an unclaimed hold; caller holds _condition."""
        hold = self._hold
        if hold is not None and hold.sequence == sequence and not hold.claimed:
            hold.rejected = True
            hold.done.set()

    def capture_for_shot(
        self,
        impact_timestamp: float | None,
        *,
        timeout_s: float = 12.0,
    ) -> IWR6843Capture | None:
        """Consume the capture nearest an OPS impact timestamp."""
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                if impact_timestamp is None and self._captures:
                    return self._claim(self._captures.popleft())

                if impact_timestamp is not None:
                    cutoff = impact_timestamp - self.match_tolerance_s
                    while self._captures and self._captures[0].trigger_timestamp < cutoff:
                        stale = self._captures.popleft()
                        logger.warning(
                            "[IWR6843] Discarding unmatched capture #%d (edge %.3f, shot %.3f)",
                            stale.sequence,
                            stale.trigger_timestamp,
                            impact_timestamp,
                        )
                    matches = [
                        capture
                        for capture in self._captures
                        if abs(capture.trigger_timestamp - impact_timestamp)
                        <= self.match_tolerance_s
                    ]
                    if matches:
                        selected = min(
                            matches,
                            key=lambda capture: abs(capture.trigger_timestamp - impact_timestamp),
                        )
                        self._captures.remove(selected)
                        return self._claim(selected)

                    matching_capture_active = abs(
                        self._last_edge_timestamp - impact_timestamp
                    ) <= self.match_tolerance_s and (
                        self._capture_active or not self._events.empty()
                    )
                    if (
                        time.time() > impact_timestamp + self.match_tolerance_s
                        and not matching_capture_active
                    ):
                        return None

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(remaining)

    def stop(self) -> None:
        """Drain active capture, stop firmware, then release host resources."""
        if not self._running:
            return
        self._armed = False
        self._running = False
        if self._button is not None:
            self._button.when_pressed = None
            self._button.close()
            self._button = None
        try:
            self._events.put_nowait(None)
        except queue.Full:
            pass
        self._release_hold_for_shutdown()
        if self._worker is not None:
            # Preserve a complete debug dump and its trailing CLI prompt before
            # issuing sensorStop. Closing early strands firmware mid-transfer.
            self._worker.join(timeout=_GRACEFUL_DUMP_SHUTDOWN_S)
            if self._worker.is_alive():
                logger.warning(
                    "[IWR6843] Active dump did not finish within %.1fs; "
                    "forcing serial close (board reset may be required)",
                    _GRACEFUL_DUMP_SHUTDOWN_S,
                )
                self.radar.close()
                self._worker.join(timeout=2.0)
            else:
                self._stop_sensor_and_close()
            self._worker = None
        else:
            self._stop_sensor_and_close()
        logger.info("[IWR6843] Capture monitor stopped")

    def _release_hold_for_shutdown(self) -> None:
        with self._serial_lock:
            hold = self._hold
            if hold is None or hold.finished:
                return
            try:
                self.radar.release()
            except Exception:  # pylint: disable=broad-exception-caught
                logger.warning("[IWR6843] Release at shutdown failed", exc_info=True)
            finally:
                self._end_hold(hold)

    def _stop_sensor_and_close(self) -> None:
        """Best-effort firmware stop that never leaks the serial descriptor."""
        try:
            self.radar.stop_sensor()
            logger.info("[IWR6843] Firmware capture stopped and verified inactive")
        except Exception:  # pylint: disable=broad-exception-caught
            logger.warning(
                "[IWR6843] Firmware did not stop cleanly; board reset may be required",
                exc_info=True,
            )
        finally:
            self.radar.close()


__all__ = [
    "IWR6843Capture",
    "IWR6843CaptureMonitor",
    "tx_order_from_config",
]
