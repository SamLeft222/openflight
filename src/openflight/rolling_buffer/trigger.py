"""
Trigger strategies for rolling buffer capture.

Defines different methods for determining when to capture the rolling buffer.
"""

import logging
import threading
import time
from abc import ABC, abstractmethod
from datetime import datetime, timezone
from typing import TYPE_CHECKING, Callable, List, Optional

from .processor import RollingBufferProcessor
from .types import IQCapture

if TYPE_CHECKING:
    from ..ops243 import OPS243Radar

# Incremented per capture for log sequencing (not shot number)

logger = logging.getLogger("openflight.rolling_buffer.trigger")


class TriggerStrategy(ABC):
    """
    Base class for trigger strategies.

    A trigger strategy determines when to capture the rolling buffer.
    Different strategies trade off between simplicity, reliability, and efficiency.
    """

    MIN_VALID_OUTBOUND_MPH = 15.0

    def __init__(self, pre_trigger_segments: int = 12):
        self._diagnostics: List[dict] = []
        self.pre_trigger_segments = pre_trigger_segments

    def drain_diagnostics(self) -> List[dict]:
        """Return and clear accumulated diagnostic entries.

        Diagnostics are accumulated during wait_for_trigger() calls.
        The monitor drains these after each call to log and emit them.
        """
        diagnostics = self._diagnostics
        self._diagnostics = []
        return diagnostics

    def _append_diagnostic(
        self,
        accepted: bool,
        reason: str,
        response_bytes: int = 0,
        total_readings: int = 0,
        outbound_readings: int = 0,
        inbound_readings: int = 0,
        peak_outbound_mph: float = 0.0,
        peak_inbound_mph: float = 0.0,
        all_outbound_speeds: Optional[List[float]] = None,
        all_inbound_speeds: Optional[List[float]] = None,
        peak_outbound_magnitude: float = 0.0,
        peak_inbound_magnitude: float = 0.0,
        trigger_latency_ms: Optional[float] = None,
    ):
        """Append a diagnostic entry for the current trigger event."""
        entry = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "accepted": accepted,
            "reason": reason,
            "response_bytes": response_bytes,
            "total_readings": total_readings,
            "outbound_readings": outbound_readings,
            "inbound_readings": inbound_readings,
            "peak_outbound_mph": peak_outbound_mph,
            "peak_inbound_mph": peak_inbound_mph,
            "all_outbound_speeds": all_outbound_speeds or [],
            "all_inbound_speeds": all_inbound_speeds or [],
            "peak_outbound_magnitude": peak_outbound_magnitude,
            "peak_inbound_magnitude": peak_inbound_magnitude,
        }
        if trigger_latency_ms is not None:
            entry["trigger_latency_ms"] = trigger_latency_ms
        self._diagnostics.append(entry)

    def _summarize_capture_activity(
        self,
        processor: RollingBufferProcessor,
        capture: IQCapture,
    ) -> dict:
        """Summarize movement in a capture before accepting a sound trigger."""
        timeline = processor.process_standard(capture)
        all_readings = timeline.readings
        all_outbound = [r for r in all_readings if r.is_outbound]
        all_inbound = [r for r in all_readings if not r.is_outbound]
        outbound_speeds = [r.speed_mph for r in all_outbound]
        inbound_speeds = [r.speed_mph for r in all_inbound]
        valid_outbound = [r for r in all_outbound if r.speed_mph >= self.MIN_VALID_OUTBOUND_MPH]

        return {
            "total_readings": len(all_readings),
            "outbound_readings": len(all_outbound),
            "inbound_readings": len(all_inbound),
            "peak_outbound_mph": max(outbound_speeds, default=0),
            "peak_inbound_mph": max(inbound_speeds, default=0),
            "all_outbound_speeds": outbound_speeds,
            "all_inbound_speeds": inbound_speeds,
            "peak_outbound_magnitude": max((r.magnitude for r in all_outbound), default=0),
            "peak_inbound_magnitude": max((r.magnitude for r in all_inbound), default=0),
            "valid_outbound_count": len(valid_outbound),
            "valid_peak_outbound_mph": max(
                (r.speed_mph for r in valid_outbound),
                default=0,
            ),
        }

    def _append_activity_diagnostic(
        self,
        summary: dict,
        *,
        accepted: bool,
        reason: str,
        response_bytes: int,
        trigger_latency_ms: Optional[float] = None,
    ):
        """Append a diagnostic entry using capture-activity summary fields."""
        self._append_diagnostic(
            accepted=accepted,
            reason=reason,
            response_bytes=response_bytes,
            total_readings=summary["total_readings"],
            outbound_readings=summary["outbound_readings"],
            inbound_readings=summary["inbound_readings"],
            peak_outbound_mph=summary["peak_outbound_mph"],
            peak_inbound_mph=summary["peak_inbound_mph"],
            all_outbound_speeds=summary["all_outbound_speeds"],
            all_inbound_speeds=summary["all_inbound_speeds"],
            peak_outbound_magnitude=summary["peak_outbound_magnitude"],
            peak_inbound_magnitude=summary["peak_inbound_magnitude"],
            trigger_latency_ms=trigger_latency_ms,
        )

    @abstractmethod
    def wait_for_trigger(
        self,
        radar: "OPS243Radar",
        processor: RollingBufferProcessor,
        timeout: float = 30.0,
    ) -> Optional[IQCapture]:
        """
        Wait for trigger condition and capture buffer.

        Args:
            radar: Connected OPS243Radar instance in rolling buffer mode
            processor: Processor for parsing capture response
            timeout: Maximum time to wait for trigger

        Returns:
            IQCapture if triggered and captured, None if timeout or error
        """
        pass

    def finish_capture(
        self,
        radar: "OPS243Radar",
        capture: IQCapture,
        *,
        sync_clock: bool,
    ) -> None:
        """Finish post-capture radar work for a capture returned by wait_for_trigger().

        The monitor calls this exactly once per returned capture, after it has
        shown the shot. ``sync_clock`` is True only for accepted shots. The
        default is a no-op: strategies whose wait_for_trigger() leaves the
        radar ready for the next capture have nothing deferred.
        """
        del radar, capture, sync_clock

    @abstractmethod
    def reset(self):
        """Reset trigger state for next capture."""
        pass


class SpeedTriggeredCapture(TriggerStrategy):
    """
    Speed-triggered rolling buffer capture per OmniPreSense recommendation.

    This implements the manufacturer's recommended approach for golf:
    1. Run in fast speed detection mode (~150-200Hz report rate)
    2. When outbound speed >20mph detected, immediately switch to rolling buffer
    3. Capture ball impact and flight with S#0 (no pre-trigger history)

    Advantages over polling:
    - Much faster trigger response (~5-6ms vs 300ms+ polling)
    - Captures club speed in speed mode, ball in rolling buffer
    - Minimal data loss during mode switch

    Per manufacturer:
    "You'll lose a little data (~5-6ms) from the initial club speed detection
    time while the sensor determines there's probably a golf swing event to
    capture with the Rolling Buffer. But assuming the club speed detected to
    ball impact is around 20-40ms, that should be ok."
    """

    def __init__(
        self,
        min_trigger_speed_mph: float = 20.0,
        min_ball_speed_mph: float = 35.0,
        trigger_to_capture_delay_ms: float = 15.0,
        pre_trigger_segments: int = 0,  # Speed trigger defaults to 0 (no pre-trigger)
    ):
        """
        Initialize speed-triggered capture.

        Args:
            min_trigger_speed_mph: Minimum speed to trigger capture (default 20mph)
            min_ball_speed_mph: Minimum ball speed to consider valid shot (default 35mph)
            trigger_to_capture_delay_ms: Delay after trigger before capture (default 15ms)
                This allows ball impact to happen before we dump the buffer.
            pre_trigger_segments: Pre-trigger segments (default 0 for speed trigger)
        """
        super().__init__(pre_trigger_segments=pre_trigger_segments)
        self.min_trigger_speed_mph = min_trigger_speed_mph
        self.min_ball_speed_mph = min_ball_speed_mph
        self.trigger_to_capture_delay_ms = trigger_to_capture_delay_ms
        self._last_trigger_speed: float = 0
        self._needs_reconfigure = True

    def wait_for_trigger(
        self,
        radar: "OPS243Radar",
        processor: RollingBufferProcessor,
        timeout: float = 30.0,
    ) -> Optional[IQCapture]:
        """
        Wait for speed trigger, switch to rolling buffer, and capture.

        Flow:
        1. Configure radar for fast speed detection (if needed)
        2. Poll for speed readings at ~150-200Hz
        3. When speed >= threshold detected:
           a. Record club speed
           b. Switch to rolling buffer mode (GC + S#0)
           c. Wait for ball impact (~15-25ms)
           d. Trigger capture (S!)
        4. Return to speed detection mode for next shot
        """
        # Configure for speed trigger mode if needed
        if self._needs_reconfigure:
            radar.configure_for_speed_trigger()
            self._needs_reconfigure = False
            # Clear any buffered data from mode switch
            if radar.serial:
                radar.serial.reset_input_buffer()
            time.sleep(0.1)

        start_time = time.time()
        logger.info(
            "[TRIGGER] Waiting for speed trigger >= %.1f mph...", self.min_trigger_speed_mph
        )

        while (time.time() - start_time) < timeout:
            # Non-blocking speed read
            reading = radar.read_speed_nonblocking()

            if reading and reading.speed >= self.min_trigger_speed_mph:
                # Speed detected - this is likely the club
                self._last_trigger_speed = reading.speed
                trigger_time = time.time()

                logger.info(
                    "[TRIGGER] Speed trigger: %.1f mph %s detected, switching to rolling buffer...",
                    reading.speed,
                    reading.direction.value,
                )

                # Immediately switch to rolling buffer mode
                radar.switch_to_rolling_buffer()

                # Wait for ball impact
                # Club to ball is typically 20-40ms, we wait a portion of that
                delay_sec = self.trigger_to_capture_delay_ms / 1000.0
                time.sleep(delay_sec)

                # Capture the rolling buffer
                response = radar.trigger_capture(timeout=5.0)
                capture = processor.parse_capture(response)

                # Calculate timing
                capture_time = time.time()
                total_delay_ms = (capture_time - trigger_time) * 1000
                logger.info("[TRIGGER] Buffer captured %.1fms after trigger", total_delay_ms)

                if capture:
                    # Validate capture has ball speed
                    timeline = processor.process_standard(capture)
                    outbound = [
                        r
                        for r in timeline.readings
                        if r.is_outbound and r.speed_mph >= self.min_ball_speed_mph
                    ]

                    if outbound:
                        peak = max(r.speed_mph for r in outbound)
                        logger.info("[TRIGGER] Ball detected: %.1f mph in capture", peak)

                        # Mark for reconfigure on next call
                        self._needs_reconfigure = True
                        return capture
                    else:
                        logger.info(
                            "[TRIGGER] No speed >= %.1f mph in capture", self.min_ball_speed_mph
                        )

                # Reconfigure for speed mode and continue
                self._needs_reconfigure = True
                radar.configure_for_speed_trigger()
                if radar.serial:
                    radar.serial.reset_input_buffer()

            # Brief sleep to avoid busy-waiting but stay responsive
            time.sleep(0.002)  # 2ms = 500Hz poll rate

        logger.info("[TRIGGER] Speed trigger timeout")
        return None

    def reset(self):
        """Reset trigger state and mark for reconfiguration."""
        self._last_trigger_speed = 0
        self._needs_reconfigure = True

    @property
    def last_trigger_speed(self) -> float:
        """Get the speed that triggered the last capture (likely club speed)."""
        return self._last_trigger_speed


class SoundTrigger(TriggerStrategy):
    """
    Hardware sound trigger using SparkFun SEN-14262.

    IMPORTANT: Rolling buffer mode must be configured BEFORE using this trigger.
    Call radar.configure_for_rolling_buffer() or radar.enter_rolling_buffer_mode()
    before calling wait_for_trigger().

    Wiring: SEN-14262 GATE → OPS243-A J3 Pin 3 (HOST_INT)
    The GATE output goes HIGH on loud sound (club impact).
    OPS243-A uses rising edge detection on HOST_INT as trigger.

    No software trigger (S!) needed — the radar triggers itself
    via hardware. We just need to wait for data to appear on serial.

    """

    CLOCK_SYNC_SAMPLES = 36
    CLOCK_SYNC_MAX_ROLLOVER_UNCERTAINTY_MS = 40.0
    CLOCK_SYNC_MAX_TIMEOUT_READ_MS = 50.0
    CLOCK_SYNC_MAX_FALLBACK_AGE_S = 60.0
    # After a failed re-arm, waits are shortened to this so the re-arm is
    # retried about once a second instead of after the full idle timeout.
    REARM_RETRY_INTERVAL_S = 1.0

    def __init__(
        self,
        pre_trigger_segments: int = 12,
    ):
        """
        Initialize sound trigger.

        Args:
            pre_trigger_segments: Number of pre-trigger segments for S# command.
                Each segment = 128 samples = ~4.27ms at 30ksps.
                Default 12 gives ~51ms pre-trigger, ~85ms post-trigger.
                NOTE: This is passed to enter_rolling_buffer_mode() by the caller.
                The trigger does NOT configure rolling buffer mode itself.
        """
        super().__init__(pre_trigger_segments=pre_trigger_segments)
        # Accepted capture handed to the caller whose clock sync + re-arm
        # are deferred to finish_capture(). While set, the radar is Idle.
        self._unfinished_capture: Optional[IQCapture] = None
        # A re-arm the radar would not take (write timeout). While set the
        # radar may be Idle (blue light on) and every wait retries it.
        self._rearm_pending = False

    def _rearm(self, radar: "OPS243Radar") -> bool:
        """Re-arm and remember a failure so the next wait retries it.

        ``rearm_rolling_buffer`` returns False when the port refused the
        commands; any other result (including None from older radars and
        test fakes) counts as success.
        """
        ok = radar.rearm_rolling_buffer(self.pre_trigger_segments) is not False
        if not ok:
            logger.warning(
                "[TRIGGER] Re-arm failed; retrying every %.0fs", self.REARM_RETRY_INTERVAL_S
            )
        self._rearm_pending = not ok
        return ok

    @staticmethod
    def _clock_sync_last_read_host_time(clock_sync: dict) -> Optional[float]:
        """Return the host time of the last C? read in a clock-sync summary."""
        reads = clock_sync.get("reads") or []
        if not reads or not isinstance(reads[-1], dict):
            return None
        return reads[-1].get("host_after") or reads[-1].get("host_mid")

    @classmethod
    def _clock_sync_age_s(cls, clock_sync: dict) -> Optional[float]:
        """Return age in seconds for a clock-sync summary."""
        last_host_time = cls._clock_sync_last_read_host_time(clock_sync)
        if last_host_time is None:
            return None
        try:
            return time.time() - float(last_host_time)
        except (TypeError, ValueError):
            return None

    @classmethod
    def _clock_sync_quality(cls, clock_sync: object) -> tuple[bool, str]:
        """Return whether a clock sync is trustworthy enough for shot timing."""
        if not isinstance(clock_sync, dict):
            return False, "missing"

        if not clock_sync.get("usable_for_trigger_timestamps"):
            return False, f"unusable_method:{clock_sync.get('clock_sync_method', 'unknown')}"

        if clock_sync.get("best_offset_s") is None:
            return False, "missing_best_offset"

        reads = clock_sync.get("reads") or []
        slow_invalid_reads = [
            read
            for read in reads
            if isinstance(read, dict)
            and read.get("radar_clock_s") is None
            and (read.get("read_latency_ms") or 0.0) >= cls.CLOCK_SYNC_MAX_TIMEOUT_READ_MS
        ]
        if slow_invalid_reads:
            return False, f"timeout_reads:{len(slow_invalid_reads)}"

        method = clock_sync.get("clock_sync_method")
        if method == "integer_rollover":
            uncertainty = clock_sync.get("rollover_uncertainty_ms")
            if uncertainty is None:
                return False, "missing_rollover_uncertainty"
            try:
                if float(uncertainty) > cls.CLOCK_SYNC_MAX_ROLLOVER_UNCERTAINTY_MS:
                    return False, f"rollover_uncertainty:{float(uncertainty):.1f}ms"
            except (TypeError, ValueError):
                return False, "invalid_rollover_uncertainty"
            return True, "valid_integer_rollover"

        if method == "fractional_clock":
            return True, "valid_fractional_clock"

        return False, f"unsupported_method:{method or 'unknown'}"

    @classmethod
    def _clock_sync_summary_for_log(cls, clock_sync: object) -> Optional[dict]:
        """Return a compact JSONL-safe summary of a clock-sync candidate."""
        if not isinstance(clock_sync, dict):
            return None
        valid, reason = cls._clock_sync_quality(clock_sync)
        return {
            "valid": valid,
            "reason": reason,
            "source": clock_sync.get("source"),
            "samples": clock_sync.get("samples"),
            "valid_samples": clock_sync.get("valid_samples"),
            "clock_sync_method": clock_sync.get("clock_sync_method"),
            "best_offset_s": clock_sync.get("best_offset_s"),
            "raw_best_offset_s": clock_sync.get("raw_best_offset_s"),
            "best_read_latency_ms": clock_sync.get("best_read_latency_ms"),
            "offset_spread_ms": clock_sync.get("offset_spread_ms"),
            "rollover_uncertainty_ms": clock_sync.get("rollover_uncertainty_ms"),
            "age_s": (
                round(cls._clock_sync_age_s(clock_sync), 3)
                if cls._clock_sync_age_s(clock_sync) is not None
                else None
            ),
        }

    def _select_clock_sync_for_capture(
        self,
        radar: "OPS243Radar",
        capture: IQCapture,
    ) -> Optional[dict]:
        """Choose and apply the best OPS clock sync for this capture."""
        previous_sync = getattr(radar, "last_clock_sync", None)
        previous_valid, previous_reason = self._clock_sync_quality(previous_sync)
        previous_age_s = (
            self._clock_sync_age_s(previous_sync) if isinstance(previous_sync, dict) else None
        )

        fresh_sync = None
        fresh_error = None
        if hasattr(radar, "read_clock_sync"):
            try:
                fresh_sync = radar.read_clock_sync(
                    samples=self.CLOCK_SYNC_SAMPLES,
                    store=False,
                )
                if isinstance(fresh_sync, dict):
                    fresh_sync["source"] = "per_shot"
            except Exception as exc:  # pylint: disable=broad-except
                fresh_error = str(exc)
                logger.warning("[TRIGGER] Per-shot OPS clock sync failed: %s", exc, exc_info=True)

        fresh_valid, fresh_reason = self._clock_sync_quality(fresh_sync)

        selected_sync = None
        selected_source = "first_byte"
        selected_reason = "no_valid_clock_sync"

        if fresh_valid and isinstance(fresh_sync, dict):
            selected_sync = fresh_sync
            selected_source = "fresh"
            selected_reason = fresh_reason
            radar.last_clock_sync = fresh_sync
        elif (
            previous_valid
            and isinstance(previous_sync, dict)
            and previous_age_s is not None
            and previous_age_s <= self.CLOCK_SYNC_MAX_FALLBACK_AGE_S
        ):
            selected_sync = previous_sync
            selected_source = "previous"
            selected_reason = f"fresh_rejected:{fresh_reason};previous_age:{previous_age_s:.1f}s"
        elif previous_valid and previous_age_s is not None:
            selected_reason = (
                f"fresh_rejected:{fresh_reason};previous_too_old:{previous_age_s:.1f}s"
            )
        elif previous_reason != "missing":
            selected_reason = f"fresh_rejected:{fresh_reason};previous_rejected:{previous_reason}"
        elif fresh_error:
            selected_reason = f"fresh_error:{fresh_error}"
        else:
            selected_reason = f"fresh_rejected:{fresh_reason}"

        selected_offset_s = None
        selected_age_s = None
        if isinstance(selected_sync, dict):
            selected_offset_s = selected_sync.get("best_offset_s")
            selected_age_s = self._clock_sync_age_s(selected_sync)
            try:
                capture.apply_trigger_timestamp_from_clock_sync(float(selected_offset_s))
            except (TypeError, ValueError):
                logger.warning(
                    "[TRIGGER] Ignoring invalid selected OPS clock-sync offset: %r",
                    selected_offset_s,
                )
                selected_sync = None
                selected_source = "first_byte"
                selected_reason = "selected_offset_invalid"
                selected_offset_s = None

        previous_offset_s = (
            previous_sync.get("best_offset_s") if isinstance(previous_sync, dict) else None
        )
        fresh_offset_s = fresh_sync.get("best_offset_s") if isinstance(fresh_sync, dict) else None

        selection_log = {
            "selection": selected_source,
            "selection_reason": selected_reason,
            "selected_offset_s": selected_offset_s,
            "selected_age_s": round(selected_age_s, 3) if selected_age_s is not None else None,
            "fresh": self._clock_sync_summary_for_log(fresh_sync),
            "previous": self._clock_sync_summary_for_log(previous_sync),
            "fresh_error": fresh_error,
            "fresh_delta_from_previous_ms": (
                round((fresh_offset_s - previous_offset_s) * 1000.0, 3)
                if fresh_offset_s is not None and previous_offset_s is not None
                else None
            ),
        }

        if selected_sync is not None:
            logger.info(
                "[TRIGGER] OPS clock sync selected: %s offset=%.6fs age=%sms "
                "(fresh=%s, previous=%s, delta=%sms)",
                selected_source,
                selected_offset_s,
                "n/a" if selected_age_s is None else f"{selected_age_s:.1f}",
                fresh_reason,
                previous_reason,
                "n/a"
                if selection_log["fresh_delta_from_previous_ms"] is None
                else f"{selection_log['fresh_delta_from_previous_ms']:.1f}",
            )
        else:
            logger.info(
                "[TRIGGER] OPS clock sync fallback to first-byte timing: %s",
                selected_reason,
            )

        return selection_log

    def wait_for_trigger(
        self,
        radar: "OPS243Radar",
        processor: RollingBufferProcessor,
        timeout: float = 30.0,
        cancel_event: Optional[threading.Event] = None,
        capture_started_callback: Optional[Callable[[], None]] = None,
    ) -> Optional[IQCapture]:
        """
        Wait for hardware sound trigger and capture buffer.

        PREREQUISITE: Rolling buffer mode must already be configured via
        radar.configure_for_rolling_buffer() or radar.enter_rolling_buffer_mode().

        Unlike other triggers, no S! command is sent. The radar's
        HOST_INT pin receives the trigger from the SEN-14262 GATE
        output, causing the radar to dump its rolling buffer automatically.
        We just block on serial read waiting for the I/Q data to arrive.
        """
        if self._unfinished_capture is not None:
            # A caller skipped finish_capture(): the radar is still Idle from
            # the last dump and would ignore every trigger. Re-arm now.
            logger.warning("[TRIGGER] Previous capture was never finished; re-arming")
            self._unfinished_capture = None
            self._rearm(radar)
        elif self._rearm_pending:
            self._rearm(radar)
        if self._rearm_pending:
            timeout = min(timeout, self.REARM_RETRY_INTERVAL_S)

        logger.info("[TRIGGER] Waiting for sound trigger (timeout=%.0fs)...", timeout)

        response = radar.wait_for_hardware_trigger(
            timeout=timeout,
            cancel_event=cancel_event,
            on_first_byte=capture_started_callback,
        )

        if not response:
            if cancel_event is not None and cancel_event.is_set():
                return None
            orphan_bytes = getattr(radar, "last_hardware_trigger_orphan_bytes", 0) or 0
            if orphan_bytes:
                # A dump whose header was lost (e.g. a net-impact trigger
                # between re-arm and this wait): the radar is Idle with the
                # blue light on. Re-arm now rather than at the watchdog.
                logger.warning(
                    "[TRIGGER] Orphaned dump (%d bytes, no header); re-arming now",
                    orphan_bytes,
                )
                self._append_diagnostic(
                    accepted=False, reason="orphan_dump", response_bytes=orphan_bytes
                )
            else:
                # Watchdog: a radar left Idle for an unexplained reason can
                # never dump again, so every idle timeout re-arms. PA on an
                # already-armed radar just restarts sampling.
                logger.info(
                    "[TRIGGER] Sound trigger timeout — no hardware trigger received; re-arming"
                )
            self._rearm(radar)
            return None

        dump_done = time.time()
        stage_timings: dict = {}
        first_byte_timestamp = getattr(radar, "last_hardware_trigger_first_byte_timestamp", None)
        if first_byte_timestamp is not None:
            stage_timings["dump_ms"] = max(0.0, (dump_done - first_byte_timestamp) * 1000.0)

        # Once a dump has been read the radar is Idle until re-armed. Rejected
        # and failed dumps re-arm right here, and any exception still re-arms
        # (field symptom otherwise: blue light stuck on, no more hits read).
        # Accepted captures are handed back un-synced and un-armed so the
        # caller can show the shot first; finish_capture() does the rest.
        capture = None
        try:
            capture = self._handle_dump(processor, response, first_byte_timestamp, stage_timings)
            return capture
        finally:
            if capture is not None:
                self._unfinished_capture = capture
            else:
                self._rearm(radar)

    def finish_capture(
        self,
        radar: "OPS243Radar",
        capture: IQCapture,
        *,
        sync_clock: bool,
    ) -> None:
        """Clock-sync (accepted shots only) and re-arm after the shot is shown.

        ORDERING MATTERS: after its dump the radar sits idle, where HOST_INT
        pulses are harmless. A real shot produces a second loud sound ~1s
        later (ball hitting the net); if we re-arm first, that sound starts a
        new dump exactly when the clock-sync exchange writes to the port — a
        two-way serial deadlock observed in the field (capture thread wedged
        in serial.write, radar frozen mid-dump). So: do all wire-talk (clock
        sync) while idle, and re-arm LAST, even if the sync raises.

        Idempotent: only the capture most recently returned by
        wait_for_trigger() is finished, and only once.
        """
        if capture is None or capture is not self._unfinished_capture:
            return
        self._unfinished_capture = None
        timings = capture.stage_timings_ms
        try:
            if sync_clock:
                sync_start = time.time()
                try:
                    self._select_clock_sync_for_capture(radar, capture)
                finally:
                    timings["clock_sync_ms"] = (time.time() - sync_start) * 1000.0
                self._log_trigger_wall_time(capture)
        finally:
            rearm_start = time.time()
            try:
                self._rearm(radar)
            finally:
                timings["rearm_ms"] = (time.time() - rearm_start) * 1000.0

    @staticmethod
    def _log_trigger_wall_time(capture: IQCapture) -> None:
        if capture.trigger_timestamp is not None and capture.first_byte_timestamp is not None:
            logger.info(
                "[TRIGGER] Sound trigger wall time %.3f "
                "(source=%s, first byte %.3f, post-trigger %.1fms)",
                capture.trigger_timestamp,
                capture.trigger_timestamp_source or "unknown",
                capture.first_byte_timestamp,
                capture.post_trigger_duration_ms,
            )

    def _handle_dump(
        self,
        processor: RollingBufferProcessor,
        response: str,
        first_byte_timestamp: Optional[float],
        stage_timings: dict,
    ) -> Optional[IQCapture]:
        """Parse and validate a hardware-triggered dump. No serial I/O."""
        response_len = len(response)
        logger.info("[TRIGGER] Sound trigger fired, %d bytes received", response_len)

        parse_start = time.time()
        capture = processor.parse_capture(
            response,
            first_byte_timestamp=first_byte_timestamp,
        )
        stage_timings["parse_ms"] = (time.time() - parse_start) * 1000.0

        if not capture:
            logger.warning("[TRIGGER] Sound trigger parse failed (%d bytes received)", response_len)
            self._append_diagnostic(
                accepted=False,
                reason="parse_failed",
                response_bytes=response_len,
            )
            return None

        if first_byte_timestamp is not None and capture.first_byte_timestamp is None:
            capture.first_byte_timestamp = float(first_byte_timestamp)
        # First-byte timing now, so the early shot preview has an impact
        # time; finish_capture() refines it from the OPS clock when usable.
        if capture.first_byte_timestamp is not None and capture.trigger_timestamp is None:
            capture.apply_trigger_timestamp_from_first_byte()

        # Quick validation: does the capture contain any real swing data?
        # At a driving range, a nearby player's impact sound can trip the
        # trigger even though nothing was moving in front of our radar.
        # Discard these false triggers immediately (no clock sync) so the
        # caller re-arms fast and the next real swing isn't missed.
        check_start = time.time()
        summary = self._summarize_capture_activity(processor, capture)
        stage_timings["activity_check_ms"] = (time.time() - check_start) * 1000.0
        capture.stage_timings_ms.update(stage_timings)

        if not summary["valid_outbound_count"]:
            logger.info(
                "[TRIGGER] Sound trigger rejected — no outbound speed >= %.0f mph "
                "(peak=%.1f mph, %d readings)",
                self.MIN_VALID_OUTBOUND_MPH,
                summary["peak_outbound_mph"],
                summary["total_readings"],
            )
            self._append_activity_diagnostic(
                summary,
                accepted=False,
                reason="no_outbound_speed",
                response_bytes=response_len,
            )
            return None

        logger.info(
            "[TRIGGER] Sound trigger accepted — peak %.1f mph, %d outbound readings",
            summary["valid_peak_outbound_mph"],
            summary["valid_outbound_count"],
        )
        self._append_activity_diagnostic(
            summary,
            accepted=True,
            reason="accepted",
            response_bytes=response_len,
        )

        return capture

    def reset(self):
        """Reset trigger state."""
        pass  # No state to reset


def create_trigger(trigger_type: str = "sound", **kwargs) -> TriggerStrategy:
    """
    Factory function to create trigger strategy.

    Args:
        trigger_type: "sound" (production) or "speed" (fallback)
        **kwargs: Arguments passed to trigger constructor

    Returns:
        Configured TriggerStrategy instance

    Trigger types:
        - "sound": Hardware sound trigger via SparkFun SEN-14262 GATE → HOST_INT.
                   Requires GATE voltage to reach 3.3V threshold.
        - "speed": Fast speed detection triggers rolling buffer capture.
                   Recommended fallback by OmniPreSense. ~5-6ms response time.
    """
    triggers = {
        "speed": SpeedTriggeredCapture,
        "sound": SoundTrigger,
    }

    if trigger_type not in triggers:
        raise ValueError(
            f"Unknown trigger type: {trigger_type}. Available: {list(triggers.keys())}"
        )

    return triggers[trigger_type](**kwargs)
