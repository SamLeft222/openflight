"""Regression tests: the sound trigger must never leave the radar un-armed.

Field symptom: after a hit the OPS243's blue dump light stays on and no
further hits are read. After a hardware-triggered dump the radar sits in
Idle until it receives PA; if the software never re-arms it, HOST_INT
pulses are ignored forever.

Paths that used to leave the radar idle:
1. A re-arm that failed (write timeout) or a dump that arrived without a
   recognizable start marker. The next wait just timed out every 30s
   without ever re-arming, so the system never recovered.
2. An exception between reading the dump and re-arming (activity summary,
   clock-sync selection) skipped the re-arm entirely.

Accepted captures are returned *un-synced and un-armed* so the monitor can
show the shot before the slow clock-sync exchange; the monitor then calls
finish_capture(), which clock-syncs (radar still idle) and re-arms last.
Rejected and failed dumps are re-armed inside wait_for_trigger().
"""

import json
import threading
import time
from unittest.mock import patch

import pytest
from spin_synth import synth_capture

from openflight.rolling_buffer.processor import RollingBufferProcessor
from openflight.rolling_buffer.trigger import SoundTrigger


def _dump_response(i_samples, q_samples) -> str:
    return "\n".join(
        [
            '{"sample_time": "964.003"}',
            '{"trigger_time": "964.105"}',
            json.dumps({"I": [int(v) for v in i_samples]}),
            json.dumps({"Q": [int(v) for v in q_samples]}),
        ]
    )


def _swing_response() -> str:
    i_samples, q_samples = synth_capture(rpm=3000, ball_speed_mph=80.0, amplitude=400.0)
    return _dump_response(i_samples, q_samples)


class ScriptedRadar:
    """Mock radar recording the order of serial-touching calls."""

    def __init__(self, response: str = ""):
        self.response = response
        self.calls = []
        self.last_clock_sync = None
        self.last_hardware_trigger_first_byte_timestamp = None

    def wait_for_hardware_trigger(self, timeout, cancel_event=None, on_first_byte=None):
        self.calls.append("wait")
        return self.response

    def rearm_rolling_buffer(self, pre_trigger_segments):
        self.calls.append("rearm")
        return True

    def read_clock_sync(self, samples=7, store=True, **kwargs):
        self.calls.append("clock_sync")
        return {"clock_sync_method": "no_valid_reads", "valid_samples": 0}


def test_idle_timeout_rearms_radar_as_watchdog():
    """A wait that times out must re-arm, so an idle radar can recover."""
    radar = ScriptedRadar(response="")
    trigger = SoundTrigger()

    capture = trigger.wait_for_trigger(radar, RollingBufferProcessor(), timeout=0.1)

    assert capture is None
    assert radar.calls == ["wait", "rearm"], (
        "an idle timeout must re-arm the radar; otherwise a missed re-arm "
        f"leaves the blue dump light on forever. Calls: {radar.calls}"
    )


def test_cancelled_wait_does_not_rearm():
    """Shutdown cancels the idle wait; don't talk to the radar on the way out."""
    radar = ScriptedRadar(response="")
    cancel = threading.Event()
    cancel.set()
    trigger = SoundTrigger()

    capture = trigger.wait_for_trigger(
        radar, RollingBufferProcessor(), timeout=0.1, cancel_event=cancel
    )

    assert capture is None
    assert "rearm" not in radar.calls


def test_summary_exception_still_rearms():
    radar = ScriptedRadar(response=_swing_response())
    trigger = SoundTrigger()

    with patch.object(
        SoundTrigger, "_summarize_capture_activity", side_effect=RuntimeError("boom")
    ):
        with pytest.raises(RuntimeError):
            trigger.wait_for_trigger(radar, RollingBufferProcessor(), timeout=1.0)

    assert radar.calls.count("rearm") == 1, (
        f"an exception after the dump must still re-arm exactly once; got {radar.calls}"
    )


def test_clock_sync_selection_exception_still_rearms_after_sync():
    radar = ScriptedRadar(response=_swing_response())
    trigger = SoundTrigger()
    capture = trigger.wait_for_trigger(radar, RollingBufferProcessor(), timeout=1.0)
    assert capture is not None

    # First call grades the (missing) previous sync; the second, grading the
    # fresh sync read from the radar, blows up mid-selection.
    with patch.object(
        SoundTrigger,
        "_clock_sync_quality",
        side_effect=[(False, "missing"), RuntimeError("boom")],
    ):
        with pytest.raises(RuntimeError):
            trigger.finish_capture(radar, capture, sync_clock=True)

    assert radar.calls.count("rearm") == 1, radar.calls
    assert radar.calls.index("clock_sync") < radar.calls.index("rearm"), (
        f"re-arm must still come after clock sync; got {radar.calls}"
    )


def test_parse_exception_still_rearms():
    radar = ScriptedRadar(response=_swing_response())
    processor = RollingBufferProcessor()
    trigger = SoundTrigger()

    with patch.object(processor, "parse_capture", side_effect=ValueError("bad dump")):
        with pytest.raises(ValueError):
            trigger.wait_for_trigger(radar, processor, timeout=1.0)

    assert radar.calls.count("rearm") == 1, radar.calls


def test_rejected_capture_rearms_inside_wait():
    i_samples, q_samples = synth_capture(rpm=3000, amplitude=0.0, noise_rms=1.0)
    radar = ScriptedRadar(response=_dump_response(i_samples, q_samples))
    trigger = SoundTrigger()

    capture = trigger.wait_for_trigger(radar, RollingBufferProcessor(), timeout=1.0)

    assert capture is None
    assert radar.calls == ["wait", "rearm"], radar.calls


def test_accepted_capture_defers_clock_sync_and_rearm():
    """The monitor must be able to show the shot before the slow serial work."""
    radar = ScriptedRadar(response=_swing_response())
    trigger = SoundTrigger()

    capture = trigger.wait_for_trigger(radar, RollingBufferProcessor(), timeout=1.0)

    assert capture is not None
    assert radar.calls == ["wait"], (
        f"accepted capture must return before clock sync / re-arm; got {radar.calls}"
    )


def test_finish_capture_syncs_then_rearms_once():
    radar = ScriptedRadar(response=_swing_response())
    trigger = SoundTrigger()
    capture = trigger.wait_for_trigger(radar, RollingBufferProcessor(), timeout=1.0)

    trigger.finish_capture(radar, capture, sync_clock=True)
    trigger.finish_capture(radar, capture, sync_clock=True)  # idempotent

    assert radar.calls == ["wait", "clock_sync", "rearm"], radar.calls


def test_finish_capture_without_sync_only_rearms():
    """Processing/validation failures re-arm fast; clock sync is for real shots."""
    radar = ScriptedRadar(response=_swing_response())
    trigger = SoundTrigger()
    capture = trigger.wait_for_trigger(radar, RollingBufferProcessor(), timeout=1.0)

    trigger.finish_capture(radar, capture, sync_clock=False)

    assert radar.calls == ["wait", "rearm"], radar.calls


def test_finish_capture_ignores_captures_it_does_not_own():
    """A stale/foreign capture must not trigger an extra re-arm."""
    radar = ScriptedRadar(response=_swing_response())
    trigger = SoundTrigger()
    capture = trigger.wait_for_trigger(radar, RollingBufferProcessor(), timeout=1.0)
    trigger.finish_capture(radar, capture, sync_clock=False)

    trigger.finish_capture(radar, capture, sync_clock=True)

    assert radar.calls == ["wait", "rearm"], radar.calls


def test_next_wait_rearms_if_previous_capture_was_never_finished():
    """Defensive: a caller that forgets finish_capture must not wedge the radar."""
    radar = ScriptedRadar(response=_swing_response())
    trigger = SoundTrigger()
    trigger.wait_for_trigger(radar, RollingBufferProcessor(), timeout=1.0)

    radar.response = ""
    trigger.wait_for_trigger(radar, RollingBufferProcessor(), timeout=0.1)

    assert radar.calls == ["wait", "rearm", "wait", "rearm"], radar.calls


def test_stage_timings_recorded_on_capture():
    radar = ScriptedRadar(response=_swing_response())
    radar.last_hardware_trigger_first_byte_timestamp = time.time() - 1.5
    trigger = SoundTrigger()

    capture = trigger.wait_for_trigger(radar, RollingBufferProcessor(), timeout=1.0)
    trigger.finish_capture(radar, capture, sync_clock=True)

    timings = capture.stage_timings_ms
    for key in ("dump_ms", "parse_ms", "activity_check_ms", "clock_sync_ms", "rearm_ms"):
        assert key in timings, f"missing {key}: {timings}"
        assert timings[key] >= 0.0
    assert timings["dump_ms"] == pytest.approx(1500.0, abs=250.0)
