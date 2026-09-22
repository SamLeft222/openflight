"""The monitor shows a shot before the slow post-capture serial work.

Shot latency, sound trigger: after the dump arrives, the OPS clock sync
(~36 C? round trips, up to ~2s) and the re-arm (>=0.55s of fixed sleeps) used
to run BEFORE the shot was even processed. Neither is needed to display ball
speed / spin / carry, so the monitor now:

1. processes the capture and calls shot_preview_callback (UI shows the shot);
2. calls trigger.finish_capture(sync_clock=True) — clock sync, then re-arm;
3. refreshes the shot's impact timestamps from the synced capture;
4. calls shot_callback (IWR6843 / camera matching use the refined time).

Every path that got a capture must finish it exactly once, so the radar is
always re-armed.
"""

import json
from unittest.mock import MagicMock

import pytest
from spin_synth import synth_capture

from openflight.rolling_buffer import RollingBufferMonitor, monitor as monitor_module
from openflight.rolling_buffer.processor import RollingBufferProcessor

FIRST_BYTE_EPOCH = 1_000_000.0
REFINED_TRIGGER_EPOCH = 999_998.25


def _swing_capture():
    i_samples, q_samples = synth_capture(rpm=3000, ball_speed_mph=140.0, amplitude=400.0)
    response = "\n".join(
        [
            '{"sample_time": "964.003"}',
            '{"trigger_time": "964.105"}',
            json.dumps({"I": [int(v) for v in i_samples]}),
            json.dumps({"Q": [int(v) for v in q_samples]}),
        ]
    )
    capture = RollingBufferProcessor().parse_capture(
        response, first_byte_timestamp=FIRST_BYTE_EPOCH
    )
    assert capture is not None
    return capture


class FakeTrigger:
    """Hands out one capture, records finish_capture calls into `events`."""

    def __init__(self, monitor, capture, events, finish_error=None):
        self.monitor = monitor
        self.capture = capture
        self.events = events
        self.finish_error = finish_error
        self.finish_calls = []

    def wait_for_trigger(self, **_kwargs):
        if self.capture is None:
            self.monitor._running = False
            return None
        capture, self.capture = self.capture, None
        return capture

    def finish_capture(self, radar, capture, *, sync_clock):
        self.finish_calls.append(sync_clock)
        self.events.append(("finish", sync_clock))
        if self.finish_error is not None:
            raise self.finish_error
        if sync_clock:
            capture.apply_trigger_timestamp_from_clock_sync(
                REFINED_TRIGGER_EPOCH - capture.trigger_time
            )
            capture.stage_timings_ms["clock_sync_ms"] = 900.0
            capture.stage_timings_ms["rearm_ms"] = 550.0

    @staticmethod
    def drain_diagnostics():
        return [{"accepted": True, "reason": "accepted", "response_bytes": 40000}]

    @staticmethod
    def reset():
        return None


@pytest.fixture
def session_logger(monkeypatch):
    session_log = MagicMock()
    monkeypatch.setattr(monitor_module, "get_session_logger", lambda: session_log)
    monkeypatch.setattr(monitor_module.time, "sleep", lambda _delay: None)
    return session_log


def _monitor(capture, events, **trigger_kwargs):
    monitor = RollingBufferMonitor(port=None, trigger_type="sound")
    monitor.trigger = FakeTrigger(monitor, capture, events, **trigger_kwargs)
    monitor._diagnostic_callback = None
    monitor._running = True
    return monitor


def test_preview_runs_before_clock_sync_and_final_callback_after(session_logger):
    events = []
    capture = _swing_capture()
    monitor = _monitor(capture, events)
    preview_impacts = []

    def on_preview(shot):
        events.append(("preview", shot))
        preview_impacts.append(shot.impact_timestamp)

    monitor._shot_preview_callback = on_preview
    monitor._shot_callback = lambda shot: events.append(("shot", shot))

    monitor._capture_loop()

    assert [name for name, _ in events] == ["preview", "finish", "shot"], events
    assert events[1] == ("finish", True)
    preview_shot, final_shot = events[0][1], events[2][1]
    assert preview_shot is final_shot, "preview and final must be the same Shot"
    # Preview uses first-byte timing; the final shot carries the synced time.
    assert preview_impacts[0] == pytest.approx(
        FIRST_BYTE_EPOCH - capture.post_trigger_duration_ms / 1000.0
    )
    assert final_shot.impact_timestamp == pytest.approx(REFINED_TRIGGER_EPOCH)
    assert final_shot.impact_timestamp_kld7 is not None
    assert final_shot.impact_timestamp_kld7 == pytest.approx(REFINED_TRIGGER_EPOCH, abs=0.2)


def test_session_log_records_synced_timing_and_stage_timings(session_logger):
    events = []
    capture = _swing_capture()
    monitor = _monitor(capture, events)
    monitor._shot_preview_callback = lambda shot: None
    monitor._shot_callback = lambda shot: None

    monitor._capture_loop()

    logged = session_logger.log_rolling_buffer_capture.call_args.kwargs
    assert logged["trigger_timestamp"] == pytest.approx(REFINED_TRIGGER_EPOCH)
    assert logged["trigger_timestamp_source"] == "ops_clock_sync"
    timings = logged["stage_timings_ms"]
    for key in ("process_ms", "impact_to_preview_ms", "clock_sync_ms", "rearm_ms"):
        assert key in timings, f"missing {key}: {timings}"


def test_without_preview_callback_final_callback_still_follows_sync(session_logger):
    events = []
    monitor = _monitor(_swing_capture(), events)
    monitor._shot_callback = lambda shot: events.append(("shot", shot))

    monitor._capture_loop()

    assert [name for name, _ in events] == ["finish", "shot"], events


def test_preview_failure_does_not_block_sync_or_final_callback(session_logger):
    events = []
    monitor = _monitor(_swing_capture(), events)

    def exploding_preview(_shot):
        raise RuntimeError("socket gone")

    monitor._shot_preview_callback = exploding_preview
    monitor._shot_callback = lambda shot: events.append(("shot", shot))

    monitor._capture_loop()

    assert [name for name, _ in events] == ["finish", "shot"], events
    assert events[0] == ("finish", True)


def test_finish_failure_after_preview_still_delivers_final_shot(session_logger):
    """The UI already shows the shot; the server must still finalize it."""
    events = []
    capture = _swing_capture()
    monitor = _monitor(capture, events, finish_error=OSError("serial gone"))
    monitor._shot_preview_callback = lambda shot: events.append(("preview", shot))
    monitor._shot_callback = lambda shot: events.append(("shot", shot))

    monitor._capture_loop()

    assert [name for name, _ in events] == ["preview", "finish", "shot"], events
    assert monitor.trigger.finish_calls == [True]
    # Falls back to first-byte timing.
    assert events[2][1].impact_timestamp == pytest.approx(
        FIRST_BYTE_EPOCH - capture.post_trigger_duration_ms / 1000.0
    )


def test_processing_failure_finishes_without_clock_sync(session_logger, monkeypatch):
    events = []
    monitor = _monitor(_swing_capture(), events)
    monkeypatch.setattr(monitor.processor, "process_capture", lambda *_a, **_k: None)
    monitor._shot_preview_callback = lambda shot: events.append(("preview", shot))
    monitor._shot_callback = lambda shot: events.append(("shot", shot))

    monitor._capture_loop()

    assert monitor.trigger.finish_calls == [False]
    assert events == [("finish", False)]


def test_processing_exception_finishes_without_clock_sync(session_logger, monkeypatch):
    events = []
    monitor = _monitor(_swing_capture(), events)

    def explode(*_args, **_kwargs):
        raise RuntimeError("FFT failed")

    monkeypatch.setattr(monitor.processor, "process_capture", explode)
    monitor._shot_callback = lambda shot: events.append(("shot", shot))

    monitor._capture_loop()

    assert monitor.trigger.finish_calls == [False]


def test_validation_failure_finishes_without_clock_sync(session_logger, monkeypatch):
    events = []
    monitor = _monitor(_swing_capture(), events)
    monkeypatch.setattr(monitor, "_create_shot", lambda _processed: None)
    monitor._shot_callback = lambda shot: events.append(("shot", shot))

    monitor._capture_loop()

    assert monitor.trigger.finish_calls == [False]
    assert events == [("finish", False)]


def test_final_callback_exception_does_not_double_finish(session_logger):
    events = []
    monitor = _monitor(_swing_capture(), events)

    def exploding_callback(_shot):
        raise RuntimeError("server blew up")

    monitor._shot_callback = exploding_callback

    monitor._capture_loop()

    assert monitor.trigger.finish_calls == [True]


def test_start_registers_preview_callback(monkeypatch):
    monitor = RollingBufferMonitor(port=None, trigger_type="sound")
    monkeypatch.setattr(monitor, "_capture_loop", lambda: None)

    def preview(_shot):
        return None

    monitor.start(shot_callback=lambda _s: None, shot_preview_callback=preview)
    monitor.stop()

    assert monitor._shot_preview_callback is preview
