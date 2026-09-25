"""Capture monitor in reduced-transfer mode (plans/iwr6843-on-chip-reduction.md).

On a trigger edge the monitor fetches the overview instead of the full dump.
The radar holds its frozen ring until the runtime has fetched its strips and
finishes the capture -- by releasing it, or by streaming the held ring whole.
Nothing may leave the radar frozen: an unconsumed hold is released after
``hold_budget_s``, and any overview failure falls back to an ordinary dump.
"""

from __future__ import annotations

import threading
import time

import pytest
from test_iwr6843_monitor import FakeButton
from test_iwr6843_pipeline import synth_shot
from test_iwr6843_reduced import GATE, N_FRAMES, PERIOD_US, _timed_iq8

from openflight.iwr6843 import reduced
from openflight.iwr6843.monitor import IWR6843CaptureMonitor

CAPTURE = _timed_iq8(
    synth_shot(n_frames=N_FRAMES, n_loops=12, n_tx=3, frame_period_us=PERIOD_US, trigger_frame=0)
)
REQUEST = (None,) * 6 + ((62, 4),) * 6


class FakeReducedRadar:
    """The reduced-transfer firmware, emulated with the Python reference."""

    port = "/dev/fake-iwr6843"

    def __init__(self, *, overview_error: Exception | None = None):
        self.overview_error = overview_error
        self.calls: list[str] = []
        self.held = False
        self.gate = None
        self.closed = False

    def send_config(self, path):
        self.calls.append("config")

    def configure_overview_gate(self, gate):
        self.calls.append(f"gate {gate[0]} {gate[1]}")
        self.gate = gate

    def read_overview(self):
        self.calls.append("overview")
        if self.overview_error is not None:
            raise self.overview_error
        self.held = True
        return reduced.pack_overview(reduced.build_overview(CAPTURE, gate=self.gate))

    def read_strips(self, request):
        self.calls.append("strips")
        if not self.held:
            raise RuntimeError("IWR6843 l3strip failed: Error: no held capture")
        return reduced.serve_strips(CAPTURE, request)

    def release(self):
        self.calls.append("release")
        self.held = False

    def read_dump(self):
        self.calls.append("dump")
        self.held = False
        return CAPTURE

    def stop_sensor(self):
        self.calls.append("sensorStop")

    def close(self):
        self.closed = True


def _monitor(tmp_path, radar, **kwargs):
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    monitor = IWR6843CaptureMonitor(
        config_path=config,
        output_dir=tmp_path / "dumps",
        radar=radar,
        button_factory=FakeButton,
        reduced_gate=GATE,
        **kwargs,
    )
    monitor.start()
    return monitor


def _capture(monitor):
    edge = time.time()
    assert monitor.notify_trigger(edge)
    capture = monitor.capture_for_shot(edge, timeout_s=2.0)
    assert capture is not None and capture.valid
    return capture


def test_configures_the_gate_and_delivers_an_overview(tmp_path):
    radar = FakeReducedRadar()
    monitor = _monitor(tmp_path, radar)

    capture = _capture(monitor)

    assert radar.calls[:3] == ["config", f"gate {GATE[0]} {GATE[1]}", "overview"]
    assert capture.transfer == "reduced"
    assert capture.raw is None
    assert reduced.parse_overview(capture.overview).gate == GATE
    assert radar.held
    monitor.finish_capture(capture, full_dump=False)
    monitor.stop()


def test_strips_come_from_the_held_ring_until_it_is_released(tmp_path):
    radar = FakeReducedRadar()
    monitor = _monitor(tmp_path, radar)
    capture = _capture(monitor)

    assert monitor.read_strips(capture, REQUEST) == reduced.serve_strips(CAPTURE, REQUEST)
    assert monitor.finish_capture(capture, full_dump=False) is None
    assert radar.calls[-1] == "release" and not radar.held
    with pytest.raises(RuntimeError, match="no longer held"):
        monitor.read_strips(capture, REQUEST)
    monitor.stop()


def test_finishing_twice_touches_the_radar_once(tmp_path):
    radar = FakeReducedRadar()
    monitor = _monitor(tmp_path, radar)
    capture = _capture(monitor)

    monitor.finish_capture(capture, full_dump=False)
    monitor.finish_capture(capture, full_dump=False)

    assert radar.calls.count("release") == 1
    monitor.stop()


def test_full_dump_streams_the_held_ring_and_saves_it(tmp_path):
    radar = FakeReducedRadar()
    monitor = _monitor(tmp_path, radar, save_dumps=True)
    capture = _capture(monitor)

    raw = monitor.finish_capture(capture, full_dump=True)

    assert raw == CAPTURE
    assert "release" not in radar.calls  # l3dump resumes the ring itself
    saved = sorted((tmp_path / "dumps").glob("*.l3dump"))
    assert len(saved) == 1 and saved[0].read_bytes() == CAPTURE
    assert sorted((tmp_path / "dumps").glob("*.overview"))[0].read_bytes() == capture.overview
    monitor.stop()


def test_a_held_capture_blocks_new_edges_until_it_is_finished(tmp_path):
    radar = FakeReducedRadar()
    monitor = _monitor(tmp_path, radar)
    capture = _capture(monitor)

    assert not monitor.notify_trigger(time.time() + 0.5)
    monitor.finish_capture(capture, full_dump=False)
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline and not monitor.notify_trigger(time.time() + 1.0):
        time.sleep(0.01)
    while time.monotonic() < deadline and radar.calls.count("overview") < 2:
        time.sleep(0.01)  # the capture thread fetches it asynchronously

    assert radar.calls.count("overview") == 2
    monitor.stop()


def test_an_unconsumed_hold_is_released_after_the_budget(tmp_path):
    """The OPS side may never ask for a capture; the radar must not stay frozen."""
    radar = FakeReducedRadar()
    monitor = _monitor(tmp_path, radar, hold_budget_s=0.2)
    edge = time.time()
    assert monitor.notify_trigger(edge)

    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and "release" not in radar.calls:
        time.sleep(0.01)

    assert radar.calls[-1] == "release" and not radar.held
    capture = monitor.capture_for_shot(edge, timeout_s=0.5)
    with pytest.raises(RuntimeError, match="no longer held"):
        monitor.read_strips(capture, REQUEST)
    assert monitor.finish_capture(capture, full_dump=False) is None
    monitor.stop()


def test_overview_failure_falls_back_to_an_ordinary_dump(tmp_path):
    radar = FakeReducedRadar(overview_error=RuntimeError("IWR6843 l3overview sent no response"))
    monitor = _monitor(tmp_path, radar)

    capture = _capture(monitor)

    assert capture.transfer == "full"
    assert capture.raw == CAPTURE
    assert "no response" in capture.fallback_reason
    assert radar.calls[2:5] == ["overview", "release", "dump"]
    monitor.stop()


def test_stop_releases_a_held_capture_before_stopping_the_sensor(tmp_path):
    radar = FakeReducedRadar()
    monitor = _monitor(tmp_path, radar, hold_budget_s=30.0)
    _capture(monitor)
    stopper = threading.Thread(target=monitor.stop)

    stopper.start()
    stopper.join(timeout=3.0)

    assert not stopper.is_alive()
    assert radar.calls[-2:] == ["release", "sensorStop"]
    assert radar.closed


def test_full_transfer_mode_is_unchanged(tmp_path):
    radar = FakeReducedRadar()
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    monitor = IWR6843CaptureMonitor(
        config_path=config, output_dir=tmp_path / "dumps", radar=radar, button_factory=FakeButton
    )
    monitor.start()

    capture = _capture(monitor)

    assert capture.transfer == "full" and capture.raw == CAPTURE
    assert capture.fallback_reason is None
    assert radar.calls == ["config", "dump"]
    assert monitor.finish_capture(capture, full_dump=False) == CAPTURE
    monitor.stop()


def test_firmware_without_reduced_transfer_falls_back_to_full_dumps(tmp_path):
    """The released firmware rejects overviewCfg; the IWR must still work."""

    class OldFirmware(FakeReducedRadar):
        def configure_overview_gate(self, gate):
            self.calls.append("gate")
            raise RuntimeError("config rejected: 'overviewCfg 48 98': Error: unknown command")

    radar = OldFirmware()
    monitor = _monitor(tmp_path, radar)

    capture = _capture(monitor)

    assert monitor.reduced_gate is None
    assert capture.transfer == "full" and capture.raw == CAPTURE
    assert radar.calls == ["config", "gate", "dump"]
    monitor.stop()


# --- The OPS found no shot for a sound: free its capture at once -------------


def _wait_for(predicate, timeout_s: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return predicate()


def _held(monitor, radar) -> float:
    """Trigger an edge and wait until its overview holds the ring."""
    edge = time.time()
    assert monitor.notify_trigger(edge)
    assert _wait_for(lambda: radar.held)
    return edge


def test_ops_rejection_releases_a_held_capture_well_before_the_budget(tmp_path):
    radar = FakeReducedRadar()
    monitor = _monitor(tmp_path, radar, hold_budget_s=30.0)
    edge = _held(monitor, radar)

    assert monitor.discard_trigger(edge + 0.07)  # OPS first-byte estimate, ~70 ms late

    assert _wait_for(lambda: not radar.held)
    assert radar.calls[-1] == "release"
    assert monitor.capture_for_shot(edge, timeout_s=0.2) is None
    assert _wait_for(lambda: monitor.notify_trigger(time.time()))  # radar free again
    monitor.stop()


def test_ops_rejection_before_the_overview_lands_releases_it_on_arrival(tmp_path):
    arrived = threading.Event()
    proceed = threading.Event()

    class SlowOverview(FakeReducedRadar):
        def read_overview(self):
            arrived.set()
            proceed.wait(2.0)
            return super().read_overview()

    radar = SlowOverview()
    monitor = _monitor(tmp_path, radar, hold_budget_s=30.0)
    edge = time.time()
    assert monitor.notify_trigger(edge)
    assert arrived.wait(2.0)

    monitor.discard_trigger(edge)
    proceed.set()

    assert _wait_for(lambda: "release" in radar.calls and not radar.held)
    assert monitor.capture_for_shot(edge, timeout_s=0.2) is None
    monitor.stop()


def test_rejection_of_a_different_sound_leaves_the_capture_alone(tmp_path):
    radar = FakeReducedRadar()
    monitor = _monitor(tmp_path, radar, hold_budget_s=30.0)
    edge = _held(monitor, radar)

    assert not monitor.discard_trigger(edge - 5.0)

    time.sleep(0.1)
    assert radar.held
    capture = monitor.capture_for_shot(edge, timeout_s=0.5)
    assert capture is not None
    monitor.finish_capture(capture, full_dump=False)
    monitor.stop()


def test_a_capture_claimed_for_a_shot_is_never_released_by_a_rejection(tmp_path):
    radar = FakeReducedRadar()
    monitor = _monitor(tmp_path, radar, hold_budget_s=30.0)
    edge = _held(monitor, radar)
    capture = monitor.capture_for_shot(edge, timeout_s=0.5)

    assert not monitor.discard_trigger(edge)

    time.sleep(0.1)
    assert radar.held
    assert monitor.read_strips(capture, REQUEST) == reduced.serve_strips(CAPTURE, REQUEST)
    monitor.finish_capture(capture, full_dump=False)
    monitor.stop()


def test_a_stale_rejection_does_not_match_a_later_edge(tmp_path):
    radar = FakeReducedRadar()
    monitor = _monitor(tmp_path, radar, hold_budget_s=30.0)
    monitor.discard_trigger(time.time() - 10.0)  # a sound the IWR never saw

    capture = _capture(monitor)

    assert radar.held
    monitor.finish_capture(capture, full_dump=False)
    monitor.stop()


def test_full_transfer_rejection_drops_the_queued_capture(tmp_path):
    radar = FakeReducedRadar()
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    monitor = IWR6843CaptureMonitor(
        config_path=config, output_dir=tmp_path / "dumps", radar=radar, button_factory=FakeButton
    )
    monitor.start()
    edge = time.time()
    assert monitor.notify_trigger(edge)
    assert _wait_for(lambda: "dump" in radar.calls)
    time.sleep(0.05)

    assert monitor.discard_trigger(edge)

    assert monitor.capture_for_shot(edge, timeout_s=0.2) is None
    assert radar.calls == ["config", "dump"]
    monitor.stop()


def test_discard_without_a_timestamp_is_ignored(tmp_path):
    radar = FakeReducedRadar()
    monitor = _monitor(tmp_path, radar, hold_budget_s=30.0)
    edge = _held(monitor, radar)

    assert not monitor.discard_trigger(None)

    assert radar.held
    monitor.finish_capture(monitor.capture_for_shot(edge, timeout_s=0.5), full_dump=False)
    monitor.stop()
