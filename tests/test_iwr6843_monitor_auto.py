"""Capture monitor with the radar's own shot detector (no sound edge).

The firmware detects the club reaching the tee, freezes after the post-impact
frames, holds the ring and sends an ILTRG1 notice. The monitor turns each
notice into the same trigger edge a GPIO press would, so matching, the camera
fan-out and OPS rejections work unchanged.
"""

from __future__ import annotations

import threading
import time

import pytest
from test_iwr6843_monitor import FakeButton
from test_iwr6843_monitor_reduced import CAPTURE, FakeReducedRadar, _wait_for
from test_iwr6843_reduced import GATE

from openflight.iwr6843.autotrigger import AutoTriggerConfig
from openflight.iwr6843.driver import AutoTriggerNotice
from openflight.iwr6843.monitor import IWR6843CaptureMonitor, frame_period_s_from_config

CONFIG = AutoTriggerConfig(end_lo=38, end_hi=40)


class AutoRadar(FakeReducedRadar):
    """The reduced-transfer firmware with its shot detector."""

    def __init__(self, *, reject_config: bool = False, overview_s: float = 0.0):
        super().__init__()
        self.reject_config = reject_config
        self.overview_s = overview_s
        self.auto_configs = []
        self.notices: list[AutoTriggerNotice] = []
        self.busy = 0
        self.max_busy = 0
        self._lock = threading.Lock()

    def _enter(self):
        with self._lock:
            self.busy += 1
            self.max_busy = max(self.max_busy, self.busy)

    def _leave(self):
        with self._lock:
            self.busy -= 1

    def configure_auto_trigger(self, config):
        self.calls.append("autoTrigCfg")
        if self.reject_config:
            raise RuntimeError("config rejected: 'autoTrigCfg': Error: unknown command")
        self.auto_configs.append(config)

    def detect(self, *, age_ms=50, delay=3) -> AutoTriggerNotice:
        """The club reached the tee: freeze, hold, and queue the notice."""
        notice = AutoTriggerNotice(
            seq=len(self.notices) + 1,
            age_ms=age_ms,
            delay_frames=delay,
            end_bin=39,
            slope_q=6,
            n_frames=36,
            received_at=time.time(),
        )
        self.held = True
        self.notices.append(notice)
        return notice

    def read_auto_trigger_notice(self, timeout_s):
        self._enter()
        try:
            deadline = time.monotonic() + timeout_s
            while not self.notices and time.monotonic() < deadline:
                time.sleep(0.002)
            return self.notices.pop(0) if self.notices else None
        finally:
            self._leave()

    def read_overview(self):
        self._enter()
        try:
            time.sleep(self.overview_s)
            return super().read_overview()
        finally:
            self._leave()


def _monitor(tmp_path, radar, *, auto=CONFIG, gate=GATE, factory=None, **kwargs):
    config = tmp_path / "radar.cfg"
    config.write_text("frameCfg 0 2 12 0 2 1 0\nsensorStart\n", encoding="utf-8")
    buttons = []

    def button_factory(*args, **kw):
        button = (factory or FakeButton)(*args, **kw)
        buttons.append(button)
        return button

    monitor = IWR6843CaptureMonitor(
        config_path=config,
        output_dir=tmp_path / "dumps",
        radar=radar,
        button_factory=button_factory,
        reduced_gate=gate,
        auto_trigger=auto,
        **kwargs,
    )
    monitor.start()
    return monitor, buttons


def test_frame_period_comes_from_the_config(tmp_path):
    config = tmp_path / "radar.cfg"
    config.write_text("% comment\nframeCfg 0 2 12 0 2 1 0\n", encoding="utf-8")
    assert frame_period_s_from_config(config) == 0.002
    config.write_text("sensorStart\n", encoding="utf-8")
    with pytest.raises(ValueError, match="frameCfg"):
        frame_period_s_from_config(config)


def test_auto_mode_configures_the_detector_and_skips_the_gpio(tmp_path):
    radar = AutoRadar()
    monitor, buttons = _monitor(tmp_path, radar)

    assert radar.calls[:3] == ["config", f"gate {GATE[0]} {GATE[1]}", "autoTrigCfg"]
    assert radar.auto_configs == [CONFIG]
    assert buttons == []
    assert monitor.auto_trigger == CONFIG
    monitor.stop()


def test_a_notice_becomes_a_trigger_edge_at_the_detection(tmp_path):
    radar = AutoRadar()
    edges = []
    monitor, _ = _monitor(tmp_path, radar, trigger_observers=[edges.append])
    notice = radar.detect(age_ms=50, delay=3)
    expected_edge = notice.detection_timestamp(frame_period_s=0.002)

    capture = monitor.capture_for_shot(expected_edge, timeout_s=2.0)

    assert capture is not None and capture.valid and capture.transfer == "reduced"
    assert capture.trigger_timestamp == pytest.approx(expected_edge)
    assert edges == [pytest.approx(expected_edge)]
    assert "overview" in radar.calls
    monitor.finish_capture(capture, full_dump=False)
    assert not radar.held
    monitor.stop()


def test_consecutive_shots(tmp_path):
    radar = AutoRadar()
    monitor, _ = _monitor(tmp_path, radar)

    for _ in range(3):
        time.sleep(0.15)  # shots are seconds apart; edges < 0.1 s apart are ringing
        edge = radar.detect().detection_timestamp(frame_period_s=0.002)
        capture = monitor.capture_for_shot(edge, timeout_s=2.0)
        assert capture is not None
        monitor.finish_capture(capture, full_dump=False)
        assert _wait_for(lambda: not monitor._capture_active)

    assert radar.calls.count("overview") == 3
    monitor.stop()


def test_a_notice_before_arming_releases_the_radar(tmp_path):
    radar = AutoRadar()
    config = tmp_path / "radar.cfg"
    config.write_text("frameCfg 0 2 12 0 2 1 0\nsensorStart\n", encoding="utf-8")
    monitor = IWR6843CaptureMonitor(
        config_path=config,
        output_dir=tmp_path / "dumps",
        radar=radar,
        button_factory=FakeButton,
        reduced_gate=GATE,
        auto_trigger=CONFIG,
    )
    monitor.start(armed=False)

    radar.detect()

    assert _wait_for(lambda: "release" in radar.calls)
    assert not radar.held and "overview" not in radar.calls
    monitor.stop()


def test_old_firmware_falls_back_to_the_gpio_edge(tmp_path):
    radar = AutoRadar(reject_config=True)
    monitor, buttons = _monitor(tmp_path, radar)

    assert monitor.auto_trigger is None
    assert len(buttons) == 1 and buttons[0].when_pressed is not None
    monitor.stop()


def test_auto_trigger_needs_the_reduced_transfer(tmp_path):
    radar = AutoRadar()
    monitor, buttons = _monitor(tmp_path, radar, gate=None)

    assert monitor.auto_trigger is None
    assert "autoTrigCfg" not in radar.calls
    assert len(buttons) == 1
    monitor.stop()


def test_listener_never_reads_the_port_during_a_capture(tmp_path):
    """The overview and a notice poll must never share the serial port."""
    radar = AutoRadar(overview_s=0.2)
    monitor, _ = _monitor(tmp_path, radar)

    edge = radar.detect().detection_timestamp(frame_period_s=0.002)
    capture = monitor.capture_for_shot(edge, timeout_s=2.0)
    monitor.finish_capture(capture, full_dump=False)

    assert radar.max_busy == 1
    monitor.stop()


def test_ops_rejection_still_releases_an_auto_capture(tmp_path):
    radar = AutoRadar()
    monitor, _ = _monitor(tmp_path, radar, hold_budget_s=30.0)
    edge = radar.detect().detection_timestamp(frame_period_s=0.002)
    assert _wait_for(lambda: "overview" in radar.calls and radar.held)

    monitor.discard_trigger(edge + 0.02)

    assert _wait_for(lambda: not radar.held)
    assert radar.calls[-1] == "release"
    monitor.stop()


def test_gpio_mode_is_unchanged(tmp_path):
    radar = AutoRadar()
    monitor, buttons = _monitor(tmp_path, radar, auto=None)

    assert "autoTrigCfg" not in radar.calls
    assert len(buttons) == 1
    edge = time.time()
    assert monitor.notify_trigger(edge)
    capture = monitor.capture_for_shot(edge, timeout_s=2.0)
    assert capture is not None and capture.raw is None
    assert CAPTURE  # the fake serves the shared synthetic capture
    monitor.finish_capture(capture, full_dump=False)
    monitor.stop()
