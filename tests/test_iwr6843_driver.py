"""Tests for the IWR6843 CLI and dump serial contract."""

from __future__ import annotations

import time

import numpy as np
import pytest
from test_iwr6843_pipeline import synth_shot
from test_iwr6843_reduced import N_FRAMES, PERIOD_US, _timed_iq8

from openflight.iwr6843 import reduced
from openflight.iwr6843.driver import IWR6843Radar
from openflight.iwr6843.dump import TEMP_REPORT_KEYS, pack_dump


def test_send_config_rejects_missing_cli_acknowledgement(tmp_path, monkeypatch):
    """A wedged board must not be reported as configured and armed."""
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    radar = IWR6843Radar.__new__(IWR6843Radar)
    monkeypatch.setattr(radar, "drain_stale_output", lambda: 0)
    monkeypatch.setattr(radar, "cmd", lambda *_args, **_kwargs: "")

    with pytest.raises(RuntimeError, match="did not acknowledge"):
        radar.send_config(str(config))


def test_stop_sensor_requires_acknowledgement_and_inactive_health(monkeypatch):
    """Shutdown must leave firmware idle rather than merely close the host UART."""
    radar = IWR6843Radar.__new__(IWR6843Radar)
    responses = iter(["sensorStop\nDone\nl3dump:/>", "stats\nactive=0\nDone\nl3dump:/>"])
    calls = []

    def fake_cmd(command, window):
        calls.append((command, window))
        return next(responses)

    monkeypatch.setattr(radar, "cmd", fake_cmd)

    radar.stop_sensor()

    assert calls == [("sensorStop", 3.0), ("stats", 2.0)]


def test_stop_sensor_rejects_firmware_that_remains_active(monkeypatch):
    radar = IWR6843Radar.__new__(IWR6843Radar)
    responses = iter(["sensorStop\nDone\n", "stats\nactive=1\nDone\n"])
    monkeypatch.setattr(radar, "cmd", lambda *_args: next(responses))

    with pytest.raises(RuntimeError, match="remained active"):
        radar.stop_sensor()


def test_send_config_flushes_previous_mmwave_profile_when_config_omits_flush(tmp_path, monkeypatch):
    """Repeated startup must not exhaust the firmware's mmWave profile slots."""
    config = tmp_path / "radar.cfg"
    config.write_text("dfeDataOutputMode 1\nsensorStart\n", encoding="utf-8")
    commands = []
    radar = IWR6843Radar.__new__(IWR6843Radar)
    monkeypatch.setattr(radar, "drain_stale_output", lambda: 0)

    def command(line, *_args, **_kwargs):
        commands.append(line)
        if line == "stats":
            return "active=1\nDone\n"
        return "Done\n"

    monkeypatch.setattr(radar, "cmd", command)

    radar.send_config(str(config))

    assert commands == [
        "sensorStop",
        "flushCfg",
        "dfeDataOutputMode 1",
        "sensorStart",
        "stats",
    ]


def test_send_config_waits_for_sensor_to_become_active(tmp_path, monkeypatch):
    """sensorStart may acknowledge before RF calibration and HWA startup finish."""
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    statuses = iter(
        (
            "active=0 calib=0x0 hwa_frames=0\nDone\n",
            "active=0 calib=0x1ffe hwa_frames=0\nDone\n",
            "active=1 calib=0x1ffe hwa_frames=2\nDone\n",
        )
    )
    commands = []
    radar = IWR6843Radar.__new__(IWR6843Radar)
    monkeypatch.setattr(radar, "drain_stale_output", lambda: 0)

    def command(line, *_args, **_kwargs):
        commands.append(line)
        return next(statuses) if line == "stats" else "Done\n"

    monkeypatch.setattr(radar, "cmd", command)

    radar.send_config(str(config))

    assert commands == ["sensorStop", "flushCfg", "sensorStart", "stats", "stats", "stats"]


class FakeSerial:
    """Serial double that exposes the in_waiting/read/write pieces read_dump uses."""

    def __init__(self, payload: bytes):
        self.payload = bytearray(payload)
        self.writes = []

    @property
    def in_waiting(self):
        return len(self.payload)

    def reset_input_buffer(self):
        pass

    def write(self, data: bytes):
        self.writes.append(data)

    def read(self, nbytes: int):
        nbytes = min(nbytes, len(self.payload))
        chunk = self.payload[:nbytes]
        del self.payload[:nbytes]
        return bytes(chunk)


def test_read_dump_waits_for_cli_ready_after_binary_payload():
    raw = pack_dump(np.ones((2, 6, 4, 7), dtype=complex), n_tx=3, version=3)

    class ChunkedSerial:
        def __init__(self):
            self.chunks = [bytearray(b"l3dump\r\n" + raw), bytearray(b"Done\r\nl3dump:/>")]
            self.writes = []
            self.delay_next_chunk = False

        @property
        def in_waiting(self):
            if self.delay_next_chunk:
                return 0
            return len(self.chunks[0]) if self.chunks else 0

        def reset_input_buffer(self):
            return None

        def write(self, value):
            self.writes.append(value)

        def read(self, count):
            if self.delay_next_chunk:
                self.delay_next_chunk = False
                return b""
            if not self.chunks:
                return b""
            chunk = self.chunks[0]
            data = bytes(chunk[:count])
            del chunk[:count]
            if not chunk:
                self.chunks.pop(0)
                if self.chunks:
                    self.delay_next_chunk = True
            return data

    radar = IWR6843Radar.__new__(IWR6843Radar)
    radar.ser = ChunkedSerial()

    assert radar.read_dump(timeout_s=0.1) == raw
    assert radar.ser.chunks == []
    assert radar.ser.writes == [b"l3dump\n"]


def test_read_dump_reports_firmware_restart_error_after_binary_payload():
    raw = pack_dump(np.ones((1, 3, 4, 4), dtype=complex), n_tx=3, version=3)
    radar = IWR6843Radar.__new__(IWR6843Radar)
    radar.ser = FakeSerial(b"l3dump\r\n" + raw + b"Error: RF restart failed\r\n")

    with pytest.raises(RuntimeError, match="RF restart failed"):
        radar.read_dump(timeout_s=0.1)


def test_read_dump_sizes_v5_header_extension():
    report = {key: index + 40 for index, key in enumerate(TEMP_REPORT_KEYS)}
    raw = pack_dump(
        np.zeros((2, 4, 4, 8), dtype=complex),
        n_tx=2,
        version=5,
        temperature_report=report,
    )
    serial = FakeSerial(b"cli echo\r\n" + raw + b"trailing cli noise")
    radar = IWR6843Radar.__new__(IWR6843Radar)
    radar.ser = serial

    dump = radar.read_dump(timeout_s=0.1)

    assert dump == raw
    assert serial.writes == [b"l3dump\n"]


# -- reduced transfer (plans/iwr6843-on-chip-reduction.md) --------------------------


def _synthetic_capture():
    return _timed_iq8(
        synth_shot(
            n_frames=N_FRAMES, n_loops=12, n_tx=3, frame_period_us=PERIOD_US, trigger_frame=0
        )
    )


def test_read_overview_returns_the_overview_and_consumes_done():
    overview = reduced.pack_overview(reduced.build_overview(_synthetic_capture(), gate=(48, 98)))
    radar = IWR6843Radar.__new__(IWR6843Radar)
    radar.ser = FakeSerial(b"l3overview\r\n" + overview + b"Done\r\nl3dump:/>")

    assert radar.read_overview(timeout_s=0.5) == overview
    assert radar.ser.writes == [b"l3overview\n"]
    assert radar.ser.payload == bytearray()


def test_read_overview_fails_fast_on_a_firmware_error():
    radar = IWR6843Radar.__new__(IWR6843Radar)
    radar.ser = FakeSerial(b"l3overview\r\nError: send overviewCfg before l3overview\r\n")
    start = time.monotonic()

    with pytest.raises(RuntimeError, match="send overviewCfg"):
        radar.read_overview(timeout_s=5.0)
    assert time.monotonic() - start < 1.0


def test_read_strips_sends_the_encoded_request():
    capture = _synthetic_capture()
    request = (None,) * 6 + ((62, 4),) * 6
    strips = reduced.serve_strips(capture, request)
    radar = IWR6843Radar.__new__(IWR6843Radar)
    radar.ser = FakeSerial(b"echo\r\n" + strips + b"Done\r\n")

    assert radar.read_strips(request, timeout_s=0.5) == strips
    assert radar.ser.writes == [f"l3strip {reduced.encode_strip_request(request)}\n".encode()]


def test_read_strips_fails_fast_on_a_rejected_request():
    radar = IWR6843Radar.__new__(IWR6843Radar)
    radar.ser = FakeSerial(b"Error: bad strip request (-3)\r\n")

    with pytest.raises(RuntimeError, match="bad strip request"):
        radar.read_strips((None,) * 12, timeout_s=5.0)


def test_read_dump_keeps_waiting_past_error_text():
    """l3dump's contract is unchanged: no early failure on CLI text."""
    radar = IWR6843Radar.__new__(IWR6843Radar)
    radar.ser = FakeSerial(b"Error: something\r\n")

    assert radar.read_dump(timeout_s=0.05, stall_tolerance_s=0.01) == b"Error: something\r\n"


def test_overview_gate_and_release_commands(monkeypatch):
    radar = IWR6843Radar.__new__(IWR6843Radar)
    sent = []

    def fake_cmd(line, window):
        sent.append(line)
        return "Done\n"

    monkeypatch.setattr(radar, "cmd", fake_cmd)

    radar.configure_overview_gate((48, 98))
    radar.release()

    assert sent == ["overviewCfg 48 98", "l3release"]


def test_overview_gate_rejected_by_firmware_raises(monkeypatch):
    radar = IWR6843Radar.__new__(IWR6843Radar)
    monkeypatch.setattr(radar, "cmd", lambda *_a: "Error: overviewCfg gateLo gateHi\n")

    with pytest.raises(RuntimeError, match="overviewCfg"):
        radar.configure_overview_gate((98, 48))
