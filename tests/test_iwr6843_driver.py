"""Tests for the IWR6843 CLI and dump serial contract."""

from __future__ import annotations

import time

import numpy as np
import pytest
from test_iwr6843_pipeline import synth_shot
from test_iwr6843_reduced import N_FRAMES, PERIOD_US, _timed_iq8

from openflight.iwr6843 import reduced
from openflight.iwr6843.autotrigger import AutoTriggerConfig
from openflight.iwr6843.driver import AutoTriggerNotice, IWR6843Radar, parse_auto_trigger_notice
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
    """Serial double that exposes the in_waiting/read/write pieces read_dump uses.

    Like the real port, the command's ``reply`` arrives after the command is
    written; ``waiting`` is already in the input buffer (e.g. a notice).
    """

    def __init__(self, reply: bytes = b"", *, waiting: bytes = b""):
        self.payload = bytearray(waiting)
        self.reply = bytes(reply)
        self.writes = []

    @property
    def in_waiting(self):
        return len(self.payload)

    def reset_input_buffer(self):
        pass

    def write(self, data: bytes):
        self.writes.append(data)
        self.payload.extend(self.reply)
        self.reply = b""

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
            if self.delay_next_chunk or not self.writes:
                return 0
            return len(self.chunks[0]) if self.chunks else 0

        def reset_input_buffer(self):
            return None

        def write(self, value):
            self.writes.append(value)

        def read(self, count):
            if not self.writes:
                return b""
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


class _TimedSerial:
    """Serial double that releases each chunk only after its delay."""

    def __init__(self, chunks, *, start_on_write=True):
        """Chunk delays count from the command's write (or from now)."""
        self.start = None if start_on_write else time.monotonic()
        self.chunks = [(delay, bytearray(data)) for delay, data in chunks]
        self.writes = []

    def _ready(self):
        if self.start is None:
            return None
        elapsed = time.monotonic() - self.start
        return self.chunks[0][1] if self.chunks and elapsed >= self.chunks[0][0] else None

    @property
    def in_waiting(self):
        ready = self._ready()
        return len(ready) if ready is not None else 0

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.writes.append(data)
        if self.start is None:
            self.start = time.monotonic()

    def read(self, count):
        ready = self._ready()
        if ready is None:
            time.sleep(0.005)
            return b""
        data = bytes(ready[:count])
        del ready[:count]
        if not ready:
            self.chunks.pop(0)
        return data


def test_read_overview_waits_out_the_compute_before_the_first_byte():
    """rc2 on the radar: the echo came back, then seconds of compute."""
    overview = reduced.pack_overview(reduced.build_overview(_synthetic_capture(), gate=(48, 98)))
    radar = IWR6843Radar.__new__(IWR6843Radar)
    radar.ser = _TimedSerial([(0.0, b"l3overview\r\n"), (0.3, overview + b"Done\r\n")])

    assert radar.read_overview(timeout_s=2.0, stall_tolerance_s=0.1) == overview


def test_read_overview_reports_a_missing_response():
    radar = IWR6843Radar.__new__(IWR6843Radar)
    radar.ser = _TimedSerial([(0.0, b"l3overview\r\n")])

    with pytest.raises(RuntimeError, match="l3overview sent no response"):
        radar.read_overview(timeout_s=0.3, stall_tolerance_s=0.05)


def test_read_dump_still_gives_up_after_a_stall_following_the_echo():
    radar = IWR6843Radar.__new__(IWR6843Radar)
    radar.ser = _TimedSerial([(0.0, b"l3dump\r\n")])
    start = time.monotonic()

    assert radar.read_dump(timeout_s=5.0, stall_tolerance_s=0.05) == b"l3dump\r\n"
    assert time.monotonic() - start < 1.0


# -- auto trigger: the radar detects the shot and sends an ILTRG1 notice -------------

NOTICE = b"ILTRG1 seq=7 age_ms=52 delay=3 end=39 slope_q=6 frames=36\n"


class _ClearingSerial(FakeSerial):
    """Like the real port: reset_input_buffer drops what is waiting; a write
    queues the command's reply."""

    def __init__(self, waiting: bytes, replies: dict[bytes, bytes]):
        super().__init__(waiting=waiting)
        self.replies = replies

    def reset_input_buffer(self):
        self.payload.clear()

    def write(self, data: bytes):
        self.writes.append(data)
        self.payload.extend(self.replies.get(data, b""))


def _radar(serial) -> IWR6843Radar:
    radar = IWR6843Radar.__new__(IWR6843Radar)
    radar.ser = serial
    return radar


def test_parse_auto_trigger_notice():
    notice = parse_auto_trigger_notice(NOTICE.strip(), received_at=100.0)

    assert notice == AutoTriggerNotice(
        seq=7, age_ms=52, delay_frames=3, end_bin=39, slope_q=6, n_frames=36, received_at=100.0
    )
    assert parse_auto_trigger_notice(b"stats\n", received_at=0.0) is None


def test_notice_edge_time_is_the_detection():
    notice = AutoTriggerNotice(
        seq=1, age_ms=52, delay_frames=3, end_bin=39, slope_q=6, n_frames=36, received_at=100.0
    )

    assert notice.detection_timestamp(frame_period_s=0.002) == pytest.approx(100.0 - 0.052 - 0.006)


def test_read_notice_skips_other_output():
    radar = _radar(FakeSerial(waiting=b"l3dump:/>\r\nnoise " + NOTICE))

    notice = radar.read_auto_trigger_notice(timeout_s=0.5)

    assert notice.seq == 7 and notice.end_bin == 39


def test_read_notice_assembles_a_line_split_across_reads():
    radar = _radar(_TimedSerial([(0.0, NOTICE[:20]), (0.05, NOTICE[20:])], start_on_write=False))

    assert radar.read_auto_trigger_notice(timeout_s=1.0).seq == 7


def test_read_notice_times_out_quietly():
    radar = _radar(_TimedSerial([], start_on_write=False))
    start = time.monotonic()

    assert radar.read_auto_trigger_notice(timeout_s=0.1) is None
    assert time.monotonic() - start < 0.5


def test_a_notice_waiting_before_a_command_is_kept():
    radar = _radar(_ClearingSerial(NOTICE, {b"stats\n": b"stats\r\nactive=1\r\nDone\r\n"}))

    response = radar.cmd("stats")

    assert "active=1" in response
    assert radar.read_auto_trigger_notice(timeout_s=0.0).seq == 7


def test_a_notice_inside_a_command_reply_is_kept_and_removed_from_it():
    reply = b"stats\r\n" + NOTICE + b"active=1\r\nDone\r\n"
    radar = _radar(_ClearingSerial(b"", {b"stats\n": reply}))

    response = radar.cmd("stats")

    assert "ILTRG1" not in response and "Done" in response
    assert radar.read_auto_trigger_notice(timeout_s=0.0).seq == 7


def test_a_notice_before_a_framed_reply_is_kept():
    overview = reduced.pack_overview(reduced.build_overview(_synthetic_capture(), gate=(48, 98)))
    radar = _radar(
        _ClearingSerial(NOTICE, {b"l3overview\n": b"l3overview\r\n" + overview + b"Done\r\n"})
    )

    assert radar.read_overview(timeout_s=0.5) == overview
    assert radar.read_auto_trigger_notice(timeout_s=0.0).seq == 7


def test_configure_auto_trigger_sends_the_config_line(monkeypatch):
    radar = IWR6843Radar.__new__(IWR6843Radar)
    sent = []

    def fake_cmd(line, window=1.5):
        sent.append(line)
        return "Auto trigger: on\nDone\n"

    monkeypatch.setattr(radar, "cmd", fake_cmd)
    config = AutoTriggerConfig(end_lo=38, end_hi=40)

    radar.configure_auto_trigger(config)
    radar.configure_auto_trigger(None)

    assert sent == [config.command(), "autoTrigCfg 0"]


def test_configure_auto_trigger_rejected_by_old_firmware(monkeypatch):
    radar = IWR6843Radar.__new__(IWR6843Radar)
    monkeypatch.setattr(radar, "cmd", lambda *_a, **_k: "Error: unknown command\n")

    with pytest.raises(RuntimeError, match="autoTrigCfg"):
        radar.configure_auto_trigger(AutoTriggerConfig(end_lo=38, end_hi=40))
