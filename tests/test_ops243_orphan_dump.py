"""Stuck blue light: dumps whose start marker was lost must not be ignored.

``wait_for_hardware_trigger`` starts by clearing the input buffer. If a
second HOST_INT pulse (e.g. the ball hitting the net) starts a dump between
the re-arm and the next wait, that clear discards the dump's
``{"sample_time"``/``{"trigger_time"`` header. The remaining I/Q body then
looked like idle serial noise, was dropped, and the radar sat idle after its
dump -- blue light on, no hits read -- until the 30 s watchdog re-armed it.

The wait now recognises such an orphaned dump (hundreds of bytes of
non-capture data, then quiet) and returns early with the byte count, so the
trigger can re-arm within about a second. Small idle noise (whitespace,
``{"Clock":...}`` replies) is still ignored.
"""

import time

import serial as pyserial
from test_ops243 import _ScheduledSerial

from openflight.ops243 import OPS243Radar

# I/Q body of a dump whose header was lost: >2 KB, no capture marker.
ORPHAN_BODY = b"2150,2151,2149," * 60 + b'2150]}\r\n{"Q":[' + b"2048,2047,2049," * 60 + b"2048]}"
CLOCK_NOISE = b'\n\n{"Clock":1786805707}\r\n\n'
DUMP = (
    b'{"sample_time":946.077}\r\n{"trigger_time":946.145}\r\n'
    b'{"I":[2168,2187,2155,2154]}\r\n{"Q":[2048,2050,2047,2049]}'
)


def _radar(serial_obj):
    radar = OPS243Radar.__new__(OPS243Radar)
    radar.serial = serial_obj
    radar.last_hardware_trigger_first_byte_timestamp = None
    return radar


def test_orphaned_dump_returns_early_and_reports_its_size():
    radar = _radar(_ScheduledSerial([(0.02, ORPHAN_BODY)]))
    start = time.time()

    response = radar.wait_for_hardware_trigger(timeout=5.0, dump_grace=1.0)

    assert response == ""
    assert time.time() - start < 2.0, "orphaned dump must not wait for the full timeout"
    assert radar.last_hardware_trigger_orphan_bytes == len(ORPHAN_BODY)
    assert radar.last_hardware_trigger_first_byte_timestamp is None


def test_idle_noise_is_not_an_orphaned_dump():
    radar = _radar(_ScheduledSerial([(0.02, CLOCK_NOISE)]))
    start = time.time()

    response = radar.wait_for_hardware_trigger(timeout=0.6, dump_grace=1.0)

    assert response == ""
    assert time.time() - start >= 0.55, "small noise must not end the wait early"
    assert radar.last_hardware_trigger_orphan_bytes == 0


def test_complete_dump_reports_no_orphan_bytes():
    radar = _radar(_ScheduledSerial([(0.02, CLOCK_NOISE), (0.08, DUMP)]))

    response = radar.wait_for_hardware_trigger(timeout=1.0, dump_grace=1.0)

    assert response == DUMP.decode("ascii")
    assert radar.last_hardware_trigger_orphan_bytes == 0


class _QuietSerial:
    """Port with nothing waiting that accepts writes."""

    def __init__(self):
        self.is_open = True
        self.writes = []

    @property
    def in_waiting(self):
        return 0

    def read(self, n):
        return b""

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.writes.append(data)

    def flush(self):
        pass


class _TimeoutWriteSerial(_QuietSerial):
    def write(self, data):
        raise pyserial.SerialTimeoutException("Write timeout")


def test_rearm_reports_success():
    radar = _radar(_QuietSerial())

    assert radar.rearm_rolling_buffer(16) is True
    assert b"PA" in radar.serial.writes


def test_rearm_reports_failed_write():
    radar = _radar(_TimeoutWriteSerial())

    assert radar.rearm_rolling_buffer(16) is False
