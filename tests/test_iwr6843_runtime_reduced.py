"""Runtime + monitor in reduced-transfer mode, end to end with an emulated radar.

process_shot must give the same shot as today's full dump, fetch strips
through the monitor while the radar holds the ring, and always finish the
capture. Strip failures fall back to streaming the held ring whole.
"""

from __future__ import annotations

import time

import pytest
from test_iwr6843_monitor import FakeButton
from test_iwr6843_monitor_reduced import CAPTURE, FakeReducedRadar
from test_iwr6843_pipeline import _cal  # noqa: F401
from test_iwr6843_reduced_runtime import BALL_MPH, NET_M, _same

from openflight.iwr6843.dump import parse_capture_metadata
from openflight.iwr6843.monitor import IWR6843CaptureMonitor
from openflight.iwr6843.reduced import CaptureReducedSource, ball_gate_for
from openflight.iwr6843.runtime import IWR6843Runtime


class _FailingStripsRadar(FakeReducedRadar):
    def read_strips(self, request):
        self.calls.append("strips")
        raise RuntimeError("IWR6843 l3strip sent no response")


def _setup(tmp_path, cal, radar, **runtime_kwargs):
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    monitor = IWR6843CaptureMonitor(
        config_path=config,
        output_dir=tmp_path / "dumps",
        radar=radar,
        button_factory=FakeButton,
        reduced_gate=ball_gate_for(parse_capture_metadata(CAPTURE), NET_M),
    )
    monitor.start()
    runtime = IWR6843Runtime(
        capture_monitor=monitor,
        calibration=cal,
        net_range_m=NET_M,
        tdm_sign_policy="positive",
        capture_timeout_s=2.0,
        **runtime_kwargs,
    )
    edge = time.time()
    assert monitor.notify_trigger(edge)
    return monitor, runtime, edge


def _offline_runtime(cal):
    return IWR6843Runtime(
        capture_monitor=None, calibration=cal, net_range_m=NET_M, tdm_sign_policy="positive"
    )


def _full(cal):
    """Today's full-dump measurement (what every fallback must reproduce)."""
    return _offline_runtime(cal).measure_capture(CAPTURE, ball_speed_mph=BALL_MPH, club="driver")


def _wire_format(cal):
    """The same capture through the packed overview the radar sends."""
    source = CaptureReducedSource(
        CAPTURE, gate=ball_gate_for(parse_capture_metadata(CAPTURE), NET_M), quantise=True
    )
    return _offline_runtime(cal).measure_reduced(source, ball_speed_mph=BALL_MPH, club="driver")


def test_reduced_shot_matches_the_full_dump_and_releases_the_radar(tmp_path, cal):
    radar = FakeReducedRadar()
    monitor, runtime, edge = _setup(tmp_path, cal, radar)

    result = runtime.process_shot(
        impact_timestamp=edge, ball_speed_mph=BALL_MPH, club="driver", club_speed_mph=75.0
    )

    assert result.measurement.to_dict() == _wire_format(cal).to_dict()
    assert result.measurement.angle_deg == pytest.approx(_full(cal).angle_deg, abs=1e-3)
    assert radar.calls[-1] == "release" and not radar.held
    assert "strips" in radar.calls and "dump" not in radar.calls
    assert result.transfer["mode"] == "reduced"
    assert result.transfer["fallback_reason"] is None
    assert result.transfer["overview_bytes"] > 0
    assert len(result.transfer["strip_bytes"]) >= 1
    assert result.transfer["full_dump"] is False
    assert result.club_path is None
    assert result.transfer["club_path"] == "skipped: no full dump in reduced transfer"
    monitor.stop()


def test_full_dump_option_streams_the_held_ring_for_club_path(tmp_path, cal):
    radar = FakeReducedRadar()
    monitor, runtime, edge = _setup(tmp_path, cal, radar, reduced_full_dump=True)

    result = runtime.process_shot(
        impact_timestamp=edge, ball_speed_mph=BALL_MPH, club="driver", club_speed_mph=75.0
    )

    assert result.measurement.to_dict() == _wire_format(cal).to_dict()
    assert radar.calls[-1] == "dump" and "release" not in radar.calls
    assert result.transfer["full_dump"] is True
    assert result.club_path is not None
    monitor.stop()


def test_strip_failure_falls_back_to_the_held_ring(tmp_path, cal):
    radar = _FailingStripsRadar()
    monitor, runtime, edge = _setup(tmp_path, cal, radar)

    result = runtime.process_shot(impact_timestamp=edge, ball_speed_mph=BALL_MPH, club="driver")

    _same(result.measurement, _full(cal))
    assert radar.calls[-1] == "dump"
    assert "l3strip sent no response" in result.transfer["fallback_reason"]
    assert result.transfer["full_dump"] is True
    monitor.stop()


def test_monitor_fallback_capture_is_measured_as_a_full_dump(tmp_path, cal):
    radar = FakeReducedRadar(overview_error=RuntimeError("IWR6843 l3overview sent no response"))
    monitor, runtime, edge = _setup(tmp_path, cal, radar)

    result = runtime.process_shot(impact_timestamp=edge, ball_speed_mph=BALL_MPH, club="driver")

    _same(result.measurement, _full(cal))
    assert result.transfer["mode"] == "full"
    assert "no response" in result.transfer["fallback_reason"]
    monitor.stop()


def test_a_measurement_error_still_releases_the_radar(tmp_path, cal, monkeypatch):
    radar = FakeReducedRadar()
    monitor, runtime, edge = _setup(tmp_path, cal, radar)

    def broken(*_args, **_kwargs):
        raise ValueError("boom")

    monkeypatch.setattr(runtime, "measure_reduced", broken)
    monkeypatch.setattr(runtime, "measure_capture", broken)

    with pytest.raises(ValueError, match="boom"):
        runtime.process_shot(impact_timestamp=edge, ball_speed_mph=BALL_MPH, club="driver")
    assert radar.calls[-1] == "dump"  # the fallback took the held ring
    assert not radar.held
    monitor.stop()
