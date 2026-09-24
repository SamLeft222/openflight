"""Firmware glue for the reduced transfer (plans/iwr6843-on-chip-reduction.md).

The overview/strip bytes are checked against the Python reference by
test_iwr6843_reduced_firmware.py. These checks pin the command sequencing in
l3_dump.c that cannot run off-target: validate before streaming, never lose a
frozen ring, and never leave the radar frozen.
"""

from __future__ import annotations

import re
from pathlib import Path

from openflight.iwr6843 import reduced

FIRMWARE_DIR = Path(__file__).parents[1] / "firmware" / "iwr6843"
SOURCE = (FIRMWARE_DIR / "l3_dump.c").read_text(encoding="utf-8")
HEADER = (FIRMWARE_DIR / "reduced_overview.h").read_text(encoding="utf-8")


def _function(name: str) -> str:
    """Body of the definition (not a prototype) whose signature starts with ``name``."""
    match = re.search(rf"^{re.escape(name)}\([^;{{]*?\)\s*\{{", SOURCE, re.MULTILINE | re.DOTALL)
    assert match, name
    return SOURCE[match.start() : SOURCE.index("\n}\n", match.end())]


def _before(text: str, first: str, second: str) -> bool:
    return text.index(first) < text.index(second)


def _define(text: str, name: str) -> str:
    """A macro's value, without any trailing comment."""
    line = re.search(rf"^#define\s+{name}\s+(.+)$", text, re.MULTILINE).group(1)
    return line.split("/*")[0].strip()


# -- C and Python agree on the wire contract ------------------------------------------


def test_overview_constants_match_the_python_reference():
    assert _define(HEADER, "RO_OVERVIEW_MAGIC") == f'"{reduced.OVERVIEW_MAGIC.decode()}"'
    assert _define(HEADER, "RO_OVERVIEW_VERSION") == f"{reduced.OVERVIEW_VERSION}U"
    assert _define(HEADER, "RO_OVERVIEW_HEADER_BYTES") == f"{reduced.OVERVIEW_HEADER.size}U"
    assert _define(HEADER, "RO_ZERO_POWER_CODE") == f"({reduced.ZERO_POWER_CODE})"
    assert _define(HEADER, "RO_LOG_STEPS_PER_OCTAVE") == str(reduced.LOG_POWER_STEPS_PER_OCTAVE)
    assert _define(HEADER, "RO_MAX_STRIP_REQUEST_FRAMES") == f"{reduced.MAX_STRIP_REQUEST_FRAMES}U"


def test_core_is_built_into_the_image():
    makefile = (FIRMWARE_DIR / "makefile").read_text(encoding="utf-8")

    assert re.search(r"^SOURCES\s*=.*\breduced_overview\.c\b", makefile, re.MULTILINE)


def test_reduced_transfer_needs_the_ring_streamed_as_stored():
    guard = SOURCE[SOURCE.index("#define L3_REDUCED_TRANSFER") - 300 :]
    guard = guard[: guard.index("#define L3_REDUCED_TRANSFER")]

    for flag in ("CONFIGURABLE_CAPTURE", "HYBRID_CADENCE_CAPTURE", "SNAPSHOT_DUMP"):
        assert f"defined({flag})" in guard
    assert "!defined(L3_DUMP_IQ8)" in guard


# -- command sequencing -----------------------------------------------------------------


def test_overview_freezes_validates_then_streams_then_holds():
    body = _function("static int32_t l3_cli_overview")

    assert _before(body, "l3_stopCaptureAtBoundary", "l3_describeHeldCapture")
    assert _before(body, "ro_check_overview", "l3_writeHeldHeader")
    # every computation that can fail happens before the first response byte
    assert _before(body, "ro_prepare_overview", "l3_writeHeldHeader")
    assert _before(body, "l3_writeHeldHeader", "ro_stream_overview")
    assert _before(body, "ro_stream_overview", "gReducedHeld = 1U")
    assert "a capture is already held" in body


def test_failed_overview_resumes_capture_instead_of_staying_frozen():
    body = _function("static int32_t l3_cli_overview")

    failures = body.split("gReducedErrors++")[1:]
    assert len(failures) == 2
    for failure in failures:
        assert "l3_resumeCapture()" in failure.split("return -1")[0]


def test_strip_validates_the_request_before_streaming():
    body = _function("static int32_t l3_cli_strip")

    assert _before(body, "ro_parse_strip_request", "l3_writeHeldHeader")
    assert _before(body, "ro_check_strips", "l3_writeHeldHeader")
    assert "no held capture" in body
    assert "gReducedHoldTick = Clock_getTicks()" in body


def test_release_is_harmless_without_a_hold():
    body = _function("static int32_t l3_cli_release")

    assert "if (gReducedHeld)" in body
    assert _before(body, "gReducedHeld = 0U", "l3_resumeCapture")


def test_l3dump_streams_a_held_capture_without_refreezing():
    body = _function("int32_t l3_cli_dump")

    assert _before(body, "l3_claimReducedHold", "l3_stopCaptureAtBoundary")
    assert "if (!heldCapture)" in body
    assert "l3_writeDumpHeader(&h, heldCapture)" in body
    assert "l3_resumeCapture()" in body


def test_sensor_start_and_stop_drop_any_hold():
    for name in ("static int32_t l3_cli_sensorStart", "static int32_t l3_cli_sensorStop"):
        assert "l3_dropReducedHold()" in _function(name)


def test_abandoned_hold_times_out_and_resumes():
    body = _function("static void l3_reducedWatchTask")

    assert "L3_REDUCED_HOLD_TIMEOUT_MS" in body
    assert _before(body, "gReducedHeld = 0U", "l3_resumeCapture")
    assert "gReducedTimeouts++" in body
    assert _define(SOURCE, "L3_REDUCED_HOLD_TIMEOUT_MS") == "10000U"


def test_watchdog_never_preempts_the_hwa_rearm_task():
    assert _define(SOURCE, "L3_REDUCED_TASK_PRIORITY") == "(L3_CLI_TASK_PRIORITY - 2)"
    assert "L3_HWA_REARM_TASK_PRIORITY (L3_CLI_TASK_PRIORITY - 1U)" in SOURCE


def test_every_hold_state_change_is_locked():
    for name in (
        "static int32_t l3_cli_overview",
        "static int32_t l3_cli_strip",
        "static int32_t l3_cli_release",
        "static uint8_t l3_claimReducedHold",
        "static void l3_dropReducedHold",
        "static void l3_reducedWatchTask",
    ):
        body = _function(name)
        assert "l3_reducedLock()" in body, name
        assert body.count("l3_reducedLock()") <= body.count("l3_reducedUnlock()"), name


def test_commands_append_after_the_last_registered_one():
    """The SDK CLI stops at the first empty table slot."""
    add = _function("static void l3_addCommand")
    init = SOURCE[SOURCE.index("static void l3_initTask") :]

    assert "cliCfg->tableEntry[index].cmd != NULL" in add
    assert _before(init, "l3_registerReducedCommands(&cliCfg)", "CLI_open(&cliCfg)")
    assert _before(init, "l3_startReducedWatch()", "l3_registerReducedCommands(&cliCfg)")
    for command in ("overviewCfg", "l3overview", "l3strip", "l3release"):
        assert f'"{command}"' in _function("static void l3_registerReducedCommands")


def test_stats_reports_the_reduced_transfer():
    body = _function("static int32_t l3_cli_stats")

    assert "reduced: held=%u" in body
    for counter in (
        "gReducedOverviewMs",
        "gReducedPrepareMs",
        "gReducedTimeouts",
        "gReducedErrors",
    ):
        assert counter in body
