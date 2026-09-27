"""Firmware glue for the on-radar shot detector (auto_trigger.c in l3_dump.c).

The detector's decisions are checked against the Python reference by
test_iwr6843_auto_trigger_firmware.py. These checks pin the sequencing in
l3_dump.c that cannot run off-target: the detector never delays a frame
re-arm, its freeze reaches the right waiter, a command can never restart or
lose a detected capture, and nothing leaves the radar frozen.
"""

from __future__ import annotations

import re

from test_iwr6843_firmware_reduced import FIRMWARE_DIR, SOURCE, _before, _define, _function

from openflight.iwr6843 import autotrigger
from openflight.iwr6843.driver import AUTO_TRIGGER_NOTICE_MAGIC

AT_HEADER = (FIRMWARE_DIR / "auto_trigger.h").read_text(encoding="utf-8")


def test_constants_match_the_python_reference():
    assert _define(SOURCE, "L3_AUTO_NOTICE_MAGIC") == f'"{AUTO_TRIGGER_NOTICE_MAGIC}"'
    assert _define(AT_HEADER, "AT_MAX_FRAMES") == f"{autotrigger.MAX_FRAMES}U"
    assert _define(AT_HEADER, "AT_MAX_WINDOW_BINS") == f"{autotrigger.MAX_WINDOW_BINS}U"
    assert _define(AT_HEADER, "AT_BIN_SPACE") == f"{autotrigger.BIN_SPACE}U"


def test_core_is_built_into_the_image():
    makefile = (FIRMWARE_DIR / "makefile").read_text(encoding="utf-8")

    assert re.search(r"^SOURCES\s*=.*\bauto_trigger\.c\b", makefile, re.MULTILINE)


def test_needs_the_iq16_scratch_and_the_reduced_hold():
    guard = SOURCE[: SOURCE.index("#define L3_AUTO_TRIGGER 1")]
    guard = guard[guard.rindex("#if") :]

    assert "defined(L3_REDUCED_TRANSFER)" in guard
    assert "defined(L3_RING_IQ8)" in guard


# -- the detector in the frame loop ---------------------------------------------------


def test_detector_runs_after_the_frame_is_rearmed_and_packed():
    task = _function("static void l3_hwaRearmTask")

    assert _before(task, "l3_restartCompletedHwaFrame()", "l3_autoTriggerFrame(")
    assert _before(task, "l3_startIq8EdmaPack(pendingSlot, pendingScratch)", "l3_autoTriggerFrame(")


def test_detector_only_looks_at_pre_impact_frames_while_enabled():
    body = _function("static void l3_autoTriggerFrame")

    guard = body[body.index("if (!gAutoEnabled") :].split("\n")[0]
    for condition in ("!gAutoEnabled", "gHwaFreezeRequested", "slot >= gCapturePlan.preFrames"):
        assert condition in guard
    assert "gCapturePlan.preStart, gCapturePlan.preBins" in body


def test_configuration_changes_apply_between_frames():
    body = _function("static void l3_autoTriggerFrame")
    cli = _function("static int32_t l3_cli_autoTrigCfg")

    assert _before(body, "Hwi_disable()", "gAutoCfg = gAutoCfgNext")
    assert _before(body, "gAutoCfg = gAutoCfgNext", "at_push_frame(")
    assert _before(body, "gAutoResetRequested = 0U", "at_push_frame(")
    assert _before(cli, "Hwi_disable()", "gAutoCfgNext = cfg")
    assert _before(cli, "gAutoCfgNext = cfg", "Hwi_restore(key)")


def test_a_detection_requests_the_same_post_frame_freeze_as_l3dump():
    body = _function("static void l3_autoTriggerFrame")
    request = body[body.rindex("Hwi_disable()") :]

    assert "if (gCaptureActive && !gHwaFreezeRequested)" in request
    for line in (
        "gHwaFreezeRequested = 1U",
        "gPostCaptureStarted = 0U",
        "gPostFramesCaptured = 0U",
        "gAutoFreezePending = 1U",
        "gAutoFireTick = Clock_getTicks()",
    ):
        assert line in request
    assert _before(request, "gAutoFreezePending = 1U", "Hwi_restore(key)")


def test_the_detector_freeze_wakes_the_auto_task_not_a_command():
    task = _function("static void l3_hwaRearmTask")
    completion = task[task.index("if (freezeAfterPack)") :]

    assert _before(completion, "if (gAutoFreezePending)", "Semaphore_post(gAutoFrozenSemaphore)")
    assert _before(
        completion, "Semaphore_post(gAutoFrozenSemaphore)", "Semaphore_post(gHwaFreezeSemaphore)"
    )
    assert "gAutoFrozen = 1U" in completion.split("Semaphore_post(gAutoFrozenSemaphore)")[0]


def test_a_command_adopts_a_detector_freeze_instead_of_restarting_it():
    body = _function("static int32_t l3_freezeHwaAfterPostFrames")
    adopt = body[body.index("if (gAutoFreezePending)") :]

    assert _before(body, "if (gAutoFreezePending)", "gPostFramesCaptured = 0U")
    assert "gAutoFreezePending = 0U" in adopt.split("return")[0]
    assert "Semaphore_pend(gHwaFreezeSemaphore, 250U)" in adopt.split("}")[0]


def test_shutdown_cancels_a_detector_freeze():
    body = _function("static int32_t l3_freezeHwaForShutdown")

    assert _before(body, "gHwaShutdownRequested = 1U", "Hwi_restore(key)")
    assert "gAutoFreezePending = 0U" in body


def test_every_ring_restart_resets_the_detector():
    for name in ("static int32_t l3_resumeCapture", "static int32_t l3_armHwaChain"):
        assert "gAutoResetRequested = 1U" in _function(name), name


# -- the hold ------------------------------------------------------------------------


def test_every_hold_entry_point_takes_a_detected_capture_first():
    for name in (
        "static int32_t l3_cli_overview",
        "static uint8_t l3_claimReducedHold",
        "static void l3_dropReducedHold",
        "static int32_t l3_cli_release",
        "static void l3_autoTriggerTask",
    ):
        body = _function(name)
        assert _before(body, "l3_reducedLock()", "l3_autoTakeHold()"), name


def test_taking_the_hold_stops_describes_and_marks_it_held():
    body = _function("static int32_t l3_autoTakeHold")

    assert _before(body, "if (!gAutoFrozen)", "gAutoFrozen = 0U")
    assert _before(body, "l3_finishCaptureStop()", "gReducedHeld = 1U")
    assert _before(body, "l3_describeHeldCapture()", "gReducedHeld = 1U")
    assert "gReducedHoldTick = Clock_getTicks()" in body
    failure = body[body.index("gAutoErrors++") :].split("return -1")[0]
    assert "l3_resumeCapture()" in failure


def test_overview_serves_a_detected_hold_without_freezing_again():
    body = _function("static int32_t l3_cli_overview")
    auto = body[body.index("if (gReducedHeld && gAutoHeld)") :].split("} else")[0]

    assert "gAutoHeld = 0U" in auto
    assert "ro_check_overview" in auto
    assert "l3_stopCaptureAtBoundary" not in auto
    assert _before(body, "if (gReducedHeld && gAutoHeld)", "a capture is already held")


def test_failed_overview_of_a_detected_hold_does_not_stay_held():
    body = _function("static int32_t l3_cli_overview")
    gate_check, _describe, prepare = body.split("gReducedErrors++")[1:]

    # a detected hold is marked held before these can fail
    for failure in (gate_check, prepare):
        assert _before(failure, "gReducedHeld = 0U", "l3_resumeCapture()")


def test_notice_is_written_whole_and_outside_the_lock():
    task = _function("static void l3_autoTriggerTask")
    writer = _function("static void l3_autoWriteNotice")

    assert _before(task, "l3_reducedUnlock()", "l3_autoWriteNotice(")
    assert "if (taken == 1)" in task
    assert _before(writer, "Task_disable()", "UART_writePolling(gCliUart")
    assert _before(writer, "UART_writePolling(gCliUart", "Task_restore(taskKey)")
    for field in ("seq=", "age_ms=", "delay=", "end=", "slope_q=", "frames="):
        assert field in writer


def test_hold_timeout_also_covers_detected_captures():
    body = _function("static void l3_reducedWatchTask")

    assert "gAutoHeld = 0U" in body


def test_auto_task_never_preempts_the_hwa_rearm_task():
    assert _define(SOURCE, "L3_AUTO_TASK_PRIORITY") == "(L3_CLI_TASK_PRIORITY - 2)"


def test_configuration_is_validated_and_needs_the_overview_gate():
    cli = _function("static int32_t l3_cli_autoTrigCfg")

    assert _before(cli, "at_check_config(&cfg)", "gAutoCfgNext = cfg")
    assert _before(cli, "!gOverviewGateSet", "gAutoCfgNext = cfg")
    assert '"autoTrigCfg"' in _function("static void l3_registerReducedCommands")


def test_stats_reports_the_auto_trigger():
    body = _function("static int32_t l3_cli_stats")

    assert "auto: enabled=%u fires=%u holds=%u adopted=%u errors=%u notices=%u" in body
