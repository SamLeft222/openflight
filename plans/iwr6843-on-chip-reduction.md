# Plan: IWR6843 On-Chip Data Reduction

> Source: offline study on 2026-09-24 of 70 shots paired with a Garmin R10
> (sessions `20260923_130739`, `131647`, `134628`; dense IQ8 profile).

## Problem

The IWR6843 result arrives ~6.4 s after impact. 5.3 s of that is streaming
the full 537 KB capture over the 1 Mbaud UART; Pi processing is ~1.1 s. The
launch-angle pipeline only ever reads a small part of that capture.

## Approach

The chip sends a small **power overview** first; the Pi runs its **unchanged**
ball tracker and OPS-guided track search on it, then requests only the thin
**strips** of complex data along the chosen track (or each candidate track).
Every tracking and estimation decision stays on the Pi, so estimator changes
never require reflashing.

Measured with the Phase 0 reference model on all 77 saved captures
(`scripts/iwr6843/evaluate_reduced_transfer.py`):

| Shot type | Captures | Bytes (median) | Transfer | Today |
|---|---|---|---|---|
| Baseline track (1 strip request) | 46 | 105 KB | ~1.0 s | 537 KB, 5.3 s |
| OPS-guided search (2 strip requests) | 31 | 280 KB | ~2.7 s | 537 KB, 5.3 s |
| All | 77 | | **~1.0 s median** | 5.3 s |

Transfer times assume today's measured ~103 KB/s. The Pi must still wait for
the OPS ball speed (~1.9 s) before confirming a track, so the expected
end-to-end IWR result is ~2.5-3 s instead of ~6.4 s. These are estimates from
byte counts, not hardware measurements.

## Evidence (offline, 70 paired shots)

1. **The angle fit is local to the track.** Given the same track and the full
   capture's noise power, a +/-1-bin strip (38.5 KB IQ8) reproduced the LCMF
   angle bit-for-bit on 62/62 shots (burst MTI scope). Horizontal was
   identical on every shot that had one.
2. **The tracker and OPS-guided search read only `loop_power(mti)`** inside the
   ball gates (2.25 m to net - 0.25 m; bins 48-98 on the current profiles).
   Bins outside the gates cannot change a detection.
3. **Overview precision.** A log-encoded uint16 power map (1/256-octave steps)
   left accuracy vs the R10 unchanged (solid shots, R10 LA > 5, n = 50: typical
   2.16 deg, mean 3.81 deg, 24 within 2 deg, same as full data); median shift
   from the full-data angle 0.00 deg, one shot moved 10.7 deg. uint8 at 0.5 dB
   or 1 dB was worse (mean 4.08 / 4.53 deg, more >5 deg misses): rejected.
4. **Both MTI scopes are needed.** Running the OPS-guided search on the burst
   map only raised the mean error from 3.81 to 4.48 deg and moved one shot
   37.5 deg. The window-scope map is sent every time (27.6 KB).
5. **Tracker sensitivity.** `find_ball` is a seeded RANSAC; changing a single
   detection reshuffles its draws. Bit-identical output is therefore not a
   usable acceptance test for any change to the overview; accuracy vs a
   reference is.

## Architectural decisions

- **Decision logic stays on the Pi.** The chip computes MTI, power, and noise
  and serves strips. It never chooses a track or an angle.
- **`l3dump` is kept unchanged** as the fallback path and for `--debug` raw
  captures.
- **Overview contents** (vertical TX pair 0/2, the same data the Pi uses today):
  - burst-scope and window-scope `loop_power`, rows = frame x loop, columns =
    gate bins only, log-encoded int16 (`round(log2(p) * 256)`, with
    -32768 reserved for exactly zero);
  - burst- and window-scope noise power as float32, computed exactly as
    `PreparedShotDump.noise_power` (median of `|mti|^2` over all valid bins);
  - window-scope per-bin static means (complex64, vertical pair, every bin
    any frame window holds), so the Pi can form window-scope MTI from strips
    (~6 KB);
  - the existing per-frame descriptors (start, count, time delta) and header.
- **Strips reuse the existing v7 variable-width timed format**, IQ8 as stored,
  all 3 TX (the horizontal proxy uses the third TX). One contiguous window per
  frame; the union of candidate corridors when several tracks are requested.
  The format needs at least one bin per frame, so an unrequested frame carries
  the first bin of its window.
- **The ring stays frozen** from `l3overview` until `l3release`, or until a
  firmware timeout (proposal: 10 s) re-arms it if the Pi disappears.
- **Gate bins are configured by the Pi** (`overviewCfg <loBin> <hiBin>`,
  derived from the net distance) so firmware and tracker cannot drift.
- **Lossless vertical projection.** Today `project_tx_pair` re-quantises IQ8
  with a new per-frame scale derived from the frame's maximum. With strips
  that maximum changes, so the projection must keep the stored IQ8 values
  exactly. Offline, the lossless projection moved individual angles (median
  0.43 deg in the full pipeline) without changing accuracy vs the R10
  (typical 1.94 vs 1.79 deg, n = 43), so this is a behaviour change to accept
  deliberately, not a fix.
- **Feature flag.** The reduced transfer ships behind
  `--iwr6843-reduced-transfer` with automatic fallback to `l3dump` on any
  protocol error, timeout, or malformed response.

## Out of scope / open questions

- **Club path** (experimental) uses pre-impact frames outside the ball gates.
  Either drop it in reduced mode or request an additional club strip.
- **On-chip compute time** for MTI + power + two medians on the R4F is not
  measured; the estimate is tens of ms. The DSP stays unused.
- **UART round-trip latency** for request/response is not measured.
- **Build toolchain.** Resolved: the Docker toolchain builds the release
  byte-for-byte (Phase 1).

---

## Phase 0: Python chip model and host refactor (no firmware)

**User stories**: As a developer, I can prove on saved dumps that the reduced
transfer produces the same shots before any firmware is written.

### What to build

A reference "chip model" that turns a full dump into an overview and serves
strip requests, and a host path that consumes overview + strips through the
existing tracker, OPS-guided search, and LCMF. `find_ball` and
`_candidate_tracks_for_scope` accept a precomputed power map;
`PreparedShotDump` accepts supplied noise values and window-scope means.

### Acceptance criteria

- [x] The chip model's overview and strips are packed/parsed by one
      executable definition (`openflight.iwr6843.reduced`), with round-trip
      and malformed-input tests.
- [x] With an unquantised overview, the reduced path matches the full-data
      path: 77/77 saved captures, including the 31 that ran the OPS-guided
      search -- bit-for-bit with the original float overview; with the rc2
      integer maths, identical statuses and launch angles within 3.4e-12 deg.
- [x] With the int16 log overview, accuracy vs the R10 pairs is unchanged:
      solid shots (n = 50) typical 2.16 deg, mean 3.81 deg, 24 within 2 deg,
      9 beyond 5 deg for both paths. No status changed; angle shift median
      0.00001 deg, p90 0.0001 deg; one capture moved 10.7 deg (a top the R10
      read at 2.0 deg, wrong on both paths).
- [x] Byte counts per shot are reported by
      `scripts/iwr6843/evaluate_reduced_transfer.py` (table above).
- [x] `project_tx_pair` preserves IQ8 values exactly; the change is covered by
      tests and its effect on the paired sessions is recorded. Replaying the
      70 paired shots (tilt 11.5): all 70 still accepted; angles moved median
      0.43 deg, p90 5.0 deg, max 12.3 deg. Solid shots vs R10 (n = 50):
      typical 2.03 -> 2.16 deg, mean 4.00 -> 3.81 deg, within 2 deg 25 -> 24,
      beyond 5 deg 8 -> 9 -- no net accuracy change.

---

## Phase 1: Reproducible firmware build

### What to build

Build the current release from source with the Docker toolchain once the TI
installers are available.

### Acceptance criteria

- [x] `make -C firmware docker-build` produces an image matching the
      checked-in `l3_dump_configurable_capture_20260818.bin`. Verified
      2026-09-24 on Apple Silicon (Docker Desktop 29.8, default Rosetta
      setting): the rebuild is byte-identical (SHA-256 `823ddd18...`), so no
      static-capture comparison is needed. Build with
      `RELEASE_NAME=<scratch>.bin` to avoid overwriting the release.

---

## Phase 2: Firmware overview, strips, and release

### What was built

* `firmware/iwr6843/reduced_overview.c/.h`: portable C89 core (no TI headers,
  no libm) that writes the overview and strips through a byte sink. Noise
  medians are exact (radix selection on the bits of non-negative doubles);
  log-power codes use a generated threshold table
  (`gen_reduced_log_table.py`). ~98 KB of working memory in MSS data RAM
  (151 KB of 192 KB used).
* `l3_dump.c` commands, registered after the build's last CLI entry:

  | Command | Effect |
  |---|---|
  | `overviewCfg <lo> <hi>` | Absolute range bins [lo, hi) the power maps cover |
  | `l3overview` | Freeze like `l3dump`, stream header + overview, **hold** the ring |
  | `l3strip <hex>` | 4 hex digits (start, count) per frame, `0000` for none; streams a timed dump of those bins from the held ring |
  | `l3release` | Resume capture; harmless when nothing is held |
  | `l3dump` (held) | Streams the held ring unchanged, then resumes |

  Requests are validated before any byte is streamed. `sensorStart` and
  `sensorStop` drop a hold. A priority-1 watch task resumes a hold the Pi has
  been silent on for 10 s. Every response from one freeze carries the
  temperature report read at the freeze.
* `IWR6843Radar.configure_overview_gate / read_overview / read_strips /
  release` on the Pi; `scripts/hardware-test/test_iwr6843_reduced_transfer.py`
  for the on-radar check.
* Candidate image: `firmware/candidates/l3_dump_reduced_transfer_rc3.bin`
  (not the release until this phase passes on hardware).

### Hardware results

**rc1 (2026-09-24, float maths):** all 23 checks passed on the radar --
overview and strips byte-identical to the reference from the same freeze
(3 cycles), release, error paths, and the 10 s timeout. But the overview
took 2.96 s in firmware (~2.3 s compute + ~0.6 s UART): ~200 cycles per
sample read, ~10x the estimate. That would leave the IWR result at ~4.5 s.

**rc2 (integer maths):** stored codes decoded once per pass into fast
memory; burst MTI power is exactly s^2 |L c - sum c|^2 / L^2 and window power
|n s c - T|^2 / n^2, each rounded once (reduced.py uses the same maths, so C
and Python still match byte-for-byte on all 80 captures). Against the float
full-capture path: 0 status changes, launch angles within 3.4e-12 deg
unquantised; accuracy vs the R10 unchanged. `stats` now also reports
`prepare_ms` (compute only). On the radar the Pi received only the command
echo: rc2 computes before its first byte, and the reader's 4 s stall timer
counted from the echo. Either the compute exceeded 4 s or it hung -- the
suspect is 64-bit integer multiplies and int64-to-double conversions, which
the R4F runs as library calls.

**rc3:** no 64-bit integers in the maths -- every intermediate is an exact
integer below 2^53, so it is carried in doubles with bit-identical results
(C and Python still match on all 80 captures). Compiling rc2's core with
the TI toolchain confirmed the suspicion: it called `__aeabi_lmul` and
`__aeabi_l2d` per value. rc3 calls neither, and the median's key is split
into 32-bit halves so its per-value work needs no 64-bit shift helper
either (those remain once per pass). The reader now starts its stall timer
only once the response has begun, and reports a missing response instead
of returning the echo.

On the radar (2026-09-24) rc3 passed all 23 checks: overview and strips
byte-identical to the reference from the same freeze in 3 cycles, release,
error paths, 10 s timeout, l3dump after the timeout. Timings (median):

| | rc1 | rc3 |
|---|---|---|
| compute before the first byte (`prepare_ms`) | ~2.3 s | 1.26 s |
| overview in firmware (`overview_ms`) | 2.96 s | 2.16 s |
| overview as the Pi receives it | 3.01 s | 2.22 s |
| strips (diagonal / whole windows / none) | 0.29 / 0.54 / 0.10 s | same |
| full `l3dump` | 5.28 s | 5.28 s |

The overview now lands ~0.3 s after the OPS speed (~1.9 s). Estimated IWR
result ~3.5-3.8 s vs ~6.4 s today (to be measured in Phase 3). The compute
is ~9 passes over the frozen ring at ~0.15 s each (sums, 3 per noise
median, 2 for the power maps); sharing the two medians' passes would remove
3 of them.

### Acceptance criteria

- [x] From one freeze, `l3overview` + `l3strip` output equals the Phase 0 chip
      model applied to an `l3dump` of the same freeze (rc1 and rc3 on the
      radar, 3 cycles each; host: all 80 saved captures).
- [ ] `stats` reports overview compute time, strip requests, release, and
      timeout re-arms (done); no HWA misses or EDMA errors across repeated
      cycles (not yet checked -- the hardware script does not read those
      counters).
- [x] A host that never sends `l3release` does not leave the radar frozen
      (timeout resumes capture after 10 s; checked on rc1 and rc3).
- [x] `l3dump` behaviour is unchanged (refactored into shared header, frame
      plan, and resume helpers; verified on the radar before, between, and
      after reduced transfers).

## Phase 3: Host integration behind a flag

### What was built

* `--iwr6843-reduced-transfer`: on a trigger edge the capture monitor sends
  `overviewCfg` (at start) and `l3overview`, and the radar holds its ring.
  `IWR6843Runtime.process_shot` measures the held capture with
  `measure_reduced`, fetching strips through `IWR6843CaptureMonitor.read_strips`
  (one serial lock), then `finish_capture` releases the ring.
* `--iwr6843-reduced-full-dump`: stream each held ring whole after measuring
  (offline analysis, club path) at the full dump's time cost. Without it the
  experimental club path is skipped (recorded in the transfer record).
* Fallbacks, each tested: overview failure -> ordinary `l3dump`; any failure
  of the reduced measurement -> the held ring streamed whole and measured as
  today; released firmware without `overviewCfg` -> full dumps (the runtime
  config reports the transfer actually in use); an unconsumed hold is
  released after 8 s (the firmware's own timeout is 10 s); shutdown releases
  a hold before `sensorStop`.
* `iwr6843_capture` session entries carry a `transfer` record: mode, overview
  bytes and seconds, strip bytes and seconds, fallback reason, full dump.

### Acceptance criteria

- [x] Per-stage timings (overview, strips, full dump) in the session log;
      processing is the shot's existing `pipeline_ms.iwr6843`.
- [x] Fallback to `l3dump` is exercised by tests for timeout, missing
      responses, strip failures, measurement errors, and old firmware.
- [ ] One R10-paired field session: IWR result latency and launch-angle error
      reported against the full-dump path.

---

## Phase 4: Default on

- [ ] Enabled by default after Phase 3 field validation; the flag remains to
      force `l3dump`.
