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

| Shot type | Share | Bytes | Transfer | Today |
|---|---|---|---|---|
| Baseline track | 61% | ~93 KB | ~0.9 s | 537 KB, 5.3 s |
| OPS-guided search (up to 9 candidate strips) | 39% | ~223 KB | ~2.2 s (max ~2.7 s) | 537 KB, 5.3 s |

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
    gate bins only, log-encoded uint16 (`round(log2(p) * 256)`);
  - burst- and window-scope noise power as float32, computed exactly as
    `PreparedShotDump.noise_power` (median of `|mti|^2` over all valid bins);
  - window-scope per-bin static means (complex, vertical pair, gate bins),
    so the Pi can form window-scope MTI from strips (~3.4 KB);
  - the existing per-frame descriptors (start, count, time delta) and header.
- **Strips reuse the existing v7 variable-width timed format**, IQ8 as stored,
  all 3 TX (the horizontal proxy uses the third TX). One contiguous window per
  frame; the union of candidate corridors when several tracks are requested.
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
- **Build toolchain.** Phases 2+ need the licence-gated TI installers
  (`firmware/ti_installers/`), which are not yet available.

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

- [ ] The chip model's overview and strips are packed/parsed by one
      executable definition (like `pack_dump`), with round-trip tests.
- [ ] With an unquantised overview, the reduced path matches the full-data
      path bit-for-bit on every saved dump, for both MTI scopes, including
      OPS-guided candidates in window scope.
- [ ] With the uint16 overview, accuracy vs the R10 pairs is not worse than
      the full-data path (typical and mean error, count within 2 deg).
- [ ] Byte counts per shot are logged and match the budget above.
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

- [ ] `make -C firmware docker-build` produces an image; its behaviour matches
      the checked-in `l3_dump_configurable_capture_20260818.bin` on a static
      capture (same header, dimensions, and byte count).

---

## Phase 2: Firmware overview, strips, and release

### What to build

`overviewCfg`, `l3overview`, `l3strip`, and `l3release` CLI commands in
`l3_dump.c`, matching the Phase 0 chip model byte-for-byte. The ring stays
frozen between `l3overview` and `l3release`, with a re-arm timeout.

### Acceptance criteria

- [ ] From one freeze, `l3overview` + `l3strip` output equals the Phase 0 chip
      model applied to an `l3dump` of the same freeze.
- [ ] `stats` reports overview compute time, strip requests, release, and
      timeout re-arms; no HWA misses or EDMA errors across repeated cycles.
- [ ] A host that never sends `l3release` does not leave the radar frozen.
- [ ] `l3dump` behaviour is unchanged.

---

## Phase 3: Host integration behind a flag

### What to build

`--iwr6843-reduced-transfer` in the IWR6843 monitor: overview, OPS-speed
wait, track selection, strip requests, release. Falls back to `l3dump` on any
failure. `--debug` can still request a full dump.

### Acceptance criteria

- [ ] Per-stage timings (overview, strips, processing) in the session log.
- [ ] Fallback to `l3dump` is exercised by tests for timeout, short reads, and
      malformed responses.
- [ ] One R10-paired field session: IWR result latency and launch-angle error
      reported against the full-dump path.

---

## Phase 4: Default on

- [ ] Enabled by default after Phase 3 field validation; the flag remains to
      force `l3dump`.
