# Test Roadmap: 280-Key Analog Keyboard Bring-Up

This roadmap is optimized for getting a working 280-key analog keyboard quickly.

It separates:
- **Gating tests** (is hardware alive / routing correct?)
- **Runtime reality tests** (full 280-key scan + peak detect + MIDI/LED load)

Outputs should be machine-readable (JSONL / CSV) and logged with `ver` into `TEST_LOG.md`.

## Goals

- **Idle stability**: no (or near-zero) false triggers when untouched.
- **Reliable detection**: no missed presses for typical key dynamics.
- **Dynamics**: usable `maxSwing` and `dyn/sigma` across keys.
- **Responsiveness**: as fast as possible without false triggers.
- **Repeatability**: tunable defaults (`avg`, `sd`) with consistent logs.

## Conventions

- Keys: 5 boards x 56 keys = 280 total.
- `key` is 0..279 (firmware), user-facing often 1..280.
- `board = key / 56` (0..4), `boardKey = key % 56` (0..55).
- Prefer JSONL for MCP/AI (`fmt1`).

## Phase 0: Setup & Observability

Run once per firmware revision:
- `ver` (confirm firmware version)
- `fmt1` (JSONL)
- Append a log header via `tools/log_test_entry.py`.

## Phase 1: Hardware / Routing Gates (fast)

Stop if any of these fail.

1) **Board + mux bring-up** (guided)
- Command: `testb`
- What it does: prompts board 1 keys 1..16 (on-board mux sanity) and the first key of boards 2..5 (board presence).
- Pass: each prompted press is detected in time.

2) **Analog sanity sweep** (no pressing)
- Commands: `k1,280` then `pr2` (or `pr5`)
- Purpose: detect stuck-at-rail channels and gross noise outliers.
- Pass: no large groups pinned near 0V or 3.3V; sigma is within expected band.

## Phase 2: Runtime Reality Gates (full scan ON)

These tests must run under the same conditions you will actually play.

3) **Idle false-trigger audit** (no pressing, runtime)
- Add/Use a runtime audit command that runs for N seconds and outputs:
  - total trigger count
  - top-N keys by trigger count
  - optionally per-key counts
- Pass: near-zero triggers over 30–60s.

4) **Runtime scan performance**
- Add/Use a runtime performance command that measures:
  - revisit time per key (min/mean/p95)
  - effective keys/s including peak detect + LED/MIDI overhead
- Pass: revisit time meets your responsiveness target.

## Phase 3: Stabilize Measurement Pipeline (tuning loop)

Tune one knob at a time and log the change + results.

5) **Mux settling / carryover**
- Use diag characterization as a starting point:
  - `sts<keyA>,<keyB>` (repeat and look for outliers)
- Validate changes by re-running the **runtime idle audit**.
- Typical candidates: `sd5`, `sd10`, `sd20`, `sd50`.

6) **ADC averaging**
- Use diag sweep:
  - `teste` (sigmaP95 vs keys/s)
- Validate with runtime idle audit + runtime revisit times.
- Typical defaults:
  - responsive: `avg=4`
  - cleaner: `avg=8`

## Phase 4: Calibration + Dynamics (subset first)

7) **Subset calibration per board** (guided)
- Calibrate 3–6 representative keys per board first.
- Capture per key:
  - baseline (zeroV)
  - sigma
  - threshold
  - maxSwing
  - polarity
  - dyn/sigma
- Pass: dyn/sigma above a minimum for tested keys; polarity stable per key.

8) **Expand calibration coverage**
- Calibrate per board (56 keys) and then all 280.
- After each board calibration, rerun a short idle false-trigger audit.

## Phase 5: Neighbor/Coupling + Long-Run (later)

9) **Physical neighbor coupling**
- Requires a board layout map (hex-grid neighbor definitions).
- Guided hold/press tests for a small set of keys.
- Output coupling ratios; identify problematic regions.

10) **Long-run stability**
- 10–30 minutes idle + active playing.
- Log drift in baseline, trigger rates, and any stuck-key behavior.

## Acceptance Checklist

- `testb` passes on all 5 boards.
- Idle false-trigger rate is acceptable (define threshold).
- p95 revisit time meets target under full scan.
- Dynamics usable across keys (no large dead zones).
- Calibration persistence works (versioned `calib.csv`).
