# XenControl Firmware Test & Modification Log

This log is append-only. Each entry captures:
- firmware version (`ver`)
- date/time
- what changed
- key test results + brief analysis

## 2026-02-22 10:14:58

### fw=16

Tuning run: a4 + sd5

Results (COM3)
- `sd5` applied.
- `a4` applied.
- `rate1000` after a4+sd5:
  - muxStepsPerSec: 3864
  - keysPerSec: 6440
- `pr2` after a4+sd5 (head only):
  - sigma (keys 1..5): ~2.4–3.8 mV
  - `thrSuggest` mostly clamps at 30 mV
- `sts1,57` after a4+sd5:
  - best delay_us in {0,5,10,20,50,100,200}: 100 us
  - best score (maxErr): 8.159 mV
- `teste` after a4+sd5:
  - avg=4: keysPerSec ~43120, sigmaP95 ~7.74 mV
  - avg=8: keysPerSec ~23520, sigmaP95 ~7.30 mV
  - avg=16: keysPerSec ~12320, sigmaP95 ~6.38 mV
  - avg=32: keysPerSec ~6440,  sigmaP95 ~5.13 mV

Notes
- Settling sweep appears noisy/unstable across runs; we should pick a worst-case key pair explicitly and run multiple repetitions.
- Test output is still interleaved with peak-detect debug prints; add a quiet/test-only flag to suppress those during diagnostics.

Changes
- Reverted serial command parsing to leading-letters tokens.
- Renamed tests to letter-only commands (`testa..teste`).

Non-interactive test results (COM3)
- `rate1000`:
  - muxStepsPerSec: 3696
  - adcReadsPerSec / keysPerSec: 6182
  - full-scan-equivalent (280 keys): ~45 ms
- `pr2` (2s noise survey):
  - typical sigma: ~3–5 mV
  - `thrSuggest = max(30 mV, 8*sigma)` => often 30 mV clamp
  - most keys uncalibrated (`hasZero=0`), expected
- `sts1,57` (settle sweep, keyA=1 keyB=57, 50 samples):
  - best delay_us in {0,5,10,20,50,100,200}: 5 us
  - best score (maxErr): 6.749 mV
- `teste` (averaging sweep; p95 sigma vs throughput):
  - avg=1:  ~85960 keys/s, sigmaP95 ~14.36 mV
  - avg=2:  ~85960 keys/s, sigmaP95 ~14.14 mV
  - avg=4:  ~38360 keys/s, sigmaP95 ~7.77 mV
  - avg=8:  ~22120 keys/s, sigmaP95 ~7.29 mV
  - avg=16: ~12040 keys/s, sigmaP95 ~6.15 mV
  - avg=32: ~6440 keys/s,  sigmaP95 ~4.98 mV

Analysis
- Averaging sweet spot looks like 4 or 8 (big noise reduction vs large speed gain over 32).
- Mux settle delay could likely be reduced from 10 us to ~5 us, but should be re-tested on a worst-case key pair.
- Some debug prints suggest scan/peak logic may still run during tests; tests should run in a quiet mode for cleaner logs.

## 2026-02-22 10:20:03

### fw=16

Tuning run: a4 sd5 rate/pr/sts/teste

## 2026-02-22 10:22:00

### fw=17

Changes
- Added diagnostic mode gating so helpers/tests run without normal scanning or peak detection.
- Wrapped long-running helpers/tests with diag begin/end so they take over LEDs and timing.

Run (COM3)
- `fmt1`, `a4`, `sd5`
- `rate1000`
- `k1,280` + `pr2`
- `sts1,57`
- `teste`

Results
- `ver`: fw=17
- `rate1000` (after a4+sd5, diag mode):
  - muxStepsPerSec: 25872
  - keysPerSec: 43274
  - full-scan-equivalent (280 keys): ~6.5 ms
- `pr2` (2s noise survey, head):
  - typical sigma (keys 1..30): ~1.1–1.9 mV
  - `thrSuggest` still clamps at 30 mV for most keys
- `sts1,57` (settle sweep, 50 samples):
  - best delay_us in {0,5,10,20,50,100,200}: 50 us
  - best score (maxErr): 5.339 mV
  - observed outlier at 0 us (maxErrAB ~17.4 mV)
- `teste` (averaging sweep, p95 sigma vs throughput):
  - avg=1:  keysPerSec ~115640, sigmaP95 ~10.14 mV
  - avg=4:  keysPerSec ~43120,  sigmaP95 ~5.14 mV
  - avg=8:  keysPerSec ~23520,  sigmaP95 ~3.98 mV
  - avg=16: keysPerSec ~12320,  sigmaP95 ~2.86 mV
  - avg=32: keysPerSec ~6440,   sigmaP95 ~2.38 mV

Analysis
- With diag gating, rate/noise measurements are much cleaner and faster.
- Averaging sweet spot remains 4–8 depending on responsiveness vs cleanliness.
- Mux settle delay of 0 us can create rare but large carryover spikes; 10–50 us looks safer.

## 2026-02-22 10:34:00

### fw=18

Changes
- Added discard-first-sample ADC reads after each mux switch in runtime scan and noise scan.

Results (COM3)
- `teste` (averaging sweep, p95 sigma vs throughput):
  - avg=1:  keysPerSec ~85960, sigmaP95 ~12.39 mV
  - avg=2:  keysPerSec ~85960, sigmaP95 ~12.60 mV
  - avg=4:  keysPerSec ~38360, sigmaP95 ~5.69 mV
  - avg=8:  keysPerSec ~22120, sigmaP95 ~4.31 mV
  - avg=16: keysPerSec ~12040, sigmaP95 ~3.26 mV
  - avg=32: keysPerSec ~6440,  sigmaP95 ~2.59 mV

Analysis
- After discard-first-sample, avg=4 is still a good default for responsiveness.
- avg=8 is a nice "clean" mode with ~1.3x lower sigmaP95 than avg=4 but ~1.7x slower.

## 2026-02-22 10:40:00

### fw=19

Changes
- Set default ADC averaging to 4 on boot (previously 32).

Verification
- `ver`: fw=19

## 2026-02-22 12:54:02

### fw=28

Results (testb JSONL)
- `testb_done`: passCount=20 failCount=0 total=20 pass=1
- Typical baseline: ~1.66–1.73 V
- Typical sigma: ~1.3–2.0 mV
- Threshold used: 30 mV (minThr clamp)
- Notable low swing sentinel: key 169 (board 4 key 1) maxSwing ~58 mV (still passed)

testb full run (JSONL) pass=20 fail=0

## 2026-02-22 13:06:42

### fw=30

Results (testb JSONL)
- `testb_done`: passCount=23 failCount=0 total=23 pass=1
- Polarity distribution (from `testb_row.pol`): +1 = 16, -1 = 7
- Weakest sentinel: key 169 (board 4 key 1) maxSwing ~58.9 mV, pol=-1 (still passed at thr=30 mV)

testb updated: includes pol + extra subtest keys

## 2026-02-22 14:37:10

### fw=38

Calibrate board 1 (k1,56) after calib/testb parity

Run (COM3)
- Connected later (DTR) -> debug banner includes `fw=38` and `calibAutoLoadOk=0`.
- `fmt1`
- `k1,56`
- `ccalib`
- `x` (guided calibration for keys 1..56)

Key findings
- Calibration now emits `testb`-style baseline logs per key (baseline/sigma/thr/min/max) and uses discard-first-sample after mux switch during baseline capture.
- Key 3, which previously produced inflated sigma/threshold, calibrated normally.
- Calibration completed for all 56 keys with no issues:
  - `calib_done`: Skipped=0, NotTriggering=0, SelfTriggering=0, NotReleasing=0
- Persistence/coverage after run:
  - `calstat`: hasZero=56, hasThr=56, nSigma=56
  - observed sigma range on board 1: minSigma~0.001365, maxSigma~0.001911
