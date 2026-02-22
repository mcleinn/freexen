# Test Guidelines (Safe + Repeatable)

This document describes how to run XenControl firmware tests safely and repeatably.

## Safety Rules

- **One serial connection at a time**: COM3 can only be opened by one process.
  - Close Arduino Serial Monitor / PlatformIO Monitor / MCP2Serial / Python scripts before starting another.
- **Avoid accidental key presses** during idle/noise tests.
- **Calibrate before using `idle`**:
  - `idle<seconds>` only counts false triggers for keys with `hasZero=1`.
- **Prefer JSONL during analysis**:
  - `fmt1` enables JSONL output.
- **Log everything**:
  - Record `ver`, date/time, and the test outputs in `TEST_LOG.md`.

## Quick Start (Recommended Order)

1) Identify firmware + enable JSONL
- `ver`
- `fmt1`

2) Gate hardware routing (guided)
- `testb`
  - Confirms: on-board mux sequencing (board 1 keys 1..16) + board presence (boards 2..5 first key).

3) Baseline analog sanity (no pressing)
- Select full range: `k1,280`
- Noise report: `pr2` (or `pr5`)
  - Use `thrSuggest` to see if a key is inherently noisy.

4) Calibrate a small subset (guided)
- Select a subset (example: first board): `k1,56`
- Calibrate: `x`

5) Runtime stability (no pressing)
- Idle false-trigger audit: `idle30`
  - If `total` is high, increase threshold/min-threshold or move averaging 4->8.

6) Runtime responsiveness
- `rperf5`
  - Watch `keysPerSec` and `revisitMsEst`.

7) Tuning sweeps (diagnostic, no pressing)
- Settling sweep: `sts1,57` (better: pick a worst-case pair)
- Averaging sweep: `teste`

## Command Notes

- `a<avg>` changes ADC averaging (1,2,4,8,16,32).
- `sd<us>` changes mux settle delay in microseconds.
- `pr<seconds>` measures per-key noise and prints `thrSuggest = max(0.03, 8*sigma)`.
- `idle<seconds>` checks false triggers (requires calibration on those keys).
- `rperf<seconds>` measures scan throughput under full scan.

## Repeatability Tips

- Run tests with consistent settings:
  - set `a` and `sd` explicitly before each run.
- For comparisons, keep:
  - the same key range (`k`)
  - the same duration (`pr2`, `idle30`)
  - the same environment (USB cable, power, no touching keys)

## Troubleshooting

- **"Access is denied" opening COM3**: another process still has COM3 open.
- **Windows console Unicode crash (cp1252)**: some outputs contain non-ASCII (e.g. `σ`).
  - If a Python script crashes printing serial output, print via UTF-8 bytes:

```python
import sys
sys.stdout.buffer.write(text.encode("utf-8", "replace"))
```

  - Or set: `set PYTHONUTF8=1` before running scripts.
- **MCP2Serial timeouts**: mcp2serial v0.1.0 only reads what is available immediately; use the firmware `OK` ACK or run direct serial for long outputs.
- **Old `calib.csv`**: if `loadCalibrationCSV()` detects old format it starts uncalibrated and prints instructions.
