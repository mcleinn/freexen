# XenControl Firmware (Teensy 4.1)

This repo contains the firmware for the XenControl controller board (Teensy 4.1) used with 5 IO boards (56 keys each, 280 total) arranged like a Lumatone-style hex keyboard.

The firmware multiplexes analog hull sensor signals and turns them into MIDI note events. It is designed to be tuned empirically: noise, settling/carryover, scan rate, and per-key dynamics vary with sensors/magnets/wiring.

## Architecture (Condensed)

- **Keys**: 280 total (`NUM_KEYS`), 5 boards x 56 keys (`NUM_KEYS_PER_BOARD`).
- **Indexing**:
  - `key` is 0..279.
  - `board = key / 56` (0..4)
  - `boardKey = key % 56` (0..55)
- **Analog path (unscaled-only)**:
  - XenControl mux selects a key, output is converted to a 3.3V-safe signal (`MUX*_OUT_3V3`).
  - Teensy reads the unscaled ADC path only (see `src/adc.*`).
- **Scanning**:
  - The mux is stepped once per address; for each mux address two keys are read (lower + upper bank) because the same mux position is reused across the two ADC channels.
  - Settling delay after mux switching is configurable (`sd<us>`).
  - ADC averaging is configurable (`a<avg>`).
- **Calibration (unscaled-only)**:
  - Baseline (zero) per key: robust mean over a time window.
  - Noise sigma per key: Welford running variance.
  - Threshold per key: `max(minThr, kSigma * sigma)`.
  - Dynamics per key: max swing captured on first press.
  - Polarity per key: inferred at press time.

## Build

This is a PlatformIO project.

From the project folder:

```sh
"C:\\Users\\tobia\\.platformio\\penv\\Scripts\\platformio.exe" run
```

## Serial Command Protocol

- Commands are ASCII, terminated by `\n`.
- Format: `<letters><comma-separated-numbers>`
  - Example: `k1,280\n` (select all keys)
  - Example: `a16\n` (set ADC averaging)

### Output format switch

- `fmt0` -> human-readable
- `fmt1` -> JSONL (machine readable; recommended for MCP)

When JSONL is enabled, helper/test commands emit one JSON object per line.

## Core Commands

- `h` help
- `k` select key range (`k`, `k<key>`, `k<from>,<to>`)
- `kc` select key range and highlight via LEDs
- `l` set loop mode (0 pause, 1 plot, 2 calib, 3 run)
- `r` enter/exit run loop
- `p` enter/exit plot loop
- `a` set ADC averaging (1,2,4,8,16,32)
- `sd` set mux settle delay in microseconds
- `t` set threshold (volts) for selected keys
- `ccalib` clear calibration for selected keys
- `x` calibration loop (guided)
- `n` measure noise for selected keys (duration seconds)

## Helper Commands (MCP-friendly)

These commands return structured outputs for analysis.

- `rk` / `rk<key>`: one key snapshot (adc, volts, baseline, swing, threshold, etc.)
- `rs<key>,<ms>`: min/mean/max/sigma for one key over time
- `pr<seconds>`: per-key noise report for selected range (streams rows)
- `rate<ms>`: scan throughput estimate (keys/s, mux steps/s)
- `st<keyA>,<keyB>,<delay_us>,<samples>`: carryover error stats for a pair
- `sts<keyA>,<keyB>`: sweep a default delay set and recommend a settle delay
- `rst`: reset Teensy (software reset)

## Test Commands (ordered)

Tests are intended to stop early if fundamentals are broken.

- `testa`: self-check/config echo
- `testb`: sentinel press test (board 1 keys 1..16 + first key of boards 2..5) to sanity-check on-board muxing + board connectivity
- `testc`: settling/carryover test (pair chosen from `testb`/baseline scan)
- `testd<seconds>`: noise survey table
- `teste`: performance sweep (averaging x settle delay)

Physical-neighbor coupling tests are planned but deferred until a key layout map is added.

## MCP2Serial

See `XenMCP.yaml` for an initial MCP2Serial mapping.
