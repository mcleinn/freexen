# FixNoiseTools.md

This document lists the firmware-side tools/commands we use to diagnose and fix:
- idle false triggers ("noise")
- calibration problems (missing baseline/threshold, stuck keys)
- baseline drift across boots and over time

The device prints an immediate ack line for every command:

```
OK:<raw-command> <-- <description>
```

Use `fmt1` to enable JSONL output (recommended for logging/scripts).

## Quick Start (Recommended)

1) Connect to serial (COM3, 115200).
2) Switch to JSONL:

```
fmt1
```

3) Basic health snapshot:

```
ver
calstat
bootdrift
noiselevel
autostat
idle5
```

Interpretation:
- `calstat hasZero=280 hasThr=280` means all keys are calibrated and have thresholds.
- `bootdrift_sum` reports how far boot baselines differ from `calib.csv` baselines.
- `noiselevel` reports a single sigmaP95 noise score and LOW/MEDIUM/HIGH class.
- `autostat enabledKeys>0` means autotune overlay thresholds are active.
- `idle5 total=0` means no idle crossings in the last 5 seconds.

## Calibration Tools

### calstat
Shows how many keys have calibration baseline (`hasZero`) and thresholds (`hasThr`).

Use when:
- you suspect calibration did not load
- thresholds were accidentally wiped

Example:

```
calstat
```

### rk<key>
Reads the current key model and live voltage/swing for a single key.

Use when:
- `idle` reports a top offender key
- you suspect a stuck key or bad calibration row

Example:

```
rk195
```

### lcalib / scalib
- `lcalib`: load `calib.csv` from SD into RAM.
- `scalib`: save current RAM calibration back to `calib.csv`.

Use when:
- you want to restore calibration state from disk
- you intentionally updated calibration and want to persist

## Calibration Backup Slots (SD)

These are safe, fast backups of `calib.csv` stored on SD.

### bcalib<slot>
Copies `calib.csv` to `calib_slot_NNN.csv`.

Example:

```
bcalib1
```

### rcalib<slot>
Copies `calib_slot_NNN.csv` back to `calib.csv` and loads it into RAM.

Example:

```
rcalib1
```

## Baseline Drift Tools

### bootdrift
Prints the cached drift summary measured at boot:
- how much boot baselines differ from the calibration baseline

Example:

```
bootdrift
```

### driftreset
Re-measures drift and re-applies baseline compensation.

Example:

```
driftreset
```

Notes:
- A large `bootdrift` does not necessarily mean instability. It means the saved baseline differs from the boot baseline.
- Stability is validated by `idleX` and/or by autotune strict validation.

## Noise Tools

### idle<seconds>
Counts false crossings at rest. This is our primary stability metric.

Examples:

```
idle5
idle60
idle120
```

If `idle` is nonzero:
- Inspect top offender keys (`idle_top` lines)
- Use `rk<key>` on offenders

### noiselevel (cached)
Prints the last computed noise score:
- `sigmaP95` across keys (200ms runtime noise snapshot)
- class `LOW/MEDIUM/HIGH`

Example:

```
noiselevel
```

### noisereset (remeasure) and noisereset1 (sweep)

- `noisereset`: remeasure noise under current `avg/sd_us` and print a quick playability prognosis:
  - `noiseprog dynStepsMin` and `dynDeadN`

- `noisereset1`: also prints a short per-avg sweep table while holding the current `sd_us`:
  - `noise_trial avg=.. sigmaP95=.. class=.. dynStepsMin=.. dynDeadN=..`

Examples:

```
noisereset
noisereset1
```

## Autotune Tools

### autotune<shortS>,<midS>,<strictS>,<topK>
Runs autotune and prints progress while it runs.

Example:

```
autotune5,20,120,2
```

Important:
- The strict phase is designed to converge to `total==0` by raising thresholds for offenders.
- After a run, use `autodump` to inspect best settings, dynamics prognosis, and any "!!!" flags.

### autodump
Prints the last autotune run log (JSONL).

Key records:
- `autodump`: best avg/sd and totals
- `autodump_rate`: `keysPerSec`, `scansPerSec`, `revisitMsEst`
- `autodump_dyn`: `dynStepsMin/Max`, `dynDeadN`, and `flag` (`!!!` if any key is effectively dead)

Example:

```
autodump
```

### autosave / autoload / autoreset / autostat

Autotune overlay thresholds + avg/sd are stored in `autotune.csv`.

- `autosave`: persist current overlay thresholds and avg/sd
- `autoload`: load and apply
- `autoreset`: disable overlay thresholds (set to 0)
- `autostat`: show whether overlay thresholds are active (`enabledKeys`)

Examples:

```
autostat
autosave
autoreset
autoload
```

## Common Fix Flows

### Flow A: device is unstable at idle

```
fmt1
calstat
bootdrift
noiselevel
idle5
```

If `calstat` is not `hasZero=280 hasThr=280`:
- `lcalib` (or `rcalib<slot>`)

If calibration is OK but idle is nonzero:
- run `autotune5,20,120,2`
- `autodump` (check `flag` and `dynDeadN`)
- `autosave`
- `idle120`

### Flow B: you suspect a bad key (stuck/noisy)

```
idle5
rk<top-offender>
rs<top-offender>,1000
```

Then decide:
- recalibrate a small range (`k<from>,<to>` + `x`)
- or accept autotune overlay threshold for that key and mark for mechanical fix
