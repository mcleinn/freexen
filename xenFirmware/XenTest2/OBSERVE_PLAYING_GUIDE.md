# Observe Human Playing (10s)

Goal: capture machine-readable peak/MIDI activity for a short human play window, then return to a quiet/idle state.

## Setup

- Connect serial at 115200.
- Use JSONL output mode: `fmt1`

## Procedure (10 seconds)

1) Select the intended layout/program (recommended)

- `l0` (pause)
- `lconf<programId>` (loads `config<programId>.csv` and refreshes LEDs)

2) Enter normal play loop

- `l3`

3) (Optional) enable debug prints if they are off

- Use your debug enable command (prints `{"type":"debug","enabled":1,...}` when enabled).

4) Play normally for ~10 seconds

- Watch for JSONL lines:
  - `{"type":"peak_begin",...}` threshold crossings
  - `{"type":"peak_fire",...}` peak-to-velocity mapping
  - `{"type":"midi","kind":"noteon"|"polypressure"|"noteoff",...}` actual MIDI events

5) Stop output / freeze state (so logs stop scrolling)

- `l0`

6) Reset LEDs to black (cleanup)

- `ca` (same as `ca0,0,0`)

## What To Watch For

- False triggers while hands-off: `midi kind=noteon` with no physical press.
- Chatter: rapid `noteon`/`noteoff` for the same `key`.
- Stuck notes: `noteon` without a corresponding `noteoff` after release.
- Weak dynamics: `peak_fire vel` clustered near 1..5 even on hard presses.

## Notes

- JSON uses 0-based `key` indices.
- During diagnostics/tests (`idle`, `autotune`, etc.) `_diagActive` suppresses peak detect; use `l3` for live playing.

## xenctl Example

Observe 10s of play, then pause and black out LEDs:

`python tools/xenctl.py --port COM3 run fmt1 l0 lconf0 l3 wait10 l0 ca`
