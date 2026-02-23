# xenctl Guide

`tools/xenctl.py` is a serial + PlatformIO supervisor for XenControl.

## Quick Start

- Run a single command:

  `python tools/xenctl.py --port COM3 run fmt1 ver`

- Run a queued script (commands executed in order):

  `python tools/xenctl.py --port COM3 run fmt1 noisereset idle30 autostat`

- Reboot by uploading current firmware and waiting for boot:

  `python tools/xenctl.py --port COM3 upload`

## Important Behavior

- Live tee: xenctl prints every incoming serial line as it arrives.
- Step markers: each sent command is printed as `[i/N] >>> <cmd>`.
- Output format: use `fmt1` to get JSONL suitable for parsing.
- Exclusive port: COM port must not be open in any other program.

## xenctl-only wait command

Inside `run`, you can insert a timed wait while still streaming serial.

- `wait10` waits 10 seconds
- `wait0.5` waits 0.5 seconds

Example: observe human playing for 10 seconds:

`python tools/xenctl.py --port COM3 run fmt1 l3 wait10 l0`

## Noise/Idle Workflow (Typical)

`python tools/xenctl.py --port COM3 run fmt1 ver calstat bootdrift noisereset idle120 autostat`

If you need to retune:

`python tools/xenctl.py --port COM3 run fmt1 autoreset autotune autodump autosave idle120 autostat`

## Common Issues

- No output:
  - device is silent, OR
  - COM port is held by another program.

- JSONL breaks:
  - ensure `fmt1` is set; in JSONL mode debug peak messages should also be JSON.
