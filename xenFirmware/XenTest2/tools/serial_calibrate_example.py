import argparse
import json
import os
import sys
import time


def _now_stamp():
    return time.strftime("%Y%m%d_%H%M%S")


def _write_cmd(ser, s: str):
    if not s.endswith("\n"):
        s += "\n"
    ser.write(s.encode("ascii", errors="replace"))


def run():
    ap = argparse.ArgumentParser(
        description="Example: run Xen calibration over serial (JSONL)."
    )
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--baud", default=115200, type=int)
    ap.add_argument("--from_key", type=int, required=True, help="1-based")
    ap.add_argument("--to_key", type=int, required=True, help="1-based")
    ap.add_argument(
        "--clear",
        action="store_true",
        help="Clear calibration for the selected range before starting",
    )
    ap.add_argument(
        "--idle_stop_s",
        type=float,
        default=15.0,
        help="Stop monitoring after this many seconds with no output",
    )
    ap.add_argument(
        "--log_dir",
        default="calib_serial_logs",
        help="Directory to write raw JSONL capture",
    )
    args = ap.parse_args()

    try:
        import serial  # type: ignore
    except Exception as e:
        sys.stderr.write(
            "pyserial is required. Install with: py -m pip install pyserial\n"
        )
        raise

    os.makedirs(args.log_dir, exist_ok=True)
    log_path = os.path.join(
        args.log_dir,
        f"calib_{args.from_key}-{args.to_key}_{_now_stamp()}.jsonl",
    )

    with (
        serial.Serial(args.port, args.baud, timeout=0.2) as ser,
        open(log_path, "w", newline="\n") as logf,
    ):
        ser.dtr = True
        time.sleep(0.6)
        ser.read(8192)

        # JSONL format and key range
        _write_cmd(ser, "fmt1")
        time.sleep(0.2)
        ser.read(8192)

        _write_cmd(ser, f"k{args.from_key},{args.to_key}")
        time.sleep(0.2)
        ser.read(8192)

        if args.clear:
            _write_cmd(ser, "ccalib")
            time.sleep(0.4)
            ser.read(8192)

        # Start calibration
        _write_cmd(ser, "x")
        time.sleep(0.2)

        sys.stdout.write(
            f"Calibrating keys {args.from_key}..{args.to_key}. Press in order.\n"
        )
        sys.stdout.write(f"Logging JSONL to: {log_path}\n")
        sys.stdout.flush()

        last = time.time()
        got_done = False

        while True:
            line = ser.readline()
            now = time.time()
            if line:
                last = now
                txt = line.decode("utf-8", errors="replace").strip()
                if not txt:
                    continue
                logf.write(txt + "\n")

                # Also echo key milestones to console
                if '"type":"calib_done"' in txt or '"type":"calib_saved"' in txt:
                    sys.stdout.write(txt + "\n")
                    sys.stdout.flush()
                if '"type":"calib_done"' in txt:
                    got_done = True
                    break

            if now - last > args.idle_stop_s:
                break

        # Always show final status
        _write_cmd(ser, "calstat")
        time.sleep(0.8)
        out = ser.read(20000).decode("utf-8", errors="replace")
        sys.stdout.write(out)
        sys.stdout.flush()

        if not got_done:
            sys.stdout.write(
                f"Stopped after {args.idle_stop_s}s idle without seeing calib_done.\n"
            )
            sys.stdout.flush()


if __name__ == "__main__":
    run()
