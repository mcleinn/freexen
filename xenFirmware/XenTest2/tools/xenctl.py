import argparse
import json
import os
import queue
import subprocess
import sys
import threading
import time
from dataclasses import dataclass


PIO_EXE_DEFAULT = r"C:\Users\tobia\.platformio\penv\Scripts\platformio.exe"


def _ts() -> str:
    return time.strftime("%H:%M:%S")


def _now_tag() -> str:
    return time.strftime("%Y%m%d_%H%M%S")


def _ensure_dir(p: str) -> None:
    os.makedirs(p, exist_ok=True)


def _print_step(idx: int, total: int, cmd: str) -> None:
    sys.stdout.write(f"[{idx}/{total}] >>> {cmd}\n")
    sys.stdout.flush()


@dataclass
class CmdSpec:
    text: str
    # Stop condition:
    # - "quiet": stop after quiet seconds of no traffic
    # - "json_type": stop after seeing a JSONL line with type==stop_value
    # - "ok_and_quiet": stop after seeing OK: ack, then quiet
    stop_kind: str = "quiet"
    stop_value: str = ""
    max_s: float = 60.0
    quiet_s: float = 2.0


class SerialSession:
    def __init__(self, port: str, baud: int, timeout_s: float):
        import serial

        self.ser = serial.Serial(
            port=port, baudrate=baud, timeout=timeout_s, write_timeout=timeout_s
        )
        self._rx_thread = None
        self._rx_stop = threading.Event()
        self._lines = queue.Queue()
        self._raw_log = []
        self._tee_to_stdout = False

    def close(self) -> None:
        self._rx_stop.set()
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=1.0)
        try:
            self.ser.close()
        except Exception:
            pass

    def start_reader(self) -> None:
        if self._rx_thread is not None:
            return

        def _run():
            while not self._rx_stop.is_set():
                try:
                    raw = self.ser.readline()
                except Exception:
                    break
                if not raw:
                    continue
                s = raw.decode("utf-8", "replace").rstrip("\r\n")
                self._raw_log.append(s)
                self._lines.put((time.monotonic(), s))
                if self._tee_to_stdout:
                    sys.stdout.write(s + "\n")
                    sys.stdout.flush()

        self._rx_thread = threading.Thread(target=_run, daemon=True)
        self._rx_thread.start()

    def drain(self, max_s: float = 1.0) -> list[str]:
        t0 = time.monotonic()
        out = []
        while time.monotonic() - t0 < max_s:
            try:
                _t, s = self._lines.get(timeout=0.05)
                out.append(s)
            except queue.Empty:
                pass
        return out

    def send(self, cmd: str) -> None:
        if not cmd.endswith("\n"):
            cmd = cmd + "\n"
        self.ser.write(cmd.encode("ascii", "strict"))
        self.ser.flush()

    def read_until(self, spec: CmdSpec) -> list[str]:
        """Collect lines until the spec stop condition or safety net triggers."""
        t0 = time.monotonic()
        last_rx = t0
        got_ok = False
        out = []

        def _is_stop_json_type(line: str) -> bool:
            if not (line.startswith("{") and line.endswith("}")):
                return False
            try:
                obj = json.loads(line)
            except Exception:
                return False
            return obj.get("type") == spec.stop_value

        while True:
            now = time.monotonic()
            if now - t0 >= spec.max_s:
                break

            try:
                _t, s = self._lines.get(timeout=0.05)
            except queue.Empty:
                # No new line.
                if now - last_rx >= spec.quiet_s:
                    if spec.stop_kind == "quiet":
                        break
                    if spec.stop_kind == "ok_and_quiet" and got_ok:
                        break
                continue

            last_rx = time.monotonic()
            out.append(s)

            if s.startswith("OK:"):
                got_ok = True

            if spec.stop_kind == "json_type" and _is_stop_json_type(s):
                break

        return out


def _extract_json_objs(lines: list[str]) -> list[dict]:
    out = []
    for s in lines:
        if not (isinstance(s, str) and s.startswith("{") and s.endswith("}")):
            continue
        try:
            out.append(json.loads(s))
        except Exception:
            pass
    return out


def _autotune_wait_complete(
    ss: SerialSession,
    transcript: list[str],
    *,
    max_wait_s: float = 900.0,
    poll_every_s: float = 2.0,
) -> bool:
    """Wait for the last autotune run to become complete.

    Newer firmware emits {"type":"autotune_done",...} at the end of autotune.
    If we don't see it (older firmware), we fall back to polling autodump.
    """
    t0 = time.monotonic()
    attempt = 0
    while time.monotonic() - t0 < max_wait_s:
        drained = ss.drain(max_s=0.25)
        if drained:
            transcript.extend(drained)
            for obj in _extract_json_objs(drained):
                if obj.get("type") == "autotune_done":
                    sys.stdout.write("xenctl: saw autotune_done\n")
                    sys.stdout.flush()
                    transcript.append("xenctl: saw autotune_done")
                    return True

        attempt += 1
        sys.stdout.write(
            f"xenctl: waiting for autotune completion (poll {attempt})...\n"
        )
        sys.stdout.flush()
        transcript.append(
            f"xenctl: waiting for autotune completion (poll {attempt})..."
        )

        ss.send("autodump")
        lines = ss.read_until(default_stop_for_cmd("autodump"))
        transcript.extend(lines)
        objs = _extract_json_objs(lines)
        for obj in objs:
            if obj.get("type") == "autodump":
                if int(obj.get("complete", 0)) == 1:
                    sys.stdout.write("xenctl: autotune complete=1\n")
                    sys.stdout.flush()
                    transcript.append("xenctl: autotune complete=1")
                    return True

        time.sleep(max(0.2, poll_every_s))

    sys.stdout.write("xenctl: timed out waiting for autotune completion\n")
    sys.stdout.flush()
    transcript.append("xenctl: timed out waiting for autotune completion")
    return False


def _wait_for_ok_or_timeout(ss: SerialSession, timeout_s: float) -> bool:
    """Return True if an OK: ack line is observed within timeout_s."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        try:
            _t, s = ss._lines.get(timeout=0.05)
        except queue.Empty:
            continue
        # Note: reader thread already tees to stdout. We must not lose lines.
        # Put back into the queue for downstream consumption.
        ss._lines.put((_t, s))
        if isinstance(s, str) and s.startswith("OK:"):
            return True
    return False


def default_stop_for_cmd(cmd: str) -> CmdSpec:
    c = cmd.strip().lower()
    # From src/main.cpp (fw>=90):
    # - idle: emits {"type":"idle",...}
    # - noisereset (no sweep): emits noiselevel then noiseprog
    # We'll handle these as explicit json_type stops where possible.
    if c.startswith("idle"):
        return CmdSpec(
            text=cmd, stop_kind="json_type", stop_value="idle", max_s=180.0, quiet_s=3.0
        )
    if c.startswith("noisereset"):
        # noisereset can emit noiselevel + noiseprog; we stop after noiseprog.
        # If sweep is enabled, it emits noise_trial lines; use quiet stop.
        if c != "noisereset":
            return CmdSpec(text=cmd, stop_kind="quiet", max_s=120.0, quiet_s=3.0)
        return CmdSpec(
            text=cmd,
            stop_kind="json_type",
            stop_value="noiseprog",
            max_s=120.0,
            quiet_s=3.0,
        )
    if c.startswith("autotune"):
        # Autotune progress is periodic JSON; wait on quiet.
        return CmdSpec(text=cmd, stop_kind="quiet", max_s=900.0, quiet_s=5.0)
    if c == "autodump":
        # Autodump can be long; stop after quiet.
        return CmdSpec(text=cmd, stop_kind="quiet", max_s=120.0, quiet_s=3.0)
    if c in ("testb", "testc", "testd", "teste", "pr2", "pr5", "rate1000"):
        return CmdSpec(text=cmd, stop_kind="quiet", max_s=240.0, quiet_s=3.0)
    return CmdSpec(text=cmd, stop_kind="ok_and_quiet", max_s=60.0, quiet_s=1.5)


def run_serial_script(
    port: str, baud: int, timeout_s: float, commands: list[str], outpath: str
) -> int:
    _ensure_dir(os.path.dirname(outpath) or ".")
    ss = None
    try:
        ss = SerialSession(port=port, baud=baud, timeout_s=timeout_s)
        ss._tee_to_stdout = True
        ss.start_reader()

        # Give device a moment (boot banners, etc.).
        time.sleep(0.35)
        boot = ss.drain(max_s=1.0)

        transcript = []
        for line in boot:
            transcript.append(line)

        total = len(commands)
        for i, cmd in enumerate(commands, start=1):
            spec = default_stop_for_cmd(cmd)
            transcript.append(f">>> {cmd}")
            _print_step(i, total, cmd)
            ss.send(cmd)

            # If the device doesn't ack quickly, assume it's not responsive.
            # Exceptions: commands that may dump without a fast OK line.
            c = cmd.strip().lower()
            # Some commands can legitimately delay their OK: line due to heavy work
            # or large output (e.g. autotune progress, dumps). For those, don't
            # fail-fast on a 1s ack watchdog.
            need_ok = not (
                c == "autodump"
                or c.startswith("autotune")
                or c.startswith("test")
                or c.startswith("pr")
                or c.startswith("sts")
                or c.startswith("st")
            )

            if need_ok and not _wait_for_ok_or_timeout(ss, timeout_s=1.0):
                msg = (
                    f"xenctl: no OK ack within 1.0s for '{cmd}'. "
                    "Device may be unresponsive or the serial link is wedged. "
                    "Suggestion: run 'xenctl reset --boot-wait 2.0' or power-cycle."
                )
                sys.stdout.write(msg + "\n")
                sys.stdout.flush()
                transcript.append(msg)
                break

            lines = ss.read_until(spec)
            transcript.extend(lines)

            # If we just launched autotune, wait until the run is complete before
            # proceeding (otherwise follow-up commands like autodump/autostat can
            # run while diag mode is still active).
            # Avoid follow-up commands that would collide with diag mode.
            # Newer firmware emits autotune_done; xenctl will wait for it.
            if cmd.strip().lower().startswith("autotune"):
                _autotune_wait_complete(ss, transcript)

            time.sleep(0.05)

        with open(outpath, "w", encoding="utf-8", newline="\n") as f:
            for line in transcript:
                f.write(line + "\n")

        return 0
    finally:
        if ss is not None:
            ss.close()


def pio_run(pio_exe: str, target: str, workdir: str) -> int:
    cmd = [pio_exe, "run", "-t", target]
    sys.stdout.write(f"xenctl: running: {cmd!r}\n")
    sys.stdout.flush()
    p = subprocess.run(cmd, cwd=workdir)
    return int(p.returncode)


def cmd_build(args) -> int:
    return pio_run(args.pio, "build", args.workdir)


def cmd_upload(args) -> int:
    return pio_run(args.pio, "upload", args.workdir)


def cmd_reset(args) -> int:
    # Prefer hardware/PIO reset when available; serial reset requires a responsive link.
    # PlatformIO reset also gives us a place to wait for USB re-enumeration.
    out = os.path.join("autotune_logs", f"reset_{_now_tag()}.txt")

    # First try PlatformIO reset.
    rc = pio_run(args.pio, "reset", args.workdir)
    if rc == 0:
        sys.stdout.write(
            f"xenctl: pio reset succeeded, waiting {args.boot_wait:.1f}s for boot...\n"
        )
        sys.stdout.flush()
        time.sleep(max(0.0, args.boot_wait))
        return 0

    sys.stdout.write(
        "xenctl: pio reset failed; falling back to serial 'rst' (requires responsive firmware).\n"
    )
    sys.stdout.flush()
    rc2 = run_serial_script(args.port, args.baud, args.timeout, ["rst"], out)
    time.sleep(max(0.0, args.boot_wait))
    return rc2


def cmd_run(args) -> int:
    out = args.outfile
    if not out:
        out = os.path.join("autotune_logs", f"xenctl_run_{_now_tag()}.txt")
    return run_serial_script(args.port, args.baud, args.timeout, args.commands, out)


def cmd_noise_fix(args) -> int:
    # Flow:
    # - ensure jsonl
    # - snapshot state
    # - disable overlays if thresholds are too high
    # - re-measure noise + idle
    # - run autotune with staged strict validation
    # - show post-check
    cmds = [
        "ver",
        "fmt1",
        "calstat",
        "bootdrift",
        "noiselevel",
        "autostat",
        f"idle{max(5, args.idle_pre)}",
        "noisereset",
    ]

    if args.autoreset:
        cmds.extend(
            [
                "autoreset",
                "autostat",
                "noisereset",
                f"idle{max(5, args.idle_pre)}",
            ]
        )

    # Run autotune and capture results.
    cmds.extend(
        [
            f"autotune{args.short},{args.mid},{args.strict},{args.topk}",
            "autodump",
            f"idle{max(30, args.idle_post)}",
            "noisereset",
            "autostat",
        ]
    )

    out = args.outfile
    if not out:
        out = os.path.join("autotune_logs", f"noise_fix_{_now_tag()}.txt")
    return run_serial_script(args.port, args.baud, args.timeout, cmds, out)


def main() -> int:
    ap = argparse.ArgumentParser(
        prog="xenctl", description="XenControl supervisor tool"
    )
    ap.add_argument(
        "--aim",
        default="",
        help="human-readable goal printed at start of the queue",
    )
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=0.25)
    ap.add_argument("--pio", default=PIO_EXE_DEFAULT)
    ap.add_argument(
        "--workdir", default=os.path.dirname(os.path.abspath(__file__)) + os.sep + ".."
    )

    sub = ap.add_subparsers(dest="sub")

    sp = sub.add_parser("build")
    sp.set_defaults(fn=cmd_build)

    sp = sub.add_parser("upload")
    sp.set_defaults(fn=cmd_upload)

    sp = sub.add_parser("reset")
    sp.add_argument("--boot-wait", type=float, default=1.0)
    sp.set_defaults(fn=cmd_reset)

    sp = sub.add_parser("run", help="run commands in order")
    sp.add_argument("commands", nargs="+", help="commands like fmt1 idle30 rk195")
    sp.add_argument("--outfile", default="")
    sp.set_defaults(fn=cmd_run)

    sp = sub.add_parser("noise-fix", help="reset overlays, analyze, autotune")
    sp.add_argument("--outfile", default="")
    sp.add_argument(
        "--autoreset",
        action="store_true",
        help="disable autotune overlay before tuning",
    )
    sp.add_argument("--idle-pre", type=int, default=30)
    sp.add_argument("--idle-post", type=int, default=120)
    sp.add_argument("--short", type=int, default=5)
    sp.add_argument("--mid", type=int, default=20)
    sp.add_argument("--strict", type=int, default=120)
    sp.add_argument("--topk", type=int, default=2)
    sp.set_defaults(fn=cmd_noise_fix)

    args = ap.parse_args()
    if not hasattr(args, "fn"):
        ap.print_help()
        return 2
    if args.aim:
        sys.stdout.write(f"aim: {args.aim}\n")
        sys.stdout.flush()
    return int(args.fn(args))


if __name__ == "__main__":
    raise SystemExit(main())
