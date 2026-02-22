import argparse
import datetime as _dt
from pathlib import Path


def now_timestamp() -> str:
    # Local time, second precision.
    return _dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def main() -> int:
    ap = argparse.ArgumentParser(description="Append a TEST_LOG.md entry header")
    ap.add_argument("--fw", type=int, required=True, help="Firmware version (ver)")
    ap.add_argument(
        "--file",
        default="TEST_LOG.md",
        help="Log file path (default: TEST_LOG.md)",
    )
    ap.add_argument(
        "--title",
        default="",
        help="Optional short title text (one line)",
    )
    args = ap.parse_args()

    log_path = Path(args.file)
    if not log_path.exists():
        raise SystemExit(f"Log file not found: {log_path}")

    ts = now_timestamp()
    header = [
        "\n",
        f"## {ts}\n",
        "\n",
        f"### fw={args.fw}\n",
    ]
    if args.title:
        header += [
            "\n",
            f"{args.title.strip()}\n",
        ]

    with log_path.open("a", encoding="utf-8", newline="\n") as f:
        f.writelines(header)

    print(f"Appended entry header to {log_path} @ {ts} (fw={args.fw})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
