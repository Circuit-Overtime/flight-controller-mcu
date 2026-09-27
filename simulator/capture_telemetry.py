"""Capture extended flight-controller telemetry to a timestamped CSV file.

The firmware emits a schema header after boot. This tool preserves that schema,
adds a host monotonic timestamp, writes firmware event lines to a companion
file, and prints basic RX/IMU health statistics when capture ends.

Keep propellers removed for stationary RX/IMU profiling.

Usage:
    python capture_telemetry.py /dev/ttyACM0 460800 60 logs/stationary.csv
"""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
import time
from pathlib import Path

import serial


def _summary(rows: list[dict[str, str]]) -> None:
    if not rows:
        print("No telemetry rows captured.")
        return

    print(f"Captured {len(rows)} valid telemetry rows.")
    for name in ("raw_ch1", "raw_ch2", "raw_ch3", "raw_ch4"):
        values = [int(float(row[name])) for row in rows if row.get(name)]
        if values:
            print(
                f"{name}: min={min(values)} max={max(values)} "
                f"mean={statistics.mean(values):.1f} "
                f"std={statistics.pstdev(values):.2f} us"
            )

    for name in ("ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps"):
        values = [float(row[name]) for row in rows if row.get(name)]
        if values:
            print(
                f"{name}: mean={statistics.mean(values):+.4f} "
                f"std={statistics.pstdev(values):.4f}"
            )

    failsafe_rows = sum(int(float(row.get("failsafe", "0"))) != 0 for row in rows)
    print(f"failsafe rows: {failsafe_rows}")
    if "rx_alive_mask" in rows[0]:
        masks = sorted({int(float(row["rx_alive_mask"])) for row in rows})
        print(f"observed RX alive masks: {masks} (flight channels require bits 0-3)")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("port", nargs="?", default="/dev/ttyACM0")
    parser.add_argument("baud", nargs="?", type=int, default=460800)
    parser.add_argument("seconds", nargs="?", type=float, default=60.0)
    parser.add_argument(
        "output", nargs="?", type=Path, default=Path("logs/telemetry.csv")
    )
    args = parser.parse_args()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    events_path = args.output.with_suffix(".events.txt")
    rows: list[dict[str, str]] = []
    header: list[str] | None = None
    opened_at = time.monotonic()
    capture_started: float | None = None

    print(
        f"Capturing {args.port} @ {args.baud} for {args.seconds:.1f} s "
        f"to {args.output}"
    )
    try:
        with (
            serial.Serial(args.port, args.baud, timeout=1) as ser,
            args.output.open("w", newline="", encoding="utf-8") as csv_file,
            events_path.open("w", encoding="utf-8") as events_file,
        ):
            writer = csv.writer(csv_file)
            while True:
                now = time.monotonic()
                if capture_started is None:
                    if now - opened_at >= 30.0:
                        print("No telemetry header received within 30 s.", file=sys.stderr)
                        return 1
                elif now - capture_started >= args.seconds:
                    break

                payload = ser.readline()
                if not payload:
                    continue
                host_s = time.monotonic() - opened_at
                line = payload.decode("ascii", errors="replace").strip()
                if not line:
                    continue
                if line.startswith("#"):
                    events_file.write(f"{host_s:.6f},{line}\n")
                    events_file.flush()
                    continue

                parts = line.split(",")
                if parts[0] == "roll":
                    if header is None:
                        header = parts
                        capture_started = time.monotonic()
                        writer.writerow(["host_s", *header])
                        csv_file.flush()
                    elif parts != header:
                        events_file.write(
                            f"{host_s:.6f},# SCHEMA CHANGED; row ignored\n"
                        )
                    continue
                if header is None or len(parts) != len(header):
                    continue

                writer.writerow([f"{host_s:.6f}", *parts])
                rows.append(dict(zip(header, parts)))
    except (OSError, serial.SerialException) as exc:
        print(f"[capture] {exc}", file=sys.stderr)
        return 1

    _summary(rows)
    print(f"Events written to {events_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
