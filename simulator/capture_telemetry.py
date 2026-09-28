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


def _summary(
    rows: list[dict[str, str]], tx_off_cue_host_s: float | None = None
) -> None:
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

    if tx_off_cue_host_s is not None:
        first_stale = next(
            (
                row
                for row in rows
                if float(row["_host_s"]) >= tx_off_cue_host_s
                if int(float(row.get("rx_alive_mask", "0"))) != 15
            ),
            None,
        )
        first_failsafe = next(
            (
                row
                for row in rows
                if float(row["_host_s"]) >= tx_off_cue_host_s
                if int(float(row.get("failsafe", "0"))) != 0
            ),
            None,
        )
        for label, row in (
            ("first non-15 RX mask", first_stale),
            ("failsafe assertion", first_failsafe),
        ):
            if row is None:
                print(f"{label}: not observed after transmitter-off cue")
                continue
            latency_s = float(row["_host_s"]) - tx_off_cue_host_s
            print(f"cue to {label}: {latency_s:.3f} s")
        print("Latency includes the operator's reaction time after the cue.")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("port", nargs="?", default="/dev/ttyACM0")
    parser.add_argument("baud", nargs="?", type=int, default=460800)
    parser.add_argument("seconds", nargs="?", type=float, default=60.0)
    parser.add_argument(
        "output", nargs="?", type=Path, default=Path("logs/telemetry.csv")
    )
    parser.add_argument(
        "--tx-off-after",
        type=float,
        metavar="SECONDS",
        help=(
            "print and log a transmitter-off cue this many seconds after the "
            "telemetry header; propellers must be removed"
        ),
    )
    args = parser.parse_args()
    if args.tx_off_after is not None:
        if args.tx_off_after <= 3.0:
            parser.error("--tx-off-after must be greater than 3 seconds")
        if args.tx_off_after >= args.seconds:
            parser.error("--tx-off-after must occur before capture ends")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    events_path = args.output.with_suffix(".events.txt")
    rows: list[dict[str, str]] = []
    header: list[str] | None = None
    opened_at = time.monotonic()
    capture_started: float | None = None
    countdown_announced: set[int] = set()
    tx_off_cue_host_s: float | None = None

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

                if capture_started is not None and args.tx_off_after is not None:
                    elapsed = now - capture_started
                    for remaining in (3, 2, 1):
                        if (
                            elapsed >= args.tx_off_after - remaining
                            and remaining not in countdown_announced
                        ):
                            print(f"Transmitter off in {remaining}...", flush=True)
                            countdown_announced.add(remaining)
                    if elapsed >= args.tx_off_after and tx_off_cue_host_s is None:
                        tx_off_cue_host_s = time.monotonic() - opened_at
                        print("*** SWITCH TRANSMITTER OFF NOW ***", flush=True)
                        events_file.write(
                            f"{tx_off_cue_host_s:.6f},# OPERATOR CUE: "
                            "SWITCH TRANSMITTER OFF NOW\n"
                        )
                        events_file.flush()

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
                        print(
                            "Telemetry header received; capture timer started.",
                            flush=True,
                        )
                    elif parts != header:
                        events_file.write(
                            f"{host_s:.6f},# SCHEMA CHANGED; row ignored\n"
                        )
                    continue
                if header is None or len(parts) != len(header):
                    continue

                writer.writerow([f"{host_s:.6f}", *parts])
                row = dict(zip(header, parts))
                row["_host_s"] = f"{host_s:.6f}"
                rows.append(row)
                if (
                    args.tx_off_after is not None
                    and int(float(row.get("armed", "0"))) != 0
                ):
                    print(
                        "[capture] controller became armed; aborting loss test.",
                        file=sys.stderr,
                    )
                    return 1
    except (OSError, serial.SerialException) as exc:
        print(f"[capture] {exc}", file=sys.stderr)
        return 1

    _summary(rows, tx_off_cue_host_s)
    print(f"Events written to {events_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
