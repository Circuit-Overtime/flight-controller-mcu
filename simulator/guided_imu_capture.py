"""Interactively capture real MPU6050 alignment and axis-sign measurements.

Each phase discards queued serial data and records exactly N fresh extended
telemetry rows. Static level phases characterize bias/noise. Movement phases
capture attitude and gyro signs relative to explicit physical frame motions.

Keep propellers removed and leave the transmitter on.

Usage:
    python guided_imu_capture.py /dev/ttyACM0 460800 100 logs/imu-guided.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
import time
from pathlib import Path

import serial


PHASES = (
    (
        "level_start",
        "Place the aircraft LEVEL and motionless.",
        False,
    ),
    (
        "roll_right",
        "Start LEVEL. After pressing Enter, smoothly LOWER THE RIGHT SIDE "
        "about 20-30 degrees, then hold it there.",
        True,
    ),
    (
        "level_after_roll_right",
        "Return the aircraft LEVEL and hold it motionless.",
        False,
    ),
    (
        "roll_left",
        "Start LEVEL. After pressing Enter, smoothly LOWER THE LEFT SIDE "
        "about 20-30 degrees, then hold it there.",
        True,
    ),
    (
        "level_after_roll_left",
        "Return the aircraft LEVEL and hold it motionless.",
        False,
    ),
    (
        "pitch_nose_down",
        "Start LEVEL. After pressing Enter, smoothly LOWER THE NOSE "
        "about 20-30 degrees, then hold it there.",
        True,
    ),
    (
        "level_after_nose_down",
        "Return the aircraft LEVEL and hold it motionless.",
        False,
    ),
    (
        "pitch_nose_up",
        "Start LEVEL. After pressing Enter, smoothly RAISE THE NOSE "
        "about 20-30 degrees, then hold it there.",
        True,
    ),
    (
        "level_after_nose_up",
        "Return the aircraft LEVEL and hold it motionless.",
        False,
    ),
    (
        "yaw_clockwise",
        "Keep the aircraft LEVEL. After pressing Enter, smoothly rotate it "
        "CLOCKWISE when viewed from above, then stop.",
        True,
    ),
    (
        "yaw_counterclockwise",
        "Keep the aircraft LEVEL. After pressing Enter, smoothly rotate it "
        "COUNTER-CLOCKWISE when viewed from above, then stop.",
        True,
    ),
    (
        "level_end",
        "Return to the original heading; hold LEVEL and motionless.",
        False,
    ),
)

IMU_FIELDS = (
    "roll",
    "pitch",
    "yaw",
    "ax_g",
    "ay_g",
    "az_g",
    "gx_dps",
    "gy_dps",
    "gz_dps",
)


def wait_for_header(ser: serial.Serial, timeout_s: float = 30.0) -> list[str]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        payload = ser.readline()
        if not payload:
            continue
        line = payload.decode("ascii", errors="replace").strip()
        if not line.startswith("roll,"):
            continue
        header = line.split(",")
        required = (*IMU_FIELDS, "armed", "failsafe", "rx_alive_mask")
        missing = [name for name in required if name not in header]
        if missing:
            raise RuntimeError(
                "the Mega is running incompatible telemetry firmware; "
                "missing fields: " + ", ".join(missing)
            )
        return header
    raise TimeoutError("no firmware telemetry header received within 30 seconds")


def read_phase(
    ser: serial.Serial,
    header: list[str],
    count: int,
    timeout_s: float,
) -> list[tuple[float, list[str]]]:
    rows: list[tuple[float, list[str]]] = []
    armed_index = header.index("armed")
    failsafe_index = header.index("failsafe")
    mask_index = header.index("rx_alive_mask")
    deadline = time.monotonic() + timeout_s

    while len(rows) < count and time.monotonic() < deadline:
        payload = ser.readline()
        if not payload:
            continue
        line = payload.decode("ascii", errors="replace").strip()
        if not line or line.startswith("#") or line.startswith("roll,"):
            continue
        parts = line.split(",")
        if len(parts) != len(header):
            continue
        try:
            values = [float(value) for value in parts]
        except ValueError:
            continue
        if not all(math.isfinite(value) for value in values):
            continue
        if int(values[armed_index]) != 0:
            raise RuntimeError("controller became armed; capture aborted")
        if int(values[failsafe_index]) != 0:
            raise RuntimeError("receiver entered failsafe; capture aborted")
        if (int(values[mask_index]) & 0x0F) != 0x0F:
            raise RuntimeError("a flight-critical RX channel became stale")
        rows.append((time.monotonic(), parts))

    if len(rows) != count:
        raise TimeoutError(f"received only {len(rows)}/{count} valid rows")
    return rows


def print_summary(header: list[str], rows: list[list[str]]) -> None:
    index = {name: header.index(name) for name in IMU_FIELDS}
    for name in IMU_FIELDS:
        values = [float(row[index[name]]) for row in rows]
        print(
            f"  {name}: mean={statistics.mean(values):+.3f} "
            f"std={statistics.pstdev(values):.3f} "
            f"min={min(values):+.3f} max={max(values):+.3f}"
        )
    magnitudes = [
        math.sqrt(
            float(row[index["ax_g"]]) ** 2
            + float(row[index["ay_g"]]) ** 2
            + float(row[index["az_g"]]) ** 2
        )
        for row in rows
    ]
    print(
        f"  accel magnitude: mean={statistics.mean(magnitudes):.4f} g "
        f"std={statistics.pstdev(magnitudes):.4f} g"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("port", nargs="?", default="/dev/ttyACM0")
    parser.add_argument("baud", nargs="?", type=int, default=460800)
    parser.add_argument("samples", nargs="?", type=int, default=100)
    parser.add_argument(
        "output", nargs="?", type=Path, default=Path("logs/imu-guided.csv")
    )
    args = parser.parse_args()
    if args.samples < 2:
        parser.error("samples must be at least 2")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    print("PROPELLERS MUST BE REMOVED. KEEP THE TRANSMITTER ON.")
    print(f"Opening {args.port} @ {args.baud}; waiting for firmware boot/header...")

    try:
        with (
            serial.Serial(args.port, args.baud, timeout=1) as ser,
            args.output.open("w", newline="", encoding="utf-8") as output_file,
        ):
            header = wait_for_header(ser)
            writer = csv.writer(output_file)
            writer.writerow(["phase", "sample", "host_s", *header])
            capture_start = time.monotonic()

            for phase, instruction, move_after_enter in PHASES:
                print(f"\n[{phase}] {instruction}")
                if move_after_enter:
                    prompt = (
                        "Set the START position, press Enter, then perform "
                        "the movement immediately: "
                    )
                else:
                    prompt = "Set and hold the position, then press Enter: "
                input(prompt)

                ser.reset_input_buffer()
                received_rows = read_phase(
                    ser,
                    header,
                    args.samples,
                    timeout_s=max(10.0, args.samples / 20.0),
                )
                rows = [row for _, row in received_rows]
                for sample_index, (received_at, row) in enumerate(
                    received_rows, start=1
                ):
                    writer.writerow(
                        [
                            phase,
                            sample_index,
                            f"{received_at - capture_start:.6f}",
                            *row,
                        ]
                    )
                output_file.flush()
                print(f"Captured exactly {len(rows)} fresh samples.")
                print_summary(header, rows)
    except KeyboardInterrupt:
        print("\nCapture cancelled; completed phases remain in the output file.")
        return 130
    except (OSError, RuntimeError, TimeoutError, serial.SerialException) as exc:
        print(f"[guided IMU capture] {exc}", file=sys.stderr)
        return 1

    print(f"\nComplete: {len(PHASES) * args.samples} rows written to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
