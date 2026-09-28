"""Interactively capture exactly N fresh samples at defined stick positions.

The tool waits for the firmware telemetry header, asks the operator to set and
hold one transmitter position, discards any queued serial input, and records
fresh rows only. Raw ISR pulse widths are used for endpoint/noise analysis;
corrected values and all simultaneous IMU/controller fields are retained.

Keep propellers removed.

Usage:
    python guided_rx_capture.py /dev/ttyACM0 460800 100 logs/rx-guided.csv
"""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
import time
from pathlib import Path

import serial


PHASES = (
    (
        "center_low_start",
        "Set roll, pitch, and yaw to CENTER; set throttle to LOWEST.",
    ),
    (
        "roll_left",
        "Hold ROLL fully LEFT; keep pitch/yaw centered and throttle lowest.",
    ),
    (
        "roll_right",
        "Hold ROLL fully RIGHT; keep pitch/yaw centered and throttle lowest.",
    ),
    (
        "pitch_down",
        "Hold PITCH fully DOWN; keep roll/yaw centered and throttle lowest.",
    ),
    (
        "pitch_up",
        "Hold PITCH fully UP; keep roll/yaw centered and throttle lowest.",
    ),
    (
        "throttle_middle",
        "Hold THROTTLE at its physical midpoint; keep other sticks centered.",
    ),
    (
        "throttle_high",
        "Hold THROTTLE at its HIGHEST position; keep other sticks centered.",
    ),
    (
        "yaw_left",
        "Hold YAW fully LEFT; keep roll/pitch centered and throttle at MIDPOINT.",
    ),
    (
        "yaw_right",
        "Hold YAW fully RIGHT; keep roll/pitch centered and throttle at MIDPOINT.",
    ),
    (
        "center_low_end",
        "Return roll, pitch, and yaw to CENTER; throttle to LOWEST.",
    ),
)

PHYSICAL_LAYOUT_PHASES = (
    (
        "physical_center_low_start",
        "Center both sticks horizontally; center the RIGHT stick vertically; "
        "put the LEFT stick vertically at LOWEST.",
    ),
    (
        "right_horizontal_left",
        "Move only the RIGHT stick fully LEFT; keep LEFT vertical at LOWEST.",
    ),
    (
        "right_horizontal_right",
        "Move only the RIGHT stick fully RIGHT; keep LEFT vertical at LOWEST.",
    ),
    (
        "right_vertical_down",
        "Move only the RIGHT stick fully DOWN; keep LEFT vertical at LOWEST.",
    ),
    (
        "right_vertical_up",
        "Move only the RIGHT stick fully UP; keep LEFT vertical at LOWEST.",
    ),
    (
        "left_vertical_middle",
        "Center both sticks, including putting the LEFT stick vertically at "
        "its physical MIDPOINT.",
    ),
    (
        "left_vertical_high",
        "Move only the LEFT stick vertically to HIGHEST; center all other axes.",
    ),
    (
        "left_horizontal_left",
        "Keep LEFT vertical at MIDPOINT and move only the LEFT stick fully LEFT.",
    ),
    (
        "left_horizontal_right",
        "Keep LEFT vertical at MIDPOINT and move only the LEFT stick fully "
        "RIGHT. Mid-throttle prevents the arming gesture.",
    ),
    (
        "physical_center_low_end",
        "Return horizontal axes and RIGHT vertical to CENTER; return LEFT "
        "vertical to LOWEST.",
    ),
)

RAW_CHANNELS = ("raw_ch1", "raw_ch2", "raw_ch3", "raw_ch4")


def wait_for_header(ser: serial.Serial, timeout_s: float = 30.0) -> list[str]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        payload = ser.readline()
        if not payload:
            continue
        line = payload.decode("ascii", errors="replace").strip()
        if line.startswith("roll,"):
            header = line.split(",")
            required = (*RAW_CHANNELS, "rx_alive_mask", "armed", "failsafe")
            missing = [name for name in required if name not in header]
            if missing:
                raise RuntimeError(
                    "the Mega is running an older telemetry firmware; "
                    "compile and upload the current mcu/ sketch, then retry. "
                    "Missing header fields: " + ", ".join(missing)
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
            # Reject malformed rows now rather than discovering them in analysis.
            [float(value) for value in parts]
        except ValueError:
            continue
        if int(float(parts[armed_index])) != 0:
            raise RuntimeError(
                "controller became armed; capture aborted for safety"
            )
        if int(float(parts[failsafe_index])) != 0:
            raise RuntimeError("receiver entered failsafe; capture aborted")
        if (int(float(parts[mask_index])) & 0x0F) != 0x0F:
            raise RuntimeError("a flight-critical RX channel became stale")
        rows.append((time.monotonic(), parts))
    if len(rows) != count:
        raise TimeoutError(f"received only {len(rows)}/{count} valid rows")
    return rows


def print_phase_summary(header: list[str], rows: list[list[str]]) -> None:
    indices = {name: header.index(name) for name in RAW_CHANNELS}
    for name in RAW_CHANNELS:
        values = [int(float(row[indices[name]])) for row in rows]
        print(
            f"  {name}: mean={statistics.mean(values):.2f} "
            f"std={statistics.pstdev(values):.2f} "
            f"min={min(values)} max={max(values)}"
        )

    mask_index = header.index("rx_alive_mask")
    masks = sorted({int(float(row[mask_index])) for row in rows})
    if any((mask & 0x0F) != 0x0F for mask in masks):
        print(f"  WARNING: critical RX channel missing; alive masks={masks}")
    else:
        print(f"  RX flight channels alive; observed masks={masks}")


def print_physical_layout_result(
    header: list[str], phase_rows: dict[str, list[list[str]]]
) -> None:
    indices = {name: header.index(name) for name in RAW_CHANNELS}

    def means(phase: str) -> dict[str, float]:
        rows = phase_rows[phase]
        return {
            name: statistics.mean(float(row[index]) for row in rows)
            for name, index in indices.items()
        }

    comparisons = (
        ("right horizontal", "right_horizontal_left", "right_horizontal_right"),
        ("right vertical", "right_vertical_down", "right_vertical_up"),
        ("left vertical", "physical_center_low_start", "left_vertical_high"),
        ("left horizontal", "left_horizontal_left", "left_horizontal_right"),
    )
    detected: dict[str, str] = {}
    strong_movements = True
    print("\nPhysical transmitter layout:")
    for axis, low_phase, high_phase in comparisons:
        low = means(low_phase)
        high = means(high_phase)
        channel = max(RAW_CHANNELS, key=lambda name: abs(high[name] - low[name]))
        change = high[channel] - low[channel]
        if abs(change) < 150.0:
            strong_movements = False
        detected[axis] = channel
        print(
            f"  {axis}: {channel.replace('raw_', '').upper()} "
            f"({low[channel]:.1f} -> {high[channel]:.1f} us; "
            f"delta={change:+.1f} us)"
        )

    mode_2 = {
        "right horizontal": "raw_ch1",
        "right vertical": "raw_ch2",
        "left vertical": "raw_ch3",
        "left horizontal": "raw_ch4",
    }
    channels_are_unique = len(set(detected.values())) == len(detected)
    if detected == mode_2 and strong_movements:
        print("RESULT: Mode 2 mapping confirmed (roll/pitch/throttle/yaw = CH1/2/3/4).")
    elif not strong_movements or not channels_are_unique:
        print("RESULT: Inconclusive; an axis was not moved fully or channels overlap.")
    else:
        print("RESULT: Mapping is not the expected Mode 2 CH1/CH2/CH3/CH4 layout.")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("port", nargs="?", default="/dev/ttyACM0")
    parser.add_argument("baud", nargs="?", type=int, default=460800)
    parser.add_argument("samples", nargs="?", type=int, default=100)
    parser.add_argument(
        "output", nargs="?", type=Path, default=Path("logs/rx-guided.csv")
    )
    parser.add_argument(
        "--physical-layout",
        action="store_true",
        help="prompt by physical gimbal axis and identify the transmitter mode",
    )
    args = parser.parse_args()
    if args.samples < 2:
        parser.error("samples must be at least 2")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    phases = PHYSICAL_LAYOUT_PHASES if args.physical_layout else PHASES
    print("PROPELLERS MUST BE REMOVED.")
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
            captured_by_phase: dict[str, list[list[str]]] = {}

            for phase, instruction in phases:
                print(f"\n[{phase}] {instruction}")
                input("Set and hold the position, then press Enter to capture: ")

                # Drop samples accumulated while the operator moved the sticks.
                ser.reset_input_buffer()
                received_rows = read_phase(
                    ser,
                    header,
                    args.samples,
                    timeout_s=max(10.0, args.samples / 20.0),
                )
                rows = [row for _, row in received_rows]
                captured_by_phase[phase] = rows
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
                print_phase_summary(header, rows)

            if args.physical_layout:
                print_physical_layout_result(header, captured_by_phase)
    except KeyboardInterrupt:
        print("\nCapture cancelled; completed phases remain in the output file.")
        return 130
    except (OSError, RuntimeError, TimeoutError, serial.SerialException) as exc:
        print(f"[guided capture] {exc}", file=sys.stderr)
        return 1

    print(f"\nComplete: {len(phases) * args.samples} rows written to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
