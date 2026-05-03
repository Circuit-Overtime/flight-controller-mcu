"""Profile RX stick noise.

Listens to the firmware's CSV telemetry stream for N seconds, records the
four AETR channels, and prints noise statistics so we can pick the right
filter:

  - mean / std deviation       -> static noise floor
  - peak-to-peak range         -> worst case excursion at rest
  - sample-to-sample |delta|   -> single-frame jitter
  - delta histogram            -> distribution of sudden jumps

Run with sticks at rest in their typical hover position (centered for
roll/pitch/yaw, throttle wherever you'd trim it). 60 s is enough to see
both fast and slow noise.

Usage:
    python rx_profile.py                                       # /dev/ttyACM0, 460800, 60s
    python rx_profile.py /dev/ttyACM0 460800 60
"""

from __future__ import annotations

import statistics
import sys
import time
from collections import Counter

import serial


CHANNEL_NAMES = {
    0: "CH1 roll    ",
    1: "CH2 pitch   ",
    2: "CH3 throttle",
    3: "CH4 yaw     ",
}


def main() -> int:
    port  = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    baud  = int(sys.argv[2]) if len(sys.argv) > 2 else 460800
    secs  = float(sys.argv[3]) if len(sys.argv) > 3 else 60.0

    print(f"Logging RX from {port} @ {baud} for {secs:.0f} s")
    print("Hold sticks in TYPICAL flight position. Don't move them.")
    print()

    samples: dict[int, list[int]] = {0: [], 1: [], 2: [], 3: []}
    line_count = 0
    start = time.monotonic()

    try:
        with serial.Serial(port, baud, timeout=1) as ser:
            while time.monotonic() - start < secs:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="ignore").strip()
                if not line or line.startswith("#") or line.startswith("roll,"):
                    continue
                parts = line.split(",")
                if len(parts) != 16:
                    continue
                try:
                    samples[0].append(int(parts[4]))   # ch1
                    samples[1].append(int(parts[5]))   # ch2
                    samples[2].append(int(parts[6]))   # ch3
                    samples[3].append(int(parts[7]))   # ch4
                    line_count += 1
                except ValueError:
                    continue
    except serial.SerialException as exc:
        print(f"[serial] {exc}", file=sys.stderr)
        return 1

    elapsed = time.monotonic() - start
    if line_count == 0:
        print("No samples received. Check port, baud, and that the FC is streaming.",
              file=sys.stderr)
        return 1

    print(f"Captured {line_count} samples over {elapsed:.1f} s "
          f"({line_count / elapsed:.0f} Hz)")
    print()

    # ---- absolute statistics ------------------------------------------------
    print(f"{'channel':<13}{'mean':>9}{'std':>8}{'min':>8}{'max':>8}{'p2p':>8}")
    print("-" * 54)
    for i in range(4):
        s = samples[i]
        if len(s) < 2:
            continue
        mean = statistics.mean(s)
        std  = statistics.stdev(s)
        lo, hi = min(s), max(s)
        print(f"{CHANNEL_NAMES[i]:<13}"
              f"{mean:>9.1f}{std:>8.2f}{lo:>8d}{hi:>8d}{hi - lo:>8d}")

    # ---- frame-to-frame jitter ---------------------------------------------
    print()
    print(f"{'channel':<13}{'mean|d|':>10}{'p95|d|':>9}{'max|d|':>9}{'>10us':>10}{'>30us':>10}")
    print("-" * 61)
    for i in range(4):
        s = samples[i]
        if len(s) < 2:
            continue
        deltas = [abs(s[k] - s[k - 1]) for k in range(1, len(s))]
        deltas_sorted = sorted(deltas)
        p95 = deltas_sorted[int(0.95 * (len(deltas_sorted) - 1))]
        big10 = sum(1 for d in deltas if d > 10)
        big30 = sum(1 for d in deltas if d > 30)
        print(f"{CHANNEL_NAMES[i]:<13}"
              f"{statistics.mean(deltas):>10.2f}"
              f"{p95:>9d}"
              f"{max(deltas):>9d}"
              f"{big10:>10d}"
              f"{big30:>10d}")

    # ---- delta histogram (combined across channels) ------------------------
    print()
    print("Sample-to-sample |delta| histogram (all 4 channels combined):")
    all_deltas: list[int] = []
    for i in range(4):
        s = samples[i]
        for k in range(1, len(s)):
            all_deltas.append(abs(s[k] - s[k - 1]))
    bins = Counter()
    for d in all_deltas:
        if d == 0:
            bins["0"] += 1
        elif d <= 4:
            bins["1-4"] += 1
        elif d <= 10:
            bins["5-10"] += 1
        elif d <= 20:
            bins["11-20"] += 1
        elif d <= 50:
            bins["21-50"] += 1
        else:
            bins[">50"] += 1
    total = sum(bins.values()) or 1
    for label in ("0", "1-4", "5-10", "11-20", "21-50", ">50"):
        n   = bins[label]
        pct = 100.0 * n / total
        bar = "#" * int(pct / 2)
        print(f"  {label:<7} {n:>7d} ({pct:5.1f}%) {bar}")

    # ---- recommendation ----------------------------------------------------
    print()
    worst_std = max(statistics.stdev(s) for s in samples.values() if len(s) >= 2)
    worst_max_d = max(
        max(abs(s[k] - s[k - 1]) for k in range(1, len(s)))
        for s in samples.values() if len(s) >= 2
    )
    print(f"Suggestion: worst std={worst_std:.2f} us, worst |delta|={worst_max_d} us")
    if worst_max_d <= 10 and worst_std <= 3:
        print("  -> noise is fine; current ±10 us display hysteresis is enough.")
    elif worst_max_d <= 30:
        print("  -> add a light EMA on the FC side (alpha ~0.7) for cleaner control.")
    else:
        print("  -> outliers >30 us; consider a 3-sample median filter to reject spikes.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
