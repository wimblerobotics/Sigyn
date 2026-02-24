#!/usr/bin/env python3
"""
analyze_battery_single.py
--------------------------
Analyze the 36V LiPo battery data from a single Board-2 log file.

Usage:
    python3 analyze_battery_single.py <log_file> [--location 36VLIPO] [--no-plot]

The script:
  - Parses all BATT2 JSON messages for the specified battery location.
  - Detects whether the session is a charge, discharge, or mixed cycle.
  - Reports full-charge voltage, cutoff voltage, average current, Ah consumed,
    estimated run time, and battery health versus rated capacity.
  - Generates a dual-axis plot (voltage + current vs time).
  - Prints behavior-tree-ready thresholds based on the observed data.

Notes on the current sensor:
  Logs generated before approximately LOG00072 carry live current readings
  (A != 0.00).  Later logs show A=0.00 for all readings – the sensor appears
  to have stopped reporting.  When current is absent the script uses the
  voltage-drop rate together with an assumed nominal capacity to estimate
  discharge current.
"""

import argparse
import os
import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")          # headless – writes a PNG
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np


# ── Constants for a 10S (36 V nominal) LiPo pack ──────────────────────────────
CELLS           = 10
CELL_FULL_V     = 4.20          # V/cell  → 42.0 V pack
CELL_CUTOFF_V   = 3.65          # V/cell  → 36.5 V pack (safe cutoff)
PACK_FULL_V     = CELLS * CELL_FULL_V      # 42.0 V
PACK_CUTOFF_V   = CELLS * CELL_CUTOFF_V   # 36.5 V
RATED_AH        = 30.0          # A·h rated capacity
LOCATION        = "36VLIPO"

# Rough LiPo SOC table (V/cell → fraction remaining, 0–1)
# Derived from typical 18650 / LiPo discharge curves
SOC_TABLE = [
    (4.20, 1.00),
    (4.15, 0.95),
    (4.10, 0.90),
    (4.05, 0.85),
    (4.00, 0.79),
    (3.95, 0.73),
    (3.90, 0.66),
    (3.85, 0.59),
    (3.80, 0.52),
    (3.75, 0.46),
    (3.70, 0.40),
    (3.65, 0.33),
    (3.60, 0.26),
    (3.55, 0.18),
    (3.50, 0.11),
    (3.40, 0.04),
    (3.30, 0.01),
]

def soc_from_voltage(pack_v: float) -> float:
    """Return estimated state-of-charge (0–1) from pack voltage."""
    vc = pack_v / CELLS
    if vc >= SOC_TABLE[0][0]:
        return SOC_TABLE[0][1]
    if vc <= SOC_TABLE[-1][0]:
        return SOC_TABLE[-1][1]
    for i in range(len(SOC_TABLE) - 1):
        v1, s1 = SOC_TABLE[i]
        v2, s2 = SOC_TABLE[i + 1]
        if v2 <= vc <= v1:
            frac = (vc - v2) / (v1 - v2)
            return s2 + frac * (s1 - s2)
    return 0.0


def parse_log(path: str, location: str = LOCATION):
    """
    Parse BATT2 records from a log file.

    Returns list of (time_s, voltage_V, current_A) tuples, sorted by time.
    """
    pattern = re.compile(
        r'\[([0-9.]+)\]\s+BATT2:'
        r'\{.*?"V":([0-9.]+),"A":([0-9.-]+),'
        r'.*?"location":"' + re.escape(location) + r'"'
    )
    data = []
    with open(path, "r", errors="ignore") as fh:
        for line in fh:
            m = pattern.search(line)
            if m:
                data.append((float(m.group(1)),
                              float(m.group(2)),
                              float(m.group(3))))
    data.sort(key=lambda x: x[0])
    return data


def detect_cycle_type(voltages):
    """Classify session: 'charge', 'discharge', 'mixed', or 'idle'."""
    if len(voltages) < 10:
        return "unknown"
    v_start = np.median(voltages[:10])
    v_end   = np.median(voltages[-10:])
    delta   = v_end - v_start
    spread  = max(voltages) - min(voltages)
    if spread < 0.5:
        return "idle"
    if delta > 1.0:
        return "charge"
    if delta < -1.0:
        return "discharge"
    # Check for a rise then fall, or fall then rise
    peak_idx = int(np.argmax(voltages))
    trough_idx = int(np.argmin(voltages))
    midpoint = len(voltages) // 2
    if peak_idx < midpoint and trough_idx > midpoint:
        return "charge_then_discharge"
    if trough_idx < midpoint and peak_idx > midpoint:
        return "discharge_then_charge"
    return "mixed"


def compute_stats(data, has_current: bool):
    """Return a dict of key statistics."""
    times  = np.array([d[0] for d in data])
    volts  = np.array([d[1] for d in data])
    amps   = np.array([d[2] for d in data])
    duration_s = times[-1] - times[0]
    duration_h = duration_s / 3600.0

    stats = {
        "n_readings"    : len(data),
        "t_start_s"     : times[0],
        "t_end_s"       : times[-1],
        "duration_s"    : duration_s,
        "duration_h"    : duration_h,
        "v_start"       : float(volts[0]),
        "v_end"         : float(volts[-1]),
        "v_min"         : float(volts.min()),
        "v_max"         : float(volts.max()),
        "v_mean"        : float(volts.mean()),
        "v_delta"       : float(volts[-1] - volts[0]),
        "soc_start"     : soc_from_voltage(float(volts[0])),
        "soc_end"       : soc_from_voltage(float(volts[-1])),
        "cycle_type"    : detect_cycle_type(list(volts)),
        "has_current"   : has_current,
    }

    if has_current:
        stats["a_mean"]        = float(amps.mean())
        stats["a_min"]         = float(amps.min())
        stats["a_max"]         = float(amps.max())
        # Numeric integration of Ah (trapezoidal)
        dt_h = np.diff(times) / 3600.0
        ah   = float(np.sum((amps[:-1] + amps[1:]) / 2.0 * dt_h))
        stats["ah_consumed"]   = ah
        capacity_factor = stats["soc_start"] - stats["soc_end"]
        if capacity_factor > 0.05:
            stats["estimated_capacity_ah"] = ah / capacity_factor
        else:
            stats["estimated_capacity_ah"] = None
    else:
        # Voltage-drop estimation: assume steady discharge
        soc_delta = stats["soc_start"] - stats["soc_end"]
        if soc_delta > 0.01 and duration_h > 0.05:
            ah_est = soc_delta * RATED_AH           # rough, using rated capacity
            stats["a_mean_estimated"] = ah_est / duration_h
            stats["ah_consumed_estimated"] = ah_est
        else:
            stats["a_mean_estimated"] = None
            stats["ah_consumed_estimated"] = None

    return stats


def print_report(stats: dict, log_path: str):
    """Pretty-print analysis report to stdout."""
    sep = "=" * 70
    print(sep)
    print(f"  BATTERY ANALYSIS REPORT  –  {Path(log_path).name}")
    print(sep)
    print(f"  Location tag   : {LOCATION}")
    print(f"  Readings       : {stats['n_readings']}")
    print(f"  Session type   : {stats['cycle_type'].upper()}")
    print()
    print(f"  Time span      : {stats['t_start_s']:.1f} s → {stats['t_end_s']:.1f} s")
    print(f"  Duration       : {stats['duration_s']:.0f} s  ({stats['duration_h']:.2f} h)")
    print()
    print(f"  Voltage start  : {stats['v_start']:.2f} V   ({stats['soc_start']*100:.1f}% SOC)")
    print(f"  Voltage end    : {stats['v_end']:.2f} V   ({stats['soc_end']*100:.1f}% SOC)")
    print(f"  Voltage min    : {stats['v_min']:.2f} V")
    print(f"  Voltage max    : {stats['v_max']:.2f} V")
    print(f"  Voltage Δ      : {stats['v_delta']:+.2f} V")
    print()
    if stats["has_current"]:
        print(f"  Current avg    : {stats['a_mean']:.2f} A")
        print(f"  Current min    : {stats['a_min']:.2f} A")
        print(f"  Current max    : {stats['a_max']:.2f} A")
        print(f"  Ah consumed    : {stats['ah_consumed']:.2f} A·h")
        if stats.get("estimated_capacity_ah"):
            eff = stats["estimated_capacity_ah"] / RATED_AH * 100
            print(f"  Estimated pack : {stats['estimated_capacity_ah']:.1f} A·h  "
                  f"({eff:.0f}% of {RATED_AH:.0f} A·h rated)")
    else:
        print("  Current sensor : NOT REPORTING (A=0.00 throughout)")
        if stats.get("a_mean_estimated") is not None:
            print(f"  Est. avg Amps  : {stats['a_mean_estimated']:.2f} A  "
                  f"(voltage-drop estimate, assuming {RATED_AH:.0f} A·h capacity)")
            print(f"  Est. Ah used   : {stats['ah_consumed_estimated']:.2f} A·h")

    print()
    print("─" * 70)
    print("  BEHAVIOR TREE REFERENCE DATA")
    print("─" * 70)
    print(f"  Full charge voltage  : {PACK_FULL_V:.1f} V  "
          f"({CELL_FULL_V:.2f} V/cell × {CELLS} cells)")
    print(f"  Safe cutoff voltage  : {PACK_CUTOFF_V:.1f} V  "
          f"({CELL_CUTOFF_V:.2f} V/cell × {CELLS} cells)")
    print()
    print("  SOC → voltage mapping for BT thresholds:")
    for soc_pct, label in [(75, "75% – plenty of range"),
                            (50, "50% – plan return trip"),
                            (25, "25% – WARNING: return now"),
                            (10, "10% – CRITICAL: emergency stop")]:
        frac = soc_pct / 100.0
        # Find corresponding voltage
        for i in range(len(SOC_TABLE) - 1):
            v1, s1 = SOC_TABLE[i]
            v2, s2 = SOC_TABLE[i + 1]
            if s2 <= frac <= s1:
                ratio = (frac - s2) / (s1 - s2) if (s1 - s2) != 0 else 0
                v_thresh = v2 + ratio * (v1 - v2)
                print(f"    {soc_pct:3d}%  →  {v_thresh * CELLS:.2f} V  ({label})")
                break

    if stats.get("a_mean") is not None:
        i_avg = stats["a_mean"]
    elif stats.get("a_mean_estimated") is not None:
        i_avg = stats["a_mean_estimated"]
    else:
        i_avg = None

    if i_avg and i_avg > 0:
        print()
        # Remaining runtime at warning thresholds
        print(f"  Remaining runtime estimates at avg {i_avg:.2f} A discharge:")
        for v_warn, label in [(39.5, "39.5 V (50% SOC)"),
                               (38.5, "38.5 V (40% SOC)"),
                               (37.5, "37.5 V (25% SOC)")]:
            soc_now  = soc_from_voltage(v_warn)
            soc_cut  = soc_from_voltage(PACK_CUTOFF_V)
            ah_left  = (soc_now - soc_cut) * RATED_AH
            runtime_min = (ah_left / i_avg) * 60
            print(f"    At {v_warn:.1f} V → ~{runtime_min:.0f} min remaining")
    print(sep)


def plot_session(data, stats: dict, log_path: str, out_dir: str = None):
    """Generate and save a dual-axis voltage + current plot."""
    times  = np.array([d[0] for d in data])
    volts  = np.array([d[1] for d in data])
    amps   = np.array([d[2] for d in data])
    time_h = (times - times[0]) / 3600.0

    fig, ax1 = plt.subplots(figsize=(14, 6))
    fig.patch.set_facecolor("#1e1e2e")
    ax1.set_facecolor("#1e1e2e")

    # Voltage curve
    color_v = "#89b4fa"
    ax1.plot(time_h, volts, color=color_v, linewidth=1.5, label="Voltage (V)")
    ax1.axhline(PACK_FULL_V,   color="#a6e3a1", linestyle="--", linewidth=0.8, label=f"Full charge {PACK_FULL_V:.0f} V")
    ax1.axhline(PACK_CUTOFF_V, color="#f38ba8", linestyle="--", linewidth=0.8, label=f"Cutoff {PACK_CUTOFF_V:.1f} V")
    ax1.axhline(38.5,          color="#fab387", linestyle=":",  linewidth=0.8, label="Warning ~38.5 V")
    ax1.set_xlabel("Time (hours)", color="white", fontsize=11)
    ax1.set_ylabel("Voltage (V)", color=color_v, fontsize=11)
    ax1.tick_params(colors="white")
    ax1.tick_params(axis="y", labelcolor=color_v)
    ax1.set_ylim(35.5, 43.5)
    ax1.yaxis.set_minor_locator(ticker.MultipleLocator(0.5))
    ax1.grid(True, which="major", color="#313244", linewidth=0.5)
    ax1.grid(True, which="minor", color="#1e1e2e", linewidth=0.3)
    for spine in ax1.spines.values():
        spine.set_edgecolor("#45475a")

    # Current curve (if available)
    ax2 = ax1.twinx()
    ax2.set_facecolor("#1e1e2e")
    if stats["has_current"]:
        color_a = "#cba6f7"
        # Smooth with rolling average for display
        window = max(1, len(amps) // 200)
        amps_smooth = np.convolve(amps, np.ones(window) / window, mode="same")
        ax2.plot(time_h, amps_smooth, color=color_a, linewidth=1.2,
                 alpha=0.85, label="Current (A) – rolling avg")
        ax2.set_ylabel("Current (A)", color=color_a, fontsize=11)
        ax2.tick_params(axis="y", labelcolor=color_a)
        ax2.set_ylim(-0.5, amps.max() * 1.4)
    else:
        ax2.set_ylabel("Current (A) – NOT RECORDED", color="#585b70", fontsize=11)
        ax2.tick_params(axis="y", labelcolor="#585b70")
        ax2.set_ylim(0, 10)
        ax2.set_yticks([])

    # Title and legend
    log_name = Path(log_path).name
    cycle_label = stats["cycle_type"].replace("_", " ").title()
    title = (f"{log_name}  –  36V LiPo  –  {cycle_label}\n"
             f"Duration: {stats['duration_h']:.2f} h   "
             f"V: {stats['v_start']:.2f}→{stats['v_end']:.2f} V   "
             f"SOC: {stats['soc_start']*100:.0f}%→{stats['soc_end']*100:.0f}%")
    ax1.set_title(title, color="white", fontsize=11, pad=10)

    # Merge legends
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2,
               loc="upper right", fontsize=8,
               facecolor="#313244", edgecolor="#45475a", labelcolor="white")

    plt.tight_layout()

    if out_dir is None:
        out_dir = os.path.dirname(os.path.abspath(log_path))
    os.makedirs(out_dir, exist_ok=True)
    stem = Path(log_path).stem
    out_path = os.path.join(out_dir, f"{stem}_battery_36VLIPO.png")
    plt.savefig(out_path, dpi=150, facecolor=fig.get_facecolor())
    plt.close()
    print(f"\n  Plot saved: {out_path}")
    return out_path


def main():
    parser = argparse.ArgumentParser(
        description="Analyze 36V LiPo battery data from a Board-2 log file."
    )
    parser.add_argument("log_file", help="Path to the .TXT log file")
    parser.add_argument("--location", default=LOCATION,
                        help=f"Battery location tag (default: {LOCATION})")
    parser.add_argument("--no-plot", action="store_true",
                        help="Skip plot generation")
    parser.add_argument("--plot-dir", default=None,
                        help="Directory to save plots (default: same as log file)")
    args = parser.parse_args()

    if not os.path.isfile(args.log_file):
        print(f"ERROR: File not found: {args.log_file}", file=sys.stderr)
        sys.exit(1)

    print(f"Parsing {args.log_file} …")
    data = parse_log(args.log_file, args.location)

    if not data:
        print(f"No BATT2 records found for location '{args.location}' in {args.log_file}")
        sys.exit(0)

    amps = [d[2] for d in data]
    has_current = any(abs(a) > 0.01 for a in amps)

    stats = compute_stats(data, has_current)
    print_report(stats, args.log_file)

    if not args.no_plot:
        plot_session(data, stats, args.log_file, args.plot_dir)


if __name__ == "__main__":
    main()
