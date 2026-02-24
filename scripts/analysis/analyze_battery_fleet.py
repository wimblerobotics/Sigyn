#!/usr/bin/env python3
"""
analyze_battery_fleet.py
-------------------------
Scan every Board-2 log file in a directory tree and produce a fleet-level
battery health report for the 36V LiPo pack.

Usage:
    python3 analyze_battery_fleet.py [log_dir]   (default: current directory)
    python3 analyze_battery_fleet.py ~/board2Logs/20260224 --plot-dir ~/board2Logs/plots

Output:
    • Console report with per-log and aggregate statistics.
    • CSV file:  <log_dir>/fleet_battery_report.csv
    • PNG plots: <plot_dir>/fleet_*.png

What the script finds:
  1. For each log file: cycle type, voltage range, SOC range, duration,
     average current (if sensor was working), estimated Ah consumed.
  2. Discharge cycle summary: full-charge V, cutoff V, typical runtime,
     average current per session.
  3. Battery health trend: effective capacity over successive discharge
     cycles (capacity fade detection).
  4. Charging cycle summary: charge duration, final voltage reached.
  5. Behavior-tree thresholds recommended from observed data.
"""

import argparse
import csv
import os
import re
import sys
from pathlib import Path
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np


# ─── LiPo constants ───────────────────────────────────────────────────────────
CELLS         = 10
CELL_FULL_V   = 4.20
CELL_CUTOFF_V = 3.65
PACK_FULL_V   = CELLS * CELL_FULL_V      # 42.0 V
PACK_CUTOFF_V = CELLS * CELL_CUTOFF_V   # 36.5 V
RATED_AH      = 30.0
LOCATION      = "36VLIPO"

SOC_TABLE = [
    (4.20, 1.00), (4.15, 0.95), (4.10, 0.90), (4.05, 0.85),
    (4.00, 0.79), (3.95, 0.73), (3.90, 0.66), (3.85, 0.59),
    (3.80, 0.52), (3.75, 0.46), (3.70, 0.40), (3.65, 0.33),
    (3.60, 0.26), (3.55, 0.18), (3.50, 0.11), (3.40, 0.04),
    (3.30, 0.01),
]

def soc_from_voltage(pack_v: float) -> float:
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


# ─── Parsing ──────────────────────────────────────────────────────────────────
PATTERN = re.compile(
    r'\[([0-9.]+)\]\s+BATT2:'
    r'\{.*?"V":([0-9.]+),"A":([0-9.-]+),'
    r'.*?"location":"' + re.escape(LOCATION) + r'"'
)

def parse_log(path: str):
    """Return sorted list of (time_s, voltage_V, current_A)."""
    data = []
    try:
        with open(path, "r", errors="ignore") as fh:
            for line in fh:
                m = PATTERN.search(line)
                if m:
                    data.append((float(m.group(1)),
                                  float(m.group(2)),
                                  float(m.group(3))))
    except OSError:
        pass
    data.sort(key=lambda x: x[0])
    return data


def classify_cycle(data):
    """
    Returns (cycle_type, sub_info) where cycle_type is one of:
      'discharge'  – voltage predominantly falling
      'charge'     – voltage predominantly rising
      'idle'       – voltage nearly flat, very little change
      'mixed'      – significant swing in both directions
    """
    if len(data) < 20:
        return "short", {}
    volts = np.array([d[1] for d in data])
    v_start = float(np.median(volts[:10]))
    v_end   = float(np.median(volts[-10:]))
    v_delta = v_end - v_start
    v_range = float(volts.max() - volts.min())

    if v_range < 0.4:
        return "idle", {"v_range": v_range}
    if v_delta < -1.0:
        return "discharge", {"delta": v_delta}
    if v_delta > 1.0:
        return "charge", {"delta": v_delta}
    return "mixed", {"delta": v_delta, "range": v_range}


def analyze_file(path: str):
    """Return per-log stats dict, or None if no data found."""
    data = parse_log(path)
    if len(data) < 20:
        return None

    times = np.array([d[0] for d in data])
    volts = np.array([d[1] for d in data])
    amps  = np.array([d[2] for d in data])

    has_current = bool(np.any(np.abs(amps) > 0.01))
    duration_h  = float((times[-1] - times[0]) / 3600.0)
    cycle_type, _ = classify_cycle(data)

    v_start = float(np.median(volts[:10]))
    v_end   = float(np.median(volts[-10:]))

    row = {
        "file"        : Path(path).name,
        "n"           : len(data),
        "duration_h"  : round(duration_h, 3),
        "cycle_type"  : cycle_type,
        "v_start"     : round(v_start, 2),
        "v_end"       : round(v_end, 2),
        "v_min"       : round(float(volts.min()), 2),
        "v_max"       : round(float(volts.max()), 2),
        "soc_start"   : round(soc_from_voltage(v_start), 3),
        "soc_end"     : round(soc_from_voltage(v_end), 3),
        "has_current" : has_current,
        "a_mean"      : None,
        "a_max"       : None,
        "ah_consumed" : None,
        "est_capacity": None,
    }

    if has_current:
        row["a_mean"] = round(float(amps.mean()), 3)
        row["a_max"]  = round(float(amps.max()), 3)
        # Trapezoidal Ah integration
        dt_h = np.diff(times) / 3600.0
        ah = float(np.sum((amps[:-1] + amps[1:]) / 2.0 * dt_h))
        row["ah_consumed"] = round(ah, 3)
        soc_delta = row["soc_start"] - row["soc_end"]
        if soc_delta > 0.05 and cycle_type == "discharge":
            row["est_capacity"] = round(ah / soc_delta, 1)
    else:
        # Voltage-drop capacity estimate (discharge sessions only)
        if cycle_type == "discharge":
            soc_delta = row["soc_start"] - row["soc_end"]
            if soc_delta > 0.01:
                ah_est = soc_delta * RATED_AH
                row["ah_consumed"]  = round(ah_est, 2)
                row["a_mean"]       = round(ah_est / duration_h, 3) if duration_h > 0 else None

    return row


def voltage_from_soc(soc_frac: float) -> float:
    """Inverse of soc_from_voltage: return pack voltage at a given SOC fraction."""
    for i in range(len(SOC_TABLE) - 1):
        v1, s1 = SOC_TABLE[i]
        v2, s2 = SOC_TABLE[i + 1]
        if s2 <= soc_frac <= s1:
            ratio = (soc_frac - s2) / (s1 - s2) if (s1 - s2) != 0 else 0
            return (v2 + ratio * (v1 - v2)) * CELLS
    if soc_frac >= SOC_TABLE[0][1]:
        return SOC_TABLE[0][0] * CELLS
    return SOC_TABLE[-1][0] * CELLS


def recommend_bt_thresholds(discharge_rows):
    """
    Derive behavior-tree thresholds from aggregate discharge session data.
    Returns a dict with voltage thresholds and runtime estimates.

    Thresholds are chosen so that remaining runtime is ABOVE zero at the
    observed cutoff voltage.  The cutoff SOC reference is taken as the SOC
    corresponding to the minimum end-voltage observed across all sessions
    (i.e. the actual lowest voltage the battery has survived to).
    """
    if not discharge_rows:
        return {}

    # Average current from sessions where the current sensor was ACTUALLY
    # reporting (not the voltage-drop estimates).
    currents = [r["a_mean"] for r in discharge_rows
                if r["a_mean"] is not None and r["has_current"]]
    # Fall back to all a_mean estimates if no real sensor data available
    if not currents:
        currents = [r["a_mean"] for r in discharge_rows
                    if r["a_mean"] is not None]
    avg_current = float(np.mean(currents)) if currents else 2.0

    # Effective capacity estimate from sessions with real current data
    capacities = [r["est_capacity"] for r in discharge_rows
                  if r["est_capacity"] is not None and r["has_current"]]
    if not capacities:
        capacities = [r["est_capacity"] for r in discharge_rows
                      if r["est_capacity"] is not None]
    eff_cap = float(np.median(capacities)) if capacities else RATED_AH

    # Observed cutoff = lowest end voltage seen (used as effective dead-band)
    obs_cutoff_v = min(r["v_min"] for r in discharge_rows)
    obs_cutoff_v = max(obs_cutoff_v, PACK_CUTOFF_V)   # never exceed design cutoff
    soc_at_cutoff = soc_from_voltage(obs_cutoff_v)

    # Thresholds: plan (60% SOC), warn (45% SOC), emergency (36% SOC)
    # These are chosen to always remain comfortably above the cutoff SOC.
    thresholds = {}
    for soc_warn_pct, label in [
        (60, "plan_return"),
        (45, "warn_return"),
        (36, "emergency"),
    ]:
        frac = soc_warn_pct / 100.0
        if frac <= soc_at_cutoff:
            # Can't go below cutoff
            frac = soc_at_cutoff + 0.02
        pack_v = voltage_from_soc(frac)
        ah_rem = (frac - soc_at_cutoff) * eff_cap
        min_rem = (ah_rem / avg_current * 60) if avg_current > 0 else None
        thresholds[label] = {
            "soc_pct": soc_warn_pct,
            "voltage": round(pack_v, 2),
            "minutes_remaining": round(min_rem, 0) if min_rem is not None else None,
        }

    thresholds["avg_discharge_current_A"] = round(avg_current, 2)
    thresholds["effective_capacity_Ah"]   = round(eff_cap, 1)
    thresholds["rated_capacity_Ah"]       = RATED_AH
    thresholds["health_pct"]              = round(eff_cap / RATED_AH * 100, 1)
    thresholds["observed_cutoff_v"]       = round(obs_cutoff_v, 2)

    return thresholds


def print_console_report(results, thresholds):
    sep = "=" * 72
    thin = "─" * 72

    discharge_rows = [r for r in results if r["cycle_type"] == "discharge"]
    charge_rows    = [r for r in results if r["cycle_type"] == "charge"]
    idle_rows      = [r for r in results if r["cycle_type"] == "idle"]

    print(sep)
    print("  FLEET BATTERY ANALYSIS  –  36V LiPo")
    print(sep)
    print(f"  Total log files analyzed          : {len(results)}")
    print(f"  Discharge sessions                : {len(discharge_rows)}")
    print(f"  Charge sessions                   : {len(charge_rows)}")
    print(f"  Idle / short sessions             : {len(idle_rows)}")
    print()

    # Voltage extremes
    all_vmax = [r["v_max"] for r in results]
    all_vmin = [r["v_min"] for r in results]
    print(f"  Highest voltage observed          : {max(all_vmax):.2f} V")
    print(f"  Lowest voltage observed           : {min(all_vmin):.2f} V")
    print()

    if discharge_rows:
        durations  = [r["duration_h"] for r in discharge_rows]
        start_volts = [r["v_start"]   for r in discharge_rows]
        end_volts   = [r["v_end"]     for r in discharge_rows]
        currents    = [r["a_mean"]    for r in discharge_rows if r["a_mean"] is not None]

        print(thin)
        print("  DISCHARGE SESSION STATISTICS")
        print(thin)
        print(f"  Sessions                          : {len(discharge_rows)}")
        print(f"  Avg start voltage                 : {np.mean(start_volts):.2f} V")
        print(f"  Avg end voltage                   : {np.mean(end_volts):.2f} V")
        print(f"  Min end voltage (deepest disch.)  : {min(end_volts):.2f} V")
        print(f"  Avg session duration              : {np.mean(durations):.2f} h")
        print(f"  Longest session                   : {max(durations):.2f} h")
        print(f"  Shortest session                  : {min(durations):.2f} h")
        currents_real = [r["a_mean"] for r in discharge_rows
                         if r["a_mean"] is not None and r["has_current"]]
        currents_all  = [r["a_mean"] for r in discharge_rows
                         if r["a_mean"] is not None]
        if currents_real:
            print(f"  Avg discharge current (sensor)    : {np.mean(currents_real):.2f} A  "
                  f"(from {len(currents_real)} sessions with live current sensor)")
        if currents_all:
            print(f"  Avg discharge current (all+est.)  : {np.mean(currents_all):.2f} A  "
                  f"({len(currents_all)} sessions, incl. voltage-drop estimates)")
        print()

    if thresholds:
        print(thin)
        print("  BATTERY HEALTH")
        print(thin)
        eff = thresholds.get("effective_capacity_Ah", RATED_AH)
        rated = thresholds.get("rated_capacity_Ah", RATED_AH)
        health = thresholds.get("health_pct", 100.0)
        print(f"  Rated capacity                    : {rated:.0f} A·h")
        print(f"  Estimated effective capacity      : {eff:.1f} A·h")
        print(f"  Health                            : {health:.0f}%")
        if health < 70:
            print("  ⚠  Battery health below 70% – consider replacement.")
        elif health < 85:
            print("  ⚠  Battery health below 85% – monitor closely.")
        else:
            print("  ✓  Battery health looks acceptable.")
        print()

        print(thin)
        print("  BEHAVIOR TREE THRESHOLDS  (copy into BT config)")
        print(thin)
        print(f"  Full charge voltage  : {PACK_FULL_V:.1f} V")
        print(f"  Safe cutoff voltage  : {PACK_CUTOFF_V:.1f} V")
        print(f"  Avg discharge current: {thresholds['avg_discharge_current_A']:.2f} A")
        print()
        labels = {
            "plan_return" : "Plan return trip (60% SOC)",
            "warn_return" : "Warning – return NOW (45% SOC)",
            "emergency"   : "EMERGENCY – stop & call for help (36% SOC)",
        }
        for key in ["plan_return", "warn_return", "emergency"]:
            if key in thresholds:
                t = thresholds[key]
                min_rem = t.get("minutes_remaining", "N/A")
                min_str = f"{min_rem:.0f} min" if min_rem else "N/A"
                print(f"  {labels[key]}")
                print(f"    Voltage: {t['voltage']:.2f} V   Remaining runtime: ~{min_str}")
                print()

    print(sep)


def save_csv(results, out_path: str):
    fieldnames = [
        "file", "n", "duration_h", "cycle_type",
        "v_start", "v_end", "v_min", "v_max",
        "soc_start", "soc_end",
        "has_current", "a_mean", "a_max",
        "ah_consumed", "est_capacity",
    ]
    with open(out_path, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)
    print(f"  CSV saved: {out_path}")


def plot_fleet_overview(results, out_dir: str):
    """Four-panel fleet plot."""
    discharge_rows = [r for r in results
                      if r["cycle_type"] == "discharge" and r["v_start"] > 40]
    charge_rows    = [r for r in results if r["cycle_type"] == "charge"]

    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    fig.patch.set_facecolor("#1e1e2e")
    for ax in axes.flat:
        ax.set_facecolor("#1e1e2e")
        ax.tick_params(colors="white")
        for spine in ax.spines.values():
            spine.set_edgecolor("#45475a")
        ax.title.set_color("white")
        ax.xaxis.label.set_color("white")
        ax.yaxis.label.set_color("white")

    idx = list(range(len(results)))

    # Panel 1 – Voltage range per log (min/max bars)
    ax = axes[0, 0]
    v_mins = [r["v_min"] for r in results]
    v_maxs = [r["v_max"] for r in results]
    colors = []
    for r in results:
        ct = r["cycle_type"]
        if ct == "discharge":   colors.append("#f38ba8")
        elif ct == "charge":    colors.append("#a6e3a1")
        elif ct == "idle":      colors.append("#585b70")
        else:                   colors.append("#cba6f7")
    ax.bar(idx, [mx - mn for mx, mn in zip(v_maxs, v_mins)],
           bottom=v_mins, color=colors, alpha=0.8, width=1.0)
    ax.axhline(PACK_FULL_V,   color="#a6e3a1", linestyle="--", linewidth=0.8)
    ax.axhline(PACK_CUTOFF_V, color="#f38ba8", linestyle="--", linewidth=0.8)
    ax.axhline(38.5,          color="#fab387", linestyle=":",  linewidth=0.8)
    ax.set_xlabel("Log file index")
    ax.set_ylabel("Voltage (V)")
    ax.set_title("Voltage Range per Log  (red=disch, green=charge, grey=idle)")
    ax.set_ylim(35, 44)

    # Panel 2 – Session duration per log (discharge only)
    ax = axes[0, 1]
    if discharge_rows:
        filenames = [r["file"] for r in discharge_rows]
        durations = [r["duration_h"] for r in discharge_rows]
        x = list(range(len(discharge_rows)))
        bars = ax.bar(x, durations, color="#89b4fa", alpha=0.85, width=0.7)
        # Annotate with log names if not too many
        if len(x) <= 30:
            ax.set_xticks(x)
            ax.set_xticklabels([f[3:8] for f in filenames],
                               rotation=45, fontsize=7, color="white")
        mean_dur = float(np.mean(durations))
        ax.axhline(mean_dur, color="#fab387", linestyle="--", linewidth=1.0,
                   label=f"Mean {mean_dur:.2f} h")
        ax.legend(facecolor="#313244", edgecolor="#45475a", labelcolor="white", fontsize=8)
        ax.set_xlabel("Discharge sessions (near-full start)")
        ax.set_ylabel("Duration (hours)")
        ax.set_title("Discharge Session Durations")
    else:
        ax.text(0.5, 0.5, "No discharge sessions found", ha="center",
                va="center", color="white", transform=ax.transAxes)
        ax.set_title("Discharge Session Durations")

    # Panel 3 – End voltage histogram (shows how deep discharges go)
    ax = axes[1, 0]
    all_end_v = [r["v_end"] for r in results if r["cycle_type"] == "discharge"]
    if all_end_v:
        ax.hist(all_end_v, bins=20, color="#cba6f7", edgecolor="#313244", alpha=0.85)
        ax.axvline(PACK_CUTOFF_V, color="#f38ba8", linestyle="--", linewidth=1.2,
                   label=f"Cutoff {PACK_CUTOFF_V:.1f} V")
        ax.axvline(38.5, color="#fab387", linestyle=":", linewidth=1.0,
                   label="Warning 38.5 V")
        ax.legend(facecolor="#313244", edgecolor="#45475a", labelcolor="white", fontsize=8)
        ax.set_xlabel("Session end voltage (V)")
        ax.set_ylabel("Count")
        ax.set_title("Discharge End-Voltage Distribution")
    else:
        ax.text(0.5, 0.5, "No discharge data", ha="center",
                va="center", color="white", transform=ax.transAxes)
        ax.set_title("Discharge End-Voltage Distribution")

    # Panel 4 – Effective capacity trend (sessions with current sensor)
    ax = axes[1, 1]
    cap_rows = [r for r in results
                if r["est_capacity"] is not None and r["cycle_type"] == "discharge"]
    if cap_rows:
        cap_x = list(range(len(cap_rows)))
        cap_y = [r["est_capacity"] for r in cap_rows]
        ax.plot(cap_x, cap_y, "o-", color="#89b4fa", linewidth=1.5,
                markersize=5, label="Est. capacity (A·h)")
        ax.axhline(RATED_AH, color="#a6e3a1", linestyle="--", linewidth=0.8,
                   label=f"Rated {RATED_AH:.0f} A·h")
        ax.axhline(RATED_AH * 0.80, color="#fab387", linestyle=":", linewidth=0.8,
                   label="80% health")
        ax.legend(facecolor="#313244", edgecolor="#45475a", labelcolor="white", fontsize=8)
        ax.set_xlabel("Session index (with current sensor)")
        ax.set_ylabel("Effective Capacity (A·h)")
        ax.set_title("Battery Capacity Trend")
    else:
        ax.text(0.5, 0.5, "No capacity data\n(current sensor needed)",
                ha="center", va="center", color="white",
                transform=ax.transAxes, fontsize=10)
        ax.set_title("Battery Capacity Trend")

    fig.suptitle("36V LiPo Fleet Battery Analysis", color="white", fontsize=14, y=0.995)
    plt.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "fleet_battery_overview.png")
    plt.savefig(out_path, dpi=150, facecolor=fig.get_facecolor())
    plt.close()
    print(f"  Fleet plot saved: {out_path}")


def plot_discharge_voltage_curves(results, log_dir: str, out_dir: str):
    """
    Overlay voltage curves for all near-full-charge discharge sessions on one plot.
    Time axis is normalized so t=0 is the start of each discharge.
    """
    discharge_rows = [r for r in results
                      if r["cycle_type"] == "discharge" and r["v_start"] > 40.0]
    if not discharge_rows:
        return

    fig, ax = plt.subplots(figsize=(14, 7))
    fig.patch.set_facecolor("#1e1e2e")
    ax.set_facecolor("#1e1e2e")

    colors = plt.cm.plasma(np.linspace(0.1, 0.9, len(discharge_rows)))  # type: ignore[attr-defined]

    for i, row in enumerate(discharge_rows):
        path = os.path.join(log_dir, row["file"])
        data = parse_log(path)
        if not data:
            continue
        times = np.array([d[0] for d in data])
        volts = np.array([d[1] for d in data])
        time_h = (times - times[0]) / 3600.0
        ax.plot(time_h, volts, linewidth=1.0, color=colors[i],
                alpha=0.8, label=row["file"].replace(".TXT", ""))

    ax.axhline(PACK_FULL_V,   color="#a6e3a1", linestyle="--", linewidth=1.0,
               label=f"Full {PACK_FULL_V:.0f} V")
    ax.axhline(PACK_CUTOFF_V, color="#f38ba8", linestyle="--", linewidth=1.0,
               label=f"Cutoff {PACK_CUTOFF_V:.1f} V")
    ax.axhline(38.5, color="#fab387", linestyle=":", linewidth=0.8,
               label="Warning 38.5 V")

    ax.set_xlabel("Time since discharge start (hours)", color="white", fontsize=11)
    ax.set_ylabel("Pack Voltage (V)", color="white", fontsize=11)
    ax.tick_params(colors="white")
    ax.set_ylim(35.5, 43.5)
    ax.yaxis.set_minor_locator(ticker.MultipleLocator(0.5))
    ax.grid(True, which="major", color="#313244", linewidth=0.5)
    for spine in ax.spines.values():
        spine.set_edgecolor("#45475a")

    n = len(discharge_rows)
    ax.set_title(f"36V LiPo – All Discharge Curves from Full Charge ({n} sessions)",
                 color="white", fontsize=12)
    # Legend only if not too crowded
    if n <= 20:
        ax.legend(loc="upper right", fontsize=7, facecolor="#313244",
                  edgecolor="#45475a", labelcolor="white", ncol=2)

    plt.tight_layout()
    out_path = os.path.join(out_dir, "fleet_discharge_curves.png")
    plt.savefig(out_path, dpi=150, facecolor=fig.get_facecolor())
    plt.close()
    print(f"  Discharge curves plot saved: {out_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Fleet-level 36V LiPo analysis across all Board-2 log files."
    )
    parser.add_argument("log_dir", nargs="?", default=".",
                        help="Directory containing .TXT log files (default: .)")
    parser.add_argument("--plot-dir", default=None,
                        help="Directory for output plots (default: <log_dir>/plots)")
    parser.add_argument("--no-plots", action="store_true",
                        help="Skip all plot generation")
    args = parser.parse_args()

    log_dir = os.path.abspath(args.log_dir)
    if not os.path.isdir(log_dir):
        print(f"ERROR: Not a directory: {log_dir}", file=sys.stderr)
        sys.exit(1)

    plot_dir = args.plot_dir or os.path.join(os.path.dirname(log_dir), "plots")

    # Discover all TXT files
    log_files = sorted(Path(log_dir).rglob("*.TXT"))
    log_files += sorted(Path(log_dir).rglob("*.txt"))
    if not log_files:
        print(f"No .TXT files found in {log_dir}")
        sys.exit(0)

    print(f"Scanning {len(log_files)} log files in {log_dir} …")
    results = []
    for lf in log_files:
        row = analyze_file(str(lf))
        if row:
            results.append(row)

    print(f"  {len(results)} files had battery data.\n")

    discharge_rows = [r for r in results if r["cycle_type"] == "discharge"]
    thresholds = recommend_bt_thresholds(discharge_rows)

    print_console_report(results, thresholds)

    csv_path = os.path.join(log_dir, "fleet_battery_report.csv")
    save_csv(results, csv_path)

    if not args.no_plots:
        plot_fleet_overview(results, plot_dir)
        plot_discharge_voltage_curves(results, log_dir, plot_dir)

    # Print BT config block for easy copy into behavior tree code
    if thresholds:
        print()
        print("# ─── Paste into behavior_tree_battery_params.yaml ───────────────────────────")
        eff   = thresholds.get("effective_capacity_Ah", RATED_AH)
        avg_a = thresholds.get("avg_discharge_current_A", 2.0)
        obs_c = thresholds.get("observed_cutoff_v", PACK_CUTOFF_V)
        print(f"battery_36vlipo:")
        print(f"  full_charge_v:      {PACK_FULL_V:.1f}")
        print(f"  cutoff_v:           {obs_c:.2f}  # lowest voltage observed in logs")
        print(f"  rated_ah:           {RATED_AH:.1f}")
        print(f"  effective_ah:       {eff:.1f}   # measured from logs")
        print(f"  avg_discharge_a:    {avg_a:.2f}  # measured from logs")
        for key in ["plan_return", "warn_return", "emergency"]:
            if key in thresholds:
                t = thresholds[key]
                min_r = t.get("minutes_remaining")
                min_s = f"~{min_r:.0f} min remaining" if min_r is not None else "N/A"
                print(f"  {key}_v:        {t['voltage']:.2f}  "
                      f"# {t['soc_pct']}% SOC  {min_s}")
        print("# ───────────────────────────────────────────────────────────────────────────")


if __name__ == "__main__":
    main()
