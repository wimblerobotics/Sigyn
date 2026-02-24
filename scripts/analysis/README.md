# Battery Analysis Scripts

Scripts for analyzing 36V LiPo battery health from Board-2 (Power_Sensors) log files.

## Location

These scripts live in `sigyn_ws/src/Sigyn/scripts/analysis/`.  
Working log data lives in `~/board2Logs/` (or wherever your SD-card logs are extracted to).

---

## Scripts

| Script | Purpose |
|--------|---------|
| `analyze_battery_single.py` | Deep analysis of **one** log file: voltage/current plot, SOC estimates, BT thresholds |
| `analyze_battery_fleet.py` | Aggregate analysis across **all** log files in a directory: health trend, fleet statistics, BT YAML block |

---

## Quick Start

```bash
# Activate your Python environment
source ~/sigyn-venv/bin/activate

# --- Single log analysis ---
cd ~/board2Logs
python3 ~/sigyn_ws/src/Sigyn/scripts/analysis/analyze_battery_single.py \
    20260224/LOG00161.TXT \
    --plot-dir 20260224/plots

# --- Fleet analysis (all logs in a directory) ---
python3 ~/sigyn_ws/src/Sigyn/scripts/analysis/analyze_battery_fleet.py \
    ~/board2Logs/20260224 \
    --plot-dir ~/board2Logs/20260224/plots
```

---

## `analyze_battery_single.py`

### Usage
```
analyze_battery_single.py <log_file> [options]
```

### Arguments
| Argument | Description | Default |
|----------|-------------|---------|
| `log_file` | Path to the `.TXT` Board-2 log file | *(required)* |
| `--location` | Battery location tag in the BATT2 JSON | `36VLIPO` |
| `--no-plot` | Skip PNG plot generation | off |
| `--plot-dir` | Directory to save the PNG plot | same dir as log file |

### Output
- Console report: session type, voltage range, SOC start/end, current stats, Ah consumed, estimated pack capacity, BT threshold table, remaining-runtime estimates at key voltages.
- PNG file: dual-axis voltage + current vs time (dark theme).

### Example output
```
======================================================================
  BATTERY ANALYSIS REPORT  –  LOG00161.TXT
======================================================================
  Session type   : DISCHARGE
  Duration       : 16213 s  (4.50 h)
  Voltage start  : 41.97 V   (99.7% SOC)
  Voltage end    : 37.09 V   (41.1% SOC)
  Est. avg Amps  : 3.90 A
  Est. Ah used   : 17.59 A·h
  ...
```

---

## `analyze_battery_fleet.py`

### Usage
```
analyze_battery_fleet.py [log_dir] [options]
```

### Arguments
| Argument | Description | Default |
|----------|-------------|---------|
| `log_dir` | Directory containing `.TXT` log files (recursive) | `.` (current dir) |
| `--plot-dir` | Directory to save PNG plots | `<log_dir>/../plots` |
| `--no-plots` | Skip all plot generation | off |

### Output
- Console report: fleet statistics, discharge session summary, battery health estimate (effective vs rated capacity), **BT threshold YAML block** ready to paste into your behavior tree config.
- `fleet_battery_report.csv` — per-log statistics.
- `fleet_battery_overview.png` — 4-panel summary chart.
- `fleet_discharge_curves.png` — overlay of all discharge-from-full-charge sessions.

### YAML block (example)
```yaml
battery_36vlipo:
  full_charge_v:      42.0
  cutoff_v:           36.55
  rated_ah:           30.0
  effective_ah:       21.5
  avg_discharge_a:    1.87
  plan_return_v:      38.57    # 60% SOC  ~181 min remaining
  warn_return_v:      37.42    # 45% SOC  ~78 min remaining
  emergency_v:        36.71    # 36% SOC  ~16 min remaining
```

---

## Legacy Scripts (pre-2026)

These older scripts were found in `~/board2Logs/` and are preserved here for reference.

| Script | Purpose | Limitation |
|--------|---------|-----------|
| `battery_analysis.py` | Aggregate min/max/avg stats by location, saves CSV | **Skips zero-current rows** — unusable on LOG00072+ where current sensor broke; reports no data for most recent logs |
| `battery_histogram_plotter.py` | Histograms of voltage/current distributions | Same zero-current filter; requires `seaborn`; useful only for LOG00001–LOG00071 |

### Running legacy scripts
```bash
source ~/sigyn-venv/bin/activate
pip install seaborn   # needed for histogram plotter

cd ~/board2Logs/20260224
python3 ~/sigyn_ws/src/Sigyn/scripts/analysis/battery_analysis.py
python3 ~/sigyn_ws/src/Sigyn/scripts/analysis/battery_histogram_plotter.py
```

---

## Requirements

```bash
pip install matplotlib numpy pandas
```

All three are already installed in `sigyn-venv`.

---

## Battery Location Tags (from logs)

| Tag | Description | IDX |
|-----|-------------|-----|
| `36VLIPO` | Main 36V LiPo drive battery | 0 |
| `5VDCDC` | 5V DC-DC converter output | 1 |
| `12VDCDC` | 12V DC-DC converter output | 2 |
| `24VDCDC` | 24V DC-DC converter output | 3 |
| `3.3VDCDC` | 3.3V DC-DC converter output | 4 |

Pass `--location 12VDCDC` (etc.) to analyze other power rails.

---

## Notes on Current Sensor

- The INA226 power/current monitors (I²C addr `0x40`, mux channels 2–6) reported live current data up through **LOG00071** (firmware compiled Jan 5 2026 16:10).
- Starting with **LOG00072** (firmware compiled Jan 5 2026 23:26), **all** current readings became `A=0.00`. This is consistent with the INA226 `CALIBRATION` register (0x05) being zeroed in a firmware change — with `CAL=0` the current register always returns 0.
- See the full analysis report for detailed recommendations.

---

## Related Files

- Analysis reports: `sigyn_ws/src/Sigyn/docs/analysis/`
- Raw logs: `~/board2Logs/` (Board-2 SD card extracts)
- Log format: `[timestamp_ms] BATT2:{JSON}` — one line per sensor per second
