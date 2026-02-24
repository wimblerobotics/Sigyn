# Sigyn Analysis Documents

This directory contains hardware and system analysis reports for the Sigyn robot.

## Battery Analysis

| Report | Description |
|--------|-------------|
| [battery_analysis_20260224/](battery_analysis_20260224/) | 36V LiPo battery health analysis from 164 Board-2 log files (Feb 24 2026) |

### Key Findings (Battery, 2026-02-24)
- Battery health: **~72%** of rated 30 Ah capacity (~21.5 Ah effective)
- Typical run time from full charge: **3.5 h average** (up to 8.25 h light load)
- BMS cutoff at 36.5 V confirmed correct and functioning
- **INA226 current sensor broken since LOG00072** (firmware calibration register bug, Jan 5 2026)
- Full recharge from cutoff: **5–6 hours** estimated

See the [analysis document](battery_analysis_20260224/battery_analysis_20260224.md) for full details, plots, BT thresholds, and fix recommendations.
