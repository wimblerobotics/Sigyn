# Topic Analysis Pipeline

A four-script toolkit for discovering, measuring, describing, and reporting on all ROS 2 topics published by Sigyn.

---

## Why this exists

When developing a robot with 100+ topics across multiple subsystems, it is easy to lose track of what is being published, at what rate, how much bandwidth it consumes, and what it actually means. This pipeline solves that problem by:

1. **Sampling** all live topics in parallel with a single command
2. **Merging** hand-written, three-level descriptions with the measured data
3. **Generating** a rich Markdown report you can read, share, or commit to the repo

The data-gathering and report-generation steps are separated on purpose: you can experiment with the report format, rewrite descriptions, or change the depth level without waiting for another 15-second sampling run.

---

## Directory layout

```
Sigyn/scripts/topic_analysis/
├── gather.py               # Step 1 – subscribe to all topics, measure Hz/BW/delay
├── describe.py             # Step 2 – merge topic_descriptions.json → enriched JSON
├── report.py               # Step 3 – generate Markdown report
├── run_all.py              # Convenience: run all three steps in order
├── sample_durations.json   # Per-topic sampling hints (auto-updated by gather.py)
├── topic_descriptions.json # Human-written 3-level topic descriptions (never auto-overwritten)
└── README.md               # This file

Sigyn/docs/artifacts/       # Generated at runtime – not committed to git
├── topic_data.json         # Raw measured data from gather.py
├── topic_data_described.json  # Enriched data from describe.py
└── topic_analysis.md       # Final report from report.py

Sigyn/docs/specs/
└── topic_analysis_system.spec.md  # Design specification for this system
```

---

## Quick start

```bash
# Source your workspace (already done automatically by ~/.bashrc)
source /opt/ros/jazzy/setup.bash
source ~/sigyn_ws/install/setup.bash

cd ~/sigyn_ws/src/Sigyn/scripts/topic_analysis

# Full pipeline with defaults (15-second sampling window)
python3 run_all.py

# Then open the report
xdg-open ../../docs/artifacts/topic_analysis.md
```

---

## Running each step individually

### Step 1: gather.py

Subscribes to every topic currently available, measures Hz, bandwidth, and end-to-end delay (for topics with a `std_msgs/Header`), then writes `topic_data.json`.

```bash
python3 gather.py                          # defaults (15-second window)
python3 gather.py --max-wait 30           # longer window for slow topics
python3 gather.py --discovery-wait 3      # wait 3 s for topic discovery before subscribing
python3 gather.py --artifacts-dir /tmp/ta # write output to a different directory
```

**Outputs**: `<artifacts-dir>/topic_data.json`, updated `sample_durations.json`

### Step 2: describe.py

Reads `topic_data.json` and merges descriptions from `topic_descriptions.json`, writing `topic_data_described.json`.

```bash
python3 describe.py                        # defaults
python3 describe.py --descriptions /path/to/alt_descriptions.json
```

**Outputs**: `<artifacts-dir>/topic_data_described.json`

### Step 3: report.py

Reads `topic_data_described.json` and generates the Markdown report.

```bash
python3 report.py                          # full deep-dive descriptions (default)
python3 report.py --depth paragraph        # medium detail
python3 report.py --depth one_liner        # summary only
python3 report.py --output /tmp/my_report.md
```

**Outputs**: `<artifacts-dir>/topic_analysis.md`

---

## Editing topic descriptions

Topic descriptions are stored in `topic_descriptions.json`. Each entry has three levels:

| Level | Purpose | Typical length |
|-------|---------|----------------|
| `one_liner` | One sentence summary, shown in tables | 5–15 words |
| `paragraph` | A short paragraph suitable for a developer | 2–5 sentences |
| `deep_dive` | Full explanation for documentation / book | Multiple paragraphs |

To add or update a description, open `topic_descriptions.json` in your editor and find the topic by name. Then re-run `describe.py && report.py` (no re-sampling needed).

To check which topics are missing descriptions:

```bash
python3 describe.py 2>&1 | grep "Missing"
```

---

## Tuning sampling durations

`sample_durations.json` controls how long `gather.py` waits per topic. After each run, gather.py automatically updates `last_measured_hz` and may update `sample_duration_s` based on the measured rate.

Manual fields you might want to set:

| Field | Meaning |
|-------|---------|
| `expected_hz` | Expected publish rate — used for health checks in the report |
| `sample_duration_s` | How long to wait for this topic (overrides the automatic recommendation) |
| `notes` | Free text — never overwritten by automation |

---

## Expected runtime

| Step | Typical time |
|------|-------------|
| `gather.py` with `--max-wait 15` | ~18–20 s total (15 s sampling + discovery + write) |
| `describe.py` | < 1 s |
| `report.py` | < 1 s |

Slow topics (maps, latched costmaps) are the main reason to increase `--max-wait`.

---

## Output file reference

| File | Created by | Purpose |
|------|-----------|---------|
| `topic_data.json` | `gather.py` | Raw measurements: Hz, bandwidth, publishers, subscribers |
| `topic_data_described.json` | `describe.py` | Enriched with descriptions + namespace grouping |
| `topic_analysis.md` | `report.py` | Human-readable Markdown report |
| `sample_durations.json` | Seed + updated by `gather.py` | Per-topic sampling hints |
| `topic_descriptions.json` | Manually maintained | Three-level description database |

---

## Design principles

- **Parallel sampling** — all topics are subscribed simultaneously; the sampling window is a wall-clock timeout, not a per-topic sequential wait.
- **Adaptive duration hints** — `sample_durations.json` is updated after every run; over time the recommended durations converge on the minimum needed to get a reliable Hz estimate.
- **Separation of concerns** — gathering data, describing topics, and rendering the report are separate steps with JSON interfaces between them, so each step can be iterated independently.
- **Never overwrite descriptions** — `describe.py` only reads `topic_descriptions.json`; it never modifies it.
- **Three-level descriptions** — `one_liner` for tables, `paragraph` for quick reading, `deep_dive` for documentation. The `--depth` flag controls which levels appear in the report.
