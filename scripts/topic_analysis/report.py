#!/usr/bin/env python3
"""
report.py  –  Step 3 of the Sigyn topic-analysis pipeline.

Reads topic_data_described.json and generates a rich Markdown report at
<output-dir>/topic_analysis.md.

The report has five sections:
  1.  Header – robot name, hostname, generation time, ROS distro
  2.  Top-20 Bandwidth – table of the 20 highest-bandwidth topics
  3.  Full Topic Summary – sortable table, all topics, one row each
  4.  Topic Details – grouped by namespace; per-topic three-level description,
      type, publishers, subscribers, hz, bandwidth, delay, health
  5.  Silent Topics – topics with publishers but no messages observed

Usage
-----
    python3 report.py                            # defaults
    python3 report.py --artifacts-dir /tmp/ta
    python3 report.py --output /tmp/ta/my_report.md
    python3 report.py --depth paragraph          # one_liner | paragraph | deep_dive (default)
"""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

SCRIPT_DIR        = Path(__file__).parent.resolve()
REPO_ROOT         = SCRIPT_DIR.parent.parent
DEFAULT_ARTIFACTS = REPO_ROOT / "docs" / "artifacts"
DEFAULT_OUTPUT    = DEFAULT_ARTIFACTS / "topic_analysis.md"
ROBOT_NAME        = "Sigyn"

# ── helpers ────────────────────────────────────────────────────────────────────

def load_json(path: Path) -> Any:
    with open(path) as f:
        return json.load(f)


def _fmt_bw(bps: Optional[float]) -> str:
    if bps is None:
        return "—"
    if bps >= 1_000_000:
        return f"{bps/1_000_000:.2f} MB/s"
    if bps >= 1_000:
        return f"{bps/1_000:.1f} kB/s"
    return f"{bps:.0f} B/s"


def _fmt_hz(hz_block: Optional[Dict]) -> str:
    """Full Hz string including min–max range, for the detail section."""
    if not hz_block:
        return "—"
    m = hz_block.get("mean")
    if m is None:
        return "—"
    mn = hz_block.get("min")
    mx = hz_block.get("max")
    if mn is not None and mx is not None:
        return f"{m:.1f} Hz ({mn:.1f}\u2013{mx:.1f})"
    return f"{m:.1f}\u00a0Hz"


def _fmt_hz_compact(hz_block: Optional[Dict]) -> str:
    """Compact Hz string (mean only, non-breaking space) for summary tables."""
    if not hz_block:
        return "—"
    m = hz_block.get("mean")
    if m is None:
        return "—"
    return f"{m:.1f}\u00a0Hz"


def _fmt_delay(delay: Optional[Dict]) -> str:
    if not delay:
        return "N/A"
    m = delay.get("mean_s")
    if m is None:
        return "N/A"
    mn = delay.get("min_s")
    mx = delay.get("max_s")
    if mn is not None and mx is not None:
        return f"{m*1000:.1f} ms ({mn*1000:.1f}–{mx*1000:.1f})"
    return f"{m*1000:.1f} ms"


def _fmt_type_short(tp: str) -> str:
    """Drop the 'msg/' layer: sensor_msgs/msg/LaserScan → sensor_msgs/LaserScan."""
    parts = tp.split("/")
    if len(parts) == 3 and parts[1] == "msg":
        return f"{parts[0]}/{parts[2]}"
    return tp


def _fmt_size(size: Optional[float]) -> str:
    if size is None:
        return "—"
    if size >= 1_000_000:
        return f"{size/1_000_000:.2f} MB"
    if size >= 1_000:
        return f"{size/1_000:.1f} kB"
    return f"{size:.0f} B"


def _dot(color: str) -> str:
    """HTML colored bullet — renders correctly in wkhtmltopdf (no emoji needed)."""
    return f'<span style="color:{color};font-size:9pt">&#9679;</span>'


def _health(entry: Dict, dur_hints: Dict) -> str:
    """Return an HTML-colored health indicator — compatible with wkhtmltopdf."""
    topic = entry.get("_topic", "")
    never = entry.get("never_published", False)
    if never:
        pubs = entry.get("publishers", [])
        if pubs:
            return f"{_dot('orange')} silent"
        return "(no publisher)"

    expected = dur_hints.get(topic, {}).get("expected_hz")
    measured = (entry.get("hz") or {}).get("mean")
    if expected and measured and expected > 0:
        ratio = measured / expected
        if ratio < 0.5:
            return f"{_dot('red')} low ({measured:.1f} Hz, exp {expected:.1f})"
        if ratio < 0.8:
            return f"{_dot('#bb8800')} low ({measured:.1f} Hz)"
    return f"{_dot('green')} OK"


def _node_lines(nodes: List[Dict]) -> str:
    """Format nodes with QoS details — used in the detail section."""
    if not nodes:
        return '<span style="color:orange">NONE</span>'
    lines = []
    for n in nodes:
        name = n.get("node", "?")
        qos  = n.get("qos", {})
        rel  = qos.get("reliability", "?")
        dur  = qos.get("durability", "?")
        lines.append(f"`{name}` {rel}/{dur}")
    return "<br>".join(lines)


def _node_names_only(nodes: List[Dict]) -> str:
    """Format node names only (no QoS) for compact summary table cells."""
    if not nodes:
        return "NONE"
    return "<br>".join(f"`{n.get('node', '?')}`" for n in nodes)


def _slug(topic: str) -> str:
    """Convert topic to a Markdown anchor slug."""
    return topic.replace("/", "-").strip("-")


def _namespace_title(ns: str) -> str:
    mapping = {
        "/":                  "Root Topics",
        "/global_costmap":    "Global Costmap",
        "/local_costmap":     "Local Costmap",
        "/sigyn/sensors":     "Sigyn Sensors",
        "/sigyn/power":       "Power",
        "/sigyn/safety":      "Safety System",
        "/sigyn":             "Sigyn (Other)",
        "/oakd_top":          "OAK-D Top Camera",
        "/oakd":              "OAK-D Detection",
        "/gripper":           "Gripper / Elevator",
        "/cmd_vel":           "Velocity Commands",
        "/amcl":              "AMCL Localisation",
        "/bt_navigator":      "BT Navigator",
        "/controller_server": "Controller Server",
        "/planner_server":    "Planner Server",
        "/smoother_server":   "Smoother Server",
        "/behavior_server":   "Behavior Server",
        "/waypoint_follower": "Waypoint Follower",
        "/velocity_smoother": "Velocity Smoother",
        "/teensy_bridge":     "Teensy Bridge",
        "/map_server":        "Map Server",
    }
    return mapping.get(ns, ns.lstrip("/").replace("/", " / ").title() or "Root Topics")


# ── main report builder ────────────────────────────────────────────────────────

def build_report(data: Dict, dur_hints: Dict, depth: str) -> str:
    topics: Dict[str, Any] = data.get("topics", {})
    generated_at = data.get("described_at") or data.get("generated_at", "unknown")
    hostname     = data.get("hostname", "unknown")
    ros_distro   = data.get("ros_distro", "unknown")
    max_wait     = data.get("max_wait_s", "?")

    # Annotate each entry with its topic name for helpers
    for t, e in topics.items():
        e["_topic"] = t

    # Pre-sort by bandwidth descending
    def bw_sort(kv: Tuple[str, Any]) -> float:
        return kv[1].get("bandwidth_bps") or 0.0

    sorted_topics = sorted(topics.items(), key=bw_sort, reverse=True)

    # Group by namespace
    ns_map: Dict[str, List[Tuple[str, Any]]] = {}
    for topic, entry in sorted(topics.items()):
        ns = entry.get("namespace", "/")
        ns_map.setdefault(ns, []).append((topic, entry))

    lines: List[str] = []

    # ── CSS for PDF export (Markdown PDF Plus / Puppeteer) ────────────────────
    # @page sets landscape orientation; the table rules keep content legible.
    lines += [
        "<style>",
        "@page {",
        "  size: A4 landscape;",
        "  margin: 1.0cm 0.3cm 1.0cm 0.3cm;",
        "}",
        "/* Override pandoc standalone template's narrow body max-width */",
        "body { font-size: 9pt; max-width: none !important; padding: 0 !important; margin: 0 !important; }",
        "#content, .content, main, article, div { max-width: none !important; }",
        "table {",
        "  font-size: 7pt;",
        "  border-collapse: collapse;",
        "  width: 100%;",
        "  table-layout: fixed;",  # honour explicit column widths
        "}",
        "th {",
        "  white-space: nowrap;",  # never wrap header text
        "  padding: 2px 4px;",
        "  vertical-align: bottom;",
        "  overflow: hidden;",
        "}",
        "td {",
        "  padding: 2px 4px;",
        "  vertical-align: top;",
        "  overflow-wrap: break-word;",  # older webkit-compatible form
        "  word-wrap: break-word;",      # alias for wkhtmltopdf compatibility
        "  word-break: break-all;",      # allows breaks anywhere for long paths
        "}",
        "/* 8-column summary table — fixed widths summing to 100% */",
        "/* col 1=Topic, 2=BW, 3=Hz, 4=AvgMsg, 5=Type, 6=Pub, 7=Sub, 8=Health */",
        "table th:nth-child(1), table td:nth-child(1) { width: 22%; }",
        "table th:nth-child(2), table td:nth-child(2) { width:  8%; }",
        "table th:nth-child(3), table td:nth-child(3) { width:  7%; }",
        "table th:nth-child(4), table td:nth-child(4) { width:  6%; }",
        "table th:nth-child(5), table td:nth-child(5) { width: 15%; }",
        "table th:nth-child(6), table td:nth-child(6) { width: 19%; }",
        "table th:nth-child(7), table td:nth-child(7) { width: 17%; }",
        "table th:nth-child(8), table td:nth-child(8) { width:  6%; }",
        "code { font-size: 6.5pt; }",
        "/* Repeat table header row on every printed page */",
        "thead { display: table-header-group; }",
        "tbody { display: table-row-group; }",
        "</style>",
        "",
    ]

    # ── Section 1: Header ──────────────────────────────────────────────────────
    lines += [
        f"# {ROBOT_NAME} — ROS 2 Topic Analysis",
        "",
        f"> **Robot:** {ROBOT_NAME}  ",
        f"> **Host:** `{hostname}`  ",
        f"> **ROS Distro:** {ros_distro}  ",
        f"> **Generated:** {generated_at}  ",
        f"> **Sampling window:** {max_wait} s (all topics measured in parallel)  ",
        "> **Source data:** `topic_data_described.json`  ",
        "> **Update descriptions:** edit `topic_descriptions.json`, re-run `describe.py`, re-run `report.py`",
        "",
        "---",
        "",
    ]

    # ── Table of Contents ──────────────────────────────────────────────────────
    lines += [
        "## Table of Contents",
        "",
        "1. [Top 20 Topics by Bandwidth](#top-20-topics-by-bandwidth)",
        "2. [Full Topic Summary Table](#full-topic-summary-table)",
        "3. [Topic Details by Namespace](#topic-details-by-namespace)",
    ]
    for i, ns in enumerate(sorted(ns_map.keys()), start=1):
        title = _namespace_title(ns)
        anchor = title.lower().replace(" ", "-").replace("/", "").replace("(", "").replace(")", "")
        lines.append(f"   - [{title}](#{anchor})")
    lines += [
        "4. [Silent Topics (Publisher Exists, No Messages Observed)](#silent-topics)",
        "",
        "---",
        "",
    ]

    # ── Section 2: Top 20 Bandwidth ───────────────────────────────────────────
    lines += [
        "## Top 20 Topics by Bandwidth",
        "",
        "Topics ordered by measured bandwidth (highest first). Silent topics excluded.",
        "",
        "| # | Topic | Bandwidth | Hz | Avg Msg Size | Type |",
        "|---|-------|-----------|-----|--------------|------|",
    ]
    rank = 0
    for topic, entry in sorted_topics:
        if entry.get("never_published", False):
            continue
        bps  = entry.get("bandwidth_bps")
        if bps is None or bps == 0:
            continue
        rank += 1
        if rank > 20:
            break
        tp   = entry.get("type", "?")
        hz   = _fmt_hz(entry.get("hz"))
        bw   = _fmt_bw(bps)
        sz   = _fmt_size(entry.get("avg_msg_size_bytes"))
        anchor = _slug(topic)
        lines.append(f"| {rank} | [`{topic}`](#{anchor}) | **{bw}** | {hz} | {sz} | `{tp}` |")
    lines += ["", "---", ""]

    # ── Section 3: Full Topic Summary Table ───────────────────────────────────
    lines += [
        "## Full Topic Summary Table",
        "",
        "All topics, sorted by topic name. "
        '<span style="color:green">&#9679;</span> OK &nbsp;'
        '<span style="color:#bb8800">&#9679;</span> slightly low &nbsp;'
        '<span style="color:red">&#9679;</span> low rate &nbsp;'
        '<span style="color:orange">&#9679;</span> silent (has publisher) &nbsp;'
        '(no publisher)',
        "",
        "| Topic | Bandwidth | Hz | Avg\u00a0Msg | Type | Publishers | Subscribers | Status |",
        "|-------|----------:|---:|--------:|------|-----------|------------|--------|",
    ]
    for topic, entry in sorted(topics.items()):
        bps   = entry.get("bandwidth_bps")
        hz    = _fmt_hz_compact(entry.get("hz"))
        bw    = _fmt_bw(bps)
        sz    = _fmt_size(entry.get("avg_msg_size_bytes"))
        tp    = _fmt_type_short(entry.get("type", "?"))
        pubs  = _node_names_only(entry.get("publishers", []))
        subs  = _node_names_only(entry.get("subscribers", []))
        hlth  = _health(entry, dur_hints)
        anchor = _slug(topic)
        lines.append(
            f"| [`{topic}`](#{anchor}) | {bw} | {hz} | {sz} | `{tp}` | {pubs} | {subs} | {hlth} |"
        )
    lines += ["", "---", ""]

    # ── Section 4: Topic Details by Namespace ─────────────────────────────────
    lines += ["## Topic Details by Namespace", ""]

    for ns in sorted(ns_map.keys()):
        title = _namespace_title(ns)
        lines += [f"### {title}", ""]

        for topic, entry in sorted(ns_map[ns]):
            anchor = _slug(topic)
            tp     = entry.get("type", "unknown")
            desc   = entry.get("description", {})
            one_l  = desc.get("one_liner", "TODO")
            para   = desc.get("paragraph", "TODO")
            deep   = desc.get("deep_dive", "TODO")
            bps    = entry.get("bandwidth_bps")
            hz     = _fmt_hz(entry.get("hz"))
            bw     = _fmt_bw(bps)
            delay  = _fmt_delay(entry.get("delay_s"))
            sz     = _fmt_size(entry.get("avg_msg_size_bytes"))
            hlth   = _health(entry, dur_hints)
            pubs   = entry.get("publishers", [])
            subs   = entry.get("subscribers", [])
            never  = entry.get("never_published", False)

            lines += [
                f"#### `{topic}` {{#{anchor}}}",
                "",
                f"**{one_l}**",
                "",
            ]

            # Description block
            if depth in ("paragraph", "deep_dive") and para and para != "TODO":
                lines += [para, ""]

            if depth == "deep_dive" and deep and deep != "TODO":
                for paragraph_text in deep.split("\n\n"):
                    lines += [paragraph_text.strip(), ""]

            # Metrics table
            lines += [
                "| Property | Value |",
                "|----------|-------|",
                f"| **Type** | `{tp}` |",
                f"| **Bandwidth** | {bw} |",
                f"| **Rate** | {hz} |",
                f"| **Avg message size** | {sz} |",
                f"| **Delay (end-to-end)** | {delay} |",
                f"| **Health** | {hlth} |",
                f"| **Sample count** | {entry.get('sample_count', '—')} |",
                "",
            ]

            # Publishers
            lines += ["**Publishers:**", ""]
            if pubs:
                for p in pubs:
                    name = p.get("node", "?")
                    qos  = p.get("qos", {})
                    rel  = qos.get("reliability", "?")
                    dur  = qos.get("durability", "?")
                    dep  = qos.get("depth", "?")
                    lines.append(f"- `{name}` — {rel} / {dur} (depth {dep})")
            else:
                lines.append('<span style="color:orange">**NONE**</span>')
            lines.append("")

            # Subscribers
            lines += ["**Subscribers:**", ""]
            if subs:
                for s in subs:
                    name = s.get("node", "?")
                    qos  = s.get("qos", {})
                    rel  = qos.get("reliability", "?")
                    dur  = qos.get("durability", "?")
                    dep  = qos.get("depth", "?")
                    lines.append(f"- `{name}` — {rel} / {dur} (depth {dep})")
            else:
                lines.append('<span style="color:orange">**NONE**</span>')
            lines += ["", "---", ""]

    # ── Section 5: Silent Topics ───────────────────────────────────────────────
    silent = [
        (topic, entry)
        for topic, entry in sorted(topics.items())
        if entry.get("never_published", False) and entry.get("publishers")
    ]
    no_pub = [
        (topic, entry)
        for topic, entry in sorted(topics.items())
        if not entry.get("publishers")
    ]

    lines += [
        "## Silent Topics",
        "",
        "These topics have **at least one registered publisher** but no message was observed "
        f"during the {max_wait}-second sampling window. They may be event-driven (low-rate), "
        "awaiting a trigger (e.g., a navigation goal), or indicate a misconfigured node.",
        "",
        "| Topic | Type | Publishers | Subscribers |",
        "|-------|------|-----------|------------|",
    ]
    for topic, entry in silent:
        tp   = entry.get("type", "?")
        pubs = _node_lines(entry.get("publishers", []))
        subs = _node_lines(entry.get("subscribers", []))
        lines.append(f"| `{topic}` | `{tp}` | {pubs} | {subs} |")

    lines += [
        "",
        "### Topics with No Publishers",
        "",
        "These topics have **subscribers but no publisher** — possible misconfiguration or a node that is not running.",
        "",
        "| Topic | Type | Subscribers |",
        "|-------|------|------------|",
    ]
    for topic, entry in no_pub:
        tp   = entry.get("type", "?")
        subs = _node_lines(entry.get("subscribers", []))
        lines.append(f"| `{topic}` | `{tp}` | {subs} |")

    lines += [
        "",
        "---",
        "",
        f"*Report generated by `report.py` — part of the {ROBOT_NAME} topic-analysis pipeline.*",
        f"*Edit `topic_descriptions.json` to improve descriptions, then re-run `describe.py && report.py`.*",
        "",
    ]

    return "\n".join(lines)


# ── entry point ────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(
        description="Generate topic_analysis.md from topic_data_described.json"
    )
    ap.add_argument(
        "--artifacts-dir", type=Path, default=DEFAULT_ARTIFACTS,
        help=f"Directory with topic_data_described.json (default: {DEFAULT_ARTIFACTS})",
    )
    ap.add_argument(
        "--output", type=Path, default=None,
        help="Output Markdown path (default: <artifacts-dir>/topic_analysis.md)",
    )
    ap.add_argument(
        "--durations", type=Path, default=SCRIPT_DIR / "sample_durations.json",
        help="Path to sample_durations.json for health check expected-rate lookup",
    )
    ap.add_argument(
        "--depth", choices=["one_liner", "paragraph", "deep_dive"], default="deep_dive",
        help="Description depth to include in topic details (default: deep_dive)",
    )
    args = ap.parse_args()

    in_path = args.artifacts_dir / "topic_data_described.json"
    if not in_path.exists():
        print(f"ERROR: {in_path} not found.  Run gather.py then describe.py first.")
        raise SystemExit(1)

    out_path = args.output or (args.artifacts_dir / "topic_analysis.md")

    print(f"Loading {in_path} …")
    data = load_json(in_path)

    dur_hints: Dict = {}
    if args.durations.exists():
        dur_hints = load_json(args.durations)

    print("Building report …")
    md = build_report(data, dur_hints, args.depth)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        f.write(md)
    print(f"  Wrote  {out_path}")
    print(f"\nReport lines: {md.count(chr(10))}")


if __name__ == "__main__":
    main()
