#!/usr/bin/env python3
"""
gather.py  –  Step 1 of the Sigyn topic-analysis pipeline.

Discovers all live ROS 2 topics, subscribes to ALL of them in parallel,
measures hz / bandwidth / delay for --max-wait seconds (default 15 s), then
writes topic_data.json and updates sample_durations.json with learned rates.

Because every topic is sampled simultaneously the wall-clock time is just
--max-wait seconds regardless of how many topics exist.

Usage
-----
    python3 gather.py                          # defaults
    python3 gather.py --max-wait 30            # longer sampling window
    python3 gather.py --artifacts-dir /tmp/ta  # custom output directory
    python3 gather.py --discovery-wait 3       # wait longer for DDS graph

Outputs
-------
    <artifacts-dir>/topic_data.json           – gathered topic data
    <script-dir>/sample_durations.json        – updated per-topic timing hints
"""

from __future__ import annotations

import argparse
import json
import math
import os
import platform
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
import rclpy.serialization
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rosidl_runtime_py.utilities import get_message

# ── paths ──────────────────────────────────────────────────────────────────────
SCRIPT_DIR        = Path(__file__).parent.resolve()
REPO_ROOT         = SCRIPT_DIR.parent.parent               # …/Sigyn/
DEFAULT_ARTIFACTS = REPO_ROOT / "docs" / "artifacts"
DEFAULT_DURATIONS = SCRIPT_DIR / "sample_durations.json"
DEFAULT_MAX_WAIT  = 15.0
DEFAULT_DISC_WAIT = 2.0
GATHERER_NODE     = "topic_analysis_gatherer"

# ── small helpers ───────────────────────────────────────────────────────────────

def load_json(path: Path) -> Any:
    if path.exists():
        with open(path) as f:
            return json.load(f)
    return {}


def save_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(data, f, indent=2, sort_keys=True)
    print(f"  Wrote  {path}")


def _fmt_qos(qos) -> Dict[str, Any]:
    r = "RELIABLE"       if qos.reliability == ReliabilityPolicy.RELIABLE       else "BEST_EFFORT"
    d = "TRANSIENT_LOCAL" if qos.durability  == DurabilityPolicy.TRANSIENT_LOCAL else "VOLATILE"
    return {"reliability": r, "durability": d, "depth": qos.depth}


def _std(vals: List[float]) -> float:
    n = len(vals)
    if n < 2:
        return 0.0
    m = sum(vals) / n
    return math.sqrt(sum((x - m) ** 2 for x in vals) / (n - 1))

# ── per-topic accumulator ──────────────────────────────────────────────────────

class TopicAccum:
    """Thread-safe accumulator for one topic's measurement window."""

    MAX_DELAYS = 500   # cap stored delay samples to bound memory

    def __init__(self, topic: str, msg_type: str, sample_s: float) -> None:
        self.topic      = topic
        self.msg_type   = msg_type
        self.sample_s   = sample_s
        self._lock      = threading.Lock()

        # state
        self.count       = 0
        self.total_bytes = 0
        self.times: List[float]  = []   # monotonic recv times
        self.delays: List[float] = []   # header-stamp latency (seconds)
        self.has_header  = False
        self._hdr_done   = False

    # ── callback, called from executor thread ──────────────────────────────────
    def record(self, msg: Any) -> None:
        now_mono = time.monotonic()
        now_wall = time.time()

        with self._lock:
            # one-time header detection
            if not self._hdr_done:
                self._hdr_done = True
                try:
                    s = msg.header.stamp
                    self.has_header = hasattr(s, "sec") and hasattr(s, "nanosec")
                except AttributeError:
                    pass

            # latency
            if self.has_header and len(self.delays) < self.MAX_DELAYS:
                try:
                    s   = msg.header.stamp
                    t   = s.sec + s.nanosec * 1e-9
                    lat = now_wall - t
                    if -1.0 < lat < 30.0:          # sanity bound
                        self.delays.append(lat)
                except Exception:
                    pass

            # serialised size
            try:
                raw = rclpy.serialization.serialize_message(msg)
                self.total_bytes += len(raw)
            except Exception:
                pass

            self.count += 1
            self.times.append(now_mono)

    # ── result ─────────────────────────────────────────────────────────────────
    def summarise(self) -> Dict[str, Any]:
        with self._lock:
            if self.count == 0:
                return {
                    "never_published":    True,
                    "sample_count":       0,
                    "has_header":         False,
                    "supports_delay":     False,
                    "sample_duration_s":  self.sample_s,
                }

            avg_bytes = self.total_bytes / self.count

            # --- hz ---
            hz_block: Dict[str, Any] = {"sample_count": self.count}
            if len(self.times) >= 2:
                span = self.times[-1] - self.times[0]
                if span > 0:
                    intervals = [
                        self.times[i + 1] - self.times[i]
                        for i in range(len(self.times) - 1)
                    ]
                    mean_hz = (self.count - 1) / span
                    hz_block.update({
                        "mean":    round(mean_hz, 3),
                        "min":     round(1.0 / max(intervals), 3) if max(intervals) > 0 else None,
                        "max":     round(1.0 / min(intervals), 3) if min(intervals) > 0 else None,
                        "std_dev": round(_std([1.0 / i for i in intervals if i > 0]), 3),
                        "span_s":  round(span, 3),
                    })
                else:
                    hz_block.update({"mean": None, "min": None, "max": None, "std_dev": None})
            else:
                hz_block.update({"mean": None, "min": None, "max": None, "std_dev": None})

            # --- bandwidth ---
            span = (self.times[-1] - self.times[0]) if len(self.times) >= 2 else 1.0
            bps  = (self.total_bytes * 8.0) / span if span > 0 else 0.0

            # --- delay ---
            delay_block: Optional[Dict] = None
            if self.delays:
                delay_block = {
                    "mean_s":       round(sum(self.delays) / len(self.delays), 4),
                    "min_s":        round(min(self.delays), 4),
                    "max_s":        round(max(self.delays), 4),
                    "std_dev_s":    round(_std(self.delays), 4),
                    "sample_count": len(self.delays),
                }

            return {
                "never_published":    False,
                "hz":                 hz_block,
                "bandwidth_bps":      round(bps, 1),
                "bandwidth_kbps":     round(bps / 1000.0, 2),
                "avg_msg_size_bytes": round(avg_bytes, 1),
                "delay_s":            delay_block,
                "has_header":         self.has_header,
                "supports_delay":     bool(self.delays),
                "sample_duration_s":  self.sample_s,
                "sample_count":       self.count,
            }


# ── gather node ────────────────────────────────────────────────────────────────

class GatherNode(Node):
    """Creates one subscription per topic and feeds TopicAccum callbacks."""

    def __init__(
        self,
        topics_meta: Dict[str, Dict],
        durations: Dict[str, Any],
    ) -> None:
        super().__init__(GATHERER_NODE)
        self.accums: Dict[str, TopicAccum] = {}
        self._subs: list = []

        for topic, meta in topics_meta.items():
            tp = meta.get("type")
            if not tp:
                continue
            try:
                msg_cls = get_message(tp)
            except Exception as e:
                self.get_logger().warning(f"Cannot load type {tp!r} for {topic}: {e}")
                continue

            dur_info  = durations.get(topic, {})
            sample_s  = float(dur_info.get("sample_duration_s", 10.0))
            accum     = TopicAccum(topic, tp, sample_s)
            self.accums[topic] = accum

            # Match publisher durability so we receive latched (TRANSIENT_LOCAL) messages
            pubs       = meta.get("publishers", [])
            transient  = any(
                p.get("qos", {}).get("durability") == "TRANSIENT_LOCAL" for p in pubs
            )
            if transient:
                qos = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=5,
                )
            else:
                qos = qos_profile_sensor_data   # best-effort, volatile, depth 10

            def _cb(msg: Any, a: TopicAccum = accum) -> None:
                a.record(msg)

            try:
                self._subs.append(self.create_subscription(msg_cls, topic, _cb, qos))
            except Exception as e:
                self.get_logger().warning(f"Subscription failed for {topic}: {e}")


# ── endpoint discovery ─────────────────────────────────────────────────────────

def discover_endpoints(node: Node) -> Dict[str, Dict]:
    """
    Enumerate every topic visible in the DDS graph and collect:
      - message type
      - list of publishers  (node name, qos)
      - list of subscribers (node name, qos)

    The gather node itself is not yet running so its subscriptions will not
    appear in the subscriber lists.
    """
    result: Dict[str, Dict] = {}
    names_and_types = node.get_topic_names_and_types()

    for topic, types in names_and_types:
        tp = types[0] if types else None
        entry: Dict[str, Any] = {
            "type":        tp,
            "publishers":  [],
            "subscribers": [],
        }

        for pub in node.get_publishers_info_by_topic(topic):
            ns = pub.node_namespace.rstrip("/")
            entry["publishers"].append({
                "node": f"{ns}/{pub.node_name}",
                "qos":  _fmt_qos(pub.qos_profile),
            })

        for sub in node.get_subscriptions_info_by_topic(topic):
            ns = sub.node_namespace.rstrip("/")
            entry["subscribers"].append({
                "node": f"{ns}/{sub.node_name}",
                "qos":  _fmt_qos(sub.qos_profile),
            })

        result[topic] = entry

    return result


# ── duration table updater ─────────────────────────────────────────────────────

def _recommend_duration(measured_hz: Optional[float], current_s: float) -> float:
    """Return recommended sample duration based on observed rate."""
    if measured_hz is None or measured_hz <= 0:
        return current_s   # keep existing guess
    if measured_hz > 50:
        return 3.0
    if measured_hz > 10:
        return 5.0
    if measured_hz > 1:
        return 8.0
    return 15.0


def update_durations(
    old: Dict[str, Any],
    accums: Dict[str, TopicAccum],
) -> Dict[str, Any]:
    now_str = datetime.now(timezone.utc).isoformat()
    new = dict(old)

    for topic, accum in accums.items():
        summary  = accum.summarise()
        hz_block = summary.get("hz") or {}
        hz_mean  = hz_block.get("mean")

        entry = dict(new.get(topic, {}))
        old_s = float(entry.get("sample_duration_s", 10.0))

        entry["sample_duration_s"] = _recommend_duration(hz_mean, old_s)
        entry["last_measured_hz"]  = hz_mean
        entry["last_run"]          = now_str
        entry["has_header"]        = accum.has_header
        entry["supports_delay"]    = bool(accum.delays)
        new[topic] = entry

    return new


# ── main ───────────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(
        description="Gather ROS 2 topic statistics and write topic_data.json."
    )
    ap.add_argument(
        "--artifacts-dir", type=Path, default=DEFAULT_ARTIFACTS,
        help=f"Directory for output files (default: {DEFAULT_ARTIFACTS})",
    )
    ap.add_argument(
        "--durations", type=Path, default=DEFAULT_DURATIONS,
        help=f"Path to sample_durations.json (default: {DEFAULT_DURATIONS})",
    )
    ap.add_argument(
        "--max-wait", type=float, default=DEFAULT_MAX_WAIT,
        help=f"Seconds to sample all topics in parallel (default: {DEFAULT_MAX_WAIT})",
    )
    ap.add_argument(
        "--discovery-wait", type=float, default=DEFAULT_DISC_WAIT,
        help=f"Seconds to let the DDS graph stabilise (default: {DEFAULT_DISC_WAIT})",
    )
    args = ap.parse_args()

    rclpy.init()

    # Phase 1: graph discovery
    disc_node = rclpy.create_node("topic_analysis_discovery")
    disc_exec = SingleThreadedExecutor()
    disc_exec.add_node(disc_node)

    print(f"Waiting {args.discovery_wait}s for DDS graph to stabilise…")
    deadline = time.monotonic() + args.discovery_wait
    while time.monotonic() < deadline:
        disc_exec.spin_once(timeout_sec=0.1)

    print("Discovering topics and endpoints…")
    topics_meta = discover_endpoints(disc_node)
    disc_node.destroy_node()

    n_topics = len(topics_meta)
    print(f"Found {n_topics} topics.")

    # Phase 2: load existing duration hints
    durations = load_json(args.durations)

    # Phase 3: subscribe and sample (all in parallel)
    print(f"Subscribing to all topics; sampling for {args.max_wait}s in parallel…")
    gather_node = GatherNode(topics_meta, durations)
    n_threads   = max(4, min(16, n_topics // 8))
    executor    = MultiThreadedExecutor(num_threads=n_threads)
    executor.add_node(gather_node)

    sample_start = time.monotonic()
    deadline     = sample_start + args.max_wait
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)

    print(f"Sampling complete ({time.monotonic() - sample_start:.1f}s elapsed).")

    # Phase 4: assemble output
    topics_out: Dict[str, Any] = {}
    for topic, meta in topics_meta.items():
        entry = dict(meta)   # type, publishers, subscribers
        accum = gather_node.accums.get(topic)
        if accum:
            entry.update(accum.summarise())
        else:
            entry.update({
                "never_published":  True,
                "sample_count":     0,
                "note": "message type unavailable – could not subscribe",
            })
        topics_out[topic] = entry

    # Clean up node before writing
    gather_node.destroy_node()

    output = {
        "generated_at":  datetime.now(timezone.utc).isoformat(),
        "hostname":      platform.node(),
        "ros_distro":    os.environ.get("ROS_DISTRO", "unknown"),
        "max_wait_s":    args.max_wait,
        "topic_count":   n_topics,
        "topics":        topics_out,
    }

    out_path = args.artifacts_dir / "topic_data.json"
    save_json(out_path, output)

    # Phase 5: update duration hints from measured rates
    print("Updating sample_durations.json with measured rates…")
    new_durations = update_durations(durations, gather_node.accums)
    save_json(args.durations, new_durations)

    n_pub   = sum(1 for t in topics_out.values() if not t.get("never_published", True))
    n_never = sum(1 for t in topics_out.values() if t.get("never_published", False))
    print(
        f"\nDone.  {n_pub} topics published data,  {n_never} topics silent during window."
    )

    rclpy.shutdown()


if __name__ == "__main__":
    main()
