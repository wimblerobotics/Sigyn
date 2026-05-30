#!/usr/bin/env python3
"""
describe.py  –  Step 2 of the Sigyn topic-analysis pipeline.

Merges built-in topic descriptions from topic_descriptions.json into
topic_data.json and writes an enriched topic_data_described.json.

This step is intentionally separated from gather.py so you can:
  - Re-run the description merge without re-sampling live topics
  - Edit topic_descriptions.json and regenerate the report in seconds
  - Point the script at a different descriptions file for experimentation

Usage
-----
    python3 describe.py                             # defaults
    python3 describe.py --artifacts-dir /tmp/ta
    python3 describe.py --descriptions /path/to/my_descriptions.json

Inputs
------
    <artifacts-dir>/topic_data.json            – from gather.py
    <script-dir>/topic_descriptions.json       – built-in description database

Outputs
-------
    <artifacts-dir>/topic_data_described.json  – enriched dataset for report.py
"""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict

SCRIPT_DIR           = Path(__file__).parent.resolve()
REPO_ROOT            = SCRIPT_DIR.parent.parent
DEFAULT_ARTIFACTS    = REPO_ROOT / "docs" / "artifacts"
DEFAULT_DESCRIPTIONS = SCRIPT_DIR / "topic_descriptions.json"


def load_json(path: Path) -> Any:
    with open(path) as f:
        return json.load(f)


def save_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(data, f, indent=2, sort_keys=True)
    print(f"  Wrote  {path}")


def _namespace(topic: str) -> str:
    """Return the leading namespace component, e.g. '/global_costmap' or '/sigyn/sensors'."""
    parts = topic.strip("/").split("/")
    if len(parts) == 1:
        return "/"
    # For deep topics like /sigyn/sensors/range/…  group at /sigyn/sensors
    if len(parts) >= 3 and parts[0] in ("sigyn", "global_costmap", "local_costmap", "oakd_top", "oakd"):
        return "/" + "/".join(parts[:2])
    return "/" + parts[0]


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Merge topic descriptions into topic_data.json → topic_data_described.json"
    )
    ap.add_argument(
        "--artifacts-dir", type=Path, default=DEFAULT_ARTIFACTS,
        help=f"Directory containing topic_data.json (default: {DEFAULT_ARTIFACTS})",
    )
    ap.add_argument(
        "--descriptions", type=Path, default=DEFAULT_DESCRIPTIONS,
        help=f"Path to topic_descriptions.json (default: {DEFAULT_DESCRIPTIONS})",
    )
    args = ap.parse_args()

    data_path = args.artifacts_dir / "topic_data.json"
    if not data_path.exists():
        print(f"ERROR: {data_path} not found.  Run gather.py first.")
        raise SystemExit(1)

    if not args.descriptions.exists():
        print(f"ERROR: {args.descriptions} not found.")
        raise SystemExit(1)

    print(f"Loading {data_path} …")
    gathered: Dict[str, Any] = load_json(data_path)

    print(f"Loading {args.descriptions} …")
    desc_db: Dict[str, Any] = load_json(args.descriptions)

    topics: Dict[str, Any] = gathered.get("topics", {})
    n_described = 0
    n_missing   = 0
    missing_list = []

    for topic, entry in topics.items():
        desc_entry = desc_db.get(topic)
        if desc_entry and not desc_entry.get("_comment"):
            entry["description"] = {
                "one_liner":  desc_entry.get("one_liner",  "TODO"),
                "paragraph":  desc_entry.get("paragraph",  "TODO"),
                "deep_dive":  desc_entry.get("deep_dive",  "TODO"),
            }
            n_described += 1
        else:
            entry["description"] = {
                "one_liner":  "TODO – no description written yet",
                "paragraph":  "TODO",
                "deep_dive":  "TODO",
            }
            n_missing += 1
            missing_list.append(topic)

        # Attach namespace grouping hint for report.py
        entry["namespace"] = _namespace(topic)

    output = dict(gathered)
    output["described_at"] = datetime.now(timezone.utc).isoformat()
    output["topics"]       = topics

    out_path = args.artifacts_dir / "topic_data_described.json"
    save_json(out_path, output)

    print(f"\nDescribed {n_described} topics;  {n_missing} topics have no description yet.")
    if missing_list:
        print("  Missing descriptions for:")
        for t in sorted(missing_list):
            print(f"    {t}")


if __name__ == "__main__":
    main()
