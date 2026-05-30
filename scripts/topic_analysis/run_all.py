#!/usr/bin/env python3
"""
run_all.py  –  Convenience wrapper for the full topic-analysis pipeline.

Runs all three steps in sequence:
  1.  gather.py   – subscribe to all live topics, measure Hz / bandwidth / delay
  2.  describe.py – merge topic_descriptions.json into the gathered data
  3.  report.py   – generate topic_analysis.md

Common arguments (--artifacts-dir, --depth) are forwarded to the relevant steps.
Step-specific arguments can be passed via --gather-args, --describe-args, --report-args.

Usage
-----
    python3 run_all.py                         # all defaults
    python3 run_all.py --max-wait 20           # longer sampling window
    python3 run_all.py --depth paragraph       # lighter report
    python3 run_all.py --skip-gather           # regenerate report from cached data

    # Forwarding extra arguments to individual steps:
    python3 run_all.py --gather-args "--discovery-wait 5"
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.resolve()


def run(step: str, cmd: list[str]) -> None:
    print(f"\n{'='*60}")
    print(f"  STEP: {step}")
    print(f"  CMD:  {' '.join(cmd)}")
    print(f"{'='*60}\n")
    result = subprocess.run(cmd)
    if result.returncode != 0:
        print(f"\nERROR: '{step}' exited with code {result.returncode}.  Aborting.", file=sys.stderr)
        sys.exit(result.returncode)


def split_extra(raw: str | None) -> list[str]:
    """Split a raw argument string into a list, honouring quoted strings."""
    if not raw:
        return []
    import shlex
    return shlex.split(raw)


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Run the full topic-analysis pipeline: gather → describe → report"
    )
    ap.add_argument(
        "--artifacts-dir", type=Path, default=None,
        help="Shared --artifacts-dir forwarded to all three steps",
    )
    ap.add_argument(
        "--descriptions", type=Path, default=None,
        help="Path to topic_descriptions.json (forwarded to describe.py)",
    )
    ap.add_argument(
        "--durations", type=Path, default=None,
        help="Path to sample_durations.json (forwarded to gather.py and report.py)",
    )
    ap.add_argument(
        "--max-wait", type=float, default=None,
        help="Maximum sample window in seconds (forwarded to gather.py)",
    )
    ap.add_argument(
        "--depth", choices=["one_liner", "paragraph", "deep_dive"], default=None,
        help="Description depth for report.py (default: deep_dive)",
    )
    ap.add_argument(
        "--output", type=Path, default=None,
        help="Output path for the Markdown report (forwarded to report.py)",
    )
    ap.add_argument(
        "--skip-gather", action="store_true",
        help="Skip gather.py and use the existing topic_data.json",
    )
    ap.add_argument(
        "--skip-describe", action="store_true",
        help="Skip describe.py and use the existing topic_data_described.json",
    )
    ap.add_argument(
        "--gather-args", type=str, default=None,
        help="Extra arguments for gather.py, as a quoted string",
    )
    ap.add_argument(
        "--describe-args", type=str, default=None,
        help="Extra arguments for describe.py, as a quoted string",
    )
    ap.add_argument(
        "--report-args", type=str, default=None,
        help="Extra arguments for report.py, as a quoted string",
    )
    args = ap.parse_args()

    python = sys.executable
    gather_script   = str(SCRIPT_DIR / "gather.py")
    describe_script = str(SCRIPT_DIR / "describe.py")
    report_script   = str(SCRIPT_DIR / "report.py")

    # Build shared flags
    shared_flags: list[str] = []
    if args.artifacts_dir:
        shared_flags += ["--artifacts-dir", str(args.artifacts_dir)]

    # ── Step 1: gather ─────────────────────────────────────────────────────────
    if not args.skip_gather:
        gather_cmd = [python, gather_script] + shared_flags
        if args.max_wait is not None:
            gather_cmd += ["--max-wait", str(args.max_wait)]
        if args.durations:
            gather_cmd += ["--durations", str(args.durations)]
        gather_cmd += split_extra(args.gather_args)
        run("gather.py", gather_cmd)
    else:
        print("\nSkipping gather step (--skip-gather).")

    # ── Step 2: describe ───────────────────────────────────────────────────────
    if not args.skip_describe:
        describe_cmd = [python, describe_script] + shared_flags
        if args.descriptions:
            describe_cmd += ["--descriptions", str(args.descriptions)]
        describe_cmd += split_extra(args.describe_args)
        run("describe.py", describe_cmd)
    else:
        print("\nSkipping describe step (--skip-describe).")

    # ── Step 3: report ─────────────────────────────────────────────────────────
    report_cmd = [python, report_script] + shared_flags
    if args.depth:
        report_cmd += ["--depth", args.depth]
    if args.output:
        report_cmd += ["--output", str(args.output)]
    if args.durations:
        report_cmd += ["--durations", str(args.durations)]
    report_cmd += split_extra(args.report_args)
    run("report.py", report_cmd)

    print("\n✓  All steps complete.")


if __name__ == "__main__":
    main()
