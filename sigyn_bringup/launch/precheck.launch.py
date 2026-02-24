# filepath: Sigyn/base/launch/precheck.launch.py
# SPDX-License-Identifier: Apache-2.0
#
# Precheck launch: verify required device nodes exist before launching full stack.
#
# Usage examples:
#   ros2 launch base precheck.launch.py
#   ros2 launch base precheck.launch.py proceed_on_failure:=true
#
# Interactive prompt:
#   If devices are missing and you run from a real TTY, you will be asked whether to proceed.
#   Non‑interactive (e.g. systemd) use proceed_on_failure:=true or export SIGYN_PRECHECK_FORCE=1.

import os
import sys
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    EmitEvent
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.events import Shutdown

REQUIRED_DEVICES = [
    "/dev/lidar_top",
    "/dev/lidar_cup",
    "/dev/teensy_sensor",
    "/dev/teensy_sensor2",
]

def _have_tty():
    try:
        return sys.stdin.isatty()
    except Exception:
        return False

def _prompt_user():
    while True:
        resp = input("Proceed anyway? [y/N]: ").strip().lower()
        if resp in ("y", "yes"):
            return True
        if resp in ("", "n", "no"):
            return False
        print("Please answer y or n.")

def precheck_fn(context, *args, **kwargs):
    proceed_flag = LaunchConfiguration("proceed_on_failure").perform(context).lower() == "true"
    force_env = os.environ.get("SIGYN_PRECHECK_FORCE", "") in ("1", "true", "yes")

    missing = [p for p in REQUIRED_DEVICES if not os.path.exists(p)]

    if not missing:
        print("[precheck] All required device nodes present.")
        return _include_main_launch()

    # Report failures
    print("\n[precheck] Missing required device nodes:")
    for m in missing:
        print(f"  - {m}")
    print("")

    # Decide whether to continue
    proceed = False
    if proceed_flag or force_env:
        print("[precheck] proceed_on_failure flag or SIGYN_PRECHECK_FORCE set -> continuing despite failures.")
        proceed = True
    elif _have_tty():
        try:
            proceed = _prompt_user()
        except EOFError:
            proceed = False
    else:
        print("[precheck] Non-interactive session and failures detected. Not proceeding.")
        proceed = False

    if not proceed:
        reason = f"Aborting launch due to missing devices: {', '.join(missing)}"
        print(f"[precheck] {reason}")
        return [EmitEvent(event=Shutdown(reason=reason))]

    print("[precheck] Continuing launch with missing devices (functionality may be degraded).")
    return _include_main_launch()

def _include_main_launch():
    # Path to the main robot launch file
    main_launch = PathJoinSubstitution([
        FindPackageShare('sigyn_bringup'),
        'launch',
        'sigyn.launch.py'
    ])
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(main_launch),
            launch_arguments={
                # Add pass-through arguments here if needed
                # Example: 'use_sim_time': 'false'
            }.items()
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "proceed_on_failure",
            default_value="false",
            description="If true, continue even when required device nodes are missing."
        ),
        OpaqueFunction(function=precheck_fn)
    ])
