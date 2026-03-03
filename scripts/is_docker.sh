#!/bin/bash
# Quick Docker environment check - returns 0 if in Docker, 1 if not
# This is a lightweight version of check_docker.sh for use in scripts

# Quick check using multiple methods (same as in runSigynAmd.sh)
if [ -f /.dockerenv ] || \
   grep -q docker /proc/1/cgroup 2>/dev/null || \
   [ -n "$DOCKER_CONTAINER" ] || \
   [ -n "$container" ]; then
    # In Docker - return 0 (success)
    exit 0
else
    # Not in Docker - return 1 (failure)
    exit 1
fi
