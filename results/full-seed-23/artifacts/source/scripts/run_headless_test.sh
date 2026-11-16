#!/usr/bin/env bash
set -euo pipefail
PHASE="${1:-full}"
SEED="${2:-11}"
case "$PHASE" in motion|planning|static|moving|full) ;; *) echo 'Phase: motion, planning, static, moving, or full' >&2;exit 2;; esac
[[ "$SEED" =~ ^[0-9]+$ ]] || exit 2
docker exec gazebo-rgb-sorting-dev bash -lc "source /opt/ros/jazzy/setup.bash; python3 /project/src/run_integration.py --phase $PHASE --seed $SEED"
