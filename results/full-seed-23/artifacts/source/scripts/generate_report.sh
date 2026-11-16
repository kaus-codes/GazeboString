#!/usr/bin/env bash
set -euo pipefail
SEED="${1:-11}"
[[ "$SEED" =~ ^[0-9]+$ ]] || exit 2
docker exec gazebo-rgb-sorting-dev python3 /project/src/report.py "/project/results/full-seed-$SEED"
