#!/usr/bin/env bash
set -euo pipefail
SEED="${1:-11}"
[[ "$SEED" =~ ^[0-9]+$ ]] || { echo 'Seed must be an integer.' >&2;exit 2; }
echo 'Open http://localhost:6082/vnc.html?autoconnect=true&resize=scale'
docker exec -it gazebo-rgb-sorting-dev bash -lc "source /opt/ros/jazzy/setup.bash; python3 /project/src/run_integration.py --phase full --seed $SEED --gui --record"

docker exec gazebo-rgb-sorting-dev python3 /project/src/acceptance.py "/project/results/full-seed-$SEED"
