#!/usr/bin/env bash
set -euo pipefail
docker exec gazebo-rgb-sorting-dev bash -lc 'source /opt/ros/jazzy/setup.bash; python3 -m unittest discover -s /project/tests -v'
