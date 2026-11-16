#!/usr/bin/env bash
set -euo pipefail
docker exec gazebo-rgb-sorting-dev python3 -m unittest discover -s /project/tests -v
