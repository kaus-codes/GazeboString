#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
exec python3 /project/src/run_integration.py --phase "${1:-motion}"
