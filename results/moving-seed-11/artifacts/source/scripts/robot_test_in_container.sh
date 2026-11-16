#!/usr/bin/env bash
set -euo pipefail
set +u
source /opt/ros/jazzy/setup.bash
set -u
export DISPLAY=:1
export GZ_SIM_SYSTEM_PLUGIN_PATH="/project/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
python3 /project/src/build_robot.py
python3 /project/src/generate_world.py --output /project/worlds/robot_test.sdf
python3 - <<'PY'
import xml.etree.ElementTree as E
p='/project/worlds/robot_test.sdf'; r=E.parse(p);w=r.getroot().find('world')
E.SubElement(w,'plugin',filename='gz-sim-contact-system',name='gz::sim::systems::Contact')
E.SubElement(w,'plugin',filename='/project/build/librgb_contact_suction.so',name='ContactSuction')
r.write(p)
PY
python3 - <<'PY' > /project/config/robot_description.yaml
from pathlib import Path
import yaml
print(yaml.safe_dump({'robot_state_publisher':{'ros__parameters':{'robot_description':Path('/project/config/panda.urdf').read_text(),'use_sim_time':True}}}))
PY
pids=()
cleanup(){ for pid in "${pids[@]}"; do kill "$pid" 2>/dev/null || true; done; wait || true; }
trap cleanup EXIT INT TERM
gz sim -s -r -v 3 /project/worlds/robot_test.sdf > /project/results/preflight/robot-world.log 2>&1 & pids+=($!)
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file /project/config/robot_description.yaml > /project/results/preflight/rsp.log 2>&1 & pids+=($!)
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/project/config/bridge.yaml > /project/results/preflight/bridge.log 2>&1 & pids+=($!)
python3 /project/src/spawn_robot.py
ros2 run controller_manager spawner joint_state_broadcaster arm_controller --controller-manager-timeout 45 > /project/results/preflight/controllers.log 2>&1
if [[ "${1:-motion}" == "planning" ]]; then
  ros2 launch /project/launch/planning.launch.py > /project/results/preflight/moveit.log 2>&1 & pids+=($!)
  python3 /project/src/probe_planner.py
else
  python3 /project/src/probe_robot.py
fi
