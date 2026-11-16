#!/usr/bin/env python3
import json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from motion import Motion
rclpy.init();n=Node('planner_integration_probe');m=Motion(n);report={'passed':False,'verified_placements':0};start=time.monotonic()
try:
    m.ready();m.add_world('/project/worlds/robot_test.sdf');report['move']=m.move([.15,0.,1.05]);report['passed']=True
except Exception as e:report['error']=str(e)
finally:
    report['wall_seconds']=time.monotonic()-start;p=Path('/project/results/planner');p.mkdir(parents=True,exist_ok=True);(p/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if report['passed'] else 1)
