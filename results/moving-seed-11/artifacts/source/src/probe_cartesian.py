#!/usr/bin/env python3
"""Isolate FK, Cartesian path coverage, time parameterization, and IK branch."""
import copy,json,os
from pathlib import Path
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionFK,GetCartesianPath
from sensor_msgs.msg import JointState
from motion import Motion
rclpy.init();n=Node('cartesian_geometry_probe');m=Motion(n);report={}
try:
    m.ready();m.add_world('/project/worlds/robot_test.sdf');m.move([0.,0.,1.02])
    fk=n.create_client(GetPositionFK,'/compute_fk');fk.wait_for_service(timeout_sec=5)
    def forward(joints):
        req=GetPositionFK.Request();req.header.frame_id='world';req.fk_link_names=['panda_link8','panda_hand','suction_cup'];req.robot_state.joint_state=joints;res=m.result(fk.call_async(req));return res.pose_stamped
    def xyz(p):return [p.pose.position.x,p.pose.position.y,p.pose.position.z]
    def plan(state,target):
        req=GetCartesianPath.Request();req.header.frame_id='world';req.start_state.joint_state=state;req.start_state.is_diff=True;req.group_name='panda_arm';req.link_name='panda_link8';req.max_step=.005;req.revolute_jump_threshold=.35;req.avoid_collisions=True;req.max_velocity_scaling_factor=.6;req.max_acceleration_scaling_factor=.5;req.cartesian_speed_limited_link='panda_link8';req.max_cartesian_speed=.18;pose=copy.deepcopy(forward(state)[0].pose);pose.position.x,pose.position.y,pose.position.z=target;req.waypoints=[pose];result=m.result(m.cartesian.call_async(req));tr=result.solution.joint_trajectory;pts=tr.points
        row={'start_fk':[xyz(p) for p in forward(state)],'target':target,'fraction':result.fraction,'points':len(pts)}
        if pts:
            q=JointState();q.name=tr.joint_names;q.position=pts[-1].positions;row['last_fk']=[xyz(p) for p in forward(q)];row['duration_sim_s']=pts[-1].time_from_start.sec+pts[-1].time_from_start.nanosec*1e-9;row['start_q']=list(pts[0].positions);row['end_q']=list(pts[-1].positions)
        return row
    actual=copy.deepcopy(m.joints);report['actual_pregrasp_fk']=[xyz(p) for p in forward(actual)];report['actual_seed_descent']=plan(actual,[0.,0.,.98])
    low=m.solve([0.,0.,.916]);high=m.solve([0.,0.,1.02],seed=low);report['grasp_seeded_descent']=plan(high,[0.,0.,.916])
except Exception as e:report['error']=str(e)
finally:
    p=Path(os.environ['SORTING_RUN_DIR']);(p/'diagnostic.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));n.destroy_node();rclpy.shutdown()
