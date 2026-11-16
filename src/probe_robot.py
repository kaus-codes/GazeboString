#!/usr/bin/env python3
"""Verify controller availability AND measured joint movement. No grasp claims."""
import json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class Probe(Node):
    def __init__(self):
        super().__init__('robot_integration_probe'); self.states={}; self.samples=[]
        self.create_subscription(JointState,'/joint_states',self.on_state,10)
        self.action=ActionClient(self,FollowJointTrajectory,'/arm_controller/follow_joint_trajectory')
    def on_state(self,m):
        self.states.update(zip(m.name,m.position)); self.samples.append({'wall_monotonic':time.monotonic(),'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'positions':dict(zip(m.name,m.position))})
    def wait(self,f,seconds):
        end=time.monotonic()+seconds
        while rclpy.ok() and time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.1)
            if f(): return True
        return False

def main():
    rclpy.init(); node=Probe(); out=Path('/project/results/robot'); out.mkdir(parents=True,exist_ok=True)
    names=[f'panda_joint{i}' for i in range(1,8)]; report={'passed':False,'verified_placements':0}
    try:
        if not node.action.wait_for_server(timeout_sec=45): raise RuntimeError('trajectory action unavailable')
        if not node.wait(lambda:all(k in node.states for k in names),20): raise RuntimeError('joint states unavailable')
        initial=[node.states[k] for k in names]; target=initial.copy(); target[0]+=.12
        goal=FollowJointTrajectory.Goal(); goal.trajectory.joint_names=names
        p=JointTrajectoryPoint(); p.positions=target; p.time_from_start.sec=2; goal.trajectory.points=[p]
        start=time.monotonic(); future=node.action.send_goal_async(goal)
        if not node.wait(future.done,10): raise TimeoutError('goal response timeout')
        handle=future.result()
        if not handle.accepted: raise RuntimeError('trajectory rejected')
        result=handle.get_result_async()
        if not node.wait(result.done,30):
            handle.cancel_goal_async(); raise TimeoutError('trajectory result timeout')
        final=[node.states[k] for k in names]; error=max(abs(a-b) for a,b in zip(final,target)); motion=abs(final[0]-initial[0])
        report.update(passed=result.result().result.error_code==0 and error<.025 and motion>.08,initial=initial,target=target,final=final,max_joint_error_rad=error,measured_joint1_motion_rad=motion,wall_seconds=time.monotonic()-start,action_error_code=result.result().result.error_code)
    except Exception as e: report['error']=str(e)
    finally:
        (out/'summary.json').write_text(json.dumps(report,indent=2)); (out/'joint_states.json').write_text(json.dumps(node.samples)); print(json.dumps(report,indent=2)); node.destroy_node(); rclpy.shutdown()
    return 0 if report['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
