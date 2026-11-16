"""ROS MoveIt IK, collision-aware planning, and controller-backed execution."""
import time,json,os
from pathlib import Path
from rclpy.action import ActionClient
from moveit_msgs.srv import GetPositionIK,GetMotionPlan,ApplyPlanningScene,GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints,JointConstraint,CollisionObject
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from configuration import CONFIG
import xml.etree.ElementTree as E

def downward_pose(xyz):
    pose=Pose();pose.position.x,pose.position.y,pose.position.z=map(float,xyz)
    pose.orientation.x=1.;pose.orientation.y=0.;pose.orientation.z=0.;pose.orientation.w=0.
    return pose

class Motion:
    def __init__(self,node):
        self.node=node;self.joints=None
        node.create_subscription(JointState,'/joint_states',self._state,20)
        self.cartesian=node.create_client(GetCartesianPath,'/compute_cartesian_path');self.last_servo_duration=.45;self.ik=node.create_client(GetPositionIK,'/compute_ik');self.planner=node.create_client(GetMotionPlan,'/plan_kinematic_path');self.scene=node.create_client(ApplyPlanningScene,'/apply_planning_scene');self.executor=ActionClient(node,ExecuteTrajectory,'/execute_trajectory')
    def _state(self,m):self.joints=m
    def until(self,predicate,wall_timeout=30):
        end=time.monotonic()+wall_timeout
        import rclpy
        while rclpy.ok() and time.monotonic()<end:
            rclpy.spin_once(self.node,timeout_sec=.03)
            if predicate():return True
        return False
    def native_call(self,kind,client,request,timeout):
        entry={'kind':kind,'wall_start_ns':time.monotonic_ns(),'source':'ROS service round trip'}
        try:
            result=self.result(client.call_async(request),timeout);entry['response_received']=True
            if hasattr(result,'motion_plan_response'):entry['server_planning_s']=result.motion_plan_response.planning_time;entry['error_code']=result.motion_plan_response.error_code.val
            elif hasattr(result,'error_code'):entry['error_code']=result.error_code.val
            return result
        except Exception as error:entry['error']=str(error);raise
        finally:
            entry['wall_end_ns']=time.monotonic_ns()
            if os.environ.get('SORTING_RUN_DIR'):
                with (Path(os.environ['SORTING_RUN_DIR'])/'planner_calls.jsonl').open('a') as f:f.write(json.dumps(entry)+'\n')
    def result(self,future,timeout=30):
        if not self.until(future.done,timeout):raise TimeoutError('MoveIt operation timed out')
        return future.result()
    def ready(self):
        for service in [self.ik,self.planner,self.scene,self.cartesian]:
            if not service.wait_for_service(timeout_sec=45):raise RuntimeError('MoveIt service unavailable: '+service.srv_name)
        if not self.executor.wait_for_server(timeout_sec=30):raise RuntimeError('MoveIt execution unavailable')
        if not self.until(lambda:self.joints is not None,15):raise RuntimeError('joint states unavailable')
    def add_world(self,path):
        req=ApplyPlanningScene.Request(); req.scene.is_diff=True
        for model in E.parse(path).getroot().find('world').findall('model'):
            if model.findtext('static')!='true':continue
            # Sensor and robot have no primitive static collision here.
            pose=[float(x) for x in model.findtext('pose','0 0 0 0 0 0').split()]
            for link in model.findall('link'):
                for collision in link.findall('collision'):
                    geo=collision.find('geometry'); box=geo.find('box') if geo is not None else None
                    if box is None:continue
                    obj=CollisionObject();obj.header.frame_id='world';obj.id=model.get('name')+'/'+collision.get('name');obj.operation=CollisionObject.ADD
                    primitive=SolidPrimitive();primitive.type=SolidPrimitive.BOX;primitive.dimensions=[float(v) for v in box.findtext('size').split()]
                    p=Pose();p.position.x,p.position.y,p.position.z=pose[:3];p.orientation.w=1.;obj.primitives=[primitive];obj.primitive_poses=[p];req.scene.world.collision_objects.append(obj)
        if not self.result(self.scene.call_async(req)).success:raise RuntimeError('Planning scene update failed')
    def solve(self,xyz,seed=None):
        req=GetPositionIK.Request();ik=req.ik_request;ik.group_name='panda_arm';ik.ik_link_name='panda_link8';ik.robot_state.joint_state=self.joints if seed is None else seed;ik.avoid_collisions=True;ik.timeout.sec=1
        ik.pose_stamped.header.frame_id='world';ik.pose_stamped.pose=downward_pose(xyz)
        res=self.native_call('ik',self.ik,req,5)
        if res.error_code.val!=1:raise RuntimeError(f'IK failed {res.error_code.val} at {xyz}')
        return res.solution.joint_state
    def move(self,xyz,on_plan_start=lambda:None,on_plan_complete=lambda:None,on_move_start=lambda:None,seed=None):
        on_plan_start();joints=self.solve(xyz,seed=seed)
        return self.plan_joints(joints,on_plan_complete,on_move_start)
    def home(self):
        joints=JointState();joints.name=[f'panda_joint{i}' for i in range(1,8)];joints.position=list(map(float,CONFIG['robot']['home']))
        current=dict(zip(self.joints.name,self.joints.position))
        error=max(abs(current[name]-target) for name,target in zip(joints.name,joints.position))
        if error<=.002:return {'already_home':True,'measured_max_joint_error_rad':error}
        return self.plan_joints(joints)
    def plan_joints(self,joints,on_plan_complete=lambda:None,on_move_start=lambda:None):
        req=GetMotionPlan.Request();r=req.motion_plan_request;r.group_name='panda_arm';r.pipeline_id='ompl';r.planner_id='RRTConnectkConfigDefault';r.num_planning_attempts=3;r.allowed_planning_time=3.;r.max_velocity_scaling_factor=.6;r.max_acceleration_scaling_factor=.5;r.start_state.is_diff=True
        r.workspace_parameters.header.frame_id='world';r.workspace_parameters.min_corner.x=-3.;r.workspace_parameters.min_corner.y=-3.;r.workspace_parameters.min_corner.z=-1.;r.workspace_parameters.max_corner.x=3.;r.workspace_parameters.max_corner.y=3.;r.workspace_parameters.max_corner.z=4.
        c=Constraints();positions=dict(zip(joints.name,joints.position))
        for i in range(1,8):
            j=JointConstraint();j.joint_name=f'panda_joint{i}';j.position=positions[j.joint_name];j.tolerance_above=.005;j.tolerance_below=.005;j.weight=1.;c.joint_constraints.append(j)
        r.goal_constraints=[c]
        for planning_attempt in range(3):
            res=self.native_call('ompl',self.planner,req,15).motion_plan_response
            if res.error_code.val==1:break
        if res.error_code.val!=1:raise RuntimeError(f'Motion planning failed {res.error_code.val} after 3 bounded requests')
        on_plan_complete();baseline=dict(zip(self.joints.name,self.joints.position));goal=ExecuteTrajectory.Goal();goal.trajectory=res.trajectory
        handle=self.result(self.executor.send_goal_async(goal),5)
        if not handle.accepted:raise RuntimeError('MoveIt execution rejected')
        completion=handle.get_result_async();observed={'started':False}
        def done():
            current=dict(zip(self.joints.name,self.joints.position))
            if not observed['started'] and any(abs(current.get(k,v)-v)>1e-4 for k,v in baseline.items() if k.startswith('panda_joint')):
                observed['started']=True;on_move_start()
            return completion.done()
        if not self.until(done,45):raise TimeoutError('Trajectory completion timeout')
        result=completion.result()
        if not observed['started']:on_move_start() # zero-distance command boundary; not a measured travel duration
        if result.result.error_code.val!=1:raise RuntimeError(f'MoveIt execution failed {result.result.error_code.val}')
        return {'planning_time_reported':res.planning_time,'trajectory_duration_sim':res.trajectory.joint_trajectory.points[-1].time_from_start.sec+res.trajectory.joint_trajectory.points[-1].time_from_start.nanosec*1e-9}
    def servo_step(self,xyz,duration=.18,contact_check=None):
        """MoveIt Cartesian sweep, collision/jump checked at 5 mm increments."""
        from control_msgs.action import FollowJointTrajectory
        if not hasattr(self,'servo'):
            self.servo=ActionClient(self.node,FollowJointTrajectory,'/arm_controller/follow_joint_trajectory')
            if not self.servo.wait_for_server(timeout_sec=5):raise RuntimeError('Cartesian execution controller unavailable')
        request=GetCartesianPath.Request();request.header.frame_id='world';request.start_state.joint_state=self.joints;request.start_state.is_diff=True;request.group_name='panda_arm';request.link_name='panda_link8';request.max_step=.005;request.revolute_jump_threshold=.35;request.avoid_collisions=True;request.max_velocity_scaling_factor=.6;request.max_acceleration_scaling_factor=.5;request.cartesian_speed_limited_link='panda_link8';request.max_cartesian_speed=.18
        predictor=xyz if callable(xyz) else None
        target=list(predictor(self.last_servo_duration+.03) if predictor else xyz)
        diagnostics=[];converged=False
        for iteration in range(5):
            request.waypoints=[downward_pose(target)]
            request.start_state.joint_state=self.joints
            planned=self.native_call('cartesian',self.cartesian,request,8)
            if planned.error_code.val!=1 or planned.fraction<.999:raise RuntimeError(f'Cartesian approach rejected: code={planned.error_code.val}, fraction={planned.fraction}')
            trajectory=planned.solution.joint_trajectory
            if len(trajectory.points)<2:raise RuntimeError('Cartesian plan contains no motion')
            final=trajectory.points[-1].time_from_start;self.last_servo_duration=final.sec+final.nanosec*1e-9
            if self.last_servo_duration<=0:raise RuntimeError('Cartesian plan has no valid time parameterization')
            revised=list(predictor(self.last_servo_duration+.03) if predictor else target)
            error=sum((a-b)**2 for a,b in zip(revised,target))**.5
            diagnostics.append({'target':target,'trajectory_sim_s':self.last_servo_duration,'prediction_revision_m':error,'points':len(trajectory.points)})
            if error<.004:converged=True;break
            target=revised
        if os.environ.get('SORTING_RUN_DIR'):
            with (Path(os.environ['SORTING_RUN_DIR'])/'cartesian_plans.jsonl').open('a') as f:f.write(json.dumps(diagnostics)+'\n')
        if not converged:raise RuntimeError('Moving interception did not converge within five planning updates')
        self.last_servo_target=target
        goal=FollowJointTrajectory.Goal();goal.trajectory=trajectory;handle=self.result(self.servo.send_goal_async(goal),5)
        if not handle.accepted:raise RuntimeError('Cartesian execution rejected')
        execution_start=time.monotonic_ns();completion=handle.get_result_async();contact_stop={'requested':False}
        def finished():
            if contact_check is not None and contact_check() and not contact_stop['requested']:
                contact_stop['requested']=True;handle.cancel_goal_async()
            return completion.done()
        if not self.until(finished,30):
            handle.cancel_goal_async();raise TimeoutError('Cartesian execution timeout')
        result=completion.result();held=bool(contact_check()) if contact_check is not None else False
        execution={'wall_start_ns':execution_start,'wall_end_ns':time.monotonic_ns(),'status':result.status,'trajectory_error_code':result.result.error_code,'contact_stop_requested':contact_stop['requested'],'contact_still_held':held}
        if os.environ.get('SORTING_RUN_DIR'):
            with (Path(os.environ['SORTING_RUN_DIR'])/'cartesian_execution.jsonl').open('a') as f:f.write(json.dumps(execution)+'\n')
        if not (contact_stop['requested'] and held) and result.result.error_code!=0:raise RuntimeError(f'Cartesian execution failed {result.result.error_code}')
        return self.last_servo_duration
