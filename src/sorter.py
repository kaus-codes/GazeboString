#!/usr/bin/env python3
"""Autonomous RGB-selected moving-conveyor controller. All successes are audited."""
import argparse,json,time,traceback
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from motion import Motion
from physical_verifier import PhysicalVerifier
from feeder import Feeder
from metrics import Recorder
from configuration import CONFIG
from sequence import SequenceValidator,VerifiedPlacement

class Sorter(Node):
    def __init__(self,out,seed,goal):
        super().__init__('rgb_sorter');self.out=out;self.seed=seed;self.goal=goal;self.latest={'tracks':[],'sim_stamp':0};self.motion=Motion(self);self.audit=PhysicalVerifier();self.feeder=Feeder(seed);self.audit.spawned=self.feeder.records;self.validator=SequenceValidator();self.failed=0;self.incorrect=0;self.incorrect_placements=0;self.finished_wall=None;self.finished_sim=None;self.attempts=0;self.blocked={};self.state='INITIALIZE';self.first_wall=time.monotonic_ns();self.first_sim=None;self.selected=None;self.latest_operation_wall_s=None
        self.rec=Recorder(out,{'kind':'actual moving Gazebo experiment','seed':seed,'goal_placements':goal,'conveyor_speed_m_s':CONFIG['belt']['speed_m_s'],'ros':'jazzy','rgb_topic':'/sorting/rgb'})
        self.create_subscription(String,'/sorting/tracks',self.tracks,10);self.pub=self.create_publisher(String,'/sorting/state',10);self.create_timer(.25,self.publish_status)
    def tracks(self,m):
        self.latest=json.loads(m.data);self.audit.evaluate_frame(self.latest)
    def status(self,state):
        self.state=state;self.publish_status();print(state,'target',self.validator.target,'placements',len(self.validator.placements),flush=True)
    def publish_status(self):
        s={'state':self.state,'current_target':self.validator.target,'selected':self.selected,'predicted_grasp_position':getattr(self.motion,'last_servo_target',None),'attempted_picks':self.attempts,'failed_attempts':self.failed,'incorrect_picks':self.incorrect,'incorrect_placements':self.incorrect_placements,'wall_elapsed':((self.finished_wall or time.monotonic_ns())-self.first_wall)/1e9,'sim_time':self.audit.sim,'sequence':self.validator.summary(),'latest_perception_ms':self.latest.get('processing_ms'),'latest_operation_wall_s':self.latest_operation_wall_s,'sim_elapsed':((self.finished_sim if self.finished_sim is not None else self.audit.sim)-self.first_sim) if self.first_sim is not None else 0,'real_time_factor':(((self.finished_sim if self.finished_sim is not None else self.audit.sim)-self.first_sim)/max(((self.finished_wall or time.monotonic_ns())-self.first_wall)/1e9,.001)) if self.first_sim is not None else 0}
        text=json.dumps(s,indent=2);temporary=self.out/'state.json.tmp';temporary.write_text(text);temporary.replace(self.out/'state.json');self.pub.publish(String(data=text))
    def event(self,oid,name,**kwargs):return self.rec.record(oid,name,self.audit.sim,**kwargs)
    def candidates(self):
        now=self.audit.sim
        if self.first_sim is not None and now-self.first_sim<.8:return []
        return [t for t in self.latest['tracks'] if t['color']==self.validator.target and t['eligible'] and t['hits']>=8 and now-t['last_seen']<.3 and -.40<t['xyz'][0]<.10 and .004<t['velocity'][0]<.08 and self.blocked.get(t['id'],0)<now]
    def forecast(self,t,lead):
        dt=max(0,self.audit.sim-t['last_seen'])+lead
        return [t['xyz'][i]+t['velocity'][i]*dt for i in range(3)]
    def current_track(self,selected):
        return next((t for t in self.latest['tracks'] if t['id']==selected['id']),selected)
    def attempt(self,selected):
        oid=f'attempt_{self.attempts:03d}_track_{selected["id"]}';self.selected=selected;self.motion.last_servo_target=None;self.status('TRACK');selection_event=self.event(oid,'target_selected',track=selected)
        x,y,_=self.forecast(selected,1.6);target_y=selected['xyz'][1];self.status('PLAN_PICK');self.event(oid,'planning_start')
        grasp_seed=self.motion.solve([x,y,CONFIG['grasp']['minimum_flange_z']])
        self.motion.move([x,y,CONFIG['grasp']['pregrasp_flange_z']],lambda:None,lambda:self.event(oid,'planning_complete'),lambda:self.event(oid,'movement_start',source='joint_state_change'),seed=grasp_seed)
        self.status('APPROACH');self.audit.enable(True);self.event(oid,'grasp_start');z=CONFIG['grasp']['minimum_flange_z'];approach_start=self.audit.sim;last=selected;command=[x,y,z]
        while not self.audit.held and self.audit.sim-approach_start<CONFIG['grasp']['max_approach_sim_s']:
            last=self.current_track(last)
            # Brief camera occlusion by the approaching tool permits bounded coasting.
            if self.audit.sim-last['last_seen']>CONFIG['grasp']['maximum_coasting_sim_s']:raise RuntimeError('Selected track stale during final approach')
            z=CONFIG['grasp']['minimum_flange_z']
            def intercept(lead):
                nonlocal last
                last=self.current_track(last)
                if self.audit.sim-last['last_seen']>CONFIG['grasp']['maximum_coasting_sim_s']:raise RuntimeError('Selected track stale during interception planning')
                px,_,_=self.forecast(last,lead)
                if not (-.48<px<.49 and abs(target_y)<.155):raise RuntimeError('Predicted target left reachable grasp zone')
                return [px,target_y,z]
            self.motion.servo_step(intercept,CONFIG['grasp']['segment_sim_s'],contact_check=lambda:bool(self.audit.held));command=self.motion.last_servo_target
            if z<=CONFIG['grasp']['minimum_flange_z'] and not self.audit.held:break
        self.status('VERIFY_GRASP')
        if not self.motion.until(lambda:bool(self.audit.held),1.5):raise RuntimeError('Missed contact grasp')
        held=self.audit.held;self.event(oid,'grasp_confirmed',entity=held)
        # Evaluation is consulted only after a physical grasp has occurred.
        true_color=self.audit.manifest.get(held,{}).get('color')
        predicted=self.forecast(last,0);actual=self.audit.pose(held)
        if true_color is None:raise RuntimeError(f'Audit metadata unavailable for attached entity {held}')
        if true_color!=selected['color']:
            self.incorrect+=1;raise RuntimeError(f'Incorrect-color grasp: selected {selected["color"]}, attached {held} evaluated {true_color}')
        if actual is None or sum((a-b)**2 for a,b in zip(actual[1][:2],predicted[:2]))**.5>.045:raise RuntimeError(f'Grasp association rejected: entity={held}, actual={actual}, camera_prediction={predicted}')
        self.status('LIFT');self.motion.move([command[0],command[1],CONFIG['grasp']['lift_flange_z']])
        if not self.motion.until(lambda:self.audit.pose(held) is not None and self.audit.pose(held)[1][2]>.83 and self.audit.held==held,5):raise RuntimeError('Lift not verified')
        self.event(oid,'lift_confirmed',entity=held);self.status('TRANSPORT')
        color=selected['color'];index=sum(p.evaluated_color==color for p in self.validator.placements);bx,by=CONFIG['bins'][color];dx=[-.07,0,.07][index%3];dy=[-.06,0,.06][index//3]
        self.motion.move([bx+dx,by+dy,CONFIG['grasp']['placement_flange_z']]);self.status('PLACE');self.audit.enable(False);self.event(oid,'release',entity=held)
        stable={'start':None,'pos':None}
        def settled():
            row=self.audit.pose(held)
            if row is None or not self.audit.in_bin(held,color):stable['start']=None;return False
            if stable['start'] is None:stable.update(start=self.audit.sim,pos=row[1]);return False
            if sum((a-b)**2 for a,b in zip(stable['pos'],row[1]))**.5>.003:stable.update(start=self.audit.sim,pos=row[1]);return False
            return self.audit.sim-stable['start']>.6
        self.status('VERIFY_PLACEMENT')
        if not self.motion.until(settled,15):
            self.incorrect_placements+=1
            raise RuntimeError('Correct-bin settled release not verified')
        e=self.event(oid,'placement_confirmed',entity=held,evaluated_color=true_color,bin_color=color,eligible_entry=self.audit.entries.get(held),camera_first_detection_sim=selected['first_seen'],camera_first_detection_wall_ns=selected.get('first_seen_wall_ns'),camera_color_confirmed_sim=selected.get('color_confirmed_sim'),camera_color_confirmed_wall_ns=selected.get('color_confirmed_wall_ns'))
        self.latest_operation_wall_s=(e['wall_monotonic_ns']-selection_event['wall_monotonic_ns'])/1e9
        self.validator.accept(VerifiedPlacement(held,selected['color'],true_color,color,held in self.audit.contacts,True,self.audit.held!=held,True,self.audit.sim,e['wall_monotonic_ns']))
        if len(self.validator.placements)==self.goal:self.finished_wall=e['wall_monotonic_ns'];self.finished_sim=self.audit.sim
        self.status('RETURN_READY');self.motion.home();self.selected=None
    def run(self):
        report={'passed':False,'error':None}
        try:
            self.motion.ready()
            self.motion.add_world('/project/worlds/robot_test.sdf')
            self.motion.home()
            if not self.motion.until(lambda:self.latest.get('frames',0)>=12,45):
                raise RuntimeError('RGB camera readiness timeout')
            self.first_sim=self.audit.sim;self.first_wall=time.monotonic_ns();(self.out/'experiment_start.json').write_text(json.dumps({'wall_monotonic_ns':self.first_wall,'sim_time':self.first_sim}));self.feeder.start();self.status('WAIT_FOR_TARGET')
            while len(self.validator.placements)<self.goal:
                if self.attempts>=CONFIG['experiment']['max_attempts'] or self.audit.sim-self.first_sim>CONFIG['experiment']['max_sim_s'] or (time.monotonic_ns()-self.first_wall)/1e9>CONFIG['experiment']['max_wall_s']:raise TimeoutError('Bounded experiment limit reached')
                wait_start=self.audit.sim
                if not self.motion.until(lambda:bool(self.candidates()) or self.audit.sim-wait_start>CONFIG['experiment']['required_color_timeout_sim_s'],CONFIG['experiment']['required_color_timeout_wall_s']) or not self.candidates():raise TimeoutError('Required camera color unavailable within bounded simulation/wall limits')
                selected=min(self.candidates(),key=lambda t:abs(t['xyz'][0]));self.attempts+=1
                try:self.attempt(selected)
                except Exception as e:
                    failed_state=self.state;self.failed+=1;self.event(f'attempt_{self.attempts:03d}_track_{selected["id"]}','attempt_failed',state=failed_state,error=str(e));self.blocked[selected['id']]=self.audit.sim+10;self.status('RECOVERY');self.audit.enable(False)
                    with (self.out/'failures.jsonl').open('a') as f:f.write(json.dumps({'attempt':self.attempts,'sim':self.audit.sim,'wall_monotonic_ns':time.monotonic_ns(),'state':failed_state,'error':str(e),'traceback':traceback.format_exc()})+'\n')
                    if self.incorrect or self.incorrect_placements or failed_state=='RETURN_READY':raise
                    self.motion.home()
                    if self.failed>=CONFIG['experiment']['max_failed_attempts']:raise RuntimeError('Retry limit exceeded')
                self.status('WAIT_FOR_TARGET')
            final_audit={p.object_id:self.audit.bin_evidence(p.object_id,p.bin_color) for p in self.validator.placements};(self.out/'final_bin_audit.json').write_text(json.dumps(final_audit,indent=2))
            if not all(row['valid'] for row in final_audit.values()):raise RuntimeError('Final bin audit rejected previously placed object')
            inventory=self.audit.bin_inventory({p.object_id for p in self.validator.placements});(self.out/'bin_inventory.json').write_text(json.dumps(inventory,indent=2))
            if inventory['violations']:raise RuntimeError('Final bin inventory includes incorrect or unverified objects')
            self.status('COMPLETE' if self.goal==14 else 'MOVING_GATE_PASSED');report['passed']=not self.incorrect and (self.validator.summary()['complete'] if self.goal==14 else len(self.validator.placements)==self.goal)
        except KeyboardInterrupt:report['error']='Interrupted';self.status('INTERRUPTED')
        except Exception as e:report['error']=str(e);self.status('FAILED')
        finally:
            end_wall=time.monotonic_ns();end_sim=self.audit.sim
            if report['passed'] and self.validator.placements:
                end_wall=self.validator.placements[-1].wall_monotonic_ns;end_sim=self.validator.placements[-1].sim_time
            self.feeder.stop() if self.feeder.thread.is_alive() else None;self.audit.enable(False);self.rec.export()
            report.update(experiment_start_wall_ns=self.first_wall,experiment_end_wall_ns=end_wall,experiment_start_sim=self.first_sim,experiment_end_sim=end_sim,seed=self.seed,verified_placements=len(self.validator.placements),attempts=self.attempts,failed_attempts=self.failed,incorrect_picks=self.incorrect,incorrect_placements=self.incorrect_placements,sequence=self.validator.summary(),wall_seconds=(end_wall-self.first_wall)/1e9,sim_seconds=end_sim-self.first_sim if self.first_sim is not None else None,seven_pair_elapsed_time_wall_s=(end_wall-self.first_wall)/1e9 if report['passed'] and self.validator.summary()['complete'] else None)
            (self.out/'summary.json').write_text(json.dumps(report,indent=2));(self.out/'spawn-evaluation-only.json').write_text(json.dumps(self.feeder.records,indent=2));(self.out/'entries-evaluation-only.json').write_text(json.dumps(self.audit.entries,indent=2));(self.out/'camera-evaluation-only.json').write_text(json.dumps(self.audit.evaluations,indent=2));(self.out/'conveyor_odometry.json').write_text(json.dumps(self.audit.odometry,indent=2));(self.out/'raw_suction_contacts.json').write_text(json.dumps(self.audit.raw_contacts,indent=2));(self.out/'final_physics_snapshot.json').write_text(json.dumps(self.audit.poses,indent=2));print(json.dumps(report,indent=2),flush=True)
        return 0 if report['passed'] else 1

def main():
    p=argparse.ArgumentParser();p.add_argument('--seed',type=int,default=11);p.add_argument('--goal',type=int,default=2);p.add_argument('--output',default='/project/results/moving-gate');a=p.parse_args();out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
    rclpy.init();n=Sorter(out,a.seed,a.goal)
    try:return n.run()
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':raise SystemExit(main())
