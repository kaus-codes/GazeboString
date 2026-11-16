#!/usr/bin/env python3
"""One stationary-camera-selected physical grasp/release gate. Never a full run."""
import json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from motion import Motion
from physical_verifier import PhysicalVerifier
from metrics import Recorder
from configuration import CONFIG
from sequence import SequenceValidator,VerifiedPlacement

rclpy.init();n=Node('static_physical_pick_probe');motion=Motion(n);verifier=PhysicalVerifier();latest={'tracks':[]}
def tracks(m):latest.update(json.loads(m.data))
n.create_subscription(String,'/sorting/tracks',tracks,10)
out=Path('/project/results/static-pick');out.mkdir(parents=True,exist_ok=True);rec=Recorder(out,{'kind':'static integration test','conveyor_speed':0,'sequence_goal':14});report={'passed':False,'verified_placements':0};validator=SequenceValidator();start=time.monotonic();target=None
try:
    motion.ready();motion.add_world('/project/worlds/robot_test.sdf')
    if not motion.until(lambda:any(t['color']=='RED' and t['eligible'] for t in latest['tracks']),20):raise RuntimeError('No camera-confirmed RED target')
    target=next(t for t in latest['tracks'] if t['color']=='RED' and t['eligible']);x,y,_=target['xyz'];oid=str(target['id']);rec.record(oid,'target_selected',verifier.sim,track=target)
    motion.move([x,y,CONFIG['grasp']['pregrasp_flange_z']],lambda:rec.record(oid,'planning_start',verifier.sim),lambda:rec.record(oid,'planning_complete',verifier.sim),lambda:rec.record(oid,'movement_start',verifier.sim))
    verifier.enable(True);motion.move([x,y,CONFIG['grasp']['minimum_flange_z']])
    if not motion.until(lambda:bool(verifier.held),8):raise RuntimeError('No contact-gated grasp attachment')
    held=verifier.held;rec.record(oid,'grasp_confirmed',verifier.sim,attached_entity=held)
    if verifier.manifest[held]['color']!='RED':raise RuntimeError('Independent evaluator found incorrect-color grasp')
    observed=verifier.pose(held)
    if observed is None or ((observed[1][0]-x)**2+(observed[1][1]-y)**2)**.5>.035:raise RuntimeError('Grasped entity does not match selected camera target')
    motion.move([x,y,CONFIG['grasp']['lift_flange_z']])
    if not motion.until(lambda:verifier.pose(held) is not None and verifier.pose(held)[1][2]>.83 and verifier.held==held,5):raise RuntimeError('Lift not physically verified')
    rec.record(oid,'lift_confirmed',verifier.sim)
    motion.move([-.35,.68,CONFIG['grasp']['placement_flange_z']]);verifier.enable(False);rec.record(oid,'release',verifier.sim)
    stable={'start':None,'position':None}
    def settled():
        row=verifier.pose(held)
        if not verifier.in_bin(held,'RED') or row is None:stable['start']=None;return False
        if stable['start'] is None:stable.update(start=verifier.sim,position=row[1]);return False
        delta=sum((a-b)**2 for a,b in zip(row[1],stable['position']))**.5
        if delta>.003:stable.update(start=verifier.sim,position=row[1]);return False
        return verifier.sim-stable['start']>.6
    if not motion.until(settled,15):raise RuntimeError('Released object not verified settled inside RED bin')
    event=rec.record(oid,'placement_confirmed',verifier.sim,entity=held)
    validator.accept(VerifiedPlacement(held,'RED',verifier.manifest[held]['color'],'RED',held in verifier.contacts,True,verifier.held!=held,True,verifier.sim,event['wall_monotonic_ns']))
    retention_start=verifier.sim
    if not motion.until(lambda:verifier.sim-retention_start>=8,30):raise RuntimeError('Retention observation timed out')
    report['retention_evidence']=verifier.bin_evidence(held,'RED')
    if not report['retention_evidence']['valid']:raise RuntimeError('Eight-second physical retention check failed')
    report.update(passed=True,verified_placements=1,entity=held,sequence=validator.summary())
except Exception as e:report['error']=str(e)
finally:
    verifier.enable(False);report['wall_seconds']=time.monotonic()-start;report['seven_pair_elapsed_time']=None;rec.export();(out/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if report['passed'] else 1)
