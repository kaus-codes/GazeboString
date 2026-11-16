#!/usr/bin/env python3
"""Strict offline acceptance of actual Gazebo evidence. Partial runs fail."""
import argparse,json,math
from pathlib import Path
from collections import defaultdict
from report import create

def main(directory):
    p=Path(directory);aggregate=create(p);summary=json.loads((p/'summary.json').read_text());checks={}
    frames=[json.loads(x) for x in (p/'camera_frames.jsonl').open()] if (p/'camera_frames.jsonl').exists() else []
    tracks=[t for f in frames for t in f['tracks']];colors={t['color'] for t in tracks}
    checks['actual_camera_frames']=len(frames)>=30
    checks['red_pixels_detected']='RED' in colors;checks['blue_pixels_detected']='BLUE' in colors
    checks['all_distractors_seen']={'GREEN','YELLOW','ORANGE'}<=colors
    checks['distractors_never_eligible']=all(not t['eligible'] for t in tracks if t['color'] in ['GREEN','YELLOW','ORANGE'])
    manifest_path=p/'artifacts/robot_test.evaluation-only.json'
    shapes={r['shape'] for r in json.loads(manifest_path.read_text())['objects']} if manifest_path.exists() else set()
    checks['three_physical_shapes']=len(shapes)>=3
    checks['calibrated_localization']=aggregate['localization_error_m']['n']>=30 and aggregate['localization_error_m']['mean']<.025
    seen=defaultdict(list)
    for f in frames:
        for t in f['tracks']:seen[t['id']].append((f['sim_stamp'],t['xyz'][0]))
    checks['camera_tracks_actual_motion']=any(len(rows)>5 and max(x for t,x in rows)-min(x for t,x in rows)>.03 for rows in seen.values())
    placements=summary['sequence']['placements']
    checks['exact_alternation']=[r['evaluated_color'] for r in placements]==['RED','BLUE']*7
    checks['fourteen_verified_releases']=len(placements)==14 and all(all(r[k] for k in ['grasp_contact','lift_confirmed','released','contained_and_settled']) and r['bin_color']==r['evaluated_color'] for r in placements)
    checks['unique_objects']=len({r['object_id'] for r in placements})==len(placements)
    checks['no_incorrect_grasps']=summary['incorrect_picks']==0
    checks['complete_sequence_and_final_bin_audit']=summary['passed'] and summary['sequence']['complete']
    events=json.loads((p/'events.json').read_text())
    checks['measured_joint_motion']=sum(e['event']=='movement_start' and e['details'].get('source')=='joint_state_change' for e in events)>=14
    checks['valid_timing_order']=not aggregate['validation_errors'] and all(math.isfinite(e['sim_time']) and e['sim_time']>=0 and e['wall_monotonic_ns']>0 for e in events)
    checks['actual_fourteen_operation_timings']=aggregate['metrics'].get('pick_and_place_wall_s',{}).get('n')==14
    checks['csv_and_json_exports']=all((p/name).exists() for name in ['events.csv','events.json','full_events.csv','full_events.json','frame_processing.csv','operations.csv','operations.json','aggregate.json','metadata.json'])
    checks['timing_plot']=(p/'timing_report.png').exists()
    audit=json.loads((p/'final_bin_audit.json').read_text()) if (p/'final_bin_audit.json').exists() else {}
    checks['fourteen_fresh_final_bin_snapshots']=len(audit)==14 and all(r['valid'] and r['checks']['fresh'] for r in audit.values())
    odometry=json.loads((p/'conveyor_odometry.json').read_text()) if (p/'conveyor_odometry.json').exists() else []
    movement=[e['sim_time'] for e in events if e['event']=='movement_start'];end=[e['sim_time'] for e in events if e['event']=='placement_confirmed']
    normal=[r for r in odometry if movement and end and min(movement)<=r['sim_time']<=max(end)]
    checks['conveyor_continues_during_picks']=len(normal)>20 and all(r['speed_m_s']>.025 for r in normal)
    contacts=json.loads((p/'raw_suction_contacts.json').read_text()) if (p/'raw_suction_contacts.json').exists() else []
    checks['raw_suction_contact_stream']=len(contacts)>=14
    shutdown_path=p/'runtime/shutdown.json'
    checks['owned_processes_reaped']=shutdown_path.exists() and json.loads(shutdown_path.read_text())['direct_children_reaped']
    passed=all(checks.values());result={'passed':passed,'checks':checks,'scope':'Actual standard-mode Gazebo run; GUI and clutter are assessed separately.'};(p/'acceptance.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));return 0 if passed else 1
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('directory');a=ap.parse_args();raise SystemExit(main(a.directory))
