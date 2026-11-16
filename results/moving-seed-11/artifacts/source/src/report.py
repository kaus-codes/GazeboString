#!/usr/bin/env python3
"""Generate reports exclusively from actual recorded events. Missing stays null."""
import argparse,csv,json,statistics
from pathlib import Path
from collections import defaultdict
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def stats(values):
    values=[v for v in values if v is not None]
    return {'n':len(values),'mean':statistics.mean(values),'median':statistics.median(values),'min':min(values),'max':max(values),'std_population':statistics.pstdev(values)} if values else {'n':0,'mean':None,'median':None,'min':None,'max':None,'std_population':None}
def difference(a,b,clock):
    if a is None or b is None:return None
    return (b[clock]-a[clock])/(1e9 if clock=='wall_monotonic_ns' else 1)
def create(directory):
    p=Path(directory);summary=json.loads((p/'summary.json').read_text());events=json.loads((p/'events.json').read_text());by=defaultdict(dict)
    for e in events:by[e['object_id']][e['event']]=e
    operations=[];validation=[]
    planner_calls=[json.loads(line) for line in (p/'planner_calls.jsonl').open()] if (p/'planner_calls.jsonl').exists() else []
    phases={'planning':('planning_start','planning_complete'),'reach_and_grasp':('movement_start','grasp_confirmed'),'grasp_and_lift':('grasp_confirmed','lift_confirmed'),'transport':('lift_confirmed','release'),'placement':('release','placement_confirmed'),'pick_and_place':('target_selected','placement_confirmed')}
    for attempt,es in by.items():
        if 'placement_confirmed' not in es:continue
        e=es['placement_confirmed'];d=e['details'];row={'attempt':attempt,'object_id':d['entity'],'color':d['evaluated_color']}
        for phase,(a,b) in phases.items():
            row[phase+'_wall_s']=difference(es.get(a),es.get(b),'wall_monotonic_ns');row[phase+'_sim_s']=difference(es.get(a),es.get(b),'sim_time')
        selected=es['target_selected']
        calls=[c for c in planner_calls if selected['wall_monotonic_ns']<=c['wall_start_ns'] and c['wall_end_ns']<=e['wall_monotonic_ns']]
        row['motion_planning_wall_s']=sum((c['wall_end_ns']-c['wall_start_ns'])/1e9 for c in calls) if calls else None
        row['native_planning_requests']=len(calls)
        row['ompl_server_planning_s']=sum(c.get('server_planning_s',0) for c in calls) if calls else None
        track=selected['details']['track'];first=track.get('first_seen_wall_ns');confirm=track.get('color_confirmed_wall_ns')
        row['color_recognition_wall_s']=(confirm-first)/1e9 if confirm is not None and first is not None else None
        row['detection_to_selection_wall_s']=(selected['wall_monotonic_ns']-first)/1e9 if first is not None else None
        entry=d.get('eligible_entry');raw=(first-entry['wall_monotonic_ns'])/1e9 if first is not None and entry else None
        row['detection_latency_wall_s_raw']=raw
        row['detection_latency_status']='left_censored' if entry and entry.get('left_censored',True) else ('boundary_alignment_uncertain' if raw is not None and raw<0 else 'observed')
        row['detection_latency_wall_s']=raw if row['detection_latency_status']=='observed' else None
        times=[es[n]['wall_monotonic_ns'] for n in ['target_selected','planning_start','planning_complete','movement_start','grasp_confirmed','lift_confirmed','release','placement_confirmed'] if n in es]
        if len(times)!=8 or times!=sorted(times):validation.append('Invalid event ordering: '+attempt)
        operations.append(row)
    frame_values=[];frame_samples=[]
    if (p/'camera_frames.jsonl').exists():
        for line in (p/'camera_frames.jsonl').open():
            try:
                fr=json.loads(line);frame_values.append(fr['processing_ms']);frame_samples.append({'frame':fr['frames'],'sim_stamp':fr['sim_stamp'],'wall_monotonic_ns':fr['wall_monotonic_ns'],'processing_ms':fr['processing_ms'],'detections':len(fr['tracks'])})
            except (ValueError,KeyError):validation.append('Malformed camera frame record')
    if frame_samples:
        with (p/'frame_processing.csv').open('w') as f:
            writer=csv.DictWriter(f,fieldnames=list(frame_samples[0]));writer.writeheader();writer.writerows(frame_samples)
    pairs=[]
    placements=[e for e in events if e['event']=='placement_confirmed'];start=summary.get('experiment_start_wall_ns') or (events[0]['wall_monotonic_ns'] if events else None)
    for i in range(0,len(placements)-1,2):
        end=placements[i+1]['wall_monotonic_ns'];previous=placements[i-1]['wall_monotonic_ns'] if i else start
        pairs.append({'pair':i//2+1,'wall_s':(end-previous)/1e9,'cumulative_wall_s':(end-start)/1e9})
    evaluation=json.loads((p/'camera-evaluation-only.json').read_text()) if (p/'camera-evaluation-only.json').exists() else []
    aggregate={'source':str(p),'verified_full_run':summary.get('passed',False) and summary['sequence']['complete'] and not validation,'sequence':summary['sequence'],'attempts':summary['attempts'],'verified_placements':summary['verified_placements'],'failed_attempts':summary['failed_attempts'],'incorrect_picks':summary['incorrect_picks'],'wall_seconds':summary['wall_seconds'],'sim_seconds':summary.get('sim_seconds'),'real_time_factor_measured':summary['sim_seconds']/summary['wall_seconds'] if summary.get('sim_seconds') is not None and summary['wall_seconds']>0 else None,'seven_pair_elapsed_time_wall_s':summary.get('seven_pair_elapsed_time_wall_s'),'placement_success_rate_per_attempt':summary['verified_placements']/summary['attempts'] if summary['attempts'] else None,'grasp_attempts':sum('grasp_start' in es for es in by.values()),'grasp_success_rate':sum('grasp_confirmed' in es for es in by.values())/sum('grasp_start' in es for es in by.values()) if any('grasp_start' in es for es in by.values()) else None,'confirmed_grasps':sum('grasp_confirmed' in es for es in by.values()),'confirmed_grasps_per_pick_attempt':sum('grasp_confirmed' in es for es in by.values())/summary['attempts'] if summary['attempts'] else None,'throughput_per_wall_minute':summary['verified_placements']*60/summary['wall_seconds'] if summary['wall_seconds'] else None,'processing_latency_ms':stats(frame_values),'matched_camera_color_accuracy':sum(x['true_color']==x['predicted_color'] for x in evaluation)/len(evaluation) if evaluation else None,'matched_camera_samples':len(evaluation),'localization_error_m':stats([x['error_m'] for x in evaluation]),'metrics':{k:stats([r[k] for r in operations]) for k in operations[0] if k.endswith('_s')} if operations else {},'pairs':pairs,'validation_errors':validation,'notes':['Wall times use host monotonic timestamps inside the same container; simulation times use Gazebo /clock.','Startup before controller readiness is excluded from experiment elapsed time.','Initial objects already visible when evaluation starts are left-censored for detection latency.','Negative raw entry-to-detection values are retained and excluded from latency aggregates as boundary alignment uncertainty.','Gripper is a contact-gated fixed-joint suction approximation, not fluid suction.']}
    # Preserve raw controller logs and export a unified view of captured boundaries.
    full_events=[]
    entries=json.loads((p/'entries-evaluation-only.json').read_text()) if (p/'entries-evaluation-only.json').exists() else {}
    for attempt,es in by.items():
        placement=es.get('placement_confirmed');entity=placement['details']['entity'] if placement else attempt
        for event in es.values():full_events.append({**event,'attempt_id':attempt,'object_id':entity,'source':'controller callback','censored':False})
        if 'target_selected' in es:
            track=es['target_selected']['details']['track']
            for name,sim_key,wall_key in [('first_detection','first_seen','first_seen_wall_ns'),('color_confirmed','color_confirmed_sim','color_confirmed_wall_ns')]:
                if track.get(wall_key) is not None:full_events.append({'object_id':entity,'attempt_id':attempt,'event':name,'wall_monotonic_ns':track[wall_key],'sim_time':track[sim_key],'source':'actual camera tracker callback','censored':False,'details':{'track_id':track['id']}})
        if placement and entity in entries:
            entry=entries[entity];censored=entry.get('left_censored',True)
            full_events.append({'object_id':entity,'attempt_id':attempt,'event':'eligible_entry','wall_monotonic_ns':None if censored else entry['wall_monotonic_ns'],'sim_time':None if censored else entry['sim_time'],'source':'Gazebo physics observer','censored':censored,'details':entry})
    full_events.sort(key=lambda e:e['wall_monotonic_ns'] or 0)
    (p/'full_events.json').write_text(json.dumps(full_events,indent=2))
    with (p/'full_events.csv').open('w') as f:
        fields=['object_id','attempt_id','event','wall_monotonic_ns','sim_time','source','censored','details'];writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for row in full_events:writer.writerow({**row,'details':json.dumps(row['details'])})
    (p/'aggregate.json').write_text(json.dumps(aggregate,indent=2));(p/'operations.json').write_text(json.dumps(operations,indent=2))
    if operations:
        with (p/'operations.csv').open('w') as f:wr=csv.DictWriter(f,fieldnames=list(operations[0]));wr.writeheader();wr.writerows(operations)
        plt.style.use('seaborn-v0_8-whitegrid');fig,axs=plt.subplots(2,2,figsize=(12,8));x=list(range(1,len(operations)+1));colors=['#d33b3b' if r['color']=='RED' else '#3275d5' for r in operations]
        axs[0,0].bar(x,[r['pick_and_place_wall_s'] for r in operations],color=colors);axs[0,0].set(title='Verified operation duration',xlabel='Placement order',ylabel='Monotonic wall seconds')
        groups=[[r['pick_and_place_wall_s'] for r in operations if r['color']==c] for c in ['RED','BLUE']]
        valid=[(c,g) for c,g in zip(['RED','BLUE'],groups) if g]
        axs[0,1].boxplot([g for c,g in valid],labels=[c for c,g in valid]);axs[0,1].set(title='Red and blue durations',ylabel='Monotonic wall seconds')
        bottom=[0.]*len(operations)
        for phase in ['planning','reach_and_grasp','grasp_and_lift','transport','placement']:
            values=[r[phase+'_wall_s'] or 0 for r in operations];axs[1,0].bar(x,values,bottom=bottom,label=phase.replace('_',' '));bottom=[a+b for a,b in zip(bottom,values)]
        axs[1,0].set(title='Measured manipulation stages',xlabel='Placement order',ylabel='Monotonic wall seconds');axs[1,0].legend(fontsize=7)
        if pairs:axs[1,1].plot([r['pair'] for r in pairs],[r['cumulative_wall_s'] for r in pairs],marker='o')
        axs[1,1].set(title='Cumulative pair completion',xlabel='Completed red-blue pairs',ylabel='Wall seconds since experiment start');fig.tight_layout();fig.savefig(p/'timing_report.png',dpi=160);plt.close(fig)
        fig,ax=plt.subplots(figsize=(9,4));offset=[0.]*len(operations)
        for label,key in [('Color confirmation across frames','color_recognition_wall_s'),('IK and motion planning','motion_planning_wall_s')]:
            values=[r.get(key) or 0 for r in operations];ax.bar(x,values,bottom=offset,label=label);offset=[a+b for a,b in zip(offset,values)]
        values=[sum(r[k+'_wall_s'] or 0 for k in ['reach_and_grasp','grasp_and_lift','transport','placement']) for r in operations];ax.bar(x,values,bottom=offset,label='Physical manipulation and verification');ax.set(xlabel='Verified placement order',ylabel='Measured wall seconds',title='Perception, planning, and manipulation intervals');ax.legend(fontsize=8);fig.tight_layout();fig.savefig(p/'pipeline_comparison.png',dpi=160);plt.close(fig)
    lines=['# Actual Gazebo experiment report','',f"Full fourteen-placement run verified: **{aggregate['verified_full_run']}**",f"Verified placements: {summary['verified_placements']}; attempts: {summary['attempts']}; failed attempts: {summary['failed_attempts']}; incorrect picks: {summary['incorrect_picks']}.",f"Wall elapsed: {summary['wall_seconds']:.3f} s. Simulation elapsed: {summary.get('sim_seconds')} s.",f"Seven-pair elapsed wall time: {summary.get('seven_pair_elapsed_time_wall_s')} s.",'','Processing latency (ms): '+json.dumps(aggregate['processing_latency_ms']),'','Missing or censored timing fields remain null. See aggregate.json and operations.csv for definitions and raw values.']
    (p/'REPORT.md').write_text('\n'.join(lines)+'\n');print(json.dumps({k:aggregate[k] for k in ['verified_full_run','verified_placements','wall_seconds','sim_seconds','seven_pair_elapsed_time_wall_s','real_time_factor_measured','processing_latency_ms']},indent=2))
    return aggregate
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('directory');args=ap.parse_args();create(args.directory)
