#!/usr/bin/env python3
"""Actual Gazebo RGB integration probe, with separate evaluation-only truth."""
import csv,json,threading,time
from pathlib import Path
import cv2,numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.camera_info_pb2 import CameraInfo
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.double_pb2 import Double
from vision import Detector,Tracker,workstation_camera,annotate

def main():
    out=Path('/project/results/camera'); out.mkdir(parents=True,exist_ok=True)
    n=Node(); ready=threading.Event(); lock=threading.Lock(); state={'image':None,'K':None,'truth':{},'received':0}; rows=[]; matches=[]; tracker=Tracker()
    manifest=json.loads(Path('/project/worlds/sorting.evaluation-only.json').read_text())
    truth_colors={p['name']:p['color'] for p in manifest['objects']}
    def camera(m):
        with lock:
            state['received']+=1; state['image']=m
        ready.set()
    def info(m):
        with lock: state['K']=list(m.intrinsics.k)
    def poses(m):
        with lock: state['truth']={p.name:[p.position.x,p.position.y,p.position.z] for p in m.pose if p.name in truth_colors}
    assert n.subscribe(Image,'/sorting/rgb',camera)
    assert n.subscribe(CameraInfo,'/sorting/camera_info',info)
    assert n.subscribe(Pose_V,'/world/rgb_sorting/dynamic_pose/info',poses)
    pub=n.advertise('/conveyor/speed',Double); start=time.monotonic(); previous=None; frame=0; best_count=0; detector=None
    while time.monotonic()-start<25:
        pub.publish(Double(data=.04))
        if not ready.wait(.25): continue
        ready.clear()
        with lock: m=state['image']; K=state['K']; truth=dict(state['truth'])
        if K is None: continue
        if detector is None: detector=Detector(workstation_camera(K)); (out/'calibration.json').write_text(json.dumps({'K':K,'camera_world':detector.geometry.origin.tolist(),'world_from_optical':detector.geometry.R_world_optical.tolist(),'nominal_surface_z':.735},indent=2))
        stamp=m.header.stamp.sec+m.header.stamp.nsec*1e-9
        if stamp==previous: continue
        previous=stamp
        if m.pixel_format_type!=3: raise RuntimeError(f'Expected RGB_INT8 enum=3; received {m.pixel_format_type}')
        rgb=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
        bgr=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR); detections,latency=detector.process(bgr); tracks=tracker.update(detections,stamp); frame+=1
        for t in tracks:
            row={'frame':frame,'sim_time':stamp,'wall_monotonic':time.monotonic(),'processing_ms':latency,**t}
            rows.append(row)
            # Evaluation only: nearest truth with a strict spatial gate. Never returned to detector.
            candidates=[(np.linalg.norm(np.array(pos[:2])-np.array(t['xyz'][:2])),name) for name,pos in truth.items()]
            if candidates:
                distance,name=min(candidates)
                if distance<.04: matches.append({'frame':frame,'track_id':t['id'],'true_name':name,'true_color':truth_colors[name],'predicted_color':t['color'],'error_m':float(distance)})
        cv2.imwrite(str(out/'live_annotated.jpg'),annotate(bgr,tracks,latency))
        if len(tracks)>best_count:
            best_count=len(tracks); cv2.imwrite(str(out/'camera_raw.png'),bgr); cv2.imwrite(str(out/'camera_annotated.png'),annotate(bgr,tracks,latency))
        rows.append({'frame':frame,'sim_time':stamp,'wall_monotonic':time.monotonic(),'processing_ms':latency,'frame_summary':True,'detections':len(tracks)})
    frame_rows=[r for r in rows if r.get('frame_summary')]; ts=[r['processing_ms'] for r in frame_rows]; detected=sorted(set(r['color'] for r in rows if 'color' in r)); errors=[r['error_m'] for r in matches]
    summary={'passed':frame>=30 and 'RED' in detected and 'BLUE' in detected and len(matches)>=30 and all(r['true_color']==r['predicted_color'] for r in matches),'frames_processed':frame,'frames_received':state['received'],'wall_seconds':time.monotonic()-start,'sim_first':frame_rows[0]['sim_time'] if frame_rows else None,'sim_last':frame_rows[-1]['sim_time'] if frame_rows else None,'detected_colors':detected,'matched_evaluation_samples':len(matches),'color_accuracy_matched':sum(r['true_color']==r['predicted_color'] for r in matches)/len(matches) if matches else None,'localization_error_mean_m':float(np.mean(errors)) if errors else None,'processing_latency_ms':{k:float(fn(ts)) for k,fn in [('mean',np.mean),('median',np.median),('min',np.min),('max',np.max),('std',np.std)]} if ts else {},'verified_placements':0,'seven_pair_elapsed_time':None}
    (out/'summary.json').write_text(json.dumps(summary,indent=2)); (out/'detections.json').write_text(json.dumps(rows,indent=2)); (out/'evaluation.json').write_text(json.dumps(matches,indent=2))
    if frame_rows:
        with (out/'frames.csv').open('w') as f:
            wr=csv.DictWriter(f,fieldnames=list(frame_rows[0])); wr.writeheader(); wr.writerows(frame_rows)
    print(json.dumps(summary,indent=2)); return 0 if summary['passed'] else 1
if __name__=='__main__': raise SystemExit(main())
