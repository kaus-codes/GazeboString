#!/usr/bin/env python3
"""Independent ground truth probe. Not imported by perception or control."""
import csv,json,threading,time
from pathlib import Path
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.double_pb2 import Double
from gz.msgs10.clock_pb2 import Clock

def main():
    out=Path('/project/results/conveyor'); out.mkdir(parents=True,exist_ok=True)
    n=Node(); rows=[]; state={'sim':None}; event=threading.Event()
    def clock(m): state['sim']=m.sim.sec+m.sim.nsec*1e-9
    def poses(m):
        for p in m.pose:
            if p.name.startswith('part_'):
                rows.append(dict(wall_monotonic=time.monotonic(),sim=state['sim'],name=p.name,x=p.position.x,y=p.position.y,z=p.position.z)); event.set()
    assert n.subscribe(Clock,'/clock',clock)
    assert n.subscribe(Pose_V,'/world/rgb_sorting/dynamic_pose/info',poses)
    pub=n.advertise('/conveyor/speed',Double)
    start=time.monotonic()
    while time.monotonic()-start<14:
        pub.publish(Double(data=.04)); event.wait(.1); event.clear()
    with (out/'poses.csv').open('w') as f:
        wr=csv.DictWriter(f,fieldnames=['wall_monotonic','sim','name','x','y','z']); wr.writeheader(); wr.writerows(rows)
    motion={}
    for name in sorted(set(r['name'] for r in rows)):
        rr=[r for r in rows if r['name']==name and r['sim'] is not None]
        if len(rr)>10:
            a,b=rr[len(rr)//4],rr[-1]; dt=b['sim']-a['sim']
            motion[name]={'dx':b['x']-a['x'],'dt_sim':dt,'speed':(b['x']-a['x'])/dt if dt>0 else None,'final_z':b['z']}
    manifest=json.loads(Path('/project/worlds/sorting.evaluation-only.json').read_text())
    shapes={p['name']:p['shape'] for p in manifest['objects']}
    # Ideal rolling sphere COM tends toward (2/7) belt speed; boxes/cylinders translate.
    # This checks real forward motion and support, not forced model velocities.
    passed=len(motion)==12 and all((.008<v['speed']<.016 if shapes[name]=='sphere' else .035<v['speed']<.045) and .69<v['final_z']<.75 for name,v in motion.items())
    report={'passed':passed,'command_m_s':.04,'wall_seconds':time.monotonic()-start,'samples':len(rows),'objects':motion}
    (out/'summary.json').write_text(json.dumps(report,indent=2)); print(json.dumps(report,indent=2)); return 0 if passed else 1
if __name__=='__main__': raise SystemExit(main())
