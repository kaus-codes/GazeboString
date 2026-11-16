#!/usr/bin/env python3
import os,time,json,subprocess
from pathlib import Path
from runtime import Runtime
from feeder import Feeder
p=Path('/project/results/feeder-probe');p.mkdir(parents=True,exist_ok=True);os.environ['SORTING_RUN_DIR']=str(p);os.environ['DISPLAY']=':1'
subprocess.run(['python3','/project/src/generate_world.py'],check=True)
with Runtime(p/'runtime') as run:
    run.start('gazebo',['gz','sim','-s','-r','/project/worlds/sorting.sdf'])
    feeder=Feeder(11);feeder.start();start=time.monotonic()
    while len(feeder.records)<5 and time.monotonic()-start<65:time.sleep(.1)
    feeder.stop();rows=feeder.records
    passed=len(rows)>=5 and all(r['first_pose'] is not None and abs(r['first_pose'][0]+1.35)<.02 and .70<r['first_pose'][2]<.74 for r in rows)
    report={'passed':passed,'confirmed_spawns':sum(r['first_pose'] is not None for r in rows),'rows':rows,'wall_seconds':time.monotonic()-start};(p/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
raise SystemExit(0 if passed else 1)
