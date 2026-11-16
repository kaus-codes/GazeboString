#!/usr/bin/env python3
"""One spawn request; observed entity state determines uncertain outcomes."""
import json,os,subprocess,threading,time
from pathlib import Path
from google.protobuf.text_format import MessageToString
from gz.transport13 import Node
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.pose_v_pb2 import Pose_V
node=Node();seen=threading.Event()
def observe(m):
    if any(p.name=='panda' for p in m.pose):seen.set()
node.subscribe(Pose_V,'/world/rgb_sorting/dynamic_pose/info',observe)
req=EntityFactory();req.sdf_filename='/project/config/panda.sdf';req.name='panda';report={'request_name':'panda','confirmed':False}
try:
    reply=subprocess.run(['gz','service','-s','/world/rgb_sorting/create/blocking','--reqtype','gz.msgs.EntityFactory','--reptype','gz.msgs.Boolean','--timeout','8000','--req',MessageToString(req)],capture_output=True,text=True,timeout=12)
    report.update(reply=reply.stdout,error=reply.stderr)
except subprocess.TimeoutExpired:report['error']='Spawn reply timed out; awaiting actual model observation'
report['confirmed']=seen.wait(20)
Path(os.environ.get('SORTING_RUN_DIR','/project/results')+'/robot_spawn.json').write_text(json.dumps(report,indent=2))
if not report['confirmed']:raise SystemExit('Robot model was not observed after the single spawn request')
print('Robot spawn verified from actual model poses',flush=True)
