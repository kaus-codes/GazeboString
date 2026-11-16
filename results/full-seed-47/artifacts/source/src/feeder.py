"""Seeded loader with unique requests and pose-confirmed, timeout-safe auditing."""
import random,time,threading,json,subprocess,os,xml.etree.ElementTree as E
from pathlib import Path
from google.protobuf.text_format import MessageToString
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.double_pb2 import Double
from gz.msgs10.entity_factory_pb2 import EntityFactory
from generate_world import body
from configuration import CONFIG

class Feeder:
    def __init__(self,seed=11,speed=None):
        self.node=Node();self.seen={};self.by_name={};self.pose_event=threading.Event();self.sim=0.;self.speed=CONFIG['belt']['speed_m_s'] if speed is None else speed;self.running=True;self.rng=random.Random(seed);self.index=12;self.records=[];self.previous_shape=None
        self.node.subscribe(Pose_V,'/world/rgb_sorting/dynamic_pose/info',self.poses);self.node.subscribe(Clock,'/clock',self.clock);self.pub=self.node.advertise('/conveyor/speed',Double);self.thread=threading.Thread(target=self.loop,daemon=True)
    def poses(self,m):
        for p in m.pose:
            if p.name.startswith('part_') and p.name not in self.seen:
                pos=[p.position.x,p.position.y,p.position.z];self.seen[p.name]=pos;self.pose_event.set()
                if p.name in self.by_name:self.by_name[p.name].update(first_pose=pos,confirmed_sim=self.sim)
    def clock(self,m):self.sim=m.sim.sec+m.sim.nsec*1e-9
    def start(self):self.thread.start()
    def spawn(self,color,shape,dims,y):
        rgba={'RED':'.95 .015 .015 1','BLUE':'.015 .04 .95 1','GREEN':'.015 .9 .025 1','YELLOW':'.95 .9 .015 1','ORANGE':'1 .28 .01 1'}[color]
        name=f'part_{self.index:03d}';self.index+=1;root=E.Element('sdf',version='1.9');body(root,name,[0,0,0,0,0,0],shape,dims,rgba,False)
        req=EntityFactory();req.sdf=E.tostring(root,encoding='unicode');req.name=name;req.pose.position.x=CONFIG['belt']['loading_x'];req.pose.position.y=y;req.pose.position.z=.725
        import math
        yaw=self.rng.uniform(-3.14,3.14);req.pose.orientation.z=math.sin(yaw/2);req.pose.orientation.w=math.cos(yaw/2)
        # Audit registration precedes the RPC: a timeout is not proof of non-creation.
        record={'name':name,'color':color,'shape':shape,'dimensions':dims,'spawn_sim':self.sim,'confirmed_sim':None,'first_pose':None,'service_reply':None,'service_error':None}
        self.records.append(record);self.by_name[name]=record
        try:
            reply=subprocess.run(['gz','service','-s','/world/rgb_sorting/create/blocking','--reqtype','gz.msgs.EntityFactory','--reptype','gz.msgs.Boolean','--timeout','3000','--req',MessageToString(req)],capture_output=True,text=True,timeout=5)
            record.update(service_reply=reply.stdout.strip(),service_error=reply.stderr.strip())
        except subprocess.TimeoutExpired:record['service_error']='CLI wall timeout; creation outcome is determined from poses'
        except OSError as e:record['service_error']=str(e)
        deadline=time.monotonic()+2
        while self.running and name not in self.seen and time.monotonic()<deadline:self.pose_event.wait(.05);self.pose_event.clear()
        if name in self.seen:record.update(first_pose=self.seen[name],confirmed_sim=self.sim)
        if os.environ.get('SORTING_RUN_DIR'):
            with (Path(os.environ['SORTING_RUN_DIR'])/'feeder.jsonl').open('a') as f:f.write(json.dumps(record)+'\n')
        return record
    def loop(self):
        palette=[];next_spawn=self.sim+CONFIG['belt']['feed_interval_sim_s']
        while self.running:
            self.pub.publish(Double(data=self.speed))
            if self.sim>=next_spawn:
                if not palette:palette=['RED']*8+['BLUE']*8+['GREEN']*3+['YELLOW']*3+['ORANGE']*3;self.rng.shuffle(palette)
                color=palette.pop();shape=self.rng.choice(['box','cylinder'] if self.previous_shape=='sphere' else ['box','cylinder','sphere']);self.previous_shape=shape;dims={'box':[.035,.035,.035],'cylinder':[.019,.035],'sphere':[.019]}[shape];y={'box':-.095,'cylinder':0.,'sphere':.095}[shape]
                self.spawn(color,shape,dims,y);next_spawn=self.sim+CONFIG['belt']['feed_interval_sim_s']
            self.pose_event.wait(.08);self.pose_event.clear() # refresh only; not a timing measurement
    def stop(self):self.running=False;self.pose_event.set();self.thread.join(timeout=8)
