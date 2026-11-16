"""Evaluation-only physical evidence. Never supplies target selection or color."""
import json,threading,math,time
from pathlib import Path
from types import SimpleNamespace
from configuration import CONFIG
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.contacts_pb2 import Contacts

def object_extents(metadata,q):
    shape=metadata['shape'];dims=metadata.get('dimensions',{'box':[.035,.035,.035],'cylinder':[.019,.035],'sphere':[.019]}[shape])
    if shape=='sphere':return (dims[0],)*3
    x,y,z,w=q.x,q.y,q.z,q.w
    rows=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    if shape=='box':return tuple(sum(abs(a)*d/2 for a,d in zip(row,dims)) for row in rows)
    return tuple(dims[0]*math.sqrt(max(0,1-row[2]**2))+dims[1]/2*abs(row[2]) for row in rows)

def footprint_extents(metadata,q):return object_extents(metadata,q)[:2]

class PhysicalVerifier:
    def __init__(self,manifest='/project/worlds/robot_test.evaluation-only.json'):
        self.spawned=[];self.entries={};self.observed_outside=set();self.evaluations=[];self.odometry=[];self.raw_contacts=[];self.node=Node();self.lock=threading.Lock();self.poses={};self.orientations={};self.held='';self.sim=0.;self.contacts=set()
        self.manifest={x['name']:x for x in json.loads(Path(manifest).read_text())['objects']}
        self.node.subscribe(Pose_V,'/evaluation/poses',self._poses)
        self.node.subscribe(StringMsg,'/suction/held',self._held)
        self.node.subscribe(Clock,'/clock',self._clock)
        self.node.subscribe(Odometry,'/model/conveyor/link/body/odometry',self._odometry)
        self.node.subscribe(Contacts,'/world/rgb_sorting/model/panda/link/suction_cup/sensor/suction_contact/contact',self._contacts)
        self.suction=self.node.advertise('/suction/enable',Boolean)
    def _clock(self,m):self.sim=m.sim.sec+m.sim.nsec*1e-9
    def _poses(self,m):
        with self.lock:
            self.manifest.update({x['name']:x for x in self.spawned})
            for p in m.pose:
                if p.name in self.manifest:
                    self.orientations[p.name]=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
                    self.poses[p.name]=(m.header.stamp.sec+m.header.stamp.nsec*1e-9,[p.position.x,p.position.y,p.position.z])
                    extent_x,extent_y=footprint_extents(self.manifest[p.name],p.orientation)
                    intersects=p.position.x+extent_x>-.53 and p.position.x-extent_x<.53 and p.position.y+extent_y>-.155 and p.position.y-extent_y<.155
                    if p.name not in self.entries and intersects and .69<p.position.z<.76:
                        self.entries[p.name]={'sim_time':self.sim,'wall_monotonic_ns':time.monotonic_ns(),'left_censored':p.name not in self.observed_outside,'definition':'first physical footprint intersection with calibrated detection region'}
                    elif not intersects:self.observed_outside.add(p.name)
    def _held(self,m):
        with self.lock:
            self.held=m.data
            if m.data:self.contacts.add(m.data)
    def enable(self,value):self.suction.publish(Boolean(data=value))
    def pose(self,name):
        with self.lock:return self.poses.get(name)
    def bin_evidence(self,name,color):
        row=self.pose(name);bx,by=CONFIG['bins'][color]
        if row is None:return {'valid':False,'reason':'no physics snapshot'}
        stamp,pos=row;age=self.sim-stamp
        q=self.orientations.get(name,[0,0,0,1]);extent=object_extents(self.manifest[name],SimpleNamespace(x=q[0],y=q[1],z=q[2],w=q[3]))
        # Inner faces of 20 mm walls at +/-150 mm; contact penetration tolerance 1 mm.
        checks={'fresh':-.1<age<.2,'horizontal':abs(pos[0]-bx)+extent[0]<=.141 and abs(pos[1]-by)+extent[1]<=.141,'vertical':pos[2]-extent[2]>=.659 and pos[2]+extent[2]<=.841,'released':self.held!=name}
        return {'valid':all(checks.values()),'position':pos,'half_extents':extent,'orientation_xyzw':q,'snapshot_sim':stamp,'clock_sim':self.sim,'age_sim_s':age,'checks':checks}
    def in_bin(self,name,color):return self.bin_evidence(name,color)['valid']

    def evaluate_frame(self,frame):
        # Independent camera/ground-truth comparison. Never influences selection.
        with self.lock:
            for t in frame['tracks']:
                candidates=[(sum((a-b)**2 for a,b in zip(pos[:2],t['xyz'][:2]))**.5,name) for name,(stamp,pos) in self.poses.items() if self.sim-stamp<.2]
                if not candidates:continue
                distance,name=min(candidates)
                if distance<.04:
                    self.evaluations.append({'sim_stamp':frame['sim_stamp'],'wall_monotonic_ns':frame['wall_monotonic_ns'],'object_id':name,'track_id':t['id'],'predicted_color':t['color'],'true_color':self.manifest[name]['color'],'error_m':distance})

    def _odometry(self,m):
        self.odometry.append({'sim_time':m.header.stamp.sec+m.header.stamp.nsec*1e-9,'wall_monotonic_ns':time.monotonic_ns(),'speed_m_s':m.twist.linear.x})
    def _contacts(self,m):
        if m.contact:
            self.raw_contacts.append({'sim_time':self.sim,'wall_monotonic_ns':time.monotonic_ns(),'contacts':[{'collision1':c.collision1.name,'collision2':c.collision2.name,'points':len(c.position)} for c in m.contact]})
