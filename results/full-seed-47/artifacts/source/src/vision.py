"""Pixel-only color detection, calibrated ray/plane localization, and tracking.
No simulator model names or spawn/evaluation metadata are consumed here.
"""
from dataclasses import dataclass
import time
import cv2
import numpy as np
from configuration import CONFIG,PERCEPTION

HSV=PERCEPTION['hsv_ranges']

@dataclass
class CameraGeometry:
    K: np.ndarray
    R_world_optical: np.ndarray
    origin: np.ndarray
    def project_plane(self,u,v,z):
        ray=self.R_world_optical @ np.linalg.solve(self.K,np.array([u,v,1.]))
        if abs(ray[2])<1e-8: raise ValueError('ray parallel to belt plane')
        scale=(z-self.origin[2])/ray[2]
        if scale<=0: raise ValueError('plane behind camera')
        return self.origin+scale*ray

def workstation_camera(K):
    roll,pitch,yaw=CONFIG['camera']['rpy']
    Ry=np.array([[np.cos(pitch),0,np.sin(pitch)],[0,1,0],[-np.sin(pitch),0,np.cos(pitch)]])
    Rz=np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
    Rx=np.array([[1,0,0],[0,np.cos(roll),-np.sin(roll)],[0,np.sin(roll),np.cos(roll)]])
    optical_to_gazebo=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    return CameraGeometry(np.asarray(K).reshape(3,3),Rz@Ry@Rx@optical_to_gazebo,np.array(CONFIG['camera']['position']))

class Detector:
    def __init__(self,geometry): self.geometry=geometry
    def process(self,bgr):
        start=time.perf_counter_ns(); hsv=cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV); found=[]
        for color,ranges in HSV.items():
            mask=np.zeros(hsv.shape[:2],np.uint8)
            for low,high in ranges: mask|=cv2.inRange(hsv,np.array(low),np.array(high))
            mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((PERCEPTION['morphology_kernel'],PERCEPTION['morphology_kernel']),np.uint8))
            mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((PERCEPTION['morphology_kernel'],PERCEPTION['morphology_kernel']),np.uint8))
            contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area=cv2.contourArea(c)
                if area<PERCEPTION['minimum_contour_area_px'] or area>PERCEPTION['maximum_contour_area_px']: continue
                m=cv2.moments(c)
                if not m['m00']: continue
                u,v=m['m10']/m['m00'],m['m01']/m['m00']; x,y,w,h=cv2.boundingRect(c)
                peri=cv2.arcLength(c,True); approx=cv2.approxPolyDP(c,.04*peri,True)
                circularity=4*np.pi*area/max(peri*peri,1)
                shape='ROUND' if circularity>.79 and len(approx)>5 else 'BOX'
                # RGB cannot reliably distinguish top-down spheres from cylinders.
                # Use nominal top-plane for boxes; round localization has height uncertainty.
                xyz=self.geometry.project_plane(u,v,CONFIG['camera']['nominal_surface_z'])
                if not (-.53<xyz[0]<.53 and abs(xyz[1])<.155): continue
                roi=hsv[y:y+h,x:x+w];roi_mask=mask[y:y+h,x:x+w]>0;sat=float(np.mean(roi[:,:,1][roi_mask])/255) if np.any(roi_mask) else 0
                found.append(dict(color=color,shape=shape,confidence=sat,u=u,v=v,bbox=[x,y,w,h],xyz=xyz.tolist(),area=area,position_uncertainty_m=.012 if shape=='ROUND' else .004))
        return found,(time.perf_counter_ns()-start)/1e6

class Tracker:
    def __init__(self,max_age=PERCEPTION['track_max_age_sim_s'],gate=PERCEPTION['association_gate_m']): self.tracks={}; self.next_id=1; self.max_age=max_age; self.gate=gate
    def update(self,detections,stamp):
        self.tracks={k:v for k,v in self.tracks.items() if 0<=stamp-v['last_seen']<=self.max_age}
        used=set(); output=[]
        for d in sorted(detections,key=lambda d:d['confidence'],reverse=True):
            pos=np.array(d['xyz']); best=None; distance=self.gate
            for key,t in self.tracks.items():
                if key in used or t['color']!=d['color']: continue
                pred=np.array(t['xyz'])+np.array(t['velocity'])*(stamp-t['last_seen'])
                dist=np.linalg.norm(pos[:2]-pred[:2])
                if dist<distance: best=key; distance=dist
            if best is None:
                best=self.next_id; self.next_id+=1; velocity=np.zeros(3); first=stamp; hits=1
            else:
                old=self.tracks[best]; dt=stamp-old['last_seen']; velocity=np.array(old['velocity'])
                if dt>1e-5: velocity=.7*velocity+.3*(pos-np.array(old['xyz']))/dt
                first=old['first_seen']; hits=old['hits']+1
            confirmed=self.tracks.get(best,{}).get('color_confirmed_sim')
            confirmed_wall=self.tracks.get(best,{}).get('color_confirmed_wall_ns')
            if hits>=3 and confirmed is None:confirmed=stamp;confirmed_wall=time.monotonic_ns()
            t={**d,'color_confirmed_sim':confirmed,'color_confirmed_wall_ns':confirmed_wall,'id':best,'first_seen':first,'last_seen':stamp,'velocity':velocity.tolist(),'hits':hits,'first_seen_wall_ns':self.tracks.get(best,{}).get('first_seen_wall_ns',time.monotonic_ns()),'eligible':d['color'] in ('RED','BLUE') and hits>=3}
            self.tracks[best]=t; used.add(best); output.append(t)
        return output

def annotate(image,tracks,latency,selected_id=None):
    out=image.copy()
    colors={'RED':(40,40,255),'BLUE':(255,100,30),'GREEN':(40,230,40),'YELLOW':(20,240,240),'ORANGE':(20,140,255)}
    for d in tracks:
        x,y,w,h=d['bbox']; c=colors[d['color']];
        if d['id']==selected_id:cv2.rectangle(out,(x-2,y-2),(x+w+2,y+h+2),(255,255,255),2)
        cv2.rectangle(out,(x,y),(x+w,y+h),c,2)
        cv2.circle(out,(round(d['u']),round(d['v'])),3,(255,255,255),-1)
        cv2.putText(out,f"{d['id']} {d['color']} {d['shape']} {d['confidence']:.2f}",(x,max(y-6,15)),cv2.FONT_HERSHEY_SIMPLEX,.45,c,1)
    cv2.putText(out,f'RGB VISION SORTING | {latency:.2f} ms',(12,24),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
    cv2.putText(out,'Camera pixels | calibrated world positions',(12,465),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
    return out
