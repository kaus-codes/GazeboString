#!/usr/bin/env python3
import json,time,os
from pathlib import Path
import cv2,numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from vision import Detector,Tracker,workstation_camera,annotate

class VisionNode(Node):
    def __init__(self):
        super().__init__('rgb_perception');cv2.setNumThreads(1); self.detector=None; self.tracker=Tracker(); self.frames=0;self.selected_id=None;self.create_subscription(String,'/sorting/state',self.state,10)
        self.pub=self.create_publisher(String,'/sorting/tracks',10)
        self.create_subscription(CameraInfo,'/sorting/camera_info',self.info,qos_profile_sensor_data)
        self.create_subscription(Image,'/sorting/rgb',self.image,qos_profile_sensor_data)
        self.out=Path(os.environ.get('SORTING_RUN_DIR','/project/results'))/'live';self.out.mkdir(parents=True,exist_ok=True)
    def state(self,m):
        selected=json.loads(m.data).get('selected');self.selected_id=selected.get('id') if selected else None
    def info(self,m):
        if self.detector is not None:return
        self.detector=Detector(workstation_camera(m.k))
        if os.environ.get('SORTING_RUN_DIR'):
            calibration={'K':list(m.k),'camera_world':self.detector.geometry.origin.tolist(),'world_from_optical':self.detector.geometry.R_world_optical.tolist(),'width':m.width,'height':m.height}
            (Path(os.environ['SORTING_RUN_DIR'])/'calibration.json').write_text(json.dumps(calibration,indent=2))
    def image(self,m):
        if self.detector is None:return
        if m.encoding not in ['rgb8','bgr8']:
            self.get_logger().error(f'Unsupported encoding {m.encoding}');return
        raw=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
        bgr=cv2.cvtColor(raw,cv2.COLOR_RGB2BGR) if m.encoding=='rgb8' else raw
        processing_start=time.monotonic_ns();stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9; detections,latency=self.detector.process(bgr); tracks=self.tracker.update(detections,stamp); self.frames+=1
        payload={'sim_stamp':stamp,'wall_monotonic_ns':time.monotonic_ns(),'processing_start_wall_ns':processing_start,'processing_ms':latency,'frames':self.frames,'tracks':tracks}
        self.pub.publish(String(data=json.dumps(payload)))
        if os.environ.get('SORTING_RUN_DIR'):
            with (Path(os.environ['SORTING_RUN_DIR'])/'camera_frames.jsonl').open('a') as f:f.write(json.dumps(payload)+'\n')
        cv2.imwrite(str(self.out/'vision.tmp.jpg'),annotate(bgr,tracks,latency,self.selected_id));(self.out/'vision.tmp.jpg').replace(self.out/'vision.jpg')
        (self.out/'perception.json').write_text(json.dumps(payload,indent=2))

def main():
    rclpy.init();n=VisionNode()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
