#!/usr/bin/env python3
"""Show actual ROS camera annotations and controller-published state on X11."""
import json,os
from pathlib import Path
import cv2,numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class Panel(Node):
    def __init__(self):
        super().__init__('sorting_display');cv2.namedWindow('Actual RGB + verified sorting state',cv2.WINDOW_NORMAL);cv2.resizeWindow('Actual RGB + verified sorting state',480,760);cv2.moveWindow('Actual RGB + verified sorting state',800,0);self.state={};self.create_subscription(String,'/sorting/state',self.status,10);self.create_timer(.2,self.draw)
    def status(self,m):self.state=json.loads(m.data)
    def draw(self):
        p=Path(os.environ.get('SORTING_RUN_DIR','/project/results'))/'live/vision.jpg'
        if not p.exists():return
        im=cv2.imread(str(p))
        if im is None:return
        canvas=np.zeros((760,480,3),np.uint8);canvas[:360]=cv2.resize(im,(480,360));s=self.state;q=s.get('sequence',{})
        selected=s.get('selected') or {};prediction=s.get('predicted_grasp_position');position_text='--' if prediction is None else ', '.join(f'{v:.3f}' for v in prediction)
        lines=[f"Target: {s.get('current_target','--')} | State: {s.get('state','INITIALIZE')}",f"Selected: #{selected.get('id','--')} {selected.get('color','')} {selected.get('shape','')}",f"Grasp xyz (m): {position_text}",f"RED {q.get('red',0)}/7   BLUE {q.get('blue',0)}/7   Pairs {q.get('pairs',0)}/7",f"Verified placements: {q.get('successful_picks',0)}/14 | Failed: {s.get('failed_attempts',0)}",f"Incorrect: {s.get('incorrect_picks',0)} | Remaining pairs: {q.get('remaining_pairs',7)}",f"Wall elapsed: {s.get('wall_elapsed',0):.2f} s",f"Sim elapsed: {s.get('sim_elapsed',0):.2f} s",f"Pixel processing: {s.get('latest_perception_ms',0) or 0:.2f} ms",f"Last operation: {s.get('latest_operation_wall_s',0) or 0:.2f} wall s",f"Measured RTF: {s.get('real_time_factor',0) or 0:.2f}"]
        for i,line in enumerate(lines):cv2.putText(canvas,line,(12,394+i*34),cv2.FONT_HERSHEY_SIMPLEX,.48,(230,230,230),1)
        cv2.imshow('Actual RGB + verified sorting state',canvas);cv2.waitKey(1)
def main():
    rclpy.init();n=Panel()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        cv2.destroyAllWindows();n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
