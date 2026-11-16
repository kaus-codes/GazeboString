import sys, unittest, tempfile,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import cv2,numpy as np
from vision import Detector,Tracker,workstation_camera,CameraGeometry
from sequence import SequenceValidator,VerifiedPlacement
from metrics import Recorder

class CoreTests(unittest.TestCase):
    def camera(self):return CameraGeometry(np.array([[550,0,320],[0,550,240],[0,0,1]]),np.array([[0.,-1,0],[-1.,0,0],[0,0,-1.]]),np.array([0.,0.,1.9]))
    def test_calibration_center_and_axes(self):
        c=self.camera(); np.testing.assert_allclose(c.project_plane(320,240,.735),[0,0,.735]); np.testing.assert_allclose(c.project_plane(375,295,.735),[-.1165,-.1165,.735])
    def test_red_wraparound_and_distractor_eligibility(self):
        detector=Detector(self.camera())
        for hue,color in [(0,'RED'),(179,'RED'),(115,'BLUE'),(60,'GREEN'),(30,'YELLOW'),(15,'ORANGE')]:
            hsv=np.zeros((480,640,3),np.uint8); cv2.rectangle(hsv,(307,220),(333,250),(hue,240,230),-1)
            detections,_=detector.process(cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)); self.assertEqual([d['color'] for d in detections],[color])
            tracker=Tracker()
            for t in [0,.1,.2]: result=tracker.update(detections,t)
            self.assertEqual(result[0]['eligible'],color in ['RED','BLUE'])
    def test_track_motion_and_expiry(self):
        tracker=Tracker(); ids=[]
        for i in range(15):
            d={'color':'RED','xyz':[i*.004,0,.735],'confidence':1}; out=tracker.update([d],i*.1); ids.append(out[0]['id'])
        self.assertEqual(len(set(ids)),1); self.assertAlmostEqual(out[0]['velocity'][0],.04,places=3)
        self.assertEqual(tracker.update([],3),[]); self.assertFalse(tracker.tracks)
    def placement(self,i,color=None,**kwargs):
        color=color or ['RED','BLUE'][i%2]
        data=dict(object_id=str(i),perceived_color=color,evaluated_color=color,bin_color=color,grasp_contact=True,lift_confirmed=True,released=True,contained_and_settled=True,sim_time=float(i+1),wall_monotonic_ns=i+1);data.update(kwargs);return VerifiedPlacement(**data)
    def test_exact_fourteen_and_duplicates(self):
        validator=SequenceValidator()
        for i in range(14):validator.accept(self.placement(i))
        self.assertTrue(validator.summary()['complete']);self.assertEqual(validator.summary()['pairs'],7)
        with self.assertRaises(ValueError):validator.accept(self.placement(14))
        self.assertFalse(validator.summary()['complete'])
    def test_reject_wrong_color_failed_grasp_and_unsettled_release(self):
        for p in [self.placement(0,'BLUE'),self.placement(0,grasp_contact=False),self.placement(0,released=False),self.placement(0,contained_and_settled=False),self.placement(0,bin_color='BLUE')]:
            v=SequenceValidator()
            with self.assertRaises(ValueError):v.accept(p)
            self.assertEqual(v.summary()['successful_picks'],0)
    def test_timing_export_and_clock_regression(self):
        with tempfile.TemporaryDirectory() as td:
            r=Recorder(td,{'evidence_type':'unit_test_only'});r.record('a','planning_start',2);r.record('a','planning_complete',2.1)
            with self.assertRaises(ValueError):r.record('a','movement_start',1)
            r.export(); data=json.loads((Path(td)/'events.json').read_text());self.assertGreater(data[1]['wall_monotonic_ns'],data[0]['wall_monotonic_ns']);self.assertTrue((Path(td)/'events.csv').exists())
class GeometryEvaluation(unittest.TestCase):
    def test_grasp_orientation_is_normalized_and_points_down(self):
        from motion import downward_pose
        p=downward_pose([.1,0,1.]);q=p.orientation
        self.assertAlmostEqual(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w,1.)
        tool_axis=[2*(q.x*q.z+q.y*q.w),2*(q.y*q.z-q.x*q.w),1-2*(q.x*q.x+q.y*q.y)]
        np.testing.assert_allclose(tool_axis,[0,0,-1])

    def test_rotated_collision_extents(self):
        from physical_verifier import object_extents
        from types import SimpleNamespace
        q=SimpleNamespace(x=0,y=0,z=np.sin(np.pi/8),w=np.cos(np.pi/8))
        ext=object_extents({'shape':'box','dimensions':[.035]*3},q)
        self.assertAlmostEqual(ext[0],.035/np.sqrt(2));self.assertAlmostEqual(ext[1],.035/np.sqrt(2));self.assertAlmostEqual(ext[2],.0175)
        self.assertEqual(object_extents({'shape':'sphere'},q),(.019,.019,.019))

class FeederRegression(unittest.TestCase):
    def test_spawn_timeout_preserves_created_object_audit(self):
        from unittest.mock import patch
        import subprocess,os
        from feeder import Feeder
        f=Feeder(11)
        def ambiguous_creation(*args,**kwargs):
            self.assertEqual(f.records[0]['name'],'part_012')
            self.assertEqual(f.records[0]['color'],'RED')
            f.seen['part_012']=[-1.35,-.095,.725]
            raise subprocess.TimeoutExpired(args[0],5)
        with patch('feeder.subprocess.run',side_effect=ambiguous_creation),patch.dict(os.environ,{},clear=True):
            row=f.spawn('RED','box',[.035]*3,-.095)
        self.assertEqual(row['first_pose'],[-1.35,-.095,.725]);self.assertIn('timeout',row['service_error']);self.assertEqual(f.index,13)

if __name__=='__main__':unittest.main()
