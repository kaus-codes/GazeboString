import json
from pathlib import Path
import rclpy
from rclpy.node import Node
from motion import Motion
from moveit_msgs.srv import GetPositionIK,GetStateValidity
rclpy.init();n=Node('collision_diagnosis');m=Motion(n);m.ready();m.add_world('/project/worlds/robot_test.sdf');req=GetPositionIK.Request();ik=req.ik_request;ik.group_name='panda_arm';ik.ik_link_name='panda_link8';ik.robot_state.joint_state=m.joints;ik.avoid_collisions=False;ik.timeout.sec=1;ik.pose_stamped.header.frame_id='world';p=ik.pose_stamped.pose;p.position.x=-.021;p.position.y=-.131;p.position.z=.856;p.orientation.x=1.;res=m.result(m.ik.call_async(req));report={'ik_without_collision_filter':res.error_code.val}
if res.error_code.val==1:
    client=n.create_client(GetStateValidity,'/check_state_validity');client.wait_for_service(timeout_sec=5);vr=GetStateValidity.Request();vr.robot_state=res.solution;vr.group_name='panda_arm';valid=m.result(client.call_async(vr));report.update(valid=valid.valid,contacts=[{'body1':c.contact_body_1,'body2':c.contact_body_2,'depth':c.depth} for c in valid.contacts])
Path('/project/results/collision-diagnosis.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));n.destroy_node();rclpy.shutdown()
