from pathlib import Path
import xml.etree.ElementTree as E
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share=Path(get_package_share_directory('moveit_resources_panda_moveit_config'))/'config'
    srdf=E.parse(share/'panda.srdf'); root=srdf.getroot()
    for v in root.findall('virtual_joint'):root.remove(v)
    removed={'panda_leftfinger','panda_rightfinger','panda_finger_joint1','panda_finger_joint2'}
    for element in list(root):
        if element.tag=='disable_collisions' and (element.get('link1') in removed or element.get('link2') in removed):root.remove(element)
        elif element.tag=='group_state' and any(j.get('name') in removed for j in element.findall('joint')):root.remove(element)
        elif element.tag=='group' and element.get('name')=='hand':
            for c in list(element):element.remove(c)
            E.SubElement(element,'link',name='panda_hand');E.SubElement(element,'link',name='suction_cup')
    E.SubElement(root,'disable_collisions',link1='panda_hand',link2='suction_cup',reason='Adjacent')
    controller={'controller_names':['arm_controller'],'arm_controller':{'action_ns':'follow_joint_trajectory','type':'FollowJointTrajectory','default':True,'joints':[f'panda_joint{i}' for i in range(1,8)]}}
    ompl=yaml.safe_load((share/'ompl_planning.yaml').read_text());ompl['panda_arm']['longest_valid_segment_fraction']=.002
    params={'robot_description':Path('/project/config/panda.urdf').read_text(),'robot_description_semantic':E.tostring(root,encoding='unicode'),'robot_description_kinematics':yaml.safe_load((share/'kinematics.yaml').read_text()),'robot_description_planning':yaml.safe_load((share/'joint_limits.yaml').read_text()),'use_sim_time':True,'planning_pipelines':['ompl'],'default_planning_pipeline':'ompl','ompl':ompl,'moveit_controller_manager':'moveit_simple_controller_manager/MoveItSimpleControllerManager','moveit_simple_controller_manager':controller,'trajectory_execution.allowed_execution_duration_scaling':2.,'trajectory_execution.allowed_goal_duration_margin':3.,'trajectory_execution.allowed_start_tolerance':.025,'publish_robot_description_semantic':True}
    return LaunchDescription([Node(package='moveit_ros_move_group',executable='move_group',output='screen',parameters=[params])])
