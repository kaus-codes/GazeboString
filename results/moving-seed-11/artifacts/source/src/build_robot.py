#!/usr/bin/env python3
"""Extend packaged Panda with gz_ros2_control and a contact-sensed suction cup."""
from pathlib import Path
import xml.etree.ElementTree as E
import subprocess
from ament_index_python.packages import get_package_share_directory
from generate_world import el
from configuration import CONFIG

home=CONFIG['robot']['home']
share=Path(get_package_share_directory('moveit_resources_panda_description'))
raw=subprocess.check_output(['xacro',str(share/'urdf/panda.urdf.xacro')],text=True)
r=E.fromstring(raw)
for child in list(r):
    if child.tag=='link' and child.get('name') in ['panda_leftfinger','panda_rightfinger']:r.remove(child)
    elif child.tag=='joint' and child.get('name') in ['panda_finger_joint1','panda_finger_joint2']:r.remove(child)
hand=r.find("link[@name='panda_hand']")
for typ in ['visual','collision']:
    v=el(hand,typ,name='vacuum_stem_'+typ);el(v,'origin',xyz='0 0 .095',rpy='0 0 0');g=el(v,'geometry');el(g,'cylinder',radius='.007',length='.16')

# Explicit world anchor; shoulder is beside the belt on a pedestal.
el(r,'link',name='world')
j=el(r,'joint',name='world_anchor',type='fixed'); el(j,'parent',link='world'); el(j,'child',link='panda_link0'); el(j,'origin',xyz=' '.join(map(str,CONFIG['robot']['base_position'])),rpy=' '.join(map(str,CONFIG['robot']['base_rpy'])))
# Keep original hand/fingers open; nozzle extends below them.
l=el(r,'link',name='suction_cup'); inert=el(l,'inertial'); el(inert,'mass',value='.025'); el(inert,'inertia',ixx='.000004',iyy='.000004',izz='.000004',ixy='0',ixz='0',iyz='0')
for typ in ['visual','collision']:
    v=el(l,typ,name='suction_collision' if typ=='collision' else 'suction_visual'); g=el(v,'geometry'); el(g,'cylinder',radius='.013',length='.01')
    if typ=='visual': mat=el(v,'material',name='suction_black'); el(mat,'color',rgba='.06 .06 .06 1')
j=el(r,'joint',name='suction_joint',type='fixed'); el(j,'parent',link='panda_hand'); el(j,'child',link='suction_cup'); el(j,'origin',xyz='0 0 '+str(CONFIG['grasp']['tool_offset_m']),rpy='0 0 0')
g=el(r,'gazebo',reference='suction_joint'); el(g,'preserveFixedJoint','true')
g=el(r,'gazebo',reference='suction_cup'); sensor=el(g,'sensor',name='suction_contact',type='contact'); el(sensor,'always_on','true'); el(sensor,'update_rate',250); con=el(sensor,'contact'); el(con,'collision','suction_collision')
rc=el(r,'ros2_control',name='GazeboSystem',type='system'); hw=el(rc,'hardware'); el(hw,'plugin','gz_ros2_control/GazeboSimSystem')
for i,q in enumerate(home,1):
    j=el(rc,'joint',name=f'panda_joint{i}'); el(j,'command_interface',name='position'); st=el(j,'state_interface',name='position'); el(st,'param',q,name='initial_value'); el(j,'state_interface',name='velocity')
g=el(r,'gazebo'); p=el(g,'plugin',filename='libgz_ros2_control-system.so',name='gz_ros2_control::GazeboSimROS2ControlPlugin'); el(p,'parameters','/project/config/controllers.yaml')
E.indent(r); Path('/project/config/panda.urdf').write_text(E.tostring(r,encoding='unicode'))
print('Built Panda URDF from',share)

# SDFormat renames URDF collision elements. Resolve the sensor reference from
# the converted model rather than assuming that the URDF name survives.
converted=E.fromstring(subprocess.check_output(['gz','sdf','-p','/project/config/panda.urdf'],text=True))
cup=converted.find(".//link[@name='suction_cup']")
collision=cup.find('collision').get('name')
cup.find('sensor/contact/collision').text=collision
Path('/project/config/panda.sdf').write_text(E.tostring(converted,encoding='unicode'))
print('Resolved suction contact collision:',collision)
