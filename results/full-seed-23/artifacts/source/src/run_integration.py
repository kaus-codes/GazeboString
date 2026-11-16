#!/usr/bin/env python3
import argparse,os,subprocess,sys,time,fcntl,shutil,hashlib
from datetime import datetime,timezone
from pathlib import Path
import xml.etree.ElementTree as E
import yaml
from runtime import Runtime
ap=argparse.ArgumentParser();ap.add_argument('--phase',choices=['motion','planning','static','moving','full','collision','cartesian'],default='planning');ap.add_argument('--seed',type=int,default=11);ap.add_argument('--gui',action='store_true');ap.add_argument('--record',action='store_true');ap.add_argument('--mode',choices=['standard','clutter'],default='standard');a=ap.parse_args()
lock_handle=open('/project/.run.lock','a')
try:fcntl.flock(lock_handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
except BlockingIOError:raise SystemExit('Another project experiment is running; stop it before launching another.')
os.environ['LP_NUM_THREADS']='2';os.environ['OMP_NUM_THREADS']='1';os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['DISPLAY']=':1';os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH']='/project/build:'+os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')
subprocess.run(['python3','/project/src/build_robot.py'],check=True)
subprocess.run(['python3','/project/src/generate_world.py','--output','/project/worlds/robot_test.sdf','--seed',str(a.seed),'--mode',a.mode],check=True)
os.environ['SORTING_RUN_DIR']=f'/project/results/{a.phase}-seed-{a.seed}'+('' if a.mode=='standard' else '-clutter')
run_dir=Path(os.environ['SORTING_RUN_DIR'])
if run_dir.exists() and any(run_dir.iterdir()):run_dir.rename(str(run_dir)+'-archived-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
run_dir.mkdir(parents=True,exist_ok=True)
log_dir=Path('/project/results/integration-'+a.phase)
if log_dir.exists() and any(log_dir.iterdir()):log_dir.rename(str(log_dir)+'-archived-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
p='/project/worlds/robot_test.sdf';r=E.parse(p);w=r.getroot().find('world')
E.SubElement(w,'plugin',filename='gz-sim-contact-system',name='gz::sim::systems::Contact');E.SubElement(w,'plugin',filename='/project/build/librgb_contact_suction.so',name='ContactSuction');E.SubElement(w,'plugin',filename='/project/build/librgb_contact_suction.so',name='StateAuditor');r.write(p)
artifacts=run_dir/'artifacts';artifacts.mkdir()
for source in ['/project/worlds/robot_test.sdf','/project/worlds/robot_test.evaluation-only.json','/project/config/panda.urdf','/project/config/panda.sdf','/project/config/workstation.json','/project/config/controllers.yaml','/project/config/bridge.yaml']:
    shutil.copy2(source,artifacts/Path(source).name)
source_dir=artifacts/'source';source_dir.mkdir()
source_hashes={}
for directory in ['src','plugins','config','launch','scripts']:
    for source in (Path('/project')/directory).rglob('*'):
        if source.is_file() and '__pycache__' not in str(source) and source.suffix not in ['.pyc','.sdf','.urdf'] and source.name!='robot_description.yaml':
            relative=source.relative_to('/project');destination=source_dir/relative;destination.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,destination);source_hashes[str(relative)]=hashlib.sha256(source.read_bytes()).hexdigest()
(run_dir/'source_sha256.json').write_text(__import__('json').dumps(source_hashes,indent=2))
Path('/project/config/robot_description.yaml').write_text(yaml.safe_dump({'robot_state_publisher':{'ros__parameters':{'robot_description':Path('/project/config/panda.urdf').read_text(),'use_sim_time':True}}}))
with Runtime(run_dir/'runtime') as run:
    if not Path('/tmp/.X11-unix/X1').exists():
        run.start('xvfb',['Xvfb',':1','-screen','0','1280x800x24'])
        deadline=time.monotonic()+5
        while not Path('/tmp/.X11-unix/X1').exists():
            if time.monotonic()>deadline:raise RuntimeError('Xvfb did not become ready')
            time.sleep(.05)
    run.start('gazebo',['gz','sim','-s','-r','-v','3',p]);run.start('rsp',['ros2','run','robot_state_publisher','robot_state_publisher','--ros-args','--params-file','/project/config/robot_description.yaml'])
    if a.gui:
        run.start('vnc',['x11vnc','-display',':1','-forever','-shared','-nopw','-rfbport','5900','-listen','127.0.0.1'])
        run.start('novnc',['websockify','--web','/usr/share/novnc','6080','127.0.0.1:5900'])
        run.start('gui',['gz','sim','-g','--gui-config','/project/config/gui.config'])
        run.start('panel',['python3','/project/src/display_node.py'])
        if a.record:run.start('recording',['ffmpeg','-y','-f','x11grab','-framerate','4','-video_size','1280x800','-i',':1','-c:v','libx264','-preset','ultrafast','-crf','24','-pix_fmt','yuv420p',str(run_dir/'actual_gazebo_screen.mp4')])
    run.start('bridge',['ros2','run','ros_gz_bridge','parameter_bridge','--ros-args','-p','config_file:=/project/config/bridge.yaml'])
    subprocess.run(['python3','/project/src/spawn_robot.py'],check=True,timeout=40)
    subprocess.run(['python3','/project/src/activate_controllers.py'],check=True,timeout=160)
    if a.phase!='motion':run.start('moveit',['ros2','launch','/project/launch/planning.launch.py'])
    if a.phase in ['static','moving','full']:run.start('vision',['python3','/project/src/vision_node.py'])
    probe={'motion':'probe_robot.py','planning':'probe_planner.py','static':'probe_static_pick.py','moving':'sorter.py','full':'sorter.py','collision':'probe_collision.py','cartesian':'probe_cartesian.py'}[a.phase]
    command=['python3','/project/src/'+probe]
    if a.phase in ['moving','full']:
        command+=['--seed',str(a.seed),'--goal','14' if a.phase=='full' else '2','--output',f'/project/results/{a.phase}-seed-{a.seed}'+('' if a.mode=='standard' else '-clutter')]
    result=subprocess.run(command,timeout=1850 if a.phase in ['moving','full'] else 180)
    sys.exit(result.returncode)
