#!/usr/bin/env python3
"""Small, mesh-free physics/perception validation world. No robot yet."""
import argparse, json, math, random
from pathlib import Path
import xml.etree.ElementTree as E
from configuration import CONFIG

def el(p, tag, text=None, **attrs):
    x=E.SubElement(p,tag,attrs)
    if text is not None: x.text=str(text)
    return x

def geometry(p, shape, dims):
    g=el(p,'geometry'); s=el(g,shape)
    if shape=='box': el(s,'size',' '.join(map(str,dims)))
    elif shape=='cylinder': el(s,'radius',dims[0]); el(s,'length',dims[1])
    else: el(s,'radius',dims[0])

def body(world,name,pose,shape,dims,color,static=True,mass=.04):
    m=el(world,'model',name=name); el(m,'pose',' '.join(map(str,pose))); el(m,'static',str(static).lower())
    l=el(m,'link',name='body')
    if not static:
        inert=el(l,'inertial'); el(inert,'mass',mass); it=el(inert,'inertia')
        # Conservative positive definite inertia; exact primitive formulas.
        if shape=='box':
            a,b,c=dims; ii=[mass*(b*b+c*c)/12,mass*(a*a+c*c)/12,mass*(a*a+b*b)/12]
        elif shape=='cylinder':
            r,h=dims; ii=[mass*(3*r*r+h*h)/12]*2+[mass*r*r/2]
        else: ii=[2*mass*dims[0]**2/5]*3
        for k,v in zip(('ixx','iyy','izz'),ii): el(it,k,v)
    c=el(l,'collision',name='collision'); geometry(c,shape,dims)
    surf=el(c,'surface'); fr=el(el(surf,'friction'),'ode'); el(fr,'mu',1); el(fr,'mu2',1)
    v=el(l,'visual',name='visual'); geometry(v,shape,dims)
    mat=el(v,'material'); el(mat,'ambient',color); el(mat,'diffuse',color)
    return m,l

def build(seed=11,mode='standard'):
    rng=random.Random(seed); root=E.Element('sdf',version='1.9'); w=el(root,'world',name='rgb_sorting')
    ph=el(w,'physics',name='physics',type='ignored'); el(ph,'max_step_size',.002); el(ph,'real_time_factor',1)
    for lib,cls in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('sensors','Sensors')]:
        p=el(w,'plugin',filename=f'gz-sim-{lib}-system',name=f'gz::sim::systems::{cls}')
        if lib=='sensors': el(p,'render_engine','ogre2')
    scene=el(w,'scene'); el(scene,'ambient','.65 .65 .65 1'); el(scene,'background','.15 .18 .22 1'); el(scene,'shadows','false')
    sun=el(w,'light',name='sun',type='directional'); el(sun,'pose','0 0 5 0 0 0'); el(sun,'diffuse','.8 .8 .8 1'); el(sun,'direction','0.1 0.2 -1'); el(sun,'cast_shadows','false')
    body(w,'robot_pedestal',[0,.48,.345,0,0,0],'box',[.24,.24,.69],'.42 .46 .5 1')
    body(w,'floor',[0,0,-.06,0,0,0],'box',[7,4,.1],'.28 .31 .34 1')
    m,l=body(w,'conveyor',[0,0,.65,0,0,0],'box',[3,.36,.1],'.12 .14 .16 1')
    p=el(m,'plugin',filename='gz-sim-track-controller-system',name='gz::sim::systems::TrackController'); el(p,'link','body'); el(p,'velocity_topic','/conveyor/speed'); el(p,'odometry_publish_frequency',20)
    for y in [-.205,.205]: body(w,f'rail_{y}',[0,y,.74,0,0,0],'box',[3,.025,.12],'.65 .68 .72 1')
    for x in [-1.25,1.25]:
        for y in [-.16,.16]: body(w,f'leg_{x}_{y}',[x,y,.3,0,0,0],'box',[.06,.06,.6],'.45 .48 .52 1')
    for name,x,color in [('red',-.35,'.85 .02 .02 1'),('blue',.35,'.02 .06 .9 1')]:
        body(w,name+'_bin_base',[x,.68,.64,0,0,0],'box',[.28,.28,.04],color)
        for j,(dx,dy,sx,sy) in enumerate([(-.15,0,.02,.32),(.15,0,.02,.32),(0,-.15,.28,.02),(0,.15,.28,.02)]):
            body(w,name+f'_bin_wall{j}',[x+dx,.68+dy,.74,0,0,0],'box',[sx,sy,.2],color)
    # Texture labels are typeset programmatically; they are scene assets, not evidence.
    from PIL import Image,ImageDraw,ImageFont
    label_dir=Path(__file__).resolve().parents[1]/'models/labels';label_dir.mkdir(parents=True,exist_ok=True)
    font_path='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
    try:font=ImageFont.truetype(font_path,72)
    except OSError:font=ImageFont.load_default()
    for label,(bx,by) in CONFIG['bins'].items():
        image=Image.new('RGB',(512,192),(160,15,15) if label=='RED' else (15,40,170));draw=ImageDraw.Draw(image);draw.text((256,96),label,font=font,fill='white',anchor='mm');path=label_dir/(label.lower()+'.png');image.save(path)
        lm=el(w,'model',name=label+'_label');el(lm,'static','true');el(lm,'pose',f'{bx} {by-.162} .755 0 0 0');ll=el(lm,'link',name='label');vv=el(ll,'visual',name='text');plane=el(el(vv,'geometry'),'plane');el(plane,'normal','0 -1 0');el(plane,'size','.22 .08');mat=el(vv,'material');el(mat,'ambient','1 1 1 1');el(mat,'diffuse','1 1 1 1');metal=el(el(mat,'pbr'),'metal');el(metal,'albedo_map',str(path));el(metal,'metalness',0);el(metal,'roughness',.8)
    # Oblique camera from opposite the arm; Gazebo sensor viewing axis is +X.
    cam=el(w,'model',name='rgb_camera'); el(cam,'static','true'); el(cam,'pose',' '.join(map(str,CONFIG['camera']['position']+CONFIG['camera']['rpy']))); cl=el(cam,'link',name='camera_link')
    s=el(cl,'sensor',name='rgb',type='camera'); el(s,'always_on','true'); el(s,'update_rate',CONFIG['camera']['rate_hz']); el(s,'topic','/sorting/rgb'); ca=el(s,'camera'); el(ca,'horizontal_fov',CONFIG['camera']['horizontal_fov'])
    im=el(ca,'image'); el(im,'width',CONFIG['camera']['width']); el(im,'height',CONFIG['camera']['height']); el(im,'format','R8G8B8'); clip=el(ca,'clip'); el(clip,'near',.05); el(clip,'far',10)
    pp=el(w,'plugin',filename='gz-sim-pose-publisher-system',name='gz::sim::systems::PosePublisher')
    # World-level pose publishing is provided by SceneBroadcaster's dynamic_pose/info.
    w.remove(pp)
    colors={'RED':'.95 .015 .015 1','BLUE':'.015 .04 .95 1','GREEN':'.015 .9 .025 1','YELLOW':'.95 .9 .015 1','ORANGE':'1 .28 .01 1'}
    palette=['RED']*9+['BLUE']*9+['GREEN']*4+['YELLOW']*4+['ORANGE']*4; rng.shuffle(palette)
    shape_order=['box','cylinder','sphere']*4;rng.shuffle(shape_order)
    manifest=[];lane_counts={'box':0,'cylinder':0,'sphere':0}
    # Phase 1: initial batch on belt; later feed must use a validated spawn service.
    for i,color in enumerate(palette[:12]):
        shape=shape_order[i]; dims={'box':[.035,.035,.035],'cylinder':[.019,.035],'sphere':[.019]}[shape]
        x=(-1.05+lane_counts[shape]*.22 if mode=='standard' else -1.30+i*.085);lane_counts[shape]+=1; y=({'box':-.095,'cylinder':0.,'sphere':.095}[shape] if mode=='standard' else rng.uniform(-.10,.10)); yaw=rng.uniform(-math.pi,math.pi)
        name=f'part_{i:03d}'; body(w,name,[x,y,.725,0,0,yaw],shape,dims,colors[color],False)
        manifest.append(dict(name=name,color=color,shape=shape,dimensions=dims,x=x,y=y,yaw=yaw))
    E.indent(root); return E.tostring(root,encoding='unicode'),manifest

if __name__=='__main__':
    ap=argparse.ArgumentParser(); ap.add_argument('--seed',type=int,default=11); ap.add_argument('--mode',choices=['standard','clutter'],default='standard'); ap.add_argument('--output',default='/project/worlds/sorting.sdf'); a=ap.parse_args()
    text,manifest=build(a.seed,a.mode); p=Path(a.output); p.parent.mkdir(parents=True,exist_ok=True); p.write_text(text)
    p.with_suffix('.evaluation-only.json').write_text(json.dumps({'seed':a.seed,'mode':a.mode,'objects':manifest},indent=2))
