"""Own every subprocess group and bound shutdown. No unrelated processes touched."""
import os,signal,subprocess,time,json
from pathlib import Path
class Runtime:
    def __init__(self,directory):self.directory=Path(directory);self.directory.mkdir(parents=True,exist_ok=True);self.processes=[];self.files=[]
    def start(self,name,cmd,env=None):
        f=(self.directory/(name+'.log')).open('w');self.files.append(f)
        p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,env=env,start_new_session=True);self.processes.append((name,p));return p
    def close(self):
        for name,p in reversed(self.processes):
            try:os.killpg(p.pid,signal.SIGINT)
            except ProcessLookupError:pass
        for name,p in reversed(self.processes):
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:os.killpg(p.pid,signal.SIGTERM)
                except ProcessLookupError:pass
                try:p.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    try:os.killpg(p.pid,signal.SIGKILL)
                    except ProcessLookupError:pass
                    p.wait(timeout=2)
        for f in self.files:f.close()
        (self.directory/'shutdown.json').write_text(json.dumps({'direct_children_reaped':all(p.returncode is not None for name,p in self.processes),'processes':[{'name':name,'pid':p.pid,'exit_code':p.returncode} for name,p in self.processes]},indent=2))
    def __enter__(self):return self
    def __exit__(self,*args):self.close()
