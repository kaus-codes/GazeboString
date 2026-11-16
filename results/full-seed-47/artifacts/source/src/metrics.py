"""Actual boundary recorder; simulated time and monotonic wall time stay separate."""
import csv,json,time
from pathlib import Path

EVENTS=['eligible_entry','first_detection','color_confirmed','target_selected','planning_start','planning_complete','movement_start','grasp_confirmed','lift_confirmed','release','placement_confirmed','attempt_failed','grasp_start']
class Recorder:
    def __init__(self,directory,metadata):
        self.directory=Path(directory); self.directory.mkdir(parents=True,exist_ok=True); self.events=[]; self.metadata=metadata
        (self.directory/'metadata.json').write_text(json.dumps(metadata,indent=2))
    def record(self,object_id,event,sim_time,**details):
        if event not in EVENTS: raise ValueError(event)
        if sim_time<0: raise ValueError('negative simulated time')
        row={'object_id':object_id,'event':event,'wall_monotonic_ns':time.monotonic_ns(),'sim_time':float(sim_time),'details':details}
        prior=[r for r in self.events if r['object_id']==object_id]
        if prior and (prior[-1]['sim_time']>sim_time or prior[-1]['wall_monotonic_ns']>=row['wall_monotonic_ns']): raise ValueError('clock regression')
        self.events.append(row)
        with (self.directory/'events.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
        return row
    def export(self):
        (self.directory/'events.json').write_text(json.dumps(self.events,indent=2))
        with (self.directory/'events.csv').open('w') as f:
            writer=csv.DictWriter(f,fieldnames=['object_id','event','wall_monotonic_ns','sim_time','details']); writer.writeheader()
            for row in self.events: writer.writerow({**row,'details':json.dumps(row['details'])})
