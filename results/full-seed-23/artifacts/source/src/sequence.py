"""Independent strict acceptance validator. Only verified releases advance it."""
from dataclasses import dataclass,asdict

@dataclass(frozen=True)
class VerifiedPlacement:
    object_id: str
    perceived_color: str
    evaluated_color: str
    bin_color: str
    grasp_contact: bool
    lift_confirmed: bool
    released: bool
    contained_and_settled: bool
    sim_time: float
    wall_monotonic_ns: int

class SequenceValidator:
    expected=['RED','BLUE']*7
    def __init__(self): self.placements=[]; self.errors=[]
    @property
    def target(self): return self.expected[len(self.placements)] if len(self.placements)<14 else None
    def accept(self,p):
        reason=None
        if self.target is None: reason='extra placement after completion'
        elif p.object_id in {x.object_id for x in self.placements}: reason='duplicate object'
        elif (p.perceived_color,p.evaluated_color,p.bin_color)!=(self.target,)*3: reason='color or bin violates sequence'
        elif not all([p.grasp_contact,p.lift_confirmed,p.released,p.contained_and_settled]): reason='incomplete physical evidence'
        elif self.placements and (p.sim_time<=self.placements[-1].sim_time or p.wall_monotonic_ns<=self.placements[-1].wall_monotonic_ns): reason='nonmonotonic placement timestamps'
        if reason:
            self.errors.append(reason); raise ValueError(reason)
        self.placements.append(p)
    def summary(self):
        n=len(self.placements)
        return {'complete':n==14 and not self.errors,'target':self.target,'red':sum(p.evaluated_color=='RED' for p in self.placements),'blue':sum(p.evaluated_color=='BLUE' for p in self.placements),'pairs':n//2,'successful_picks':n,'remaining_pairs':7-n//2,'errors':list(self.errors),'placements':[asdict(p) for p in self.placements]}
