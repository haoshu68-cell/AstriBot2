"""Retry only after persistent geometric change, independently of track IDs."""
import math
from .world_geometry import has_predictions, final_prediction


def geometry_snapshot(world):
    return tuple((b.center_m.x,b.center_m.y,b.size_m.x,b.size_m.y,
                  2*math.sqrt(b.position_covariance_m2.values[0]),
                  2*math.sqrt(b.position_covariance_m2.values[4]))
                 for t in world.tracks if has_predictions(t)
                 for b in (t.geometry, final_prediction(t)))


def equivalent(a,b,tolerance=.12):
    if not a or not b:return not a and not b
    if tolerance<=0:return False
    def covered(left,right):
        cells={}
        for point in right:
            key=(math.floor(point[0]/tolerance),math.floor(point[1]/tolerance))
            cells.setdefault(key,[]).append(point)
        for point in left:
            x,y=math.floor(point[0]/tolerance),math.floor(point[1]/tolerance)
            if not any(max(abs(a-b) for a,b in zip(point,q))<tolerance
                       for dx in (-1,0,1) for dy in (-1,0,1) for q in cells.get((x+dx,y+dy),())):
                return False
        return True
    return covered(a,b) and covered(b,a)


class ObstructionRetry:
    def __init__(self):self.reset()
    def reset(self):
        self.baseline=None;self.at=None;self.pending=None;self.pending_at=None
    def attempted(self,world,now):
        self.baseline=geometry_snapshot(world);self.at=now;self.pending=None;self.pending_at=None
    def changed(self,world,now):
        if self.baseline is None:return False
        current=geometry_snapshot(world)
        if equivalent(current,self.baseline):
            self.pending=None;self.pending_at=None;return False
        if self.pending is None or not equivalent(current,self.pending):
            self.pending=current;self.pending_at=now;return False
        return now-self.at>=1. and now-self.pending_at>=.5
