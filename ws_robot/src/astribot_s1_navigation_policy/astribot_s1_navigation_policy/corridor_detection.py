"""Generate straight-passage candidates from known occupied map boundaries.

Unknown cells never count as walls or free clearance. Candidates still pass
through the same live geometry, posture and entrance admission as annotations.
"""
import math
from dataclasses import dataclass
from .corridor import Corridor, angle
from .contracts import finite, require


@dataclass(frozen=True)
class Grid:
    width: int
    height: int
    resolution: float
    origin: tuple
    data: object

    def __post_init__(self):
        finite(self.resolution, 'grid.resolution', 0)
        require(self.resolution>0 and self.width>0 and self.height>0 and
                len(self.data)==self.width*self.height, 'grid.shape')
        require(len(self.origin)==3,'grid.origin')
        for v in self.origin:finite(v,'grid.origin')

    def at(self, x, y):
        dx,dy=x-self.origin[0],y-self.origin[1];c,s=math.cos(self.origin[2]),math.sin(self.origin[2])
        i=math.floor((c*dx+s*dy)/self.resolution);j=math.floor((-s*dx+c*dy)/self.resolution)
        return self.data[j*self.width+i] if 0<=i<self.width and 0<=j<self.height else -1


def detect_corridors(grid, path, posture, max_width_m=1.8, minimum_length_m=.8,
                     heading_tolerance_rad=.05):
    """Scan both sides of the existing route; do not generate a replacement path."""
    for value in (max_width_m, minimum_length_m, heading_tolerance_rad):
        finite(value,'corridor.detector',0);require(value>0,'corridor.detector')
    step=max(grid.resolution,.05)
    samples=[];last=None
    for a,b in zip(path,path[1:]):
        distance=math.dist(a,b)
        if distance<1e-8:continue
        heading=math.atan2(b[1]-a[1],b[0]-a[0])
        for i in range(max(1,math.ceil(distance/step))):
            t=min(distance,i*step)/distance;x=a[0]+t*(b[0]-a[0]);y=a[1]+t*(b[1]-a[1])
            if last is not None and math.dist((x,y),last)<step*.5:continue
            last=(x,y)
            if grid.at(x,y)<0:raise ValueError('UNKNOWN_CORRIDOR_ROUTE')
            if grid.at(x,y)>=65:samples.append(None);continue
            distances=[];unknown=False
            for side in (-1,1):
                hit=None
                for j in range(1,math.ceil(max_width_m/grid.resolution)+1):
                    offset=j*grid.resolution/2
                    if offset>max_width_m/2:break
                    value=grid.at(x-side*math.sin(heading)*offset,y+side*math.cos(heading)*offset)
                    if value<0:unknown=True;break
                    if value>=65:
                        hit=max(0.,offset-grid.resolution);break
                distances.append(hit)
            if unknown and any(v is not None for v in distances):raise ValueError('UNKNOWN_CORRIDOR_BOUNDARY')
            if any(v is None for v in distances):samples.append(None);continue
            right,left=distances
            width=right+left
            shift=(left-right)/2
            samples.append((x-math.sin(heading)*shift,y+math.cos(heading)*shift,heading,width))
    result=[];group=[]
    def finish():
        if len(group)<2:return
        first,last=group[0],group[-1]
        if math.dist(first[:2],last[:2])<minimum_length_m:return
        width=min(point[3] for point in group)
        if width<=0:return
        # Extend the sampled boundaries outward, retaining conservative width.
        h=first[2];dx,dy=step*math.cos(h),step*math.sin(h)
        entry=(first[0]-dx,first[1]-dy);exit=(last[0]+dx,last[1]+dy)
        identifier='auto:' + ':'.join(str(round(v/grid.resolution)) for v in (*entry,*exit))
        result.append(Corridor(identifier,entry,exit,width,(posture,),
                               boundary_margin_m=grid.resolution,tracking_margin_m=.05))
    for sample in (*samples,None):
        if sample is None:
            finish();group=[];continue
        if group and (abs(angle(sample[2]-group[0][2]))>heading_tolerance_rad or
                      math.dist(sample[:2],group[-1][:2])>step*3):
            finish();group=[]
        group.append(sample)
    return tuple(result)
