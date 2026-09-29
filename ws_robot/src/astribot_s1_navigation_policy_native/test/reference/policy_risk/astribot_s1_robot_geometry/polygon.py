"""Convex polygons and filled-polygon/box distances in metres; no ROS dependency."""
import hashlib
import json
import math
import numpy as np
from functools import lru_cache
try:
    from ._geometry_native import box_distance as _native_box_distance, convex_hull as _native_hull
except ImportError:
    # Source-only analysis remains usable; deployed observers require the
    # native module explicitly, so runtime builds cannot silently fall back.
    _native_box_distance = None
    _native_hull = None


def hull(points):
    if _native_hull is not None:return _native_hull(np.asarray(points,dtype=float))
    return _hull_reference(points)


def _hull_reference(points):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        raise ValueError('finite XY vertices required')
    pts = sorted(set(map(tuple, points)))
    def cross(o, a, b): return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    def chain(items):
        out=[]
        for p in items:
            # An absolute area epsilon can delete genuinely outward vertices
            # of micrometre serialization reserves. Remove only non-left turns.
            while len(out)>=2 and cross(out[-2],out[-1],p)<=0:out.pop()
            out.append(p)
        return out
    out = np.asarray(chain(pts)[:-1]+chain(reversed(pts))[:-1])
    if len(out)<3:raise ValueError('nondegenerate polygon required')
    return out


def validate(points):
    points=np.asarray(points,dtype=float)
    if points.ndim!=2 or points.shape[1]!=2 or not 3<=len(points)<=256 or not np.isfinite(points).all():
        raise ValueError('invalid polygon vertices')
    result=hull(points)
    # Reject concavity, repeats and self intersections, rather than silently
    # repairing an externally supplied navigation commitment.
    if len(result)!=len(points):raise ValueError('strictly convex polygon required')
    for order in (result,result[::-1]):
        for shift in range(len(order)):
            if np.allclose(points,np.roll(order,shift,axis=0),atol=1e-9,rtol=0):return result
    raise ValueError('cyclic convex polygon required')


def transform(points,x=0.,y=0.,yaw=0.):
    c,s=math.cos(yaw),math.sin(yaw)
    return np.asarray(points)@np.array([[c,s],[-s,c]])+np.array([x,y])


def inflate(points, radius, sides=32):
    if not math.isfinite(radius) or radius<0:raise ValueError('invalid inflation radius')
    if radius==0:return hull(points)
    # Circumscribed regular polygon contains the entire disk, including between
    # sampled directions. An inscribed sampled circle would underbound corners.
    angles=np.arange(sides)*2*math.pi/sides
    disk=radius/math.cos(math.pi/sides)*np.column_stack((np.cos(angles),np.sin(angles)))
    # Convex Minkowski sum by merging cyclic edge directions. Enumerating every
    # vertex pair spent most of the source lease in Python hull construction.
    # Vertices remain exact sums of the same input and circumscribed disk.
    p=hull(points)
    p=np.roll(p,-np.lexsort((p[:,0],p[:,1]))[0],axis=0)
    disk=np.roll(disk,-np.lexsort((disk[:,0],disk[:,1]))[0],axis=0)
    pe=np.roll(p,-1,axis=0)-p;de=np.roll(disk,-1,axis=0)-disk
    n,m=len(p),len(disk);i=j=0;out=[]
    while i<n or j<m:
        out.append(p[i%n]+disk[j%m])
        if i==n:j+=1;continue
        if j==m:i+=1;continue
        cross=pe[i,0]*de[j,1]-pe[i,1]*de[j,0]
        if cross>=0:i+=1
        if cross<=0:j+=1
    return hull(out)


def contains(outer, inner, tolerance=1e-9):
    outer=validate(outer);inner=np.asarray(inner)
    edges=np.roll(outer,-1,axis=0)-outer
    delta=inner[:,None,:]-outer[None,:,:]
    return bool(np.all(edges[None,:,0]*delta[:,:,1]-edges[None,:,1]*delta[:,:,0]>=
                       -tolerance*np.linalg.norm(edges,axis=1)[None,:]))


def geometry_hash(points, frame, clearance):
    # Decimal quantization is identity only. Collision geometry is never rounded
    # inward and comparisons independently verify vertex positions/containment.
    return hashlib.sha256(json.dumps(dict(frame=frame,clearance=clearance,
        vertices=np.round(validate(points),6).tolist()),sort_keys=True).encode()).hexdigest()


def projection(points, yaw=0.):
    p=transform(points,yaw=yaw)
    return float(p[:,0].min()),float(p[:,0].max()),float(p[:,1].min()),float(p[:,1].max())


def box_distance_many(points, x, y, yaw, lower, upper):
    if _native_box_distance is None:
        return _box_distance_many_reference(points,x,y,yaw,lower,upper)
    lo,hi=np.asarray(lower),np.asarray(upper)
    values=np.broadcast_arrays(x,y,yaw,lo[...,0],lo[...,1],hi[...,0],hi[...,1])
    rows=np.column_stack([np.asarray(v).ravel() for v in values])
    return _native_box_distance(points,rows).reshape(values[0].shape)


def _box_distance_many_reference(points, x, y, yaw, lower, upper):
    """Exact distance when disjoint, nonpositive SAT bound for overlap.

    Handles polygons enclosing an obstacle and zero-area scan endpoints; unlike
    edge-only grid checks, an obstacle strictly inside the polygon is occupied.
    Broadcasts arbitrary arrays of robot poses and axis-aligned obstacle boxes.
    """
    p=np.asarray(points);lo=np.asarray(lower);hi=np.asarray(upper)
    x,y,yaw,lx,ly,ux,uy=np.broadcast_arrays(x,y,yaw,lo[...,0],lo[...,1],hi[...,0],hi[...,1])
    c,s=np.cos(yaw),np.sin(yaw)
    px=x[...,None]+c[...,None]*p[:,0]-s[...,None]*p[:,1]
    py=y[...,None]+s[...,None]*p[:,0]+c[...,None]*p[:,1]
    sep=np.maximum.reduce([px.min(-1)-ux,lx-px.max(-1),py.min(-1)-uy,ly-py.max(-1)])
    distance=np.min(np.hypot(np.maximum(np.maximum(lx[...,None]-px,0.),px-ux[...,None]),
                                 np.maximum(np.maximum(ly[...,None]-py,0.),py-uy[...,None])),axis=-1)
    edges, lengths, normals, support_low, support_high = _supports(tuple(map(tuple,p)))
    ex=c[...,None]*edges[:,0]-s[...,None]*edges[:,1]
    ey=s[...,None]*edges[:,0]+c[...,None]*edges[:,1]
    nx=c[...,None]*normals[:,0]-s[...,None]*normals[:,1]
    ny=s[...,None]*normals[:,0]+c[...,None]*normals[:,1]
    # Rotate support axes instead of re-projecting every vertex onto every axis.
    origin=nx*x[...,None]+ny*y[...,None]
    center=nx*((lx+ux)/2)[...,None]+ny*((ly+uy)/2)[...,None]
    extent=np.abs(nx)*((ux-lx)/2)[...,None]+np.abs(ny)*((uy-ly)/2)[...,None]
    sep=np.maximum(sep,np.maximum(origin+support_low-center-extent,
                                  center-extent-origin-support_high).max(-1))
    for bx,by in ((lx,ly),(lx,uy),(ux,ly),(ux,uy)):
        dx=bx[...,None]-px;dy=by[...,None]-py
        t=np.clip((dx*ex+dy*ey)/(lengths*lengths),0.,1.)
        distance=np.minimum(distance,np.hypot(dx-t*ex,dy-t*ey).min(-1))
    return np.where(sep>0,distance,sep)


@lru_cache(maxsize=128)
def _supports(vertices):
    p=np.asarray(vertices);edges=np.roll(p,-1,axis=0)-p
    lengths=np.linalg.norm(edges,axis=1)
    normals=np.column_stack((edges[:,1],-edges[:,0]))/lengths[:,None]
    projections=p@normals.T
    return edges,lengths,normals,projections.min(0),projections.max(0)
