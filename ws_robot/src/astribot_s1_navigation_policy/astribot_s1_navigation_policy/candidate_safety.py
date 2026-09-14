"""Conservative candidate sweep, including slower motion and continued waiting."""
import math
import numpy as np
from .swept_geometry import clearance_many, path_samples
from .world_geometry import prediction_rows


def candidate_clearance(world, robot, route, profile, evidence=None):
    if evidence is not None:evidence.clear()
    if world.unassociated or not route or robot is None:
        return False, 0., 'UNRESOLVED_WORLD'
    minimum=float('inf');step=min(.025,profile.clearance_margin_m/2)
    heading=next((math.atan2(b[1]-a[1],b[0]-a[0]) for a,b in zip(route,route[1:])
                  if math.dist(a,b)>1e-6),robot.yaw)
    turn=math.remainder(heading-robot.yaw,2*math.pi)
    turns=max(1,math.ceil(abs(turn)/.05))
    rotation_margin=math.hypot(profile.half_length_m,profile.half_width_m)*abs(turn)/(2*turns)
    initial=[(0.,robot.x,robot.y,robot.yaw+turn*j/turns,rotation_margin) for j in range(turns+1)]
    rows=prediction_rows(world,include_current=True,swept=True)
    if not rows.owners.size:return True,minimum,'PREDICTED_SWEEP_CLEAR'
    times=sorted(set(rows.offsets_ns.tolist()));indices={t:i for i,t in enumerate(times)}
    samples=[initial+list(path_samples(route,profile.max_speed_m_s*(t*1e-9),profile)) for t in times]
    count=max(map(len,samples));templates=np.zeros((len(samples),count,5))
    lengths=np.asarray([len(s) for s in samples])
    for i,values in enumerate(samples):templates[i,:len(values)]=values
    # Bound temporary matrix size without dropping obstacles or changing sample order.
    for offset in range(0,len(rows.owners),128):
        end=offset+128
        ids=np.asarray([indices[int(t)] for t in rows.offsets_ns[offset:end]]);poses=templates[ids]
        lower,upper=rows.lower[offset:end],rows.upper[offset:end]
        gaps=clearance_many(poses[:,:,1],poses[:,:,2],poses[:,:,3],
                            lower[:,None,:],upper[:,None,:],profile,poses[:,:,4])
        gaps[np.arange(count)[None,:]>=lengths[ids,None]]=np.inf
        failed=np.flatnonzero(gaps<=0)
        if failed.size:
            row,column=divmod(int(failed[0]),count)
            minimum=min(minimum,float(np.min(gaps.reshape(-1)[:int(failed[0])+1])))
            if evidence is not None:
                track=world.tracks[int(rows.owners[offset+row])].fused_track_id
                t=int(rows.offsets_ns[offset+row])*1e-9;distance,x,y,yaw,_=poses[row,column]
                evidence.update(track=track,prediction_s=t,position=[float(x),float(y)],clearance_m=float(gaps[row,column]))
                if column<len(initial):evidence['phase']='TAKEOVER_ROTATION'
                else:evidence.update(route_distance_m=float(distance),obstacle_lower=lower[row].tolist(),
                                     obstacle_upper=upper[row].tolist(),footprint_yaw_rad=float(yaw),
                                     clearance_margin_m=profile.clearance_margin_m+profile.payload_extra_margin_m+step/2)
            return False,minimum,'PREDICTED_OCCUPANCY'
        minimum=min(minimum,float(np.min(gaps)))
    return True,minimum,'PREDICTED_SWEEP_CLEAR'
