"""Shared immutable array view of explicit and parametric obstacle predictions."""
from dataclasses import dataclass
import numpy as np
from .swept_geometry import bounds_many


@dataclass(frozen=True)
class PredictionRows:
    owners: np.ndarray
    offsets_ns: np.ndarray
    lower: np.ndarray
    upper: np.ndarray


def prediction_rows(world, include_current=False, swept=False):
    groups={};owners=[];times=[];lower=[];upper=[]
    for i,track in enumerate(world.tracks):
        model=track.prediction_model
        if model is not None:
            groups.setdefault(model.steps,[]).append((i,track))
        else:
            previous=track.geometry
            pairs=([(0,previous,previous)] if include_current else [])
            for sample in track.predictions:
                pairs.append((sample.offset_ns,sample.geometry,previous));previous=sample.geometry
            if pairs:
                lo,hi=bounds_many([row[1] for row in pairs],
                                  [row[2] for row in pairs] if swept else None)
                owners.append(np.full(len(pairs),i,dtype=np.int64));times.append(np.array([row[0] for row in pairs],dtype=np.int64))
                lower.append(lo);upper.append(hi)
    for steps,tracks in groups.items():
        offsets=np.array([0,*(step[0] for step in steps)],dtype=np.int64)
        t=np.array([0.,*(step[1] for step in steps)])
        boxes=np.array([(b.geometry.center_m.x,b.geometry.center_m.y,
                         b.geometry.size_m.x/2,b.geometry.size_m.y/2,
                         b.geometry.position_covariance_m2.values[0],b.geometry.position_covariance_m2.values[4])
                        for _,b in tracks])
        velocity=np.array([(b.prediction_model.velocity.x,b.prediction_model.velocity.y) for _,b in tracks])
        variance=np.array([b.prediction_model.variance_m2_s2 for _,b in tracks])
        centers=boxes[:,None,:2]+velocity[:,None,:]*t[None,:,None]
        covariances=boxes[:,None,4:]+variance[:,None,None]*(t*t)[None,:,None]
        lo=centers-boxes[:,None,2:4];hi=centers+boxes[:,None,2:4]
        if swept:
            before=np.maximum(np.arange(len(t))-1,0)
            lo=np.minimum(lo,lo[:,before,:]);hi=np.maximum(hi,hi[:,before,:])
            covariances=np.maximum(covariances,covariances[:,before,:])
        uncertainty=2*np.sqrt(covariances)
        lo-=uncertainty;hi+=uncertainty
        first=0 if include_current else 1
        lower.append(lo[:,first:,:].reshape((-1,2)));upper.append(hi[:,first:,:].reshape((-1,2)))
        owners.append(np.repeat([i for i,_ in tracks],len(t)-first))
        times.append(np.tile(offsets[first:],len(tracks)))
    if owners:
        owner=np.concatenate(owners);offset=np.concatenate(times)
        order=np.lexsort((offset,owner))
        owner,offset=owner[order],offset[order]
        lo,hi=np.concatenate(lower)[order],np.concatenate(upper)[order]
    else:
        owner=np.empty(0,dtype=np.int64);offset=np.empty(0,dtype=np.int64)
        lo=np.empty((0,2));hi=np.empty((0,2))
    if not np.all(np.isfinite(lo)) or not np.all(np.isfinite(hi)) or np.any(lo>hi):
        raise ValueError('prediction bounds must be finite and ordered')
    for array in (owner,offset,lo,hi):array.setflags(write=False)
    return PredictionRows(owner,offset,lo,hi)


def has_predictions(track):
    return bool(track.predictions or (track.prediction_model is not None and track.prediction_model.steps))


def prediction_count(track):
    return len(track.prediction_model.steps) if track.prediction_model is not None else len(track.predictions)


def final_prediction(track):
    model=track.prediction_model
    if model is not None and model.steps:
        from .fusion import translate
        t=model.steps[-1][1]
        return translate(track.geometry,model.velocity,t,t*t*model.variance_m2_s2)
    return track.predictions[-1].geometry if track.predictions else track.geometry
