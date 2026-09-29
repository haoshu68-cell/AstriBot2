"""Scriptable inference-only replacements for GraspNet PointNet2 operators.

Algorithms follow the pinned upstream pointnet2/_ext_src kernels. The original
LICENSE (including Facebook/VoteNet notices) must accompany prepared artifacts.
No trainable layers, learned weights, or grasp candidates are introduced here.
Numerical parity with the legacy CUDA extension is not claimed.
"""
from typing import List, Tuple
import torch


@torch.jit.script
def furthest_point_sample(xyz: torch.Tensor, npoint: int) -> torch.Tensor:
    batch, count, _ = xyz.shape
    selected = torch.zeros((batch, npoint), dtype=torch.long, device=xyz.device)
    distances = torch.full((batch, count), 1.e10, dtype=xyz.dtype, device=xyz.device)
    valid = (xyz * xyz).sum(-1) > 1.e-3
    farthest = torch.zeros((batch,), dtype=torch.long, device=xyz.device)
    rows = torch.arange(batch, device=xyz.device)
    # The CUDA kernel takes the first maximum within each strided lane, then
    # reduces offsets block_size/2,...,1 while keeping the left subtree on
    # ties. That is bit-reversed lane order, not increasing thread index.
    threads = 1
    while threads * 2 <= count and threads < 512:
        threads *= 2
    ids = torch.arange(count, device=xyz.device)
    lane = ids % threads
    lane_rank = torch.zeros_like(ids)
    remaining = threads
    while remaining > 1:
        lane_rank = lane_rank * 2 + lane % 2
        lane = lane // 2
        remaining = remaining // 2
    priority = torch.argsort(lane_rank * count + ids // threads)
    for i in range(npoint):
        selected[:, i] = farthest
        center = xyz[rows, farthest].unsqueeze(1)
        squared = ((xyz - center) ** 2).sum(-1)
        distances = torch.minimum(distances, squared)
        eligible = torch.where(valid, distances, torch.full_like(distances, -1.))
        farthest = priority[eligible[:, priority].argmax(-1)]
    return selected


@torch.jit.script
def gather_operation(features: torch.Tensor, idx: torch.Tensor) -> torch.Tensor:
    return torch.gather(features, 2, idx.long().unsqueeze(1).expand(-1, features.size(1), -1))


@torch.jit.script
def grouping_operation(features: torch.Tensor, idx: torch.Tensor) -> torch.Tensor:
    batch, channels, _ = features.shape
    grouped = torch.gather(features, 2, idx.long().reshape(batch, 1, -1).expand(-1, channels, -1))
    return grouped.reshape(batch, channels, idx.size(1), idx.size(2))


@torch.jit.script
def _first_neighbors(mask: torch.Tensor, nsample: int) -> torch.Tensor:
    count = mask.size(-1)
    ids = torch.arange(count, device=mask.device).view(1, 1, count).expand_as(mask)
    candidates = torch.where(mask, ids, torch.full_like(ids, count))
    values = torch.topk(candidates, min(nsample, count), dim=-1, largest=False, sorted=True)[0]
    first = torch.where(values[:, :, :1] == count, torch.zeros_like(values[:, :, :1]), values[:, :, :1])
    values = torch.where(values == count, first.expand_as(values), values)
    if nsample > count:
        values = torch.cat((values, first.expand(-1, -1, nsample - count)), dim=-1)
    return values


@torch.jit.script
def ball_query(radius: float, nsample: int, xyz: torch.Tensor, new_xyz: torch.Tensor) -> torch.Tensor:
    parts = torch.jit.annotate(List[torch.Tensor], [])
    for start in range(0, new_xyz.size(1), 128):
        delta = xyz.unsqueeze(1) - new_xyz[:, start:start+128].unsqueeze(2)
        mask = (delta * delta).sum(-1) < radius * radius
        parts.append(_first_neighbors(mask, nsample))
    return torch.cat(parts, dim=1)


@torch.jit.script
def cylinder_query(radius: float, hmin: float, hmax: float, nsample: int,
                   xyz: torch.Tensor, new_xyz: torch.Tensor, rot: torch.Tensor) -> torch.Tensor:
    parts = torch.jit.annotate(List[torch.Tensor], [])
    rotations = rot.reshape(rot.size(0), rot.size(1), 3, 3)
    for start in range(0, new_xyz.size(1), 128):
        delta = xyz.unsqueeze(1) - new_xyz[:, start:start+128].unsqueeze(2)
        local = torch.matmul(delta, rotations[:, start:start+128])
        mask = (local[:, :, :, 1:].square().sum(-1) < radius * radius)
        mask = mask & (local[:, :, :, 0] > hmin) & (local[:, :, :, 0] < hmax)
        parts.append(_first_neighbors(mask, nsample))
    return torch.cat(parts, dim=1)


@torch.jit.script
def three_nn(unknown: torch.Tensor, known: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
    squared = ((unknown.unsqueeze(2) - known.unsqueeze(1)) ** 2).sum(-1)
    distances, indices = torch.sort(squared, dim=-1, stable=True)
    return distances[:, :, :3].sqrt(), indices[:, :, :3]


@torch.jit.script
def three_interpolate(features: torch.Tensor, idx: torch.Tensor, weights: torch.Tensor) -> torch.Tensor:
    return (grouping_operation(features, idx) * weights.unsqueeze(1)).sum(-1)


def install_into_upstream():
    """Replace only operators before importing the official network class."""
    import builtins
    import importlib
    import sys
    import types
    previous = getattr(builtins, '__POINTNET2_SETUP__', None)
    builtins.__POINTNET2_SETUP__ = True
    try:
        upstream = importlib.import_module('pointnet2_utils')
    finally:
        if previous is None:
            del builtins.__POINTNET2_SETUP__
        else:
            builtins.__POINTNET2_SETUP__ = previous
    for name in ('furthest_point_sample', 'gather_operation', 'grouping_operation',
                 'ball_query', 'cylinder_query', 'three_nn', 'three_interpolate'):
        setattr(upstream, name, globals()[name])
    # Imported by upstream training labels, but never needed by inference.
    # Fail explicitly if a future upstream change starts using that path.
    training_knn = types.ModuleType('knn_modules')
    def unavailable_training_knn(*args, **kwargs):
        raise RuntimeError('training KNN is not part of the inference export')
    training_knn.knn = unavailable_training_knn
    sys.modules['knn_modules'] = training_knn
