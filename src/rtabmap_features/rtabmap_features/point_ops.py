"""Deterministic FPS/KNN in torch, including CPU support. No model parameters."""
import torch


@torch.no_grad()
def fps(data, number):
    if data.ndim != 3 or data.shape[-1] != 3 or not 1 <= number <= data.shape[1]:
        raise ValueError('FPS expects BxNx3 and 1 <= sample count <= N')
    batch, count, _ = data.shape
    distances = torch.full((batch, count), float('inf'), device=data.device)
    indices = torch.empty((batch, number), dtype=torch.long, device=data.device)
    farthest = torch.zeros(batch, dtype=torch.long, device=data.device)
    batch_indices = torch.arange(batch, device=data.device)
    for i in range(number):
        indices[:, i] = farthest
        center = data[batch_indices, farthest].unsqueeze(1)
        distances = torch.minimum(distances, ((data - center) ** 2).sum(-1))
        farthest = distances.argmax(-1)
    return data[batch_indices[:, None], indices].contiguous()


class TorchKNN:
    def __init__(self, k, transpose_mode=True):
        if k < 1 or not transpose_mode:
            raise ValueError('Only positive-k, BxNxC KNN is supported')
        self.k = k

    @torch.no_grad()
    def __call__(self, reference, query):
        if reference.ndim != 3 or query.ndim != 3 or self.k > reference.shape[1]:
            raise ValueError('Invalid KNN input or insufficient reference points')
        distances = torch.cdist(query, reference, compute_mode='donot_use_mm_for_euclid_dist')
        return distances.topk(self.k, dim=-1, largest=False, sorted=True)
