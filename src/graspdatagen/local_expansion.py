"""One-generation local position expansion; all candidates require full PhysX validation."""
import numpy as np
import trimesh
from graspdatagen.config import digest
from graspdatagen.records import CandidateBatch
from graspdatagen.sampling import distinct_indices, posture_mask


def outside(sampler, batch, poses, widths):
    # Contact width estimates the closed opening, not the pregrasp opening.
    joined = np.concatenate((poses, batch.target))
    opening = np.concatenate((widths, batch.contact_width))
    unique = distinct_indices(joined, opening, sampler.config)
    return batch.select(unique[unique >= len(poses)] - len(poses))


def expand(sampler, seeds, poses, widths):
    """4.5/9 mm at default resolution, two local axes, at most 24 seeds/batch."""
    if not len(seeds):
        return seeds
    indices = distinct_indices(poses, widths, sampler.config)
    if len(indices) > 24:
        indices = indices[np.unique(np.linspace(0, len(indices) - 1, 24, dtype=int))]
    seeds, poses, widths = seeds.select(indices), poses[indices], widths[indices]
    approach = poses[:, :3, :3] @ sampler.pair.arrays['approach_axis_tcp']
    opening = poses[:, :3, :3] @ sampler.pair.arrays['opening_axis_tcp']
    tangent = np.cross(approach, opening)
    step = 1.5 * sampler.config.dedup_translation_m
    variants, parent, ordinal = [], [], []
    # Interleave offsets so each seed gets equal opportunity.
    for number, (x, y) in enumerate(((1,0),(-1,0),(0,1),(0,-1),
                                    (2,0),(-2,0),(0,2),(0,-2),
                                    (1,1),(1,-1),(-1,1),(-1,-1))):
        target = poses.copy()
        target[:, :3, 3] += step * (x * tangent + y * approach)
        variants.append(target)
        parent.extend(range(len(seeds)))
        ordinal.extend([number] * len(seeds))
    target = np.concatenate(variants)
    parent = np.asarray(parent)
    # Estimate contact center from the calibrated finger patches at the seed opening.
    patch_centers = []
    for side in range(2):
        patch = trimesh.Trimesh(sampler.pair.arrays[f'contact_patch_{side}_vertices_body_m'],
                               sampler.pair.arrays[f'contact_patch_{side}_faces'], process=False)
        transforms = sampler.pair.arrays['T_B_fingers'][:, side]
        patch_centers.append(transforms[:, :3, :3] @ patch.centroid + transforms[:, :3, 3])
    centers_B = np.mean(patch_centers, axis=0)
    offset = np.column_stack([np.interp(widths[parent], sampler.pair.arrays['opening_m'],
                                      centers_B[:, k]) for k in range(3)])
    base = target @ np.linalg.inv(sampler.pair.arrays['T_B_tcp'])
    centers = np.einsum('nij,nj->ni', base[:, :3, :3], offset) + base[:, :3, 3]
    n = opening[parent]
    radius = sampler.radius
    origins = centers + radius * n
    first = sampler.source.rays(origins, -n, 2 * radius)
    p = origins - first[:, :1] * n
    eps = max(radius * 1e-5, 1e-7)
    second = sampler.source.rays(p - eps * n, -n, 2 * radius)
    width = second[:, 0] + eps
    valid = ((first[:, 4] > 0) & (second[:, 4] > 0)
             & (np.sum(first[:, 1:4] * n, axis=1) > 0.0)
             & (np.sum(second[:, 1:4] * n, axis=1) < -0.0)
             & (width > sampler.pair.arrays['opening_m'][0])
             & (width + 2 * sampler.config.clearance_m < sampler.pair.arrays['opening_m'][-1]))
    # Only recenter across the jaws; retain the requested tangent/depth displacement.
    midpoint = p - 0.5 * width[:, None] * n
    # Preserve seed pose plus requested displacement; no automatic recentering.
    valid &= posture_mask(target, sampler.pair, sampler.posture)
    ids = np.array([int(digest([sampler.identity, 'local-v1', int(seeds.ids[i]), int(j)])[:15],16)+1
                    for i,j in zip(parent,ordinal)], dtype=np.int64)
    pregrasp = target.copy()
    pregrasp[:, :3, 3] -= sampler.config.pregrasp_distance_m * approach[parent]
    chosen = np.full(len(target), -1, dtype=np.int64)
    base = target @ np.linalg.inv(sampler.pair.arrays['T_B_tcp'])
    # Re-evaluate every calibrated opening and the whole approach path, without retracting.
    for state in reversed(range(len(sampler.samples))):
        pending = np.flatnonzero(valid & (chosen < 0) &
                   (width + 2 * sampler.config.clearance_m < sampler.pair.arrays['opening_m'][state]))
        if not len(pending):
            continue
        clear = np.ones(len(pending), dtype=bool)
        for fraction in np.linspace(0, 1, sampler.config.path_samples):
            points = np.einsum('nij,pj->npi',base[pending,:3,:3],sampler.samples[state])
            points += (base[pending,:3,3] + fraction *
                       (pregrasp[pending,:3,3]-target[pending,:3,3]))[:,None,:]
            distance = sampler.proxy.distances(points.reshape(-1,3), radius+1)
            clear &= (distance.reshape(len(pending),-1).min(axis=1) > sampler.config.clearance_m)
            clear &= ((points @ sampler.object_up_axis).min(axis=1) >
                      sampler.bottom + sampler.posture.bottom_clearance_m)
        chosen[pending[clear]] = state
    keep = np.flatnonzero(chosen >= 0)
    batch = CandidateBatch(ids[keep], target[keep], pregrasp[keep],
                           sampler.pair.arrays['opening_m'][chosen[keep]], width[keep],
                           sampler.pair.arrays['joint_positions_m'][chosen[keep]],
                           np.full(len(keep),sampler.pair.definition['closed_command_m']))
    print(f'LOCAL expansion: seeds={len(seeds)} generated={len(target)} geometry_passed={len(batch)}', flush=True)
    return batch
