"""Conservative isolated-speck removal for a derived SIMULATION map."""
import numpy as np
from scipy.ndimage import label, convolve, distance_transform_edt


def isolated_specks(occupied, free, resolution, max_cells=3):
    if occupied.shape != free.shape or resolution <= 0 or max_cells < 1:
        raise ValueError('Invalid cleanup input')
    components, count = label(occupied, np.ones((3, 3), bool))
    sizes = np.bincount(components.ravel())
    small = (components > 0)&(sizes[components] <= max_cells)
    kernel = np.ones((7, 7), int)
    support = convolve(free.astype(float), kernel, mode='constant')/kernel.sum()
    unknown = ~(occupied | free)
    known_surround = convolve(unknown.astype(int), kernel, mode='constant', cval=1) == 0
    substantial = occupied & ~small
    wall_distance = distance_transform_edt(~substantial)*resolution
    eligible = small & known_surround & (support >= .85) & (wall_distance > .15)
    eligible_counts = np.bincount(components[eligible], minlength=len(sizes))
    good = (sizes > 0)&(sizes <= max_cells)&(eligible_counts == sizes)
    good[0] = False
    removed = good[components]
    return removed, dict(occupied_components=int(count), removed_components=int(good.sum()),
                         removed_cells=int(removed.sum()), max_component_cells=max_cells,
                         required_free_neighborhood_fraction=.85,
                         neighborhood_cells=7, minimum_distance_to_large_obstacles_m=.15)
