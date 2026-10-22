import numpy as np
from astribot_nav_sim.map_cleanup import isolated_specks


def test_only_isolated_specks_inside_known_free_are_removed():
    occupied = np.zeros((60, 60), bool); free = np.ones_like(occupied)
    occupied[:, 0] = True                    # real wall
    occupied[20:25, 20:25] = True             # substantial obstacle
    occupied[40, 40] = True                  # isolated candidate
    occupied[10, 2] = True                   # too close to wall
    occupied[40, 10] = True                  # adjacent to unknown
    free[occupied] = False; free[40, 12] = False
    removed, report = isolated_specks(occupied, free, .05)
    assert removed[40, 40]
    assert removed.sum() == 1
    assert report['removed_components'] == 1
    assert not removed[:, 0].any()
    assert not removed[20:25, 20:25].any()
    assert not removed[10, 2] and not removed[40, 10]
    assert not (removed & ~occupied).any()


def test_thin_walls_and_clusters_above_limit_are_preserved():
    occupied = np.zeros((30, 30), bool)
    occupied[10, 5:25] = True
    occupied[20:22, 20:22] = True
    removed, _ = isolated_specks(occupied, ~occupied, .05)
    assert not removed.any()
