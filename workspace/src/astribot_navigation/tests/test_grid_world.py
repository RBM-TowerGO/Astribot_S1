import math
import numpy as np
from PIL import Image
import pytest
import yaml
from astribot_nav_sim.grid_world import GridWorld


def world(tmp_path, origin=(0., 0., 0.)):
    image = np.full((60, 60), 254, np.uint8)
    image[[0, -1], :] = 0; image[:, [0, -1]] = 0
    image[25:35, 40:45] = 205
    Image.fromarray(image).save(tmp_path/'map.pgm')
    (tmp_path/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm', resolution=.1,
        origin=list(origin), negate=0, occupied_thresh=.65, free_thresh=.196)))
    return GridWorld(tmp_path/'map.yaml')


def test_recorded_map_coordinates_and_footprint(tmp_path):
    w = world(tmp_path)
    assert w.safe([2., 2., 0.])
    assert not w.safe([.3, 2., 0.])
    assert not w.safe([4.2, 3., 0.])
    assert not w.safe([-1., 3., 0.])
    assert not w.safe([float('nan'), 2., 0.])
    assert w.safe(w.initial_pose())
    assert w.png.startswith(b'\x89PNG')


def test_collision_map_matches_physics_without_changing_source(tmp_path):
    w = world(tmp_path)
    before = (tmp_path/'map.pgm').read_bytes()
    collision = np.array(w.collision_image())
    np.testing.assert_array_equal(collision == 254, w.free[::-1])
    assert np.all(collision[25:35, 40:45] == 0)
    assert (tmp_path/'map.pgm').read_bytes() == before


def test_rays_hit_walls_but_do_not_turn_unknown_into_wall(tmp_path):
    w = world(tmp_path)
    ranges = w.raycast([2., 3.], np.array([0., math.pi/2, math.pi]))
    assert np.isinf(ranges[0])  # unknown patch blocks without fabricated return
    assert ranges[1] == pytest.approx(2.9, abs=.1)
    assert ranges[2] == pytest.approx(1.9, abs=.1)


def test_rotated_map_origin(tmp_path):
    w = world(tmp_path, (10., 20., math.pi/2))
    np.testing.assert_array_equal(w.cells([7.95, 22.05]), [20, 20])
    assert w.safe([7.95, 22.05, 0.])
    assert not w.safe([10., 20., 0.])


def test_narrow_passage_is_not_reported_as_robot_reachable(tmp_path):
    w = world(tmp_path)
    image = np.array(Image.open(tmp_path/'map.pgm'))
    image[:, 30] = 0
    image[29:32, 30] = 254  # 30 cm opening; endpoints individually have room
    Image.fromarray(image).save(tmp_path/'map.pgm')
    w = GridWorld(tmp_path/'map.yaml')
    start, goal = [2., 2., 0.], [5., 2., 0.]
    assert w.safe(start) and w.safe(goal)
    assert not w.connected(start, goal)
    assert w.connected(start, [2., 3., 0.])
    assert not w.connected(start, [-1., 2., 0.])
    import io
    overlay = np.array(Image.open(io.BytesIO(w.reachable_png(start))))
    assert overlay[:, :30, 3].max() > 0
    assert overlay[:, 31:, 3].max() == 0


def test_oriented_rectangle_fits_corridor_that_circumscribed_circle_rejected(tmp_path):
    image = np.zeros((80, 80), np.uint8)
    image[31:49, 2:78] = 254  # 0.9 m corridor, sufficient when aligned
    Image.fromarray(image).save(tmp_path/'map.pgm')
    (tmp_path/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm', resolution=.05,
        origin=[0., 0., 0.], negate=0, occupied_thresh=.65, free_thresh=.196)))
    w = GridWorld(tmp_path/'map.yaml')
    assert w.safe([2., 2., 0.])
    assert not w.safe([2., 2., math.pi/4])  # rotated corners still collide
    assert not w.safe([2., 1.7, 0.])
    assert w.connected([1., 2., 0.], [3., 2., 0.])
