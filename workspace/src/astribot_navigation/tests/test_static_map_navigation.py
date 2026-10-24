from pathlib import Path
import sys
import yaml
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from real_navigation.session import nav_params

@pytest.mark.parametrize('replay',[False,True])
def test_both_costmaps_use_only_saved_map(replay):
    config=yaml.safe_load((ROOT/'scripts/real_navigation/navigation.yaml').read_text())
    params=nav_params(config,Path('/tmp/unchanged-map.yaml'),replay)['towergo']['nav']
    for name in ('global_costmap','local_costmap'):
        p=params[name][name]['ros__parameters']
        assert p['plugins']==['static_layer','inflation_layer']
        assert 'obstacle_layer' not in p and 'voxel_layer' not in p
        assert p['static_layer']['map_topic']=='/towergo/nav/map'
        assert p['inflation_layer']['inflation_radius']==.2
        assert p['track_unknown_space']
    assert params['controller_server']['ros__parameters']['FollowPath']['use_collision_detection']
    assert params['map_server']['ros__parameters']['yaml_filename']=='/tmp/unchanged-map.yaml'
    assert params['amcl']['ros__parameters']['scan_topic']=='/towergo/nav/scan'
