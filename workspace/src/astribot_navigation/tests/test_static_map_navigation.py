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
        assert p['inflation_layer']['inflation_radius']==.85
        assert p['footprint_padding']==.05
        assert p['track_unknown_space']
    assert params['controller_server']['ros__parameters']['FollowPath']['use_collision_detection']
    assert params['map_server']['ros__parameters']['yaml_filename']=='/tmp/unchanged-map.yaml'
    assert params['amcl']['ros__parameters']['scan_topic']=='/towergo/nav/scan'


def test_wide_turn_profile_uses_actual_one_meter_primitives():
    import json
    config=yaml.safe_load((ROOT/'scripts/real_navigation/navigation.yaml').read_text())
    params=nav_params(config,Path('/tmp/map.yaml'),False)['towergo']['nav']
    planner=params['planner_server']['ros__parameters']['GridBased']
    assert '/1m_turning_radius/ackermann/' in planner['lattice_filepath']
    assert planner['prefer_straight_route'] is False and planner['smooth_path'] is False
    assert set(map(tuple,config['footprint']))==set(map(tuple,config['self_filter']))
    path=Path(planner['lattice_filepath'])
    if path.exists():
        assert json.loads(path.read_text())['lattice_metadata']['turning_radius']==1


def test_start_inside_enlarged_clearance_is_rejected_before_executor_prepare():
    import ast
    from types import SimpleNamespace as S
    import numpy as np
    from real_navigation.core import speed,controller_parameters
    from real_navigation.geometry import safe_pose
    tree=ast.parse((ROOT/'scripts/real_navigation/web.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Web')
    fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='begin')
    ns=dict(speed=speed,controller_parameters=controller_parameters,safe_pose=safe_pose)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),'begin','exec'),ns)
    config=yaml.safe_load((ROOT/'scripts/real_navigation/navigation.yaml').read_text())
    grid=S(origin=np.array([0.,0.,0.]),rotation=np.eye(2),resolution=.05,width=200,height=200,free=np.ones((200,200),bool))
    grid.free[100,100]=False
    w=S(mode='hardware',phase='idle',state=dict(ready=True,initialized=True,pose=[5.,5.,0.]),config=config,grid=grid,values=lambda obj:[8.,8.,0.])
    with pytest.raises(ValueError,match='当前位置不满足'):ns['begin'](w,dict(speed=.05))


def test_inflation_covers_outer_padded_corners_for_smac_fast_collision_check():
    import math
    config=yaml.safe_load((ROOT/'scripts/real_navigation/navigation.yaml').read_text())
    padding=config['footprint_padding']
    outer=max(math.hypot(abs(x)+padding,abs(y)+padding) for x,y in config['footprint'])
    assert config['inflation_radius']>=outer+math.sqrt(2)*.05


def test_small_saved_inflation_is_raised_to_cover_padded_corners():
    import math
    from real_navigation.core import costmap_inflation_radius
    config=yaml.safe_load((ROOT/'scripts/real_navigation/navigation.yaml').read_text())
    config['inflation_radius']=.2
    effective=costmap_inflation_radius(config)
    outer=max(math.hypot(abs(x)+config['footprint_padding'],abs(y)+config['footprint_padding']) for x,y in config['footprint'])
    assert effective>=outer+math.sqrt(2)*.05
    p=nav_params(config,Path('/tmp/map.yaml'),False)['towergo']['nav']
    for key in ['global_costmap','local_costmap']:
        assert p[key][key]['ros__parameters']['inflation_layer']['inflation_radius']==effective
