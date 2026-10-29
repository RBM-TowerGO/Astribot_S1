"""Deterministic confidence and no-motion update policy; no ROS/SDK."""
import math
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from real_navigation.confidence import Confidence,StaticReassessment


def covariance(x=.02,y=.02,yaw=.01,xy=0):
    c=[0.]*36;c[0]=x;c[7]=y;c[35]=yaw;c[1]=c[6]=xy
    return c

def ready():
    c=Confidence();c.observe(covariance())
    assert not c.tick(0,True,False)
    assert c.tick(2,True,False)
    return c

def test_single_spike_requires_continuous_two_seconds():
    c=ready();c.observe(covariance(yaw=.2919435364))
    assert c.tick(3,True,True) and c.status=='warning'
    assert c.tick(4.99,True,True)
    assert not c.tick(5,True,True) and c.status=='blocked'
    assert '朝向' in c.details['reason']

def test_transient_spike_clears_without_cancellation():
    c=ready();c.observe(covariance(yaw=.277))
    assert c.tick(3,True,True)
    c.observe(covariance());assert c.tick(4,True,True)
    assert c.bad_since is None and c.status=='trusted'

def test_persistent_high_requires_stable_recovery():
    c=ready();c.observe(covariance(y=.28));c.tick(3,True,True)
    assert not c.tick(5,True,True)
    c.observe(covariance());assert not c.tick(6,True,False)
    assert not c.tick(7,False,False)
    assert not c.tick(8,True,False)
    assert not c.tick(9.99,True,False)
    assert c.tick(10,True,False)

@pytest.mark.parametrize('value',[float('nan'),float('inf'),-.001])
def test_invalid_covariance_blocks_immediately(value):
    c=ready();c.observe(covariance(yaw=value));assert not c.tick(3,True,True)
    assert c.status=='invalid'

def test_cross_covariance_and_units_are_preserved():
    c=ready();c.observe(covariance(.2,.2,.01,.1))
    assert c.details['position_variance']==pytest.approx(.3)
    assert c.details['sigma_yaw_deg']==pytest.approx(math.degrees(.1))
    assert '位置' in c.details['reason'] and '朝向' not in c.details['reason']
    c.observe(covariance(.1,.1,.01,.2));assert not c.tick(4,True,True)

def test_no_grace_for_uninitialized_or_bad_matching():
    for initialized,matched in [(False,True),(True,False)]:
        c=ready();c.observe(covariance(yaw=.28))
        assert not c.tick(3,matched,initialized)

def test_bounded_updates_require_new_result_and_expire():
    r=StaticReassessment()
    assert not r.due(0,False,100)
    assert r.started is None
    assert r.due(1,True,100);r.sent(1,100)
    assert not r.due(2,True,100)
    assert r.due(2,True,101);r.sent(2,101)
    assert r.due(3,True,102);r.sent(3,102)
    assert not r.due(4,True,103) and r.count==3
    assert not r.due(11,False,104) and r.status=='manual_required'
    assert not r.due(12,True,105)
    r.reset();assert r.due(13,True,105)
