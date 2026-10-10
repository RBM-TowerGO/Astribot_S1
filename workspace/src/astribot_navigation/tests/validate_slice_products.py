#!/usr/bin/env python3
"""Read back navigation products and MCAP streams; no nodes, SDK or hardware."""
import csv,json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
import numpy as np
from PIL import Image
from mapping_slice_finalize import messages
from mapping_support import check_metadata,RAW_TOPICS


def validate(run):
    run=Path(run);s=json.loads((run/'session.json').read_text());d=run/'navigation';p=json.loads((d/'slice.json').read_text())
    assert s['status']=='complete' and not s['error'] and not s['shutdown_errors']
    assert s['replay'] and s['hardware_commands_sent'] is False
    topics=s['topics'];raw=check_metadata(run/'raw_bag',RAW_TOPICS[:3])
    counts=check_metadata(run/'lio_bag',[topics['odom'],topics['cloud'],topics['occupancy']])
    actual={};last={};grids=0
    for topic,m in messages(run/'lio_bag',list(counts)):
        ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        assert ns>=last.get(topic,0);last[topic]=ns;actual[topic]=actual.get(topic,0)+1
        if topic==topics['occupancy']:
            assert m.header.frame_id==p['frame']
            assert len(m.data)==m.info.width*m.info.height
            assert set(m.data)<={-1,0,100};grids+=1
    assert actual==counts
    with (d/'trajectory.csv').open() as f:rows=list(csv.DictReader(f))
    stamps=[int(r['stamp_ns']) for r in rows];assert all(b>a for a,b in zip(stamps,stamps[1:]))
    assert len(rows)==counts[topics['odom']]
    rotation=np.array(p['map_from_world_R']);anchor=np.array(p['world_front_anchor'])
    for row,(_,m) in zip(rows,messages(run/'lio_bag',[topics['odom']])):
        q=m.pose.pose.position
        np.testing.assert_allclose([float(row[k]) for k in 'xyz'],rotation@(np.array([q.x,q.y,q.z])-anchor),atol=1e-8)
    image=np.array(Image.open(d/'map.pgm'));assert set(np.unique(image))=={0,205,254}
    assert image.shape==(p['grid']['height'],p['grid']['width'])
    assert (image==0).sum()==p['grid']['occupied'];assert (image==254).sum()==p['grid']['free']
    assert p['statistics']['front_hits']>0 and p['statistics']['back_hits']>0
    if p['export_pcd']:
        with (d/'map.pcd').open('rb') as f:
            while f.readline()!=b'DATA binary\n':pass
            points=np.frombuffer(f.read(),dtype='<f4').reshape(-1,3)
        assert len(points)==p['points_3d'] and np.isfinite(points).all()
    live=json.loads((run/'slice_live.json').read_text())
    result=dict(passed=True,session=str(run),grid=p['grid'],poses=len(rows),recorded_grids=grids,
                raw_counts=raw,lio_counts=counts,points_3d=p.get('points_3d'),live_statistics=live['statistics'])
    (run/'product_validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))


if __name__=='__main__':validate(sys.argv[1])
