import importlib.util
from pathlib import Path
import numpy as np
import pytest
import yaml
from PIL import Image
spec=importlib.util.spec_from_file_location('map_editor',Path(__file__).parents[1]/'scripts/real_navigation/map_editor.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
def make(tmp_path,negate=0):
    a=np.full((20,30),255 if not negate else 0,dtype=np.uint8);a[3:6,4:7]=0 if not negate else 255;a[10,10]=205
    Image.fromarray(a).save(tmp_path/'map.pgm')
    (tmp_path/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.05,origin=[-1,2,.3],negate=negate,occupied_thresh=.65,free_thresh=.196)))
    return module.Editor(tmp_path/'map.yaml',dict(footprint=[[.335,.33],[.335,-.33],[-.335,-.33],[-.335,.33]],footprint_padding=.02))
@pytest.mark.parametrize('negate',[0,1])
def test_erase_undo_save(tmp_path,negate):
    e=make(tmp_path,negate);original=e.image.read_bytes()
    e.edit(dict(op='erase',rect=[4,3,7,6],revision=0))
    assert np.all(e.pixels[3:6,4:7]==(0 if negate else 255))
    assert e.pixels[10,10]==205
    e.edit(dict(op='radius',radius=.35,revision=1))
    saved=e.edit(dict(op='save',revision=2));folder=Path(saved['saved']).parent
    assert yaml.safe_load((folder/'navigation.yaml').read_text())['inflation_radius']==.35
    assert yaml.safe_load((folder/'map.yaml').read_text())['origin']==[-1,2,.3]
    assert e.image.read_bytes()==original
    e.edit(dict(op='undo',revision=2));assert np.array_equal(e.pixels,e.original)
def test_bounds_stale_radius(tmp_path):
    e=make(tmp_path)
    for obj in [dict(op='erase',rect=[-1,0,2,3],revision=0),dict(op='undo',revision=10),dict(op='radius',radius=float('nan'),revision=0)]:
        with pytest.raises(ValueError):e.edit(obj)
def test_overlay_updates(tmp_path):
    import io
    e=make(tmp_path);a=np.asarray(Image.open(io.BytesIO(e.png(True))))
    assert a[3,4,3]==0 and a[3,7,3]>0
    e.edit(dict(op='erase',rect=[0,0,30,20],revision=0))
    assert not np.asarray(Image.open(io.BytesIO(e.png(True))))[:,:,3].any()
