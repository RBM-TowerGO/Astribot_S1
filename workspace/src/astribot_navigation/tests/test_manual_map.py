from pathlib import Path
import subprocess
import sys
import numpy as np
from PIL import Image
import yaml


def test_manual_clear_preserves_crossing_wall_unknown_and_source(tmp_path):
    pixels = np.full((30, 30), 254, np.uint8)
    pixels[:, 2] = 0
    pixels[8:10, 8] = 0
    pixels[8, 9] = 205
    Image.fromarray(pixels).save(tmp_path/'source.pgm')
    (tmp_path/'source.yaml').write_text(yaml.safe_dump(dict(image='source.pgm',
        resolution=.05, origin=[-1.,2.,0.], mode='trinary', negate=0,
        occupied_thresh=.65, free_thresh=.196)))
    script = Path(__file__).resolve().parents[1]/'scripts/clear_sim_regions.py'
    out = tmp_path/'result'
    command = [sys.executable, str(script), '--map', str(tmp_path/'source.yaml'),
               '--rect', '1,6,12,12', '--output-directory', str(out)]
    result = subprocess.run(command, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    edited = np.array(Image.open(out/'map.pgm'))
    assert (edited[8:10,8] == 254).all()
    assert (edited[:,2] == 0).all()
    assert np.array_equal(edited == 205, pixels == 205)
    assert np.array_equal(np.array(Image.open(tmp_path/'source.pgm')), pixels)
    assert yaml.safe_load((out/'map.yaml').read_text())['origin'] == [-1.,2.,0.]
    assert subprocess.run(command, capture_output=True).returncode != 0


def test_attached_fragments_require_explicit_polygon_and_stay_inside_it(tmp_path):
    pixels = np.full((30,30),254,np.uint8); pixels[15,2:28]=0
    pixels[12:16,10]=0; pixels[12,11]=205
    Image.fromarray(pixels).save(tmp_path/'source.pgm')
    (tmp_path/'source.yaml').write_text(yaml.safe_dump(dict(image='source.pgm',resolution=.05,
        origin=[0.,0.,0.],negate=0,mode='trinary')))
    script=Path(__file__).resolve().parents[1]/'scripts/clear_sim_regions.py'
    command=[sys.executable,str(script),'--map',str(tmp_path/'source.yaml'),
             '--polygon','9,11;12,11;12,14;9,14']
    subprocess.run(command+['--output-directory',str(tmp_path/'kept')],check=True,capture_output=True)
    assert np.array_equal(np.array(Image.open(tmp_path/'kept/map.pgm')),pixels)
    subprocess.run(command+['--clear-attached-in-polygons','--output-directory',str(tmp_path/'edited')],check=True,capture_output=True)
    result=np.array(Image.open(tmp_path/'edited/map.pgm'))
    assert (result[12:15,10]==254).all() and (result[15,2:28]==0).all()
    assert np.array_equal(result==205,pixels==205)
