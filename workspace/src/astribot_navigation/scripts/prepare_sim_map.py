#!/usr/bin/env python3
"""Create a non-overwriting, explicitly labelled denoised simulation map."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from PIL import Image
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'astribot_nav_sim'))
from astribot_nav_sim.grid_world import GridWorld
from astribot_nav_sim.map_cleanup import isolated_specks


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--map', required=True); p.add_argument('--output-directory', required=True)
    args = p.parse_args()
    source = Path(args.map).resolve(); output = Path(args.output_directory).resolve()
    if output.exists(): raise ValueError('Output directory already exists; choose a new name')
    w = GridWorld(source)
    removed, report = isolated_specks(w.occupied, w.free, w.resolution)
    pixels = np.full(w.free.shape, 205, np.uint8)
    pixels[w.free | removed] = 254; pixels[w.occupied & ~removed] = 0
    cfg = dict(image='map.pgm', mode='trinary', resolution=w.resolution,
               origin=w.origin.tolist(), negate=0, occupied_thresh=.65, free_thresh=.196)
    report.update(source_yaml=str(source), source_yaml_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  source_image_sha256=hashlib.sha256((source.parent/yaml.safe_load(source.read_text())['image']).read_bytes()).hexdigest(),
                  simulation_only=True, unknown_cells_changed=0,
                  original_occupied_cells=int(w.occupied.sum()), original_free_cells=int(w.free.sum()),
                  cleaned_occupied_cells=int((w.occupied & ~removed).sum()),
                  cleaned_free_cells=int((w.free | removed).sum()),
                  note='Small isolated obstacles are treated as noise for simulation; original recordings/maps are preserved.')
    output.mkdir(parents=True, exist_ok=False)
    Image.fromarray(pixels[::-1]).save(output/'map.pgm')
    Image.fromarray(pixels[::-1]).save(output/'preview.png')
    (output/'map.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))
    (output/'cleanup.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(json.dumps(dict(output=str(output), **report), ensure_ascii=False, indent=2))


if __name__ == '__main__': main()
