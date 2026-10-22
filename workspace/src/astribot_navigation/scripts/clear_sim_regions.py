#!/usr/bin/env python3
"""Clear user-confirmed occupied pixels in explicit image rectangles, simulation only."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw
import yaml
from scipy.ndimage import label


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', required=True)
    parser.add_argument('--rect', action='append', default=[], help='Image pixels x0,y0,x1,y1 (top-left origin)')
    parser.add_argument('--polygon', action='append', default=[], help='Image pixel polygon x,y;x,y;x,y')
    parser.add_argument('--clear-attached-in-polygons', action='store_true', help='Also clear occupied fragments inside explicitly confirmed polygons')
    parser.add_argument('--output-directory', required=True)
    parser.add_argument('--preview-only', action='store_true')
    args = parser.parse_args()
    source = Path(args.map).resolve(); cfg = yaml.safe_load(source.read_text())
    if cfg.get('negate', 0) or cfg.get('mode', 'trinary') != 'trinary':
        raise ValueError('Only non-negated trinary maps are supported')
    path = source.parent/cfg['image']; pixels = np.asarray(Image.open(path).convert('L')).copy()
    rectangles = []; polygons = []; boxes = []; polygon_mask = np.zeros(pixels.shape, bool)
    if not args.rect and not args.polygon: raise ValueError('At least one selected region is required')
    mask = np.zeros(pixels.shape, bool)
    for text in args.rect:
        x0, y0, x1, y1 = [int(x) for x in text.split(',')]
        if not (0 <= x0 < x1 <= pixels.shape[1] and 0 <= y0 < y1 <= pixels.shape[0]):
            raise ValueError('Invalid rectangle bounds')
        mask[y0:y1, x0:x1] = True; rectangles.append([x0,y0,x1,y1]); boxes.append([x0,y0,x1,y1])
    for text in args.polygon:
        points = [tuple(int(v) for v in p.split(',')) for p in text.split(';')]
        if len(points) < 3 or any(len(p)!=2 or not (0<=p[0]<pixels.shape[1] and 0<=p[1]<pixels.shape[0]) for p in points):
            raise ValueError('Invalid polygon')
        selection = Image.new('1', (pixels.shape[1],pixels.shape[0]))
        ImageDraw.Draw(selection).polygon(points, fill=1)
        polygon_mask |= np.asarray(selection, bool)
        mask |= np.asarray(selection, bool); polygons.append(points)
        boxes.append([min(p[0] for p in points),min(p[1] for p in points),max(p[0] for p in points)+1,max(p[1] for p in points)+1])
    occupied = (255.-pixels)/255. > cfg.get('occupied_thresh', .65)
    # Never cut through a wall merely because its edge enters a screenshot box.
    # Only remove complete obstacle components contained within selected boxes.
    components, _ = label(occupied, np.ones((3,3), bool))
    sizes = np.bincount(components.ravel())
    selected = np.bincount(components[mask & occupied], minlength=len(sizes))
    contained = (sizes > 0)&(sizes == selected)
    contained[0] = False
    cleared = contained[components]
    if args.clear_attached_in_polygons:
        cleared |= polygon_mask & occupied
    before = Image.fromarray(pixels).convert('RGB')
    draw = ImageDraw.Draw(before)
    for i, box in enumerate(rectangles):
        draw.rectangle(box, outline=(255, 50, 40), width=1)
        draw.text((box[0], box[1]-12), str(i+1), fill=(255, 50, 40))
    for i, points in enumerate(polygons):
        draw.line(points+[points[0]], fill=(255,50,40), width=1)
        draw.text((points[0][0],points[0][1]-12), str(len(rectangles)+i+1), fill=(255,50,40))
    crop = [min(r[0] for r in boxes)-20, min(r[1] for r in boxes)-25,
            max(r[2] for r in boxes)+20, max(r[3] for r in boxes)+20]
    crop = [max(0,crop[0]),max(0,crop[1]),min(pixels.shape[1],crop[2]),min(pixels.shape[0],crop[3])]
    output = Path(args.output_directory).resolve()
    output.mkdir(parents=True, exist_ok=False)
    nearest = getattr(Image, 'Resampling', Image).NEAREST
    before.crop(crop).resize(((crop[2]-crop[0])*3, (crop[3]-crop[1])*3), nearest).save(output/'marked_regions.png')
    if args.preview_only:
        print(json.dumps(dict(preview=str(output/'marked_regions.png'), occupied_pixels=int(cleared.sum())))); return
    pixels[cleared] = 254
    Image.fromarray(pixels).save(output/'map.pgm')
    Image.fromarray(pixels).save(output/'preview.png')
    cfg['image'] = 'map.pgm'
    (output/'map.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))
    after = Image.fromarray(pixels).convert('RGB')
    after = after.crop(crop).resize(((crop[2]-crop[0])*3, (crop[3]-crop[1])*3), nearest)
    left = Image.open(output/'marked_regions.png')
    comparison = Image.new('RGB',(left.width*2,left.height),'white')
    comparison.paste(left,(0,0)); comparison.paste(after,(left.width,0)); comparison.save(output/'comparison.png')
    report = dict(source_yaml=str(source), source_image_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  simulation_only=True, authorization='User confirmed empty locations and marked screenshot regions',
                  image_rectangles=rectangles, image_polygons=polygons, cleared_occupied_cells=int(cleared.sum()),
                  cleared_components=int(contained.sum()),
                  selection_rule=('complete contained components plus occupied fragments inside user-confirmed polygons'
                                  if args.clear_attached_in_polygons else 'complete contained components; crossing walls preserved'),
                  polygon_fragments_cleared=args.clear_attached_in_polygons,
                  per_region_cells=[int(cleared[y0:y1,x0:x1].sum()) for x0,y0,x1,y1 in rectangles],
                  unknown_cells_changed=0)
    (output/'manual_edits.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps(dict(output=str(output),**report),ensure_ascii=False,indent=2))


if __name__ == '__main__': main()
