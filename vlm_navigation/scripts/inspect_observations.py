"""Inspect captured payloads; no depth units or RGB-D registration are assumed."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from PIL import Image


def main():
    p=argparse.ArgumentParser();p.add_argument('capture_dir');p.add_argument('--output',required=True);a=p.parse_args()
    directory=Path(a.capture_dir);r=json.loads((directory/'capture.json').read_text())
    result=dict(capture_dir=str(directory),streams={},payloads=[],calibration={},limitations=[])
    for key,rows in r['streams'].items():
        stamps=np.array([x['stamp_ns'] for x in rows],dtype=np.int64)
        receives=np.array([x['receive_wall_ns'] for x in rows],dtype=np.int64)
        intervals=np.diff(stamps)/1e9
        result['streams'][key]=dict(messages=len(rows),
            observed_hz=(len(rows)-1)/(rows[-1]['receive_monotonic']-rows[0]['receive_monotonic']),
            nonmonotonic_stamps=int((intervals<=0).sum()),frame_ids=sorted(set(x['frame_id'] for x in rows)),
            formats=sorted(set(x['format'] for x in rows)),
            receive_minus_stamp_ms=dict(p50=float(np.percentile((receives-stamps)/1e6,50)),p95=float(np.percentile((receives-stamps)/1e6,95))))
    for entry in r['saved']:
        path=directory/entry['file'];raw=path.read_bytes();meta=r['metadata'].get(entry['stream'])
        item=dict(file=entry['file'],stream=entry['stream'],sha256=hashlib.sha256(raw).hexdigest(),sha256_matches=hashlib.sha256(raw).hexdigest()==entry['sha256'])
        if entry['stream'].endswith('/color'):
            with Image.open(path) as im:
                rgb=np.asarray(im.convert('RGB'));gray=rgb.mean(axis=2)
                item.update(decoded=True,size=list(im.size),mode=im.mode,
                    mean_luminance=float(gray.mean()),dark_fraction=float((gray<10).mean()),bright_fraction=float((gray>245).mean()),
                    matches_message_size=meta is not None and list(im.size)==[meta['width'],meta['height']])
        else:
            if meta is None or meta['encoding']!='mono16' or len(raw)!=meta['width']*meta['height']*2:
                item.update(decoded=False,error='Unverified encoding or payload size')
            else:
                values=np.frombuffer(raw,dtype='>u2' if meta['is_bigendian'] else '<u2').reshape(meta['height'],meta['width'])
                nonzero=values[values>0]
                item.update(decoded=True,shape=list(values.shape),dtype=str(values.dtype),units='unverified raw counts; NOT metres',
                    nonzero_fraction=float((values>0).mean()),percentile_raw_counts=None if not nonzero.size else [float(x) for x in np.percentile(nonzero,[5,50,95])])
        result['payloads'].append(item)
    calibration=json.loads((directory/'sensor_calib.json').read_text())
    for name,c in calibration['camera'].items():
        T=np.array(c['extrinsics']['matrix']).reshape(4,4);R=T[:3,:3]
        resolution=c.get('resolution',c['intrinsics'].get('resolution'))
        result['calibration'][name]=dict(vendor_resolution=resolution,parent_frame=c['extrinsics']['parent_frame'],
            rotation_determinant=float(np.linalg.det(R)),rotation_orthogonality_error=float(np.max(np.abs(R.T@R-np.eye(3)))),
            intrinsics=c['intrinsics'],distortions=c.get('distortions'),metric_calibration_verified=False)
    for camera in ['head_rgbd','torso_rgbd','left_wrist_rgbd','right_wrist_rgbd']:
        color=np.array([x['stamp_ns'] for x in r['streams'][camera+'/color']],dtype=np.int64)
        depth=np.array([x['stamp_ns'] for x in r['streams'][camera+'/depth']],dtype=np.int64)
        nearest=np.min(np.abs(color[:,None]-depth[None,:]),axis=1)/1e6
        result['streams'][camera+'/color']['nearest_depth_header_delta_ms']={f'p{q}':float(np.percentile(nearest,q)) for q in [50,95]}
    result['limitations']=[
        'Header-to-receive differences do not establish sensor exposure time or hardware synchronisation.',
        'Nearest header timestamps are a diagnostic only, not validated RGB-D frame pairing.',
        'Depth counts unit, RGB-depth alignment and colour/depth extrinsics are not verified.',
        'Calibration stream-size mismatches must not be repaired by guessed scaling.',
        'Camera frame IDs empty; captured TF transform count '+str(len(r['transforms']))+'. No camera-to-base chain verified.',
        'Vendor torso flip configuration exists; relation to intrinsic/extrinsic calibration requires verification.']
    out=Path(a.output)
    if out.exists():raise FileExistsError(out)
    out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ['streams','limitations']},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
