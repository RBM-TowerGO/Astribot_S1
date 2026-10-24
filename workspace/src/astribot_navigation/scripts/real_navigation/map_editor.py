"""Offline map editor. Never publishes ROS messages or changes the source map."""
import argparse, hashlib, io, json, math, secrets, shlex, threading
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt

class Editor:
    def __init__(self, filename, config):
        self.source=Path(filename).resolve()
        self.meta=yaml.safe_load(self.source.read_text())
        if self.meta.get('mode','trinary')!='trinary': raise ValueError('仅支持 trinary 地图')
        self.image=(self.source.parent/self.meta['image']).resolve()
        self.original=np.asarray(Image.open(self.image).convert('L')).copy()
        self.pixels=self.original.copy()
        self.config=dict(config);self.history=[];self.revision=0;self.lock=threading.RLock()
        self.radius=float(config.get('inflation_radius',.2))
        self.token=secrets.token_urlsafe(32)
    def png(self,overlay=False):
        if overlay:
            p=self.pixels/255.
            probability=p if self.meta.get('negate',0) else 1-p
            occupied=probability>float(self.meta['occupied_thresh'])
            distance=distance_transform_edt(~occupied)*self.meta['resolution'] if occupied.any() else np.full(p.shape,np.inf)
            # Nav2 quantizes the inflation radius upwards to whole cells.
            radius=math.ceil(self.radius/self.meta['resolution'])*self.meta['resolution']
            mask=(distance<=radius)&~occupied
            a=np.zeros((*p.shape,4),dtype=np.uint8);a[mask]=[255,145,40,130]
            image=Image.fromarray(a)
        else:image=Image.fromarray(self.pixels)
        b=io.BytesIO();image.save(b,format='PNG');return b.getvalue()
    def state(self):
        return dict(source=str(self.source),width=self.pixels.shape[1],height=self.pixels.shape[0],
                    resolution=self.meta['resolution'],origin=self.meta['origin'],revision=self.revision,
                    undo=len(self.history),radius=self.radius,footprint=self.config['footprint'],
                    padding=self.config.get('footprint_padding',0))
    def edit(self,obj):
        if obj.get('revision')!=self.revision:raise ValueError('地图已更新，请刷新后重试')
        op=obj['op']
        if op=='erase':
            rect=obj['rect']
            if len(rect)!=4 or any(type(v)!=int for v in rect):raise ValueError('选区坐标无效')
            x0,y0,x1,y1=rect;h,w=self.pixels.shape
            if not (0<=x0<x1<=w and 0<=y0<y1<=h):raise ValueError('请在地图内框选')
            self.history.append((x0,y0,self.pixels[y0:y1,x0:x1].copy()))
            self.history=self.history[-30:]
            self.pixels[y0:y1,x0:x1]=0 if self.meta.get('negate',0) else 255
        elif op=='undo':
            if self.history:
                x,y,a=self.history.pop();self.pixels[y:y+a.shape[0],x:x+a.shape[1]]=a
        elif op=='radius':
            r=float(obj['radius'])
            if not math.isfinite(r) or not 0<=r<=2:raise ValueError('膨胀半径范围为 0–2 m')
            self.radius=r
        elif op=='save':return self.save()
        else:raise ValueError('未知操作')
        self.revision+=1
        return self.state()
    def save(self):
        folder=self.source.parent.parent/('manual_edit_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+secrets.token_hex(3))
        folder.mkdir()
        Image.fromarray(self.pixels).save(folder/'map.pgm')
        meta=dict(self.meta,image='map.pgm')
        (folder/'map.yaml').write_text(yaml.safe_dump(meta,sort_keys=False))
        config=dict(self.config,inflation_radius=self.radius)
        (folder/'navigation.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
        (folder/'edit.json').write_text(json.dumps(dict(source=str(self.source),
            source_image_sha256=hashlib.sha256(self.image.read_bytes()).hexdigest(),
            changed_cells=int(np.count_nonzero(self.original!=self.pixels)),inflation_radius=self.radius),indent=2))
        launcher=Path(__file__).resolve().parent.parent/'tg-navigation.sh'
        command='bash '+shlex.quote(str(launcher))+' --mode hardware --map '+shlex.quote(str(folder/'map.yaml'))+' --config '+shlex.quote(str(folder/'navigation.yaml'))
        return dict(saved=str(folder/'map.yaml'),command=command,message='已保存新地图。当前导航不变；结束当前导航后用下方命令启动。')

def serve(editor,port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def output(self,status,data,kind='application/json'):
            self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
        def do_GET(self):
            with editor.lock:
                route=self.path.split('?')[0]
                if route=='/':return self.output(200,Path(__file__).with_name('map_editor.html').read_bytes().replace(b'__TOKEN__',editor.token.encode()),'text/html; charset=utf-8')
                if route in ('/map.png','/inflation.png'):return self.output(200,editor.png(route=='/inflation.png'),'image/png')
                if route=='/api/state':return self.output(200,json.dumps(editor.state()).encode())
            self.output(404,b'{}')
        def do_POST(self):
            if self.path!='/api/edit' or self.headers.get('X-Session-Token')!=editor.token:return self.output(403,b'{}')
            try:
                self.connection.settimeout(3);size=int(self.headers.get('Content-Length',0))
                if not 0<size<=4096:raise ValueError('请求过大')
                obj=json.loads(self.rfile.read(size))
                with editor.lock:result=editor.edit(obj)
                self.output(200,json.dumps(result,ensure_ascii=False).encode())
            except (ValueError,KeyError,TypeError) as exc:self.output(400,json.dumps(dict(error=str(exc)),ensure_ascii=False).encode())
    ThreadingHTTPServer(('127.0.0.1',port),Handler).serve_forever()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--map',required=True);p.add_argument('--config',default=str(Path(__file__).with_name('navigation.yaml')));p.add_argument('--port',type=int,default=8768)
    args=p.parse_args();serve(Editor(args.map,yaml.safe_load(Path(args.config).read_text())),args.port)
