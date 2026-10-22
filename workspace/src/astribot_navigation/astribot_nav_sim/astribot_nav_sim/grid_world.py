"""Static recorded-map physics. Unknown and out-of-map cells are solid."""
import io
import math
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt, label


class GridWorld:
    def __init__(self, filename, padding=.02):
        path = Path(filename).resolve()
        cfg = yaml.safe_load(path.read_text())
        if cfg.get('mode', 'trinary') != 'trinary':
            raise ValueError('Only trinary maps are supported')
        self.resolution = float(cfg['resolution'])
        self.padding = float(padding)
        if not 0 <= self.padding <= .2: raise ValueError('Invalid footprint padding')
        self.origin = np.asarray(cfg['origin'], dtype=float)
        if self.resolution <= 0 or not np.isfinite(self.resolution) or not np.isfinite(self.origin).all():
            raise ValueError('Invalid map geometry')
        image = np.array(Image.open(path.parent / cfg['image']).convert('L'))
        probability = image / 255. if cfg.get('negate', 0) else (255.-image)/255.
        self.free = (probability < cfg.get('free_thresh', .196))[::-1].copy()
        self.occupied = (probability > cfg.get('occupied_thresh', .65))[::-1].copy()
        self.height, self.width = self.free.shape
        a = self.origin[2]
        self.rotation = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
        # Pad so that image edges are also treated as obstacles.
        self.clearance = distance_transform_edt(np.pad(self.free, 1))[1:-1, 1:-1]*self.resolution
        # Centre connectivity is an orientation-independent *necessary* condition.
        # Use the inscribed radius; exact oriented rectangle checks below govern motion.
        radius = .35+self.padding
        self.navigation_mask = self.free & (self.clearance > radius)
        self.regions, _ = label(self.navigation_mask)
        pixels = np.full(image.shape, 205, np.uint8)
        pixels[self.occupied[::-1]] = 35
        pixels[self.free[::-1]] = 245
        stream = io.BytesIO(); Image.fromarray(pixels).save(stream, format='PNG')
        self.png = stream.getvalue()

    def collision_image(self):
        return Image.fromarray(np.where(self.free[::-1], 254, 0).astype(np.uint8))

    def cells(self, points):
        return np.floor(((np.asarray(points)-self.origin[:2]) @ self.rotation)/self.resolution).astype(int)

    def safe(self, pose, padding=None):
        """Exact rectangle/cell SAT overlap; unknown and touching obstacles block."""
        if len(pose) != 3 or not np.isfinite(pose).all():
            return False
        padding = self.padding if padding is None else padding
        hx, hy = .4+padding, .35+padding
        centre = (np.asarray(pose[:2])-self.origin[:2]) @ self.rotation
        angle = pose[2]-self.origin[2]; c, s = math.cos(angle), math.sin(angle)
        half = self.resolution/2
        ex, ey = hx*abs(c)+hy*abs(s), hx*abs(s)+hy*abs(c)
        if (centre[0]-ex < 0 or centre[1]-ey < 0 or
                centre[0]+ex >= self.width*self.resolution or
                centre[1]+ey >= self.height*self.resolution): return False
        low = np.maximum(0, np.floor((centre-[ex+half, ey+half])/self.resolution).astype(int))
        high = np.minimum([self.width-1, self.height-1], np.floor((centre+[ex+half, ey+half])/self.resolution).astype(int))
        yy, xx = np.mgrid[low[1]:high[1]+1, low[0]:high[0]+1]
        dx, dy = (xx+.5)*self.resolution-centre[0], (yy+.5)*self.resolution-centre[1]
        u, v = c*dx+s*dy, -s*dx+c*dy
        cell_projection = half*(abs(c)+abs(s))
        intersects = ((np.abs(dx) <= ex+half)&(np.abs(dy) <= ey+half)&
                      (np.abs(u) <= hx+cell_projection)&(np.abs(v) <= hy+cell_projection))
        return bool(np.all(self.free[yy, xx][intersects]))

    def initial_pose(self):
        y, x = np.unravel_index(np.argmax(self.clearance), self.clearance.shape)
        p = np.array([x+.5, y+.5])*self.resolution @ self.rotation.T+self.origin[:2]
        pose = np.array([*p, 0.])
        if not self.safe(pose):
            raise ValueError('Map has no space for the simulated robot footprint')
        return pose

    def raycast(self, origin, angles, max_range=12.):
        direction = np.stack([np.cos(angles), np.sin(angles)], axis=1)
        ranges = np.full(len(angles), np.inf)
        alive = np.ones(len(angles), bool)
        for distance in np.arange(self.resolution*.5, max_range, self.resolution*.5):
            ids = np.flatnonzero(alive)
            if not len(ids): break
            cells = self.cells(origin + direction[ids]*distance)
            x, y = cells.T
            inside = (x >= 0)&(y >= 0)&(x < self.width)&(y < self.height)
            known_free = np.zeros(len(ids), bool)
            known_free[inside] = self.free[y[inside], x[inside]]
            hit = ~known_free
            # Unknown blocks the ray but does not manufacture a laser return.
            occupied = np.zeros(len(ids), bool)
            occupied[inside] = self.occupied[y[inside], x[inside]]
            ranges[ids[hit & occupied]] = distance
            alive[ids[hit]] = False
        return ranges

    def metadata(self):
        return dict(width=self.width, height=self.height, resolution=self.resolution,
                    origin=self.origin.tolist(), footprint=[.8, .7], padding=self.padding,
                    collision_model='oriented_rectangle', reachability='inscribed_radius_precheck')

    def region(self, pose):
        if not self.safe(pose): return 0
        x, y = self.cells(pose[:2])
        return int(self.regions[y, x])

    def connected(self, start, goal):
        # Necessary only: a path through the green region may still need rotations
        # which Nav2 must plan and the full rectangle must check at every step.
        region = self.region(start)
        return region != 0 and region == self.region(goal)

    def reachable_png(self, start):
        region = self.region(start)
        rgba = np.zeros((self.height, self.width, 4), np.uint8)
        if region: rgba[(self.regions == region)[::-1]] = [30, 190, 120, 85]
        stream = io.BytesIO(); Image.fromarray(rgba, 'RGBA').save(stream, format='PNG')
        return stream.getvalue()
