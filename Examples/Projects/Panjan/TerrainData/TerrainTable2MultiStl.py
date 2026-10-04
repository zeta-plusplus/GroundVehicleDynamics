#!/usr/bin/env python3
"""Split a Modelica CombiTable2D text height map into color-band ASCII STL meshes.
Standard library only. Edit TerrainTable2MultiStl_ctrl.txt, then double-click this script.
"""
from __future__ import annotations
import math
import re
import sys
import traceback
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parent
CONTROL = ROOT / 'TerrainTable2MultiStl_ctrl.txt'
HEADER = re.compile(r'^\s*double\s+([^\s(]+)\s*\(\s*(\d+)\s*,\s*(\d+)\s*\)', re.I)
EPS = 1e-12

@dataclass
class Config:
    source: Path
    output: Path
    table_name: str
    levels: int
    z_scale: float
    pause: bool

def get_config():
    if not CONTROL.is_file():
        raise FileNotFoundError(f'Control file not found: {CONTROL}')
    opts = {}
    for num, raw in enumerate(CONTROL.read_text(encoding='utf-8-sig').splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith(('#', ';', '//')):
            continue
        if '=' not in line:
            raise ValueError(f'Control line {num}: expected KEY = VALUE')
        key, value = line.split('=', 1)
        opts[key.strip().upper()] = value.strip().strip('"').strip("'")
    for key in ('INPUT_FILE', 'TABLE_NAME', 'OUTPUT_FILE'):
        if not opts.get(key):
            raise ValueError(f'Missing {key} in {CONTROL.name}')
    def local(name):
        p = Path(opts[name])
        return p if p.is_absolute() else ROOT / p
    output = local('OUTPUT_FILE')
    if output.suffix.lower() != '.stl':
        raise ValueError('OUTPUT_FILE must end in .stl (e.g. terrain.stl)')
    levels = int(opts.get('LEVELS', '10'))
    if not 1 <= levels <= 256:
        raise ValueError('LEVELS must be between 1 and 256')
    scale = float(opts.get('Z_SCALE', '1.0'))
    if not math.isfinite(scale):
        raise ValueError('Z_SCALE must be finite')
    pause = opts.get('PAUSE_ON_EXIT', 'true').lower()
    if pause not in ('true', 'false', '1', '0', 'yes', 'no'):
        raise ValueError('PAUSE_ON_EXIT must be true or false')
    return Config(local('INPUT_FILE'), output, opts['TABLE_NAME'], levels,
                  scale, pause in ('true', '1', 'yes'))

def read_table(path, name):
    if not path.is_file():
        raise FileNotFoundError(f'Table file not found: {path}')
    lines = path.read_text(encoding='utf-8-sig').splitlines()
    if not any(line.strip() == '#1' for line in lines):
        raise ValueError('Expected Modelica text-table signature #1')
    for pos, line in enumerate(lines):
        match = HEADER.match(line)
        if not match or match[1] != name:
            continue
        nr, nc = int(match[2]), int(match[3])
        if nr < 3 or nc < 3:
            raise ValueError('Table needs at least 3 rows and 3 columns')
        vals = []
        for row in lines[pos + 1:]:
            row = row.split('#', 1)[0].strip()
            if not row:
                continue
            for token in re.split(r'[\s,;]+', row):
                if token:
                    vals.append(float(token))
                    if len(vals) == nr * nc:
                        break
            if len(vals) == nr * nc:
                break
        if len(vals) != nr * nc:
            raise ValueError(f'Expected {nr*nc} numeric entries, got {len(vals)}')
        t = [vals[i*nc:(i+1)*nc] for i in range(nr)]
        if any(not math.isfinite(v) for r in t for v in r):
            raise ValueError('Table contains non-finite values')
        for axis in ([t[i][0] for i in range(1,nr)], t[0][1:]):
            if any(b <= a for a,b in zip(axis, axis[1:])):
                raise ValueError('u1/u2 grid points must be strictly increasing')
        return t
    raise ValueError(f'Table {name!r} not found in {path}')

def vertex(t, i, j, scale):
    return (t[i+1][0], t[0][j+1], t[i+1][j+1]*scale)

def interpolate(a, b, bound):
    dz = b[2] - a[2]
    if abs(dz) <= EPS:
        return a
    f = (bound - a[2])/dz
    return (a[0]+f*(b[0]-a[0]), a[1]+f*(b[1]-a[1]), bound)

def clip(poly, bound, keep_above, closed):
    if not poly:
        return []
    def inside(p):
        return p[2] >= bound - EPS if keep_above else (p[2] <= bound + EPS if closed else p[2] < bound - EPS)
    result = []
    previous = poly[-1]
    old = inside(previous)
    for current in poly:
        new = inside(current)
        if old != new:
            result.append(interpolate(previous, current, bound))
        if new:
            result.append(current)
        previous, old = current, new
    return result

def normal(a, b, c):
    ux,uy,uz = (b[i]-a[i] for i in range(3))
    vx,vy,vz = (c[i]-a[i] for i in range(3))
    n = (uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx)
    length = math.sqrt(sum(v*v for v in n))
    return tuple(v/length for v in n) if length > EPS else None

def facet(file, a, b, c):
    n = normal(a,b,c)
    if n is None:
        return False
    file.write('  facet normal ' + ' '.join(f'{v:.17g}' for v in n) + '\n    outer loop\n')
    for p in (a,b,c):
        file.write('      vertex ' + ' '.join(f'{v:.17g}' for v in p) + '\n')
    file.write('    endloop\n  endfacet\n')
    return True

def generate(cfg, t):
    nu, nv = len(t)-1, len(t[0])-1
    zs = [t[i][j]*cfg.z_scale for i in range(1,nu+1) for j in range(1,nv+1)]
    lo, hi = min(zs), max(zs)
    cfg.output.parent.mkdir(parents=True, exist_ok=True)
    prefix = cfg.output.stem
    active_levels = cfg.levels if hi > lo else 1
    handles = []
    temps = []
    paths = []
    counts = [0] * cfg.levels
    try:
        for band in range(1,cfg.levels+1):
            path = cfg.output.with_name(f'{prefix}[{band}].stl')
            temp = path.with_suffix('.stl.tmp')
            f = temp.open('w', encoding='ascii', newline='\n')
            f.write(f'solid terrain_band_{band}\n')
            handles.append(f)
            temps.append(temp)
            paths.append(path)
        for i in range(nu-1):
            for j in range(nv-1):
                p00=vertex(t,i,j,cfg.z_scale)
                p10=vertex(t,i+1,j,cfg.z_scale)
                p11=vertex(t,i+1,j+1,cfg.z_scale)
                p01=vertex(t,i,j+1,cfg.z_scale)
                for tri in ((p00,p10,p11),(p00,p11,p01)):
                    if active_levels == 1:
                        if facet(handles[0], *tri):
                            counts[0] += 1
                        continue
                    tri_min=min(p[2] for p in tri)
                    tri_max=max(p[2] for p in tri)
                    for band in range(active_levels):
                        lower=lo+(hi-lo)*band/active_levels
                        upper=lo+(hi-lo)*(band+1)/active_levels
                        if tri_max < lower-EPS or tri_min > upper+EPS:
                            continue
                        poly=clip(list(tri),lower,True,True)
                        poly=clip(poly,upper,False,band==active_levels-1)
                        for k in range(1,len(poly)-1):
                            if facet(handles[band],poly[0],poly[k],poly[k+1]):
                                counts[band]+=1
        for band,f in enumerate(handles,1):
            f.write(f'endsolid terrain_band_{band}\n')
            f.close()
        handles.clear()
        for temp,path in zip(temps,paths):
            temp.replace(path)
    finally:
        for f in handles:
            f.close()
        for temp in temps:
            if temp.exists():
                temp.unlink()
    return lo,hi,counts,paths

def main():
    cfg = None
    try:
        cfg=get_config()
        t=read_table(cfg.source,cfg.table_name)
        lo,hi,counts,paths=generate(cfg,t)
        print(f'Completed: {cfg.table_name}, grid {len(t)-1} x {len(t[0])-1}')
        print(f'Z range after scale: {lo:.9g} to {hi:.9g}; levels: {cfg.levels}')
        for i,(path,count) in enumerate(zip(paths,counts),1):
            print(f'  [{i}]: {count} facets -> {path}')
        print('Assign distinct colors to the same-indexed STL shapes in Modelica.')
        return 0
    except Exception as exc:
        print(f'ERROR: {exc}',file=sys.stderr)
        traceback.print_exc()
        return 1
    finally:
        if (cfg is None or cfg.pause) and sys.stdin.isatty():
            try:
                input('Press Enter to close...')
            except EOFError:
                pass

if __name__ == '__main__':
    sys.exit(main())
